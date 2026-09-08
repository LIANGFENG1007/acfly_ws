"""Isolated ROS regression for corridor watchdogs across independent clocks."""

import argparse
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as RosPath
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool


def stamp(seconds):
    nanoseconds = round(seconds * 1e9)
    return Time(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)


def zero(command):
    return all(abs(component) < 1e-8 for component in command)


class InputStream:
    def __init__(self, node, process):
        self.node = node
        self.process = process
        self.clock_pub = node.create_publisher(Clock, '/clock', 10)
        self.odom_pub = node.create_publisher(Odometry, '/aft_mapped_to_init', 10)
        self.cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', 10)
        self.odom_time = 10.0
        self.cloud_time = 500.0
        self.ros_time = 1788825600.0
        self.yaw = 0.0
        self.commands = []
        self.active = False
        self.finished = False
        node.create_subscription(TwistStamped, '/exploration/cmd_vel', self.on_cmd, 10)
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, '/exploration/corridor_active', self.on_active, latch)
        node.create_subscription(Bool, '/exploration/finished', self.on_finished, latch)
        self.cloud = PointCloud2()
        self.cloud.header.frame_id = 'camera_init'
        # Keep the entrance open and include actual walls without blocking translation.
        points = [(x, 3.4 - index * 0.04, 0.8)
                  for index in range(130) for x in (7.0, 8.5)]
        self.cloud.height, self.cloud.width = 1, len(points)
        self.cloud.fields = [PointField(name=name, offset=index * 4,
                                        datatype=PointField.FLOAT32, count=1)
                             for index, name in enumerate(('x', 'y', 'z'))]
        self.cloud.point_step = 12
        self.cloud.row_step = 12 * len(points)
        self.cloud.is_dense = True
        self.cloud.data = b''.join(struct.pack('<fff', *point) for point in points)

    def on_cmd(self, message):
        value = message.twist
        command = (value.linear.x, value.linear.y, value.linear.z,
                   value.angular.x, value.angular.y, value.angular.z)
        assert all(math.isfinite(component) for component in command), command
        self.commands.append((time.monotonic(), command))

    def on_active(self, message):
        self.active = message.data

    def on_finished(self, message):
        self.finished = message.data

    def pump(self, duration, odom='advance', cloud='advance', advance_ros=True):
        begin = time.monotonic()
        while time.monotonic() - begin < duration:
            assert self.process.poll() is None, 'Planner exited unexpectedly'
            if advance_ros:
                self.ros_time += 0.02
            self.clock_pub.publish(Clock(clock=stamp(self.ros_time)))
            if odom != 'stop':
                if odom != 'repeat':
                    self.odom_time += 0.02
                message = Odometry()
                message.header.frame_id = 'camera_init'
                message.header.stamp = stamp(self.odom_time)
                message.pose.pose.position.x = float('nan') if odom == 'nan' else 7.0
                message.pose.pose.position.y = 4.25
                message.pose.pose.position.z = 0.8
                message.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
                message.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
                self.odom_pub.publish(message)
            if cloud != 'stop':
                if cloud != 'repeat':
                    self.cloud_time += 0.02
                self.cloud.header.stamp = stamp(self.cloud_time)
                self.cloud_pub.publish(self.cloud)
            deadline = time.monotonic() + 0.02
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.002)
        assert not self.finished, 'Mission completed prematurely at the red point'
        return [(received, command) for received, command in self.commands if received >= begin]

    @staticmethod
    def assert_stopped(samples, label, tail=0.18):
        assert samples, f'{label}: no commands received'
        last = samples[-1][0]
        final = [command for received, command in samples if received >= last - tail]
        assert len(final) >= 4, f'{label}: insufficient stop samples'
        assert all(zero(command) for command in final), f'{label}: still moving: {final}'

    @staticmethod
    def assert_translation(samples, label):
        assert any(command[1] > 0.03 for _, command in samples), \
            f'{label}: no positive body-lateral entry command'
        assert all(abs(command[0]) < 0.01 for _, command in samples), \
            f'{label}: entry command did not hold the requested -90 degree heading'


def run(binary, expect_old_failure=False):
    os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 40)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='acfly-corridor-freshness-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        rclpy.init()
        node = rclpy.create_node('corridor_freshness_ros_test')
        logfile = Path(directory) / 'planner.log'
        process = None
        try:
            for _ in range(30):
                rclpy.spin_once(node, timeout_sec=0.01)
            peers = [name for name in node.get_node_names() if name != node.get_name()]
            assert not peers, f'Test domain occupied; refusing to publish: {peers}'
            latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            goal_pub = node.create_publisher(PointStamped, '/exploration/goal', latch)
            route_pub = node.create_publisher(RosPath, '/exploration/corridor_route', latch)
            with logfile.open('w') as output:
                process = subprocess.Popen([
                    binary, '--ros-args', '-p', 'viz:=false', '-p', 'use_sim_time:=true',
                    '-p', 'use_position_control:=true', '-p', 'done_coverage:=0.0',
                    '-p', 'corridor_enabled:=true', '-p', 'corridor_width:=1.5',
                ], stdout=output, stderr=subprocess.STDOUT, env=os.environ.copy())
                stream = InputStream(node, process)
                discovery_deadline = time.monotonic() + 8.0
                while (goal_pub.get_subscription_count() != 1 or
                       route_pub.get_subscription_count() != 1 or
                       stream.odom_pub.get_subscription_count() != 1 or
                       stream.cloud_pub.get_subscription_count() != 1):
                    assert time.monotonic() < discovery_deadline, 'Planner discovery timed out'
                    stream.pump(0.1)
                stream.pump(0.2)
                goal = PointStamped()
                goal.header.frame_id = 'camera_init'
                goal.header.stamp = stamp(stream.ros_time)
                goal.point.x, goal.point.y = 7.0, 4.25
                route = RosPath()
                route.header = goal.header
                for x, y in ((7.75, 4.25), (7.75, -4.25)):
                    pose = PoseStamped()
                    pose.header = goal.header
                    pose.pose.position.x, pose.pose.position.y = x, y
                    pose.pose.orientation.w = 1.0
                    route.poses.append(pose)
                goal_pub.publish(goal)
                route_pub.publish(route)
                samples = stream.pump(1.2)
                assert stream.active, 'Planner never entered corridor mode'
                if expect_old_failure:
                    assert len(samples) >= 30 and all(zero(command) for _, command in samples)
                    assert 'Waiting for fresh odometry' in logfile.read_text()
                    print('PASS: reproduced old failure: 1.2 s of fresh sensor frames '
                          'from independent clocks still yield zero motion commands')
                    return
                assert any(command[5] < -0.1 for _, command in samples), \
                    'Independent odometry clock prevented initial rotation'
                stream.yaw = -math.pi / 2.0
                stream.assert_translation(stream.pump(0.8), 'heading settled')

                for mode, label in (('stop', 'odometry dropout'),
                                    ('repeat', 'frozen odometry timestamp'),
                                    ('nan', 'invalid odometry')):
                    stream.assert_stopped(stream.pump(0.7, odom=mode), label)
                    stream.assert_translation(stream.pump(0.4), label + ' recovery')

                for mode, label in (('stop', 'cloud dropout'),
                                    ('repeat', 'frozen cloud timestamp')):
                    stream.assert_stopped(stream.pump(0.85, cloud=mode), label)
                    stream.assert_translation(stream.pump(0.4), label + ' recovery')

                stream.odom_time -= 5.0
                stream.assert_stopped(stream.pump(0.30, odom='repeat'),
                                      'odometry source clock restart')
                stream.assert_translation(stream.pump(0.4), 'odometry restart recovery')
                stream.cloud_time -= 5.0
                stream.assert_stopped(stream.pump(0.30, cloud='repeat'),
                                      'cloud source clock restart')
                stream.assert_translation(stream.pump(0.4), 'cloud restart recovery')

                # Watchdog expiry must continue even when ROS /clock stops advancing.
                stream.assert_stopped(stream.pump(0.7, odom='stop', advance_ros=False),
                                      'odometry dropout with paused ROS clock')
                stream.assert_translation(stream.pump(0.4, advance_ros=False),
                                          'paused ROS clock recovery')
                print('PASS: independent ROS/odom/cloud clocks, initial rotation, fixed-heading '
                      'entry, odometry dropout/freeze/NaN stop and recovery, cloud dropout/freeze '
                      'stop and recovery, both source clock restarts, watchdog expiry while '
                      'ROS clock is paused')
        except Exception:
            print(logfile.read_text()[-14000:] if logfile.exists() else 'No planner log',
                  file=sys.stderr)
            raise
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', help='Path to exploration_planner_node executable')
    parser.add_argument('--expect-old-failure', action='store_true')
    arguments = parser.parse_args()
    run(arguments.binary, arguments.expect_old_failure)
