"""Check the real arena's occluded, voxelized entrance cloud outside FIELD bounds."""

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
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool


def run(binary, scene_binary):
    text = subprocess.check_output([scene_binary, '--dump-entry-cloud'], text=True)
    points = [tuple(map(float, line.split())) for line in text.splitlines()]
    assert points and any(x > 7.5 for x, _ in points)
    assert any(y > 4.9 for _, y in points), 'Fixture must include the rear wall'
    os.environ['ROS_DOMAIN_ID'] = str(90 + os.getpid() % 30)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='acfly-arena-ros-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        rclpy.init()
        node = rclpy.create_node('corridor_arena_ros_test')
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        goal_pub = node.create_publisher(PointStamped, '/exploration/goal', latch)
        route_pub = node.create_publisher(RosPath, '/exploration/corridor_route', latch)
        odom_pub = node.create_publisher(Odometry, '/aft_mapped_to_init', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', 10)
        commands = []
        finished = []
        node.create_subscription(TwistStamped, '/exploration/cmd_vel',
                                 lambda msg: commands.append(msg.twist), 10)
        node.create_subscription(Bool, '/exploration/finished',
                                 lambda msg: finished.append(msg.data), latch)
        process = None
        logfile = Path(directory) / 'planner.log'
        try:
            for _ in range(30):
                rclpy.spin_once(node, timeout_sec=0.01)
            peers = [name for name in node.get_node_names() if name != node.get_name()]
            assert not peers, f'Test domain occupied: {peers}'
            with logfile.open('w') as output:
                process = subprocess.Popen([
                    binary, '--ros-args', '-p', 'viz:=false', '-p', 'done_coverage:=0.0',
                    '-p', 'corridor_enabled:=true', '-p', 'field_max_x:=7.5',
                    '-p', 'corridor_width:=1.5', '-p', 'corridor_robot_width:=0.5',
                ], stdout=output, stderr=subprocess.STDOUT, env=os.environ.copy())
                sent = False
                started = time.monotonic()
                step = 0
                while time.monotonic() - started < 4.0:
                    assert process.poll() is None, 'Planner exited'
                    step += 1
                    odom = Odometry()
                    odom.header.frame_id = 'camera_init'
                    odom.header.stamp = Time(
                        sec=10 + step // 50, nanosec=(step % 50) * 20000000)
                    odom.pose.pose.position.x = 8.25
                    odom.pose.pose.position.y = 4.25
                    odom.pose.pose.position.z = 0.71
                    odom.pose.pose.orientation.z = math.sin(-math.pi / 4)
                    odom.pose.pose.orientation.w = math.cos(-math.pi / 4)
                    odom_pub.publish(odom)
                    cloud = PointCloud2()
                    cloud.header.frame_id = 'camera_init'
                    cloud.header.stamp = Time(
                        sec=500 + step // 50, nanosec=(step % 50) * 20000000)
                    cloud.height, cloud.width = 1, len(points)
                    cloud.fields = [PointField(name=name, offset=i * 4,
                                               datatype=PointField.FLOAT32, count=1)
                                    for i, name in enumerate(('x', 'y', 'z'))]
                    cloud.point_step, cloud.row_step = 12, len(points) * 12
                    cloud.is_dense = True
                    cloud.data = b''.join(struct.pack('<fff', x, y, 0.71) for x, y in points)
                    cloud_pub.publish(cloud)
                    if not sent and goal_pub.get_subscription_count() == 1 and step > 20:
                        goal = PointStamped()
                        goal.header.frame_id = 'camera_init'
                        goal.header.stamp = node.get_clock().now().to_msg()
                        goal.point.x, goal.point.y = 8.25, 4.25
                        route = RosPath()
                        route.header = goal.header
                        for x, y in ((8.25, 4.25), (8.25, -4.25)):
                            pose = PoseStamped()
                            pose.header = goal.header
                            pose.pose.position.x, pose.pose.position.y = x, y
                            pose.pose.orientation.w = 1.0
                            route.poses.append(pose)
                        route_pub.publish(route)
                        goal_pub.publish(goal)
                        sent = True
                    until = time.monotonic() + 0.02
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.002)
                assert sent and commands
                assert not any(finished), 'Must not finish at the entrance'
                assert any(cmd.linear.x > 0.03 and cmd.linear.y < -0.001 for cmd in commands), \
                    'Did not move toward the first measured opening'
                assert 'Approaching door center' in logfile.read_text()
                print(f'PASS: {len(points)} actual arena voxelized visible returns, rear wall '
                      'ignored as a goal, forward door acquired outside FIELD_MAX_X=7.5')
        except Exception:
            print(logfile.read_text()[-12000:] if logfile.exists() else 'No log', file=sys.stderr)
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
    run(sys.argv[1], sys.argv[2])
