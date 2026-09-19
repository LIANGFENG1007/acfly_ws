"""Isolated ROS wiring test for the endpoint, entry translation, doors and H."""

import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as RosPath
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool


def corridor_points():
    """Two staggered 0.8 m openings in the user's 1.5 m corridor."""
    points = []
    for i in range(451):
        y = 4.4 - i * 0.02
        points.extend([(7.0, y, 0.8), (8.5, y, 0.8)])
    # Leave the entrance opening on the exploration side.
    points = [p for p in points if not (p[0] == 7.0 and p[1] > 3.5)]
    for i in range(36):
        points.append((7.0 + i * 0.02, 1.8, 0.8))
        points.append((8.5 - i * 0.02, -1.3, 0.8))
    return points


def run(binary):
    os.environ['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 15)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with tempfile.TemporaryDirectory(prefix='acfly-corridor-ros-') as directory:
        os.environ['ROS_LOG_DIR'] = directory
        rclpy.init()
        node = rclpy.create_node('corridor_ros_test')
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        goal_pub = node.create_publisher(PointStamped, '/exploration/goal', latch)
        route_pub = node.create_publisher(RosPath, '/exploration/corridor_route', latch)
        odom_pub = node.create_publisher(Odometry, '/aft_mapped_to_init', 10)
        cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered', 10)
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        state = {'cmd': None, 'active': False, 'finished': False, 'reset': False,
                 'position_count': 0}
        node.create_subscription(TwistStamped, '/exploration/cmd_vel',
                                 lambda m: state.update(cmd=m.twist), 10)

        def active_callback(msg):
            state['active'] = msg.data

        def finished_callback(msg):
            state['finished'] = msg.data
            if not msg.data:
                state['reset'] = True

        def pose_callback(_msg):
            state['position_count'] += 1

        node.create_subscription(Bool, '/exploration/corridor_active', active_callback, latch)
        node.create_subscription(Bool, '/exploration/finished', finished_callback, latch)
        node.create_subscription(PoseStamped, '/exploration/target_pose', pose_callback, 10)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.01)
        peers = [name for name in node.get_node_names() if name != node.get_name()]
        assert not peers, f'Test domain occupied: {peers}'
        logfile = Path(directory) / 'planner.log'
        process = None
        try:
            with logfile.open('w') as output:
                process = subprocess.Popen([
                    binary, '--ros-args', '-p', 'viz:=false', '-p', 'use_sim_time:=true',
                    '-p', 'done_coverage:=0.0',
                    '-p', 'corridor_enabled:=true', '-p', 'corridor_width:=1.5',
                    '-p', 'corridor_cloud_topic:=/cloud_registered',
                ], stdout=output, stderr=subprocess.STDOUT, env=os.environ.copy())
                x, y, yaw, vx, vy, wz = 7.0, 4.25, 0.0, 0.0, 0.0, 0.0
                simulation_time = 100.0
                dt = 0.10
                sent_goal = False
                route_sent = False
                points = corridor_points()
                max_cross_speed = 0.0
                translated = False
                first_translation_yaw = None
                stale_tested = False
                dropout_steps = 0
                dropout_started = None
                real_deadline = time.monotonic() + 100.0
                for step in range(4000):
                    assert time.monotonic() < real_deadline, 'ROS corridor test timed out'
                    assert process.poll() is None, 'Planner exited'
                    simulation_time += dt
                    clock = Clock()
                    clock.clock.sec = int(simulation_time)
                    clock.clock.nanosec = int((simulation_time % 1.0) * 1e9)
                    clock_pub.publish(clock)
                    odom = Odometry()
                    odom.header.stamp = clock.clock
                    odom.header.frame_id = 'camera_init'
                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.y = y
                    odom.pose.pose.position.z = 0.8
                    odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
                    odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
                    odom_pub.publish(odom)
                    if not sent_goal and goal_pub.get_subscription_count() == 1 and step > 15:
                        goal = PointStamped()
                        goal.header = odom.header
                        goal.point.x, goal.point.y = 7.0, 4.25
                        # Intentionally delay route: the red point must hold, never finish.
                        goal_pub.publish(goal)
                        sent_goal = True
                        goal_step = step
                    if sent_goal and not route_sent and step - goal_step > 15:
                        assert state['active'] and state['reset'] and not state['finished']
                        assert math.hypot(x - 7.0, y - 4.25) < 0.05
                        route = RosPath()
                        route.header = goal.header
                        for px, py in [(7.75, 4.25), (7.75, -4.25)]:
                            pose = PoseStamped()
                            pose.header = goal.header
                            pose.pose.position.x, pose.pose.position.y = px, py
                            pose.pose.orientation.w = 1.0
                            route.poses.append(pose)
                        route_pub.publish(route)
                        route_sent = True
                    # Exercise stale sensing once inside the corridor.
                    if translated and y < 3.5 and not stale_tested:
                        dropout_steps += 1
                        if dropout_started is None:
                            dropout_started = time.monotonic()
                        if time.monotonic() - dropout_started > 0.8:
                            cmd = state['cmd']
                            assert abs(cmd.linear.x) < 1e-8 and abs(cmd.linear.y) < 1e-8
                            stale_tested = True
                    if dropout_steps == 0 or stale_tested:
                        visible = [p for p in points if math.hypot(p[0] - x, p[1] - y) <= 6.0]
                        cloud = PointCloud2()
                        cloud.header = odom.header
                        cloud.height, cloud.width = 1, len(visible)
                        cloud.fields = [PointField(name=n, offset=i * 4,
                                                   datatype=PointField.FLOAT32, count=1)
                                        for i, n in enumerate(('x', 'y', 'z'))]
                        cloud.point_step, cloud.row_step = 12, 12 * len(visible)
                        cloud.is_dense = True
                        cloud.data = b''.join(struct.pack('fff', *p) for p in visible)
                        cloud_pub.publish(cloud)
                    end = time.monotonic() + 0.035
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.003)
                    cmd = state['cmd']
                    if cmd is None:
                        continue
                    if state['finished']:
                        assert math.hypot(x - 7.75, y + 4.25) <= 0.12
                        assert stale_tested and translated
                        break
                    assert not state['finished'] or y < -4.0
                    assert math.isfinite(cmd.linear.x) and math.isfinite(cmd.linear.y)
                    if not translated and abs(cmd.linear.y) > 0.03:
                        first_translation_yaw = yaw
                        assert abs(math.atan2(math.sin(yaw + math.pi / 2),
                                              math.cos(yaw + math.pi / 2))) < 0.12
                    if x > 7.67 and y < 4.0:
                        translated = True
                    if translated:
                        assert math.hypot(cmd.linear.x, cmd.linear.y) <= 0.300001
                        heading_error = math.atan2(math.sin(yaw + math.pi / 2),
                                                   math.cos(yaw + math.pi / 2))
                        assert abs(heading_error) < 0.10, 'Corridor or H approach changed heading'
                    max_cross_speed = max(max_cross_speed, cmd.linear.x)
                    c, s = math.cos(yaw), math.sin(yaw)
                    alpha = 1.0 - math.exp(-dt / 0.15)
                    vx += alpha * (c * cmd.linear.x - s * cmd.linear.y - vx)
                    vy += alpha * (s * cmd.linear.x + c * cmd.linear.y - vy)
                    wz += alpha * (cmd.angular.z - wz)
                    x += vx * dt
                    y += vy * dt
                    yaw += wz * dt
                else:
                    raise AssertionError('H was not reached')
                assert first_translation_yaw is not None, 'Entry did not translate'
                assert max_cross_speed > 0.27, 'Did not reach configured crossing speed'
                assert state['position_count'] == 0, 'Velocity-only exploration published a position target'
                log = logfile.read_text()
                assert '已过门 2' in log, 'Did not register both doors'
                print(f'PASS: red point held, -90 degree translation, two 0.8m doors, '
                      f'stale cloud stop/resume, H completion at ({x:.3f},{y:.3f}); '
                      f'maximum forward command={max_cross_speed:.3f}m/s')
        except Exception:
            print(logfile.read_text()[-14000:] if logfile.exists() else 'No log', file=sys.stderr)
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
    run(sys.argv[1])
