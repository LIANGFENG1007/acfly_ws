"""Single stationary synthetic scene: run the installed nodes to a real matching completion."""
from pathlib import Path
import time
import unittest

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String


PREFIX = "/survey_fixture"
ROUTES = (
    "/mavros/state", "/mavros/extended_state", "/mavros/set_mode", "/mavros/cmd/arming",
    "/mavros/setpoint_raw/local", "/aft_mapped_to_init", "/cloud_registered", "/mission/state",
    "/exploration/goal", "/exploration/context", "/exploration/control", "/exploration/finished_goal",
)


def generate_test_description():
    launch = get_package_share_path("acfly_bringup") / "launch" / "survey.launch.py"
    profile = Path(__file__).with_name("stationary_survey.yaml")
    return LaunchDescription([
        GroupAction([
            *(SetRemap(src=src, dst=PREFIX + src) for src in ROUTES),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)),
                                     launch_arguments={"planner_params": str(profile)}.items()),
        ]),
        launch_testing.actions.ReadyToTest(),
    ])


class SurveyCycleTest(unittest.TestCase):
    def test_installed_nodes_complete_a_single_survey(self, proc_output, proc_info):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node("stationary_survey_fixture")
        self.addCleanup(node.destroy_node)
        recorded = []
        landing_requests = []
        outputs = []
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, PREFIX + "/mission/state",
                                 lambda msg: recorded.append(msg.data), retained)
        node.create_subscription(PositionTarget, PREFIX + "/mavros/setpoint_raw/local",
                                 lambda msg: outputs.append(msg.velocity.z), 10)
        state_pub = node.create_publisher(State, PREFIX + "/mavros/state", 10)
        ground_pub = node.create_publisher(ExtendedState, PREFIX + "/mavros/extended_state", 10)
        odom_pub = node.create_publisher(Odometry, PREFIX + "/aft_mapped_to_init", 10)
        cloud_pub = node.create_publisher(PointCloud2, PREFIX + "/cloud_registered", 10)

        def on_mode_request(request, response):
            landing_requests.append(request.custom_mode)
            response.mode_sent = True
            return response

        node.create_service(SetMode, PREFIX + "/mavros/set_mode", on_mode_request)
        deadline = time.monotonic() + 15
        landing_time = None
        while time.monotonic() < deadline:
            if landing_requests and landing_time is None:
                self.assertEqual(landing_requests, ["AUTO.LAND"])
                landing_time = time.monotonic()
            disarmed_after_landing = landing_time is not None and time.monotonic() - landing_time > .35
            state = State()
            state.connected = True
            state.armed = bool(outputs) and "preflight" in recorded and not disarmed_after_landing
            state.mode = "AUTO.LAND" if landing_time else ("OFFBOARD" if state.armed else "MANUAL")
            height = 1.2 if "takeoff" in recorded and landing_time is None else 0.0
            odom = Odometry()
            odom.header = Header(stamp=node.get_clock().now().to_msg(), frame_id="camera_init")
            odom.child_frame_id = "body"
            odom.pose.pose.position.z = height
            odom.pose.pose.orientation.w = 1.0
            ground = ExtendedState()
            ground.landed_state = ExtendedState.LANDED_STATE_IN_AIR if height else ExtendedState.LANDED_STATE_ON_GROUND
            odom_pub.publish(odom)
            cloud_pub.publish(create_cloud_xyz32(odom.header, [(4., 4., 0.)]))
            ground_pub.publish(ground)
            state_pub.publish(state)
            if disarmed_after_landing:
                # Wait for the final state log as the process can exit before DDS delivers its last sample.
                proc_output.assertWaitFor("任务状态: finished", timeout=3)
                proc_info.assertWaitForShutdown("fly_mission_node", timeout=5)
                self.assertEqual(proc_info["fly_mission_node"].returncode, 0)
                break
            until = time.monotonic() + .025
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=max(0., until - time.monotonic()))
        # /mission/state is a keep-last snapshot, not an event journal: DDS may
        # legitimately coalesce a fast exploration -> landing transition.
        with assertSequentialStdout(proc_output, "fly_mission_node") as output:
            for stage in ("preflight", "takeoff", "exploration", "landing", "finished"):
                output.assertInStdout("任务状态: " + stage)
        self.assertTrue(landing_requests, f"No landing request; observed {recorded}")
