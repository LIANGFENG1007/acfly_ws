"""Verify the installed mission and planner agree on type, direction and latch policy."""
import time
import unittest

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
import launch_testing.actions
import rclpy
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


def generate_test_description():
    launch = get_package_share_path("acfly_bringup") / "launch" / "survey.launch.py"
    # No flight bridge or sensor is started. Also isolate their topic/service names.
    routes = ("/mavros/state", "/mavros/extended_state", "/mavros/set_mode", "/mavros/cmd/arming",
              "/mavros/setpoint_raw/local", "/aft_mapped_to_init", "/cloud_registered")
    return LaunchDescription([
        GroupAction([*(SetRemap(src=src, dst="/bench" + src) for src in routes),
                     IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)))]),
        launch_testing.actions.ReadyToTest(),
    ])


class SurveyGraphTest(unittest.TestCase):
    def test_mission_protocol(self):
        rclpy.init()
        self.addCleanup(rclpy.shutdown)
        node = rclpy.create_node("survey_graph_inspector")
        self.addCleanup(node.destroy_node)
        mission, planner = "fly_mission_node", "exploration_planner_node"
        links = {
            "goal": (mission, planner, "geometry_msgs/msg/PointStamped", True),
            "context": (mission, planner, "acfly_interfaces/msg/ExplorationContext", True),
            "control": (planner, mission, "acfly_interfaces/msg/ExplorationCommand", False),
            "finished_goal": (planner, mission, "std_msgs/msg/Header", True),
        }
        deadline = time.monotonic() + 12
        for suffix, (pub_name, sub_name, type_name, latched) in links.items():
            topic = "/exploration/" + suffix
            while True:
                pubs = [ep for ep in node.get_publishers_info_by_topic(topic)
                        if ep.node_name == pub_name]
                subs = [ep for ep in node.get_subscriptions_info_by_topic(topic)
                        if ep.node_name == sub_name]
                if pubs and subs:
                    break
                self.assertLess(time.monotonic(), deadline, f"No {pub_name} -> {topic} -> {sub_name}")
                rclpy.spin_once(node, timeout_sec=0.05)
            for ep in (*pubs, *subs):
                self.assertEqual(ep.topic_type, type_name, topic)
                self.assertEqual(ep.qos_profile.reliability, ReliabilityPolicy.RELIABLE, topic)
                self.assertEqual(ep.qos_profile.durability, DurabilityPolicy.TRANSIENT_LOCAL
                                 if latched else DurabilityPolicy.VOLATILE, topic)
