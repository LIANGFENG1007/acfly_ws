"""Run the regional surveying nodes with an existing localization and flight bridge."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def survey_nodes(context):
    nodes = []
    for package, option in (("exploration_planner", "planner_params"),
                            ("fly_mission", "mission_params")):
        override = LaunchConfiguration(option).perform(context)
        files = []
        if override:
            path = Path(override).expanduser().resolve()
            if not path.is_file():
                raise RuntimeError(f"{option}: parameter file does not exist: {path}")
            files.append(str(path))
        nodes.append(Node(package=package, executable=package + "_node",
                          parameters=files, output="screen",
                          on_exit=[Shutdown(reason="Single survey mission process exited")]
                          if package == "fly_mission" else None))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planner_params", default_value="",
                              description="Optional planner ROS parameter file."),
        DeclareLaunchArgument("mission_params", default_value="",
                              description="Optional mission ROS parameter file."),
        OpaqueFunction(function=survey_nodes),
    ])
