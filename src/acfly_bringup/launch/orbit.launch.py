"""Run only the orbit mission, with localization and MAVROS started separately."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def orbit_node(context):
    override = LaunchConfiguration("mission_params").perform(context)
    parameters = []
    if override:
        path = Path(override).expanduser().resolve()
        if not path.is_file():
            raise RuntimeError(f"mission_params: parameter file does not exist: {path}")
        parameters.append(str(path))
    parameters.append({"mission_action": "orbit"})
    return [Node(package="fly_mission", executable="fly_mission_node",
                 parameters=parameters, output="screen",
                 on_exit=[Shutdown(reason="Orbit mission process exited")])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mission_params", default_value="",
                              description="Optional YAML overrides for fly_mission_node."),
        OpaqueFunction(function=orbit_node),
    ])
