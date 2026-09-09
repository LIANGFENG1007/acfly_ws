"""Bring up competition ROS nodes for an already running PX4/Gazebo world."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def mission_node(context):
    parameter_file = LaunchConfiguration("mission_param_file").perform(context)
    return [Node(package="fly_mission", executable=LaunchConfiguration("mission_executable"),
                 parameters=[parameter_file] if parameter_file else [], output="screen")]


def generate_launch_description():
    profile = PathJoinSubstitution([
        FindPackageShare("exploration_planner"), "config", "physical_corridor.yaml"])
    arguments = [
        DeclareLaunchArgument("param_file", default_value=profile,
                              description="Planner YAML; defaults to the validated x500 profile."),
        DeclareLaunchArgument("viz", default_value="true", choices=["true", "false"],
                              description="Show the exploration and corridor trajectory window."),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"],
                              description="Start Point-LIO RViz when Point-LIO is included."),
        DeclareLaunchArgument("fcu_url", default_value="udp://:14540@127.0.0.1:14557",
                              description="MAVROS FCU connection URL."),
        DeclareLaunchArgument("with_mavros", default_value="true", choices=["true", "false"],
                              description="Start MAVROS; disable when it is already running."),
        DeclareLaunchArgument("with_point_lio", default_value="true", choices=["true", "false"],
                              description="Reused mapping must publish body scans."),
        DeclareLaunchArgument("planner_executable", default_value="exploration_planner_node",
                              description="Planner executable name or absolute snapshot path."),
        DeclareLaunchArgument("mission_executable", default_value="fly_mission_node",
                              description="Mission executable name or absolute snapshot path."),
        DeclareLaunchArgument("mission_param_file", default_value="",
                              description="Optional mission ROS parameter YAML."),
    ]

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("point_lio"), "launch", "mapping_sim.launch.py"])),
        launch_arguments={"rviz": LaunchConfiguration("rviz"),
                          "dense_body_cloud": "true",
                          "dense_cloud_adapter": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("with_point_lio")),
    )
    mavros = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("mavros"), "launch", "px4.launch"])),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        condition=IfCondition(LaunchConfiguration("with_mavros")),
    )
    dense_cloud = Node(
        package="exploration_planner",
        executable="dense_cloud_bridge",
        name="corridor_dense_cloud_bridge",
        # Apt ROS Humble and numpy are installed for the system Python. Bypass
        # an activated virtual environment's interpreter despite the env shebang.
        prefix="/usr/bin/python3",
        output="screen",
    )
    planner = Node(
        package="exploration_planner",
        executable=LaunchConfiguration("planner_executable"),
        output="screen",
        parameters=[
            {"corridor_continuous_approach": False},
            LaunchConfiguration("param_file"),
            {"corridor_cloud_topic": "/corridor/cloud_registered_dense",
             "viz": ParameterValue(LaunchConfiguration("viz"), value_type=bool)},
        ],
    )
    mission = OpaqueFunction(function=mission_node)

    # The operator still requests OFFBOARD after localization and preflight checks.
    return LaunchDescription(arguments + [point_lio, mavros, dense_cloud, planner, mission])
