cmake_minimum_required(VERSION 3.8)
project(acfly_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExplorationContext.msg" "msg/ExplorationCommand.msg" "msg/DropStatus.msg"
  "msg/ExplorationControlStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
