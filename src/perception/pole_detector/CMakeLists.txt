cmake_minimum_required(VERSION 3.8)
project(pole_detector)
find_package(ament_cmake REQUIRED)
find_package(acfly_perception_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/markers.cpp
  src/pole_detector_node.cpp
)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(${PROJECT_NAME}_node acfly_perception_common rclcpp visualization_msgs)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config rviz DESTINATION share/${PROJECT_NAME})
ament_package()
