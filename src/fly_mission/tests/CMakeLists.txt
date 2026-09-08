cmake_minimum_required(VERSION 3.8)
project(fly_mission_landing_tests)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(landing_handover_test
  landing_handover_test.cpp
  ../src/drone_controller.cpp
  ../src/avoidance.cpp
)
target_include_directories(landing_handover_test PRIVATE ../include)
ament_target_dependencies(landing_handover_test
  rclcpp geometry_msgs mavros_msgs nav_msgs std_msgs tf2
)
target_compile_options(landing_handover_test PRIVATE -Wall -Wextra -Wpedantic)
target_link_options(landing_handover_test PRIVATE -Wl,--wrap=open)

enable_testing()
add_test(NAME landing_handover COMMAND landing_handover_test)
