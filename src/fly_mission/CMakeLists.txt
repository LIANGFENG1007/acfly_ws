cmake_minimum_required(VERSION 3.8)
project(fly_mission)

find_package(ament_cmake REQUIRED)
find_package(acfly_config REQUIRED)
find_package(acfly_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(mission_core STATIC
  src/config.cpp
  src/exploration_link.cpp
  src/flight_feedback.cpp
  src/landing.cpp
  src/mavros_link.cpp
  src/mission_node.cpp
  src/orbit.cpp
  src/position_hold.cpp
  src/preflight.cpp
  src/survey.cpp
  src/takeoff.cpp
)
target_compile_features(mission_core PUBLIC cxx_std_17)
target_include_directories(mission_core PUBLIC include)
target_compile_options(mission_core PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(mission_core acfly_config acfly_interfaces geometry_msgs mavros_msgs nav_msgs rclcpp std_msgs)

add_executable(fly_mission_node src/main.cpp)
target_link_libraries(fly_mission_node mission_core)
install(TARGETS fly_mission_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(mission_lifecycle_test tests/mission_lifecycle_test.cpp)
  target_link_libraries(mission_lifecycle_test mission_core)
  add_test(NAME mission_lifecycle COMMAND mission_lifecycle_test)
  set_tests_properties(mission_lifecycle PROPERTIES TIMEOUT 25
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=208;ROS_LOG_DIR=/tmp/acfly-mission-lifecycle-logs")
endif()

ament_package()
