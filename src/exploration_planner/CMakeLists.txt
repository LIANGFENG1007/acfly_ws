cmake_minimum_required(VERSION 3.16)
project(exploration_planner LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(acfly_config REQUIRED)
find_package(acfly_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(exploration_core STATIC
  src/bezier.cpp
  src/coverage_planner.cpp
  src/global_planner.cpp
  src/grid_map.cpp
  src/obstacle_map.cpp
  src/trajectory_tracker.cpp
)
target_compile_features(exploration_core PUBLIC cxx_std_17)
target_include_directories(exploration_core PUBLIC include)

add_library(exploration_runtime STATIC
  src/node/configuration.cpp
  src/node/coverage_path.cpp
  src/node/coverage_search.cpp
  src/node/home.cpp
  src/node/interfaces.cpp
  src/node/coverage_execution.cpp
  src/node/home_execution.cpp
  src/node/mission.cpp
  src/node/path_guard.cpp
  src/node/recovery.cpp
  src/node/sensors.cpp
  src/node/telemetry.cpp
  src/node/exploration_node.cpp
)
target_compile_options(exploration_runtime PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(exploration_runtime exploration_core)
ament_target_dependencies(exploration_runtime
  acfly_config acfly_interfaces diagnostic_msgs geometry_msgs mavros_msgs nav_msgs rclcpp rosgraph_msgs sensor_msgs std_msgs tf2 visualization_msgs)

add_executable(exploration_planner_node src/main.cpp)
target_link_libraries(exploration_planner_node exploration_runtime)

install(TARGETS exploration_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(planner_contract_test tests/planner_contract_test.cpp)
  target_link_libraries(planner_contract_test exploration_runtime)
  target_link_options(planner_contract_test PRIVATE -Wl,--wrap=rcl_publish)
  add_test(NAME planner_contract COMMAND planner_contract_test)
  set_tests_properties(planner_contract PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=192;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/acfly-planner-test"
    TIMEOUT 30)
  foreach(scenario
      sensor_freshness obstacle_visibility coverage_blacklist frontier_observation
      global_path_escape global_path_progress global_path_strict global_required_path
      turn_rounding trajectory_flow trajectory_overshoot)
    add_executable(${scenario}_test tests/${scenario}_test.cpp)
    target_link_libraries(${scenario}_test exploration_core)
    add_test(NAME ${scenario} COMMAND ${scenario}_test)
  endforeach()
  add_test(NAME trajectory_flow_reference COMMAND trajectory_flow_test --production)
  add_test(NAME trajectory_overshoot_reference COMMAND trajectory_overshoot_test --production)
endif()

ament_package()
