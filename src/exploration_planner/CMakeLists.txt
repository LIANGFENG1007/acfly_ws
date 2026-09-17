cmake_minimum_required(VERSION 3.8)
project(exploration_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)

# ── 头文件目录 ──
include_directories(include ${OpenCV_INCLUDE_DIRS})

# ── 可执行 ──
add_executable(exploration_planner_node
  src/exploration_planner_node.cpp
  src/grid_map.cpp
  src/coverage_planner.cpp
  src/bezier.cpp
  src/trajectory_tracker.cpp
  src/visualizer.cpp
  src/obstacle_map.cpp
  src/global_planner.cpp
  src/corridor_perception.cpp
  src/corridor_controller.cpp
  src/vision_shm.cpp
  src/vision_candidates.cpp
  src/vision_target_queue.cpp
  src/vision_near_confirmation.cpp
)

target_compile_features(exploration_planner_node PRIVATE cxx_std_17)

ament_target_dependencies(exploration_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  diagnostic_msgs
  sensor_msgs
  visualization_msgs
  tf2
)

target_link_libraries(exploration_planner_node ${OpenCV_LIBS})

install(TARGETS exploration_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Keep the trial runner and installed launch on the same adapter/profile source.
# These resources are part of the complete acfly_ws repository.
set(ACFLY_SIM_TOOLS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/sim")
install(PROGRAMS "${ACFLY_SIM_TOOLS_DIR}/dense_cloud_bridge.py"
  DESTINATION lib/${PROJECT_NAME}
  RENAME dense_cloud_bridge
)
install(FILES "${ACFLY_SIM_TOOLS_DIR}/profiles/physical_corridor.yaml"
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  add_executable(vision_target_queue_test tests/vision_target_queue_test.cpp src/vision_target_queue.cpp)
  target_compile_features(vision_target_queue_test PRIVATE cxx_std_17)
  add_test(NAME vision_target_queue_test COMMAND vision_target_queue_test)
  add_executable(vision_near_confirmation_test tests/vision_near_confirmation_test.cpp src/vision_near_confirmation.cpp)
  target_compile_features(vision_near_confirmation_test PRIVATE cxx_std_17)
  add_test(NAME vision_near_confirmation_test COMMAND vision_near_confirmation_test)
  add_executable(vision_candidates_test tests/vision_candidates_test.cpp src/vision_candidates.cpp)
  target_compile_features(vision_candidates_test PRIVATE cxx_std_17)
  add_test(NAME vision_candidates_test COMMAND vision_candidates_test)
  add_executable(vision_shm_test tests/vision_shm_test.cpp src/vision_shm.cpp)
  target_compile_features(vision_shm_test PRIVATE cxx_std_17)
  add_test(NAME vision_shm_test COMMAND vision_shm_test)
  add_executable(frontier_observation_test tests/frontier_observation_test.cpp
    src/coverage_planner.cpp src/grid_map.cpp)
  target_compile_features(frontier_observation_test PRIVATE cxx_std_17)
  add_test(NAME frontier_observation_test COMMAND frontier_observation_test)
  add_executable(turn_rounding_test tests/turn_rounding_test.cpp
    src/bezier.cpp src/trajectory_tracker.cpp src/global_planner.cpp)
  target_compile_features(turn_rounding_test PRIVATE cxx_std_17)
  add_test(NAME turn_rounding_test COMMAND turn_rounding_test)
  add_executable(coverage_blacklist_test tests/coverage_blacklist_test.cpp src/coverage_planner.cpp)
  target_compile_features(coverage_blacklist_test PRIVATE cxx_std_17)
  add_test(NAME coverage_blacklist_test COMMAND coverage_blacklist_test)
  add_executable(required_node_test
    tests/required_node_test.cpp
    src/grid_map.cpp
    src/coverage_planner.cpp
    src/bezier.cpp
    src/trajectory_tracker.cpp
    src/visualizer.cpp
    src/obstacle_map.cpp
    src/global_planner.cpp
    src/corridor_perception.cpp
    src/corridor_controller.cpp
    src/vision_shm.cpp
    src/vision_candidates.cpp
    src/vision_target_queue.cpp
    src/vision_near_confirmation.cpp
  )
  target_compile_features(required_node_test PRIVATE cxx_std_17)
  ament_target_dependencies(required_node_test
    rclcpp geometry_msgs nav_msgs std_msgs diagnostic_msgs sensor_msgs visualization_msgs tf2
  )
  target_link_libraries(required_node_test ${OpenCV_LIBS})
  # These regression fixtures use the original 7.5 x 10 m arena, independently
  # of the operator's current field size and exploration switch defaults.
  add_test(NAME required_node_test COMMAND required_node_test
    --ros-args -p viz:=false -p corridor_enabled:=false -p exploration_enabled:=true
    -p field_min_x:=0.0 -p field_min_y:=-5.0 -p field_max_x:=7.5 -p field_max_y:=5.0
    -p fov_range:=3.0 -p v_max:=0.6)
  set_tests_properties(required_node_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=203;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/acfly-required-node-ctest-logs"
    RUN_SERIAL TRUE
    TIMEOUT 60
  )
  add_executable(global_path_progress_test tests/global_path_progress_test.cpp src/global_planner.cpp)
  target_compile_features(global_path_progress_test PRIVATE cxx_std_17)
  add_test(NAME global_path_progress_test COMMAND global_path_progress_test)
  add_executable(global_path_escape_test tests/global_path_escape_test.cpp src/global_planner.cpp)
  target_compile_features(global_path_escape_test PRIVATE cxx_std_17)
  add_test(NAME global_path_escape_test COMMAND global_path_escape_test)
  add_executable(global_path_strict_test tests/global_path_strict_test.cpp src/global_planner.cpp)
  target_compile_features(global_path_strict_test PRIVATE cxx_std_17)
  add_test(NAME global_path_strict_test COMMAND global_path_strict_test)
  add_executable(global_required_path_test tests/global_required_path_test.cpp src/global_planner.cpp)
  target_compile_features(global_required_path_test PRIVATE cxx_std_17)
  add_test(NAME global_required_path_test COMMAND global_required_path_test)
  add_executable(trajectory_overshoot_test tests/trajectory_overshoot_test.cpp
    src/trajectory_tracker.cpp)
  target_compile_features(trajectory_overshoot_test PRIVATE cxx_std_17)
  add_test(NAME trajectory_overshoot_test COMMAND trajectory_overshoot_test)
  add_test(NAME trajectory_overshoot_production_test COMMAND trajectory_overshoot_test --production)
  add_executable(trajectory_flow_test tests/trajectory_flow_test.cpp
    src/trajectory_tracker.cpp src/bezier.cpp)
  target_compile_features(trajectory_flow_test PRIVATE cxx_std_17)
  add_test(NAME trajectory_flow_test COMMAND trajectory_flow_test)
  add_test(NAME trajectory_flow_production_test COMMAND trajectory_flow_test --production)
  add_executable(sensor_freshness_test tests/sensor_freshness_test.cpp)
  target_compile_features(sensor_freshness_test PRIVATE cxx_std_17)
  add_test(NAME sensor_freshness_test COMMAND sensor_freshness_test)
  add_executable(corridor_perception_test tests/corridor_perception_test.cpp src/corridor_perception.cpp)
  target_compile_features(corridor_perception_test PRIVATE cxx_std_17)
  add_test(NAME corridor_perception_test COMMAND corridor_perception_test)
  add_executable(corridor_visualizer_test tests/corridor_visualizer_test.cpp src/visualizer.cpp)
  target_compile_features(corridor_visualizer_test PRIVATE cxx_std_17)
  target_link_libraries(corridor_visualizer_test ${OpenCV_LIBS})
  add_test(NAME corridor_visualizer_test COMMAND corridor_visualizer_test)
  add_executable(corridor_controller_test tests/corridor_controller_test.cpp
    src/corridor_controller.cpp src/corridor_perception.cpp)
  target_compile_features(corridor_controller_test PRIVATE cxx_std_17)
  add_test(NAME corridor_controller_test COMMAND corridor_controller_test)
  add_executable(corridor_arena_test tests/corridor_arena_test.cpp
    src/corridor_controller.cpp src/corridor_perception.cpp)
  target_compile_features(corridor_arena_test PRIVATE cxx_std_17)
  add_test(NAME corridor_arena_test COMMAND corridor_arena_test)
  add_executable(corridor_stability_test tests/corridor_stability_test.cpp
    src/corridor_controller.cpp src/corridor_perception.cpp)
  target_compile_features(corridor_stability_test PRIVATE cxx_std_17)
  add_test(NAME corridor_stability_test COMMAND corridor_stability_test)
  add_executable(corridor_flow_test tests/corridor_flow_test.cpp
    src/corridor_controller.cpp src/corridor_perception.cpp)
  target_compile_features(corridor_flow_test PRIVATE cxx_std_17)
  add_test(NAME corridor_flow_test COMMAND corridor_flow_test)
  add_executable(corridor_exit_test tests/corridor_exit_test.cpp
    src/corridor_controller.cpp src/corridor_perception.cpp)
  target_compile_features(corridor_exit_test PRIVATE cxx_std_17)
  add_test(NAME corridor_exit_test COMMAND corridor_exit_test)
  add_executable(corridor_continuous_approach_test tests/corridor_continuous_approach_test.cpp
    src/corridor_controller.cpp src/corridor_perception.cpp)
  target_compile_features(corridor_continuous_approach_test PRIVATE cxx_std_17)
  add_test(NAME corridor_continuous_approach_test COMMAND corridor_continuous_approach_test)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
