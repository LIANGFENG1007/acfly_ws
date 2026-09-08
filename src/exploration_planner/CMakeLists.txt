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
)

ament_target_dependencies(exploration_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
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

if(BUILD_TESTING)
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
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
