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
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
