cmake_minimum_required(VERSION 3.8)
project(fly_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json REQUIRED)   # 视觉找图解析 /cv/target_info 的 JSON

# ── 头文件目录 ──
include_directories(include)

# ── 可执行 ──
add_executable(fly_mission_node
  # 主控节点：构造 + on_timer 派发 + main
  src/fly_mission_node.cpp
  # 状态机各状态的实现(按功能域分文件，见 fly_mission_node.hpp 顶部分工说明)
  src/mission_state.cpp
  src/mission_step_basic.cpp
  src/mission_step_track.cpp
  src/mission_step_plat.cpp
  src/mission_io.cpp
  # 运动库与各功能模块
  src/drone_controller.cpp
  src/find_figure.cpp
  src/waypoint_receiver.cpp
  src/line_follower.cpp
  src/ring_driller.cpp
  src/avoidance.cpp
  src/car_tracker.cpp
)

ament_target_dependencies(fly_mission_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  std_msgs
  tf2
)

target_link_libraries(fly_mission_node nlohmann_json::nlohmann_json)

install(TARGETS fly_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# 把头文件也装上（其他包要 include 时用）
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
