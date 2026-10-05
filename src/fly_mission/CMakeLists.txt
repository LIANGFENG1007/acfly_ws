cmake_minimum_required(VERSION 3.8)
project(fly_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 依赖 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(acfly_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nlohmann_json REQUIRED)   # 视觉找图解析 /cv/target_info 的 JSON

# ── 头文件目录 ──
include_directories(include)

# ── 可执行 ──
set(FLY_MISSION_MODULES
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

add_executable(fly_mission_node src/fly_mission_node.cpp ${FLY_MISSION_MODULES})
target_compile_features(fly_mission_node PRIVATE cxx_std_17)

ament_target_dependencies(fly_mission_node
  acfly_interfaces
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
  add_executable(manual_takeoff_test tests/manual_takeoff_test.cpp ${FLY_MISSION_MODULES})
  target_compile_features(manual_takeoff_test PRIVATE cxx_std_17)
  ament_target_dependencies(manual_takeoff_test
    acfly_interfaces rclcpp geometry_msgs mavros_msgs nav_msgs std_msgs tf2)
  target_link_libraries(manual_takeoff_test nlohmann_json::nlohmann_json)
  target_link_options(manual_takeoff_test PRIVATE
    -Wl,--wrap=open -Wl,--wrap=socket -Wl,--wrap=rcl_publish -Wl,--wrap=rcl_send_request)
  add_test(NAME manual_takeoff_test COMMAND manual_takeoff_test)
  set_tests_properties(manual_takeoff_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=206;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/acfly-manual-takeoff-test-logs"
    TIMEOUT 15)
  add_executable(exploration_handoff_test tests/exploration_handoff_test.cpp ${FLY_MISSION_MODULES})
  target_compile_features(exploration_handoff_test PRIVATE cxx_std_17)
  ament_target_dependencies(exploration_handoff_test
    acfly_interfaces rclcpp geometry_msgs mavros_msgs nav_msgs std_msgs tf2)
  target_link_libraries(exploration_handoff_test nlohmann_json::nlohmann_json)
  target_link_options(exploration_handoff_test PRIVATE
    -Wl,--wrap=open -Wl,--wrap=socket -Wl,--wrap=rcl_publish -Wl,--wrap=rcl_send_request)
  add_test(NAME exploration_handoff_test COMMAND exploration_handoff_test)
  set_tests_properties(exploration_handoff_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=205;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/acfly-handoff-test-logs"
    TIMEOUT 15)
  add_executable(exploration_drop_sender_test tests/exploration_drop_sender_test.cpp)
  target_compile_features(exploration_drop_sender_test PRIVATE cxx_std_17)
  target_link_libraries(exploration_drop_sender_test util)
  target_link_options(exploration_drop_sender_test PRIVATE -Wl,--wrap=write)
  add_test(NAME exploration_drop_sender_test COMMAND exploration_drop_sender_test)
  add_executable(velocity_output_test tests/velocity_output_test.cpp
    src/drone_controller.cpp src/avoidance.cpp)
  target_compile_features(velocity_output_test PRIVATE cxx_std_17)
  ament_target_dependencies(velocity_output_test
    acfly_interfaces rclcpp geometry_msgs mavros_msgs nav_msgs std_msgs tf2)
  target_link_options(velocity_output_test PRIVATE -Wl,--wrap=open -Wl,--wrap=rcl_publish)
  add_test(NAME velocity_output_test COMMAND velocity_output_test)
  set_tests_properties(velocity_output_test PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=204;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/acfly-velocity-output-test-logs"
    TIMEOUT 15)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
