cmake_minimum_required(VERSION 3.8)
project(multi_pole_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-class-memaccess)  # -Wno-class-memaccess: 抑制系统 Eigen(NEON) 头文件的无害 memcpy 警告
endif()

# ── 依赖 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ── 可执行 ──
add_executable(multi_pole_detector_node src/multi_pole_detector_node.cpp)

ament_target_dependencies(multi_pole_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

target_include_directories(multi_pole_detector_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

install(TARGETS multi_pole_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# 装 rviz 配置(供 rviz2 -d 加载)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
