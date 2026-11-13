cmake_minimum_required(VERSION 3.8)
project(acfly_perception_common)

find_package(ament_cmake REQUIRED)
find_package(acfly_config REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cloud_window.cpp
  src/detector_node.cpp
  src/pole_fit.cpp
  src/settings.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} acfly_config nav_msgs rcl_interfaces rclcpp sensor_msgs std_msgs)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(acfly_config Eigen3 nav_msgs rcl_interfaces rclcpp sensor_msgs std_msgs)

if(BUILD_TESTING)
  enable_testing()
  add_executable(cloud_window_test tests/cloud_window_test.cpp)
  target_link_libraries(cloud_window_test ${PROJECT_NAME})
  add_test(NAME cloud_layout_and_history COMMAND cloud_window_test)
endif()
ament_package()
