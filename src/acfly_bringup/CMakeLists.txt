cmake_minimum_required(VERSION 3.8)
project(acfly_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/survey_graph.py TARGET survey_graph TIMEOUT 35
    ENV "ROS_DOMAIN_ID=183" "ROS_LOCALHOST_ONLY=1")
  add_launch_test(test/survey_cycle.py TARGET survey_cycle TIMEOUT 35
    ENV "ROS_DOMAIN_ID=184" "ROS_LOCALHOST_ONLY=1")
endif()
ament_package()
