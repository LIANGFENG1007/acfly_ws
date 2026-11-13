#include <acfly_config/initialize.hpp>
#include "node/exploration_node.hpp"
#include <iostream>
int main(int argc, char **argv) {
  try {
    acfly_config::initialize(argc, argv);
    auto node = std::make_shared<exploration::ExplorationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
  } catch (const std::exception &error) {
    std::cerr << "exploration_planner: " << error.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
  return 0;
}
