#include <acfly_config/initialize.hpp>
#include "fly_mission/mission_node.hpp"
#include <cstdio>
#include <exception>

int main(int argc, char **argv) {
  try {
    acfly_config::initialize(argc, argv);
    auto node = std::make_shared<fly_mission::MissionNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->ended())
      executor.spin_once();
    const bool aborted = node->stage() == fly_mission::Stage::Aborted;
    if (rclcpp::ok())
      rclcpp::shutdown();
    return aborted ? 2 : 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "fly_mission: %s\n", error.what());
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
