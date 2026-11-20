#pragma once

#include "fly_mission/mavros_link.hpp"
#include <acfly_interfaces/msg/exploration_command.hpp>
#include <acfly_interfaces/msg/exploration_context.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <optional>
#include <std_msgs/msg/header.hpp>
#include <string>

namespace fly_mission {

struct SurveyMotion {
  Velocity velocity;
  double height_correction_limit;
};

// 只接受本次地图库任务的平面速度；巡航高度由主控保持。
class ExplorationLink {
public:
  ExplorationLink(rclcpp::Node &node, const Config &config);
  std::string unmatched_peers() const;
  void start(const Pose &home, const Pose &cruise);
  bool finished() const {
    return active_ && finished_;
  }
  void stop() {
    active_ = false;
    latest_.reset();
  }
  void invalidate_velocity() {
    latest_.reset();
  }
  std::optional<SurveyMotion> motion(double steady_now, double yaw) const;

private:
  using Command = acfly_interfaces::msg::ExplorationCommand;
  void receive(const Command &msg);
  bool fresh(const builtin_interfaces::msg::Time &stamp) const;
  const Config &config_;
  rclcpp::Node &node_;
  std_msgs::msg::Header mission_;
  bool active_ = false, finished_ = false;
  std::optional<Command> latest_;
  std::optional<rclcpp::Time> last_stamp_;
  double received_at_{}, cruise_z_{};
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<acfly_interfaces::msg::ExplorationContext>::SharedPtr context_pub_;
  rclcpp::Subscription<Command>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr finished_sub_;
};
} // namespace fly_mission
