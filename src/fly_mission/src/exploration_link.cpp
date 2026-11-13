#include "fly_mission/exploration_link.hpp"

#include "fly_mission/exploration_completion.hpp"
#include <algorithm>
#include <cmath>

namespace fly_mission {
ExplorationLink::ExplorationLink(rclcpp::Node &node, const Config &config)
    : config_(config), node_(node) {
  const auto latched = rclcpp::QoS(1).transient_local();
  goal_pub_ =
      node_.create_publisher<geometry_msgs::msg::PointStamped>("/exploration/goal", latched);
  context_pub_ = node_.create_publisher<acfly_interfaces::msg::ExplorationContext>(
      "/exploration/context", latched);
  cmd_sub_ = node_.create_subscription<Command>(
      "/exploration/control", 1, [this](Command::ConstSharedPtr msg) { receive(*msg); });
  finished_sub_ = node_.create_subscription<std_msgs::msg::Header>(
      "/exploration/finished_goal", latched, [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        if (active_ && completion_matches_goal(*msg, mission_))
          finished_ = true;
      });
}

bool ExplorationLink::peers_ready() const {
  // 使用 RMW 已匹配的对端数，话题名存在本身不代表类型与 QoS 匹配。
  return goal_pub_->get_subscription_count() > 0 && context_pub_->get_subscription_count() > 0 &&
         cmd_sub_->get_publisher_count() > 0 && finished_sub_->get_publisher_count() > 0;
}

void ExplorationLink::start(const Pose &home, const Pose &cruise) {
  geometry_msgs::msg::PointStamped goal;
  goal.header.frame_id = "camera_init";
  goal.header.stamp = node_.now();
  goal.point.x = config_.explore_goal_x;
  goal.point.y = config_.explore_goal_y;
  mission_ = goal.header;
  cruise_z_ = cruise.z;
  acfly_interfaces::msg::ExplorationContext context;
  context.mission = mission_;
  context.home_z = home.z;
  context.cruise_z = cruise.z;
  context.height_tolerance = config_.tolerance_z_m;
  latest_.reset();
  last_stamp_.reset();
  active_ = true;
  finished_ = false;
  context_pub_->publish(context);
  goal_pub_->publish(goal);
}

bool ExplorationLink::fresh(const builtin_interfaces::msg::Time &stamp) const {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U)
    return false;
  const auto time = rclcpp::Time(stamp);
  const double age = (node_.now() - time).seconds();
  return age >= -config_.future_stamp_tolerance_s && age <= config_.control_timeout_s &&
         time >= rclcpp::Time(mission_.stamp);
}

void ExplorationLink::receive(const Command &msg) {
  if (!active_ || finished_ || !completion_matches_goal(msg.mission, mission_) ||
      msg.control_mode != Command::VELOCITY || msg.header.frame_id != "base_link" ||
      !fresh(msg.header.stamp) || (last_stamp_ && rclcpp::Time(msg.header.stamp) <= *last_stamp_) ||
      !std::isfinite(msg.velocity.linear.x) || !std::isfinite(msg.velocity.linear.y) ||
      !std::isfinite(msg.velocity.angular.z) || !std::isfinite(msg.altitude) ||
      std::abs(msg.altitude - cruise_z_) > 1e-6 || !std::isfinite(msg.vertical_speed_limit) ||
      msg.vertical_speed_limit <= 0 || msg.vertical_speed_limit > config_.max_climb_mps)
    return;
  last_stamp_ = rclcpp::Time(msg.header.stamp);
  latest_ = msg;
  received_at_ = steady_seconds();
}

std::optional<SurveyMotion> ExplorationLink::motion(double time, double yaw) const {
  if (!active_ || finished_ || !latest_ || !fresh(latest_->header.stamp) ||
      time - received_at_ > config_.control_timeout_s)
    return std::nullopt;
  const auto &v = latest_->velocity;
  const double c = std::cos(yaw), s = std::sin(yaw);
  Velocity output{c * v.linear.x - s * v.linear.y, s * v.linear.x + c * v.linear.y, 0,
                  std::clamp(v.angular.z, -config_.max_yaw_rate_radps, config_.max_yaw_rate_radps)};
  limit_xy(output, config_.max_speed_xy_mps);
  return SurveyMotion{output,
                      std::min(latest_->vertical_speed_limit, config_.max_level_correction_mps)};
}
} // namespace fly_mission
