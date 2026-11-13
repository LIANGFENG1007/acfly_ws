#pragma once

#include "fly_mission/flight_feedback.hpp"
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <optional>

namespace fly_mission {

double steady_seconds();

class MavrosLink {
public:
  MavrosLink(rclcpp::Node &node, const Config &config);
  bool state_fresh(double time) const;
  bool on_ground(double time) const;
  bool is_offboard() const {
    return state_.mode == "OFFBOARD";
  }
  bool is_landing() const {
    return state_.mode == "AUTO.LAND";
  }
  bool armed() const {
    return state_.armed;
  }
  void publish_velocity(Velocity command);
  void request_landing(double time);
  const FlightFeedback &feedback() const {
    return feedback_;
  }
  Velocity hold(const Pose &target, double xy_limit, double z_limit) const;

private:
  rclcpp::Node &node_;
  const Config &config_;
  FlightFeedback feedback_;
  mavros_msgs::msg::State state_;
  bool on_ground_ = false;
  std::optional<double> last_state_, last_ground_state_, last_request_;
  std::optional<int64_t> pending_request_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
};

void limit_xy(Velocity &velocity, double limit);

} // namespace fly_mission
