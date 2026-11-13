#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace fly_mission {

// YAML 提供全部运行默认值。本结构只保存验证后的只读启动配置。
struct Config {
  double takeoff_height_m, explore_goal_x, explore_goal_y;
  double takeoff_settle_s, takeoff_hover_s, landed_height_m;
  bool require_start_signal, offboard_preheat;
  std::int64_t control_period_ms;
  double velocity_filter_alpha, kp_xy, kd_xy, kp_z, kd_z, kp_yaw;
  double max_speed_xy_mps, max_climb_mps, max_level_correction_mps, max_hold_xy_mps;
  double max_yaw_rate_radps, tolerance_xy_m, tolerance_z_m;
  double odometry_timeout_s, fcu_state_timeout_s, control_timeout_s;
  double future_stamp_tolerance_s, takeover_debounce_s, landing_retry_s;
};

Config load_config(rclcpp::Node &node);
} // namespace fly_mission
