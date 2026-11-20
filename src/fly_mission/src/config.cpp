#include "fly_mission/config.hpp"
#include "acfly_config/defaults.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace fly_mission {
namespace {

class Settings {
public:
  explicit Settings(rclcpp::Node &node) : node_(node) {}
  template <typename T> T get(const std::string &name) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return defaults_.declare<T>(node_, name, descriptor);
  }

  double number(const std::string &name, bool positive = true) {
    const auto value = get<double>(name);
    if (!std::isfinite(value) || (positive && value <= 0)) {
      throw std::runtime_error("Invalid fly_mission setting: " + name);
    }
    return value;
  }

  void finish() {
    defaults_.check(node_);
  }

private:
  rclcpp::Node &node_;
  acfly_config::Defaults defaults_{
      "fly_mission", {"mission.yaml", "orbit.yaml", "flight_control.yaml", "watchdogs.yaml"}};
};

} // namespace

Config load_config(rclcpp::Node &node) {
  Settings p(node);
  Config c{};
  const auto action = p.get<std::string>("mission_action");
  if (action == "orbit")
    c.action = MissionAction::Orbit;
  else if (action == "survey")
    c.action = MissionAction::Survey;
  else
    throw std::runtime_error("mission_action must be orbit or survey");
  const double rad_per_degree = std::acos(-1.0) / 180.0;
  c.orbit.center_x = p.number("orbit_center_x_m", false);
  c.orbit.center_y = p.number("orbit_center_y_m", false);
  c.orbit.radius_m = p.number("orbit_radius_m");
  c.orbit.sweep_rad = p.number("orbit_angle_deg", false) * rad_per_degree;
  c.orbit.speed_mps = p.number("orbit_speed_mps");
  c.orbit.accel_mps2 = p.number("orbit_accel_mps2");
  c.orbit.radius_tolerance_m = p.number("orbit_radius_tolerance_m");
  c.orbit.angle_tolerance_rad = p.number("orbit_angle_tolerance_deg") * rad_per_degree;
  c.orbit.yaw_tolerance_rad = p.number("orbit_yaw_tolerance_deg") * rad_per_degree;
  c.orbit.stop_speed_mps = p.number("orbit_stop_speed_mps");
  c.orbit.settle_s = p.number("orbit_settle_s");
  c.orbit.timeout_s = p.number("orbit_timeout_s");
  c.orbit.face_center = p.get<bool>("orbit_face_center");
  c.takeoff_height_m = p.number("takeoff_height_m");
  c.explore_goal_x = p.number("explore_goal_x", false);
  c.explore_goal_y = p.number("explore_goal_y", false);
  c.takeoff_settle_s = p.number("takeoff_settle_s");
  c.takeoff_hover_s = p.number("takeoff_hover_s");
  c.landed_height_m = p.number("landed_height_m");
  c.require_start_signal = p.get<bool>("require_start_signal");
  c.offboard_preheat = p.get<bool>("offboard_preheat");
  c.control_period_ms = p.get<std::int64_t>("control_period_ms");
  c.velocity_filter_alpha = p.number("velocity_filter_alpha");
  c.kp_xy = p.number("kp_xy");
  c.kd_xy = p.number("kd_xy", false);
  c.kp_z = p.number("kp_z");
  c.kd_z = p.number("kd_z", false);
  c.kp_yaw = p.number("kp_yaw");
  c.max_speed_xy_mps = p.number("max_speed_xy_mps");
  c.max_climb_mps = p.number("max_climb_mps");
  c.max_level_correction_mps = p.number("max_level_correction_mps");
  c.max_hold_xy_mps = p.number("max_hold_xy_mps");
  c.max_yaw_rate_radps = p.number("max_yaw_rate_radps");
  c.tolerance_xy_m = p.number("tolerance_xy_m");
  c.tolerance_z_m = p.number("tolerance_z_m");
  c.odometry_timeout_s = p.number("odometry_timeout_s");
  c.fcu_state_timeout_s = p.number("fcu_state_timeout_s");
  c.control_timeout_s = p.number("control_timeout_s");
  c.future_stamp_tolerance_s = p.number("future_stamp_tolerance_s", false);
  c.takeover_debounce_s = p.number("takeover_debounce_s");
  c.landing_retry_s = p.number("landing_retry_s");
  c.preflight_log_period_s = p.number("preflight_log_period_s");
  p.finish();
  if (c.control_period_ms <= 0 || c.control_period_ms >= 500 || c.velocity_filter_alpha > 1 ||
      c.kd_xy < 0 || c.kd_z < 0 || c.landed_height_m >= c.takeoff_height_m ||
      c.tolerance_z_m >= c.takeoff_height_m || c.max_level_correction_mps > c.max_climb_mps ||
      c.max_hold_xy_mps > c.max_speed_xy_mps || c.future_stamp_tolerance_s < 0 ||
      c.future_stamp_tolerance_s >= c.control_timeout_s) {
    throw std::runtime_error("Inconsistent flight_control, mission or watchdog parameters");
  }
  const auto &orbit = c.orbit;
  if (orbit.radius_tolerance_m >= orbit.radius_m / 2 ||
      orbit.angle_tolerance_rad >= std::min(std::abs(orbit.sweep_rad), std::acos(-1.0)) ||
      orbit.yaw_tolerance_rad >= std::acos(-1.0) || orbit.speed_mps > c.max_speed_xy_mps ||
      orbit.stop_speed_mps >= orbit.speed_mps ||
      (orbit.face_center && orbit.speed_mps / orbit.radius_m > c.max_yaw_rate_radps) ||
      orbit.radius_m * std::abs(orbit.sweep_rad) / orbit.speed_mps + orbit.settle_s >=
          orbit.timeout_s) {
    throw std::runtime_error("Inconsistent orbit.yaml radius, angle, speed, tolerance or timeout");
  }
  return c;
}

} // namespace fly_mission
