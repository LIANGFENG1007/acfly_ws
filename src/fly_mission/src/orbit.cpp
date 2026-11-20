#include "fly_mission/mission_node.hpp"

#include <algorithm>
#include <cmath>

namespace fly_mission {
namespace {
constexpr double pi = 3.14159265358979323846;

double angle_error(double target, double current) {
  return std::remainder(target - current, 2.0 * pi);
}
} // namespace

void OrbitAction::start(const Pose &cruise, double time) {
  cruise_ = cruise;
  started_at_ = time;
  previous_command_time_ = time;
  previous_command_ = {};
  const auto &orbit = config_.orbit;
  const double dx = cruise.x - orbit.center_x, dy = cruise.y - orbit.center_y;
  // Choose the nearest entry once. At the centre, use the takeoff heading as the radial ray.
  const double bearing = std::hypot(dx, dy) > 1e-6 ? std::atan2(dy, dx) : cruise.yaw;
  entry_ = {orbit.center_x + orbit.radius_m * std::cos(bearing),
            orbit.center_y + orbit.radius_m * std::sin(bearing), cruise.z,
            orbit.face_center ? bearing + pi : cruise.yaw};
  hold_point_.reset();
  settled_since_.reset();
  hold_reason_ = nullptr;
  swept_rad_ = 0;
  tracking_ = false;
}

void OrbitAction::interrupt(const char *reason) {
  if (!hold_reason_)
    hold_reason_ = reason;
}

double OrbitAction::swept_angle_deg() const {
  return swept_rad_ * 180.0 / pi;
}

bool OrbitAction::settled(bool in_tolerance, double time) {
  if (!in_tolerance) {
    settled_since_.reset();
    return false;
  }
  if (!settled_since_)
    settled_since_ = time;
  return time - *settled_since_ >= config_.orbit.settle_s;
}

OrbitStep OrbitAction::hold(const MavrosLink &vehicle) {
  if (!hold_point_)
    hold_point_ = vehicle.feedback().pose();
  return {vehicle.hold(*hold_point_, config_.max_hold_xy_mps, config_.max_level_correction_mps)};
}

Velocity OrbitAction::slew_horizontal(Velocity command, double time) {
  const double period_s = config_.control_period_ms * 0.001;
  const double dt = std::clamp(time - previous_command_time_, 0.0, period_s);
  const double dx = command.x - previous_command_.x, dy = command.y - previous_command_.y;
  const double change = std::hypot(dx, dy);
  const double allowed = config_.orbit.accel_mps2 * dt;
  if (change > allowed) {
    command.x = previous_command_.x + dx * allowed / change;
    command.y = previous_command_.y + dy * allowed / change;
  }
  previous_command_ = command;
  previous_command_time_ = time;
  return command;
}

OrbitStep OrbitAction::step(const MavrosLink &vehicle, double time) {
  const auto &feedback = vehicle.feedback();
  if (!feedback.fresh(time)) {
    interrupt("绕圈期间定位断流");
    hold_point_.reset();
    return {};
  }
  const auto &orbit = config_.orbit;
  if (time - started_at_ >= orbit.timeout_s)
    interrupt("绕圈动作超时");
  if (hold_reason_)
    return hold(vehicle);

  const auto &pose = feedback.pose();
  const auto &velocity = feedback.velocity();
  const double dx = pose.x - orbit.center_x, dy = pose.y - orbit.center_y;
  const double radius = std::hypot(dx, dy);
  const double bearing = std::atan2(dy, dx);
  const bool stopped = std::hypot(velocity.x, velocity.y) <= orbit.stop_speed_mps &&
                       std::abs(pose.z - cruise_.z) <= config_.tolerance_z_m;
  if (!tracking_) {
    const bool heading_ready =
        !orbit.face_center || std::abs(angle_error(entry_.yaw, pose.yaw)) <= orbit.yaw_tolerance_rad;
    if (settled(stopped && heading_ready &&
                    std::hypot(pose.x - entry_.x, pose.y - entry_.y) <= orbit.radius_tolerance_m,
                time)) {
      previous_angle_ = bearing;
      settled_since_.reset();
      tracking_ = true;
    }
    return {slew_horizontal(
        vehicle.hold(entry_, orbit.speed_mps, config_.max_level_correction_mps), time)};
  }

  // A point near the centre has no usable orbit bearing.
  if (radius < orbit.radius_m / 2) {
    interrupt("绕圈期间过度偏离圆周");
    return hold(vehicle);
  }
  swept_rad_ += angle_error(bearing, previous_angle_);
  previous_angle_ = bearing;
  const double remaining = orbit.sweep_rad - swept_rad_;
  const double target_yaw = orbit.face_center ? bearing + pi : cruise_.yaw;
  const bool on_circle = std::abs(radius - orbit.radius_m) <= orbit.radius_tolerance_m;
  const bool command_settled =
      std::hypot(previous_command_.x, previous_command_.y) <= orbit.stop_speed_mps;
  if (settled(stopped && command_settled && on_circle &&
                  std::abs(remaining) <= orbit.angle_tolerance_rad &&
                  (!orbit.face_center ||
                   std::abs(angle_error(target_yaw, pose.yaw)) <= orbit.yaw_tolerance_rad),
              time))
    return {{}, true};

  // ENU commands combine radial error correction and motion along the tangent.
  const double rx = dx / radius, ry = dy / radius;
  const double radial_speed = std::clamp(
      config_.kp_xy * (orbit.radius_m - radius) -
          config_.kd_xy * (velocity.x * rx + velocity.y * ry),
      -orbit.speed_mps, orbit.speed_mps);
  const double tangent_limit =
      std::sqrt(std::max(0.0, orbit.speed_mps * orbit.speed_mps - radial_speed * radial_speed));
  const double remaining_m = orbit.radius_m * std::abs(remaining);
  const double braking_limit = std::sqrt(2.0 * orbit.accel_mps2 * remaining_m);
  const double tangent_speed = std::copysign(
      std::min({config_.kp_xy * remaining_m, braking_limit, tangent_limit}), remaining);
  auto command = vehicle.hold({orbit.center_x, orbit.center_y, cruise_.z, target_yaw},
                              orbit.speed_mps, config_.max_level_correction_mps);
  command.x = radial_speed * rx - tangent_speed * ry;
  command.y = radial_speed * ry + tangent_speed * rx;
  command = slew_horizontal(command, time);
  if (orbit.face_center) {
    const double issued_tangent = -command.x * ry + command.y * rx;
    command.yaw_rate = std::clamp(command.yaw_rate + issued_tangent / radius,
                                  -config_.max_yaw_rate_radps, config_.max_yaw_rate_radps);
  }
  return {command};
}

void MissionNode::orbit(double time) {
  const auto result = orbit_.step(vehicle_, time);
  if (const auto reason = orbit_.hold_reason(); reason && stage_ == Stage::Orbit) {
    RCLCPP_WARN(get_logger(), "%s，停止绕圈并等待飞手接管", reason);
    transition(Stage::OrbitHold);
  }
  if (result.complete) {
    RCLCPP_INFO(get_logger(), "绕圈停稳，累计角度 %.1f°，开始降落", orbit_.swept_angle_deg());
    landing_point_ = vehicle_.feedback().pose();
    transition(Stage::Landing);
    land(time);
  } else {
    vehicle_.publish_velocity(result.velocity);
  }
}
} // namespace fly_mission
