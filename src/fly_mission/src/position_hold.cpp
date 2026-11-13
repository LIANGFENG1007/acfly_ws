#include "fly_mission/mavros_link.hpp"

#include <algorithm>
#include <cmath>

namespace fly_mission {
void limit_xy(Velocity &v, double limit) {
  const double magnitude = std::hypot(v.x, v.y);
  if (magnitude > limit) {
    v.x *= limit / magnitude;
    v.y *= limit / magnitude;
  }
}

Velocity MavrosLink::hold(const Pose &goal, double xy_limit, double z_limit) const {
  const auto &pose = feedback_.pose();
  const auto &speed = feedback_.velocity();
  const double yaw_error = std::atan2(std::sin(goal.yaw - pose.yaw), std::cos(goal.yaw - pose.yaw));
  Velocity command{
      config_.kp_xy * (goal.x - pose.x) - config_.kd_xy * speed.x,
      config_.kp_xy * (goal.y - pose.y) - config_.kd_xy * speed.y,
      std::clamp(config_.kp_z * (goal.z - pose.z) - config_.kd_z * speed.z, -z_limit, z_limit),
      std::clamp(config_.kp_yaw * yaw_error, -config_.max_yaw_rate_radps,
                 config_.max_yaw_rate_radps)};
  limit_xy(command, xy_limit);
  return command;
}
} // namespace fly_mission
