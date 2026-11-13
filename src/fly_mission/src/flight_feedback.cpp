#include "fly_mission/flight_feedback.hpp"

#include <cmath>

namespace fly_mission {
bool FlightFeedback::fresh(double now_s) const {
  return receipt_ && now_s >= *receipt_ && now_s - *receipt_ <= config_.odometry_timeout_s;
}

void FlightFeedback::update(const nav_msgs::msg::Odometry &msg, double now_s) {
  const auto &p = msg.pose.pose.position;
  const auto &q = msg.pose.pose.orientation;
  const auto &time = msg.header.stamp;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) &&
        norm > 1e-9 && time.sec >= 0 && time.nanosec < 1000000000U)) {
    receipt_.reset();
    velocity_ = {};
    return;
  }
  const auto stamp = static_cast<std::int64_t>(time.sec) * 1000000000 + time.nanosec;
  if (stamp_ && stamp == *stamp_)
    return;
  if (stamp_ && stamp < *stamp_) {
    // 时钟重置后等下一帧向前推进，冻结的重复时间戳不能续命。
    receipt_.reset();
    velocity_ = {};
    stamp_ = stamp;
    return;
  }
  const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), norm - 2 * (q.y * q.y + q.z * q.z));
  // 使用来源时间差计算速度，即使回调被短时阻塞也不会被放大。
  const double dt = stamp_ ? (stamp - *stamp_) * 1e-9 : 0;
  if (fresh(now_s) && dt > 0 && dt <= config_.odometry_timeout_s) {
    const double a = config_.velocity_filter_alpha;
    velocity_.x = (1 - a) * velocity_.x + a * (p.x - pose_.x) / dt;
    velocity_.y = (1 - a) * velocity_.y + a * (p.y - pose_.y) / dt;
    velocity_.z = (1 - a) * velocity_.z + a * (p.z - pose_.z) / dt;
  } else {
    velocity_ = {};
  }
  pose_ = {p.x, p.y, p.z, yaw};
  stamp_ = stamp;
  receipt_ = now_s;
}
} // namespace fly_mission
