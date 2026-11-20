#include "fly_mission/flight_feedback.hpp"

#include <cmath>

namespace fly_mission {
bool FlightFeedback::fresh(double now_s) const {
  return receipt_ && now_s >= *receipt_ && now_s - *receipt_ <= config_.odometry_timeout_s;
}

const char *FlightFeedback::wait_reason(double now_s) const {
  if (fresh(now_s))
    return nullptr;
  return sample_issue_ ? sample_issue_ : "/aft_mapped_to_init 更新超时";
}

void FlightFeedback::update(const nav_msgs::msg::Odometry &msg, double now_s) {
  const auto &p = msg.pose.pose.position;
  const auto &q = msg.pose.pose.orientation;
  const auto &time = msg.header.stamp;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(norm) &&
        norm > 1e-9 && time.sec >= 0 && time.nanosec < 1000000000U)) {
    sample_issue_ = "/aft_mapped_to_init 位姿或时间戳无效";
    receipt_.reset();
    velocity_ = {};
    return;
  }
  const auto stamp = static_cast<std::int64_t>(time.sec) * 1000000000 + time.nanosec;
  if (stamp_ && stamp == *stamp_) {
    sample_issue_ = "/aft_mapped_to_init 时间戳未推进";
    return;
  }
  if (stamp_ && stamp < *stamp_) {
    // 时钟重置后等下一帧向前推进，冻结的重复时间戳不能续命。
    sample_issue_ = "/aft_mapped_to_init 时间戳回退，等待下一帧";
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
  sample_issue_ = nullptr;
}
} // namespace fly_mission
