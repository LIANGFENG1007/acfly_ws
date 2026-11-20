#include "fly_mission/mission_node.hpp"

#include <string_view>

namespace fly_mission {
namespace {
void append_reason(std::string &message, std::string_view reason) {
  if (!message.empty())
    message += " | ";
  message += reason;
}
} // namespace

void MissionNode::report_preflight_wait(double time, const std::string &reason) {
  if (reason == last_preflight_reason_ &&
      time - last_preflight_log_s_ < config_.preflight_log_period_s)
    return;
  last_preflight_reason_ = reason;
  last_preflight_log_s_ = time;
  RCLCPP_INFO(get_logger(), "自检等待：%s", reason.c_str());
}

void MissionNode::preflight(double time) {
  std::string pending;
  for (const char *reason : {vehicle_.state_wait_reason(time), vehicle_.feedback().wait_reason(time),
                            vehicle_.ground_wait_reason(time)}) {
    if (reason)
      append_reason(pending, reason);
  }
  if (explorer_) {
    const auto unmatched = explorer_->unmatched_peers();
    if (!unmatched.empty())
      append_reason(pending, "探索对端未匹配：" + unmatched);
  }
  if (!pending.empty()) {
    saw_disarmed_on_ground_ = false;
    report_preflight_wait(time, pending);
    return;
  }

  if (!vehicle_.armed())
    saw_disarmed_on_ground_ = true;
  // 已解锁后才重启主控时，必须重新见到一次地面上锁状态。
  if (!saw_disarmed_on_ground_) {
    report_preflight_wait(time, "飞控已解锁，但本进程尚未见到地面上锁状态");
    return;
  }
  if (config_.offboard_preheat)
    vehicle_.publish_velocity({});

  if (!vehicle_.armed())
    append_reason(pending, "飞控仍上锁，等待飞手解锁");
  if (!vehicle_.is_offboard()) {
    const auto &mode = vehicle_.mode();
    append_reason(pending, "当前模式 " + (mode.empty() ? "未知" : mode) + "，等待 OFFBOARD");
  }
  if (config_.require_start_signal && !start_signal_)
    append_reason(pending, "等待 /mission/start 的 Int32(1)");
  if (explorer_ && now().nanoseconds() <= 0)
    append_reason(pending, "等待有效的 ROS 时间");
  if (!pending.empty()) {
    report_preflight_wait(time, "飞控已连接、定位正常、地面已确认 | " + pending);
    return;
  }

  home_ = vehicle_.feedback().pose();
  takeoff_point_ = home_;
  takeoff_point_.z += config_.takeoff_height_m;
  transition(Stage::Takeoff);
}
} // namespace fly_mission
