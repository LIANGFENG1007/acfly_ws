#include "fly_mission/mission_node.hpp"

#include <chrono>

namespace fly_mission {
const char *stage_name(Stage stage) {
  switch (stage) {
  case Stage::Preflight:
    return "preflight";
  case Stage::Takeoff:
    return "takeoff";
  case Stage::Orbit:
    return "orbit";
  case Stage::OrbitHold:
    return "orbit_hold";
  case Stage::Survey:
    return "exploration";
  case Stage::Landing:
    return "landing";
  case Stage::Finished:
    return "finished";
  case Stage::Aborted:
    return "aborted";
  }
  return "unknown";
}

MissionNode::MissionNode(const rclcpp::NodeOptions &options)
    : Node("fly_mission_node", options), config_(load_config(*this)), vehicle_(*this, config_),
      orbit_(config_) {
  if (config_.action == MissionAction::Survey)
    explorer_.emplace(*this, config_);
  stage_pub_ =
      create_publisher<std_msgs::msg::String>("/mission/state", rclcpp::QoS(1).transient_local());
  if (config_.require_start_signal) {
    start_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/mission/start", 1, [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
          if (stage_ == Stage::Preflight && msg->data == 1)
            start_signal_ = true;
        });
  }
  transition(Stage::Preflight);
  timer_ =
      create_wall_timer(std::chrono::milliseconds(config_.control_period_ms), [this] { tick(); });
}

void MissionNode::transition(Stage next) {
  stage_ = next;
  std_msgs::msg::String event;
  event.data = stage_name(next);
  stage_pub_->publish(event);
  RCLCPP_INFO(get_logger(), "任务状态: %s", event.data.c_str());
  if (explorer_ && (next == Stage::Landing || ended()))
    explorer_->stop();
  if (ended())
    timer_->cancel();
}

bool MissionNode::pilot_authorizes_flight(double time) const {
  return vehicle_.state_fresh(time) && vehicle_.armed() && vehicle_.is_offboard();
}

void MissionNode::tick() {
  const double time = steady_seconds();
  if (stage_ == Stage::Takeoff || stage_ == Stage::Orbit || stage_ == Stage::OrbitHold ||
      stage_ == Stage::Survey) {
    if (!pilot_authorizes_flight(time)) {
      // 疑似接管时立即停发运动输出，去抖只影响终止判定。
      if (explorer_)
        explorer_->invalidate_velocity();
      if (stage_ == Stage::Orbit)
        orbit_.interrupt("绕圈期间飞控授权中断");
      in_tolerance_.reset();
      if (!lost_authority_)
        lost_authority_ = time;
      if (time - *lost_authority_ >= config_.takeover_debounce_s) {
        RCLCPP_WARN(get_logger(), "飞手接管、解锁状态丢失或飞控断联，终止本次任务");
        transition(Stage::Aborted);
      }
      return;
    }
    lost_authority_.reset();
  }
  switch (stage_) {
  case Stage::Preflight:
    preflight(time);
    break;
  case Stage::Takeoff:
    takeoff(time);
    break;
  case Stage::Orbit:
  case Stage::OrbitHold:
    orbit(time);
    break;
  case Stage::Survey:
    survey(time);
    break;
  case Stage::Landing:
    land(time);
    break;
  case Stage::Finished:
  case Stage::Aborted:
    break;
  }
}
} // namespace fly_mission
