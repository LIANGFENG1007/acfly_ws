#include "fly_mission/mission_node.hpp"

#include <cmath>

namespace fly_mission {
void MissionNode::preflight(double time) {
  if (!vehicle_.state_fresh(time) || !vehicle_.on_ground(time) ||
      !vehicle_.feedback().fresh(time)) {
    saw_disarmed_on_ground_ = false;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "自检等待：需要飞控连接、里程计和 MAVROS 地面判定 ON_GROUND");
    return;
  }
  if (!explorer_.peers_ready()) {
    saw_disarmed_on_ground_ = false;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "自检等待：探索 goal/context/control/finished_goal 的 DDS 对端未全部匹配");
    return;
  }
  if (!vehicle_.armed())
    saw_disarmed_on_ground_ = true;
  // 即便空中重启时已有 OFFBOARD/armed，也不会夺取原有输出或重记起飞高度。
  if (!saw_disarmed_on_ground_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "自检等待：本进程尚未见到地面上锁状态");
    return;
  }
  if (config_.offboard_preheat)
    vehicle_.publish_velocity({});
  if (!pilot_authorizes_flight(time) || (config_.require_start_signal && !start_signal_) ||
      now().nanoseconds() <= 0)
    return;
  home_ = vehicle_.feedback().pose();
  takeoff_point_ = home_;
  takeoff_point_.z += config_.takeoff_height_m;
  transition(Stage::Takeoff);
}

void MissionNode::takeoff(double time) {
  const auto &feedback = vehicle_.feedback();
  if (!feedback.fresh(time)) {
    vehicle_.publish_velocity({});
    in_tolerance_.reset();
    return;
  }
  vehicle_.publish_velocity(
      vehicle_.hold(takeoff_point_, config_.max_speed_xy_mps, config_.max_climb_mps));
  const auto &p = feedback.pose();
  if (std::hypot(takeoff_point_.x - p.x, takeoff_point_.y - p.y) > config_.tolerance_xy_m ||
      std::abs(takeoff_point_.z - p.z) > config_.tolerance_z_m) {
    in_tolerance_.reset();
    return;
  }
  if (!in_tolerance_)
    in_tolerance_ = time;
  if (time - *in_tolerance_ < config_.takeoff_settle_s + config_.takeoff_hover_s)
    return;
  cruise_ = hold_ = p;
  explorer_.start(home_, cruise_);
  transition(Stage::Survey);
}
} // namespace fly_mission
