#include "fly_mission/mission_node.hpp"

namespace fly_mission {
void MissionNode::survey(double time) {
  const auto &feedback = vehicle_.feedback();
  if (!feedback.fresh(time)) {
    vehicle_.publish_velocity({});
    explorer_->invalidate_velocity();
    holding_ = false; // 恢复定位时重新锁定当前位置，不追赶断流前的点。
    return;
  }
  const auto &p = feedback.pose();
  if (explorer_->finished()) {
    landing_point_ = p;
    transition(Stage::Landing);
    land(time);
    return;
  }
  if (auto command = explorer_->motion(time, p.yaw)) {
    command->velocity.z =
        vehicle_.hold(cruise_, config_.max_hold_xy_mps, command->height_correction_limit).z;
    vehicle_.publish_velocity(command->velocity);
    holding_ = false;
  } else {
    if (!holding_)
      hold_ = p;
    holding_ = true;
    auto hold = vehicle_.hold(hold_, config_.max_hold_xy_mps, config_.max_level_correction_mps);
    hold.yaw_rate = 0;
    vehicle_.publish_velocity(hold);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "等待 /exploration/control 的新鲜本任务速度，保持当前位置");
  }
}
} // namespace fly_mission
