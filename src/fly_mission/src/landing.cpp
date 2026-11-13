#include "fly_mission/mission_node.hpp"

namespace fly_mission {
void MissionNode::land(double time) {
  const bool connected = vehicle_.state_fresh(time);
  const bool pose_fresh = vehicle_.feedback().fresh(time);
  if (connected && !vehicle_.armed()) {
    if (pose_fresh && vehicle_.feedback().pose().z - home_.z < config_.landed_height_m) {
      transition(Stage::Finished);
    } else if (pose_fresh) {
      RCLCPP_WARN(get_logger(), "已上锁但没有到达落地高度，任务中止");
      transition(Stage::Aborted);
    }
    return;
  }
  if (connected && vehicle_.is_landing()) {
    landing_confirmed_ = true;
    lost_authority_.reset();
    return; // 模式已交接，不再发任何 OFFBOARD 指令。
  }
  if (!connected || !vehicle_.is_offboard() || landing_confirmed_) {
    if (!lost_authority_)
      lost_authority_ = time;
    if (time - *lost_authority_ >= config_.takeover_debounce_s) {
      RCLCPP_WARN(get_logger(), "降落期间飞手接管或飞控断联，停止请求模式");
      transition(Stage::Aborted);
    }
    return;
  }
  lost_authority_.reset();
  vehicle_.publish_velocity(pose_fresh ? vehicle_.hold(landing_point_, config_.max_hold_xy_mps,
                                                       config_.max_level_correction_mps)
                                       : Velocity{});
  vehicle_.request_landing(time);
}
} // namespace fly_mission
