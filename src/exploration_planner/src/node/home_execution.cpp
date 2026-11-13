#include "exploration_node.hpp"
namespace exploration {
bool ExplorationNode::home_step(const Obstacles &obstacles, geometry_msgs::msg::TwistStamped &cmd) {
  Vec2 vb = home_target_velocity(obstacles); // 先到安全入口，再保持航向、巡航高度直线返回

  if (global_failed_) {
    // 贴障无路先后退脱困(与探索分支同一套)，退出后下一规划周期重试；否则原地保持。
    //   累计后退次数有上限，防止"后退→找到路→接近→又失效"无限循环。
    if (!required_retreat_allowed(obstacles, cmd)) {
      retreating_ = false;
      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;
      cmd.twist.angular.z = 0.0;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Required home goal (%.2f, %.2f) has no validated path; %s", goal_.x,
                         goal_.y, retreating_ ? "retreating from obstacle" : "holding");
  } else {
    retreating_ = false; // A* 又通了 → 退出后退态
    cmd.twist.linear.x = vb.x;
    cmd.twist.linear.y = vb.y;
    cmd.twist.angular.z = last_yaw_rate_;
  }

  // 确认精确终点、安全路线和实测水平速度后，才允许主控接手降落。
  const double d = std::hypot(goal_.x - px_, goal_.y - py_);
  const bool exact_path =
      !global_traj_.empty() &&
      std::hypot(global_traj_.back().p.x - goal_.x, global_traj_.back().p.y - goal_.y) < 1e-6;
  const bool at_endpoint =
      !global_failed_ && !global_braking_blocked_ && global_at_goal_ &&
      (home_approach_active_ || global_tracker_->remaining_distance() <= goal_tol_) && exact_path &&
      (d <= goal_tol_);
  return at_endpoint && std::hypot(v_fwd_est_, v_lat_est_) <= goal_stop_v_;
}

} // namespace exploration
