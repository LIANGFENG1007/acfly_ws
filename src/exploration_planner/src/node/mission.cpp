#include "exploration_node.hpp"

namespace exploration {

void ExplorationNode::on_goal(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) || msg->header.stamp.sec < 0 ||
      msg->header.stamp.nanosec >= 1000000000u || (msg->header.frame_id != "camera_init")) {
    RCLCPP_WARN(get_logger(), "[探索任务] 忽略无效坐标、坐标系或时间戳的目标，保留当前任务");
    return;
  }
  std::lock_guard<std::mutex> lk(mtx_);
  const bool identified = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0;
  if (!identified ||
      (has_goal_ && rclcpp::Time(msg->header.stamp) < rclcpp::Time(goal_header_.stamp)))
    return;
  if (has_goal_ && msg->header == goal_header_) {
    if (msg->point.x != goal_.x || msg->point.y != goal_.y)
      RCLCPP_WARN(get_logger(),
                  "[探索任务] 同一任务标识对应不同终点，忽略冲突消息；新任务需使用新时间戳");
    return; // Duplicate retained goal.
  }
  trail_.clear();
  backtracking_ = false;
  backtrack_path_.clear();
  route_rejoin_target_.reset();
  route_rejoin_path_.clear();
  goal_ = {msg->point.x, msg->point.y};
  required_retreats_ = 0;
  fcfg_.goal = goal_; // 收尾顺路：选点知道红点在哪
  fcfg_.goal_valid = true;
  goal_stamp_ = msg->header.stamp;
  goal_header_ = msg->header;
  has_goal_ = true;
  finished_ = false;
  homing_ = false;
  last_command_world_ = {};
  yaw_command_history_.clear();
  no_route_since_ = last_explore_tick_ = -1.0;
  global_has_ = global_failed_ = global_goal_blocked_ = global_at_goal_ = global_braking_blocked_ =
      false;
  reset_home_approach();
  tracker_->set_trajectory({});
  traj_.clear();
  explore_raw_.clear();
  explore_has_committed_ = explore_failed_ = explore_goal_projected_ = false;
  cur_band_ = -1;
  look_valid_ = false;
  last_yaw_rate_ = 0.0;
  recovery_holding_ = false;
  required_goal_holding_ = false;
  retreating_ = retreat_exhausted_ = false;
  command_projection_blocked_ = measured_projection_blocked_ = false;
  required_blocked_time_valid_ = false;
  global_tracker_->set_trajectory({});
  global_raw_.clear();
  global_traj_.clear();
  std_msgs::msg::Bool reset;
  reset.data = false;
  finished_pub_->publish(reset);
  plan_pending_ = true;
  candidate_time_valid_ = false;
  unreachable_.clear(); // 换终点=换任务：清空够不到黑名单，所有区重新给机会
  last_unreach_clear_cov_ = 0.0;
  turning_for_solution_ = false; // 换任务：打断原地转身找解
  have_observation_target_ = false;
  observation_arrived_ = false;
  observation_yaw_rate_ = 0.0;
  observation_turn_active_ = false;
  has_unreachable_marker_ = false; // 清红叉
}

void ExplorationNode::on_timer() {
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now();
  cmd.header.frame_id = "base_link";
  std::lock_guard<std::mutex> lock(mtx_);
  const double t = steady_seconds();
  const bool pose_ok = has_pose_ && pose_freshness_.fresh(t, pose_timeout_);
  const bool map_ok = obstacle_mapping_start_ns_ && cloud_freshness_.fresh(t, cloud_timeout_);
  const bool mission_ok = has_goal_ && context_ && context_->mission == goal_header_;
  const Obstacles obstacles = obs_map_->snapshot();
  if (pose_ok && map_ok) {
    grid_->mark_scan(px_, py_, yaw_, obstacles);
    grid_->fill_obstacle_cells(obstacles);
  }
  if (pose_ok && map_ok && mission_ok && !finished_) {
    yaw_rate_est_ = current_yaw_rate(t);
    explore_tracker_ticked_ = global_tracker_ticked_ = recovery_requested_ = false;
    if (!step_route_rejoin(obstacles, cmd)) {
      if (!homing_ && grid_->coverage_ratio() >= search_coverage_) {
        homing_ = true;
        reset_home_approach();
        global_tracker_->set_endpoint_slow(home_slow_radius_, home_slow_speed_);
        turning_for_solution_ = global_has_ = global_failed_ = false;
        global_tracker_->set_trajectory({});
        global_tracker_->inherit_motion(*tracker_);
        tracker_->set_trajectory({});
        RCLCPP_INFO(get_logger(), "区域覆盖已达到阈值，开始沿已检查的路线返回终点");
      }
      if (homing_)
        finished_ = home_step(obstacles, cmd);
      else
        coverage_step(obstacles, cmd);
      const bool blocked =
          homing_ ? global_failed_
                  : (explore_failed_ && !tracker_->has_trajectory() && !turning_for_solution_);
      if (!finished_ && blocked && std::hypot(cmd.twist.linear.x, cmd.twist.linear.y) < 1e-9) {
        const Vec2 v = recovery_velocity(obstacles, homing_);
        cmd.twist.linear.x = v.x;
        cmd.twist.linear.y = v.y;
      }
      if (!obstacle_segment_clear({px_, py_}, {px_, py_}, obstacles, ggcfg_))
        cmd.twist.angular.z = 0;
    }
    if (finished_) {
      cmd.twist = geometry_msgs::msg::Twist();
      finished_goal_pub_->publish(goal_header_);
      std_msgs::msg::Bool done;
      done.data = true;
      finished_pub_->publish(done);
    }
    if (!recovery_requested_) {
      recovery_holding_ = false;
      recovery_status_ = "none";
    }
    // Smith 预估要积分【实际发出】的指令：守卫裁剪/转身/恢复都在 tracker 之后改写了 cmd。
    if (explore_tracker_ticked_)
      tracker_->set_last_command(cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z);
    if (global_tracker_ticked_)
      global_tracker_->set_last_command(cmd.twist.linear.x, cmd.twist.linear.y,
                                        cmd.twist.angular.z);
    const double command_c = std::cos(yaw_), command_s = std::sin(yaw_);
    last_command_world_ = {command_c * cmd.twist.linear.x - command_s * cmd.twist.linear.y,
                           command_s * cmd.twist.linear.x + command_c * cmd.twist.linear.y};
    last_yaw_rate_ = cmd.twist.angular.z;
    remember_yaw_command(last_yaw_rate_);

  } else {
    last_command_world_ = {};
    last_yaw_rate_ = 0;
    yaw_command_history_.clear();
    recovery_holding_ = false;
  }
  if (mission_ok && pose_ok) {
    acfly_interfaces::msg::ExplorationCommand control;
    control.header = cmd.header;
    control.mission = goal_header_;
    control.velocity = cmd.twist;
    control.altitude = context_->cruise_z;
    control.vertical_speed_limit = cruise_vertical_limit_;
    control_pub_->publish(control);
  }
  cmd_pub_->publish(cmd);
  if (++diagnostic_ticks_ % diagnostics_every_ == 0) {
    publish_status(!pose_ok      ? "waiting_for_pose"
                   : !map_ok     ? "waiting_for_cloud"
                   : !mission_ok ? "waiting_for_mission"
                   : finished_   ? "finished"
                   : homing_     ? "returning"
                                 : "exploring");
  }
}

} // namespace exploration
