#include "exploration_node.hpp"
namespace exploration {
void ExplorationNode::coverage_step(const Obstacles &obstacles,
                                    geometry_msgs::msg::TwistStamped &cmd) {
  traj_ = tracker_->trajectory();
  //   绕障全由 replan_locked 内的 A* 完成(含离墙)，跟随手感与原状一致；无障碍时
  //   A* 退化成直线，行为不变。A* 无解 → replan_locked 内已跳带悬停(放弃该区)。
  // ★原路后退★优先：连续 backtrack_after_s_ 没路时沿来路倒退，退完重新找路。
  bool handled = update_backtrack(obstacles, cmd);

  // ★原地转身找解★：锥内无解但侧向有路时，本拍原地转身改朝向重搜(画旋转标志)。
  //   step 返回 true=本拍已写 cmd(继续转/刚判真围死悬停)；false=刚解出(traj_已设)→落到下面跟随。
  if (!handled && turning_for_solution_) {
    handled = step_turn_for_solution(obstacles, cmd);
  }

  if (!handled) {
    if (has_goal_ && !finished_) {
      bool need = plan_pending_; // 收到新任务时立即规划
      if (!need) {
        if (!tracker_->has_trajectory()) {
          need = true;
        } else {
          const double age = (now() - last_plan_time_).seconds();
          if (age >= replan_period_)
            need = true;
          else if (tracker_->last_nearest_dist() > replan_dev_) {
            const Vec2 cur{px_, py_};
            need = !path_clear(cur, tracker_->remaining_path(px_, py_), obstacles, ggcfg_);
          }
        }
      }
      if (need)
        replan_locked({px_, py_});
    }

    // tracker 跟随当前 traj_(已是 A* 绕障轨迹)。A* 无解时 traj_ 被清空。
    if (tracker_->has_trajectory()) {
      retreating_ = false; // 有轨迹可走 → 退出后退态
      const bool was_reorienting = tracker_->reorienting();
      const bool tracker_ticks = !(observation_arrived_ && have_observation_target_);
      if (tracker_->hold_sliding() && !turn_correction_clear({v_fwd_est_, v_lat_est_}, obstacles))
        tracker_->end_hold_slide();
      const double previous_forward = tracker_->forward_command();
      VelCmd vc = !tracker_ticks ? VelCmd{}
                                 : tracker_->update(px_, py_, yaw_, v_fwd_est_, v_lat_est_,
                                                    goal_tol_, yaw_rate_est_);
      explore_tracker_ticked_ = tracker_ticks;
      if ((explore_handoff_deferred_ && handoff_brake_) && !vc.holding_position && !vc.at_goal) {
        // 换路前预减速：沿旧路线继续跟踪航向，前进指令按减速斜坡收到交接速度。
        // 以上一拍实际发出的前进指令为基准(审查：tracker 本拍已先加了 accel·dt，净减速只有一半)。
        const double braked = std::max(0.0, previous_forward - gains_.max_decel * gains_.dt);
        vc.v_fwd = std::min(vc.v_fwd, std::max(braked, handoff_brake_speed_ * .5));
        tracker_->constrain_forward_command(vc.v_fwd);
      }
      if (vc.needs_replan) {
        plan_pending_ = true;
        explore_has_committed_ = false;
        tracker_->set_trajectory({});
      }
      finish_observation(vc, obstacles);
      vc = guard_explore_velocity(vc, obstacles);
      if (vc.holding_position && !turn_correction_clear({vc.v_fwd, vc.v_lat}, obstacles))
        vc.v_fwd = vc.v_lat = 0.0;
      if (vc.holding_position && !obstacle_segment_clear({px_, py_}, {px_, py_}, obstacles, ggcfg_))
        vc.yaw_rate = 0.0;
      if (tracker_->reorienting() != was_reorienting) {
        if (tracker_->reorienting())
          ++recovery_entries_;
        RCLCPP_INFO(get_logger(), "[探索纠偏] %s",
                    tracker_->reorienting() ? "大角度偏差：XY定点纠偏并持续转向"
                                            : "航向与角速度稳定，平滑恢复前进");
      }
      last_look_ = tracker_->last_lookahead();
      look_valid_ = true;
      cmd.twist.linear.x = vc.v_fwd;
      cmd.twist.linear.y = vc.v_lat;
      cmd.twist.angular.z = vc.yaw_rate;
      if (vc.at_goal && (explore_goal_projected_ || !have_observation_target_)) {
        // An exhausted route without a separate observation
        // action must release control too, even when nearby
        // unseen cells make its target appear productive.
        unreachable_.push_back(explore_target_);
        explore_has_committed_ = false;
        tracker_->set_trajectory({});
        traj_.clear();
        plan_pending_ = true;
      }
    } else if (turning_for_solution_) {
      // replan_locked 本拍刚进入转身态(锥内无解+侧向有路) → 本拍即转身一步，别落到悬停。
      step_turn_for_solution(obstacles, cmd);
    } else if (explore_failed_) {
      // A* 无解被清空轨迹：先试贴障后退脱困(退出去下拍重规划接着扫)；
      //   非贴脸卡死/退到极限仍无路 → 本拍悬停(replan_locked 已跳带，下拍去别处)。
      if (!try_retreat(obstacles, cmd)) {
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.angular.z = 0.0;
      } else {
        plan_pending_ = true; // 后退后下一拍立刻重规划(此时可能已能绕)
      }
    }
  }
}

} // namespace exploration
