#include "exploration_node.hpp"

namespace exploration {

void ExplorationNode::enter_turn_for_solution(const Vec2 &cur, const Vec2 &scan_target,
                                              const GlobalResult &probe, int candidate_band,
                                              const FrontierSelection *observation) {
  ++turn_entries_;
  turning_for_solution_ = true;
  turn_target_ = scan_target;
  begin_hold_from_motion(turn_hold_);
  turn_candidate_band_ = candidate_band;
  turn_observation_ = observation ? *observation : FrontierSelection{};
  retreating_ = false; // 与后退互斥
  turn_prev_yaw_ = yaw_;
  turn_accum_ = 0.0;
  turn_start_time_ = now();
  turn_research_tick_ = 0;
  has_unreachable_marker_ = false;
  // probe 首段方位 = 存在解的起步方向；锥内失败⇒|该方位−yaw|>锥半角，sign 必有定义。
  double bx = (probe.path.size() >= 2) ? probe.path[1].x - cur.x : scan_target.x - cur.x;
  double by = (probe.path.size() >= 2) ? probe.path[1].y - cur.y : scan_target.y - cur.y;
  const double bearing = (std::hypot(bx, by) > 1e-6)
                             ? std::atan2(by, bx)
                             : std::atan2(scan_target.y - cur.y, scan_target.x - cur.x);
  turn_heading_ = bearing;
  turn_heading_valid_ = false; // Validate once at the actual turn-start position.
  turn_probe_path_.clear();
  turn_command_rate_ = 0.0;
  turn_settled_s_ = 0.0;
  turn_dir_ = preferred_turn_direction(wrap_pi(bearing - yaw_));
}

int ExplorationNode::preferred_turn_direction(double error) const {
  if (std::abs(error) > 2.6 && std::abs(yaw_rate_est_) > 0.1)
    return yaw_rate_est_ > 0.0 ? 1 : -1;
  return error >= 0.0 ? 1 : -1;
}

bool ExplorationNode::step_turn_for_solution(const Obstacles &obs,
                                             geometry_msgs::msg::TwistStamped &cmd) {
  const Vec2 cur{px_, py_};

  // 1) 累计净转角(带符号 *turn_dir_)：小幅来回正负相消不虚增，firm yaw_rate
  // 远大于里程计噪声→单调逼近
  turn_accum_ += turn_dir_ * wrap_pi(yaw_ - turn_prev_yaw_);
  turn_prev_yaw_ = yaw_;

  if (!turn_heading_valid_) {
    auto fresh = search_explore_global(cur, turn_target_, obs);
    if (fresh.ok && fresh.path.size() >= 2) {
      fresh.path = clean_explore_guides(fresh.path, obs);
      turn_probe_path_ = fresh.path;
      turn_heading_ = std::atan2(fresh.path[1].y - cur.y, fresh.path[1].x - cur.x);
      turn_heading_valid_ = true;
      turn_dir_ = preferred_turn_direction(wrap_pi(turn_heading_ - yaw_));
      turn_accum_ = 0.0;
      turn_settled_s_ = 0.0;
      turn_research_tick_ = 0;
    } else {
      turning_for_solution_ = false;
      unreachable_.push_back(turn_target_);
      plan_pending_ = false;
      last_plan_time_ = now();
      last_replan_reason_ = "turn_start_probe_unavailable";
      cmd.twist.linear.x = cmd.twist.linear.y = cmd.twist.angular.z = 0.0;
      RCLCPP_WARN(get_logger(), "转向起点到当前观察点暂无路径，等待下一轮选点 (%.2f,%.2f)",
                  turn_target_.x, turn_target_.y);
      return true;
    }
  }
  double error = wrap_pi(turn_heading_ - yaw_);
  if (std::abs(error) > 2.6 && turn_dir_ * error < 0.0)
    error += turn_dir_ * 2.0 * M_PI;
  const double heading_tolerance =
      std::min(gains_.align_resume_rad,
               ggcfg_.head_cone_half > 0 ? ggcfg_.head_cone_half * .5 : gains_.align_resume_rad);
  // 角速度正在向零收敛(误差同号且剩余制动角不超过误差)即可重搜，不必等完全停稳：
  // 采纳后 tracker 自己还会按航向误差决定继续转或前进。原来的 ≤0.12 rad/s 门槛让每次
  // 转身末尾多等约 1 s。
  const double stopping_angle =
      yaw_rate_est_ * yaw_rate_est_ / (2.0 * std::max(1e-3, gains_.max_yaw_accel));
  const bool turn_braking =
      error * yaw_rate_est_ >= 0.0 && stopping_angle <= std::abs(error) + heading_tolerance;
  const bool settled = turn_heading_valid_ &&
                       (std::abs(yaw_rate_est_) <= gains_.align_stop_yaw_rate ||
                        (turn_braking && std::abs(yaw_rate_est_) <= 0.5)) &&
                       std::abs(error) <= heading_tolerance;
  turn_settled_s_ = settled ? turn_settled_s_ + gains_.dt : 0.0;
  // A transient open cone while still spinning is not a safe handoff.
  // Keep one heading target until angular momentum has settled.
  if (++turn_research_tick_ >= turn_solve_research_every_ &&
      turn_settled_s_ >= std::max(gains_.dt, gains_.align_settle_s)) {
    turn_research_tick_ = 0;
    GlobalResult gr;
    if (turn_probe_path_.size() >= 2) {
      Path2 cached = turn_probe_path_;
      cached.front() = cur;
      const double bearing = std::atan2(cached[1].y - cur.y, cached[1].x - cur.x);
      if (std::abs(wrap_pi(bearing - yaw_)) <= heading_tolerance &&
          path_inside_safe_field(cached) && path_clear(cur, cached, obs, ggcfg_)) {
        gr.ok = true;
        gr.path = std::move(cached);
      }
    }
    if (!gr.ok)
      gr = search_explore_global(cur, turn_target_, obs, yaw_);
    if (!gr.ok) {
      auto aligned = search_explore_global(cur, turn_target_, obs);
      if (aligned.ok && aligned.path.size() >= 2) {
        aligned.path = clean_explore_guides(aligned.path, obs);
        const auto &a = aligned.path[0];
        const auto &b = aligned.path[1];
        const double fresh_heading = std::atan2(b.y - a.y, b.x - a.x);
        if (std::abs(wrap_pi(fresh_heading - yaw_)) <= heading_tolerance) {
          gr = std::move(aligned);
        } else {
          // The obstacle view or stopping position changed. A
          // reachable target is not made unreachable by facing
          // the stale probe bearing from before that change.
          turn_heading_ = fresh_heading;
          turn_probe_path_ = aligned.path;
          turn_settled_s_ = 0.0;
          turn_dir_ = preferred_turn_direction(wrap_pi(fresh_heading - yaw_));
        }
      }
    }
    if (gr.ok && gr.path.size() >= 2 && adopt_explore_path(gr, turn_target_, &turn_observation_)) {
      turning_for_solution_ = false;
      cur_band_ = turn_candidate_band_;
      RCLCPP_INFO(get_logger(),
                  "原地转身找解成功 → 机头方向出现可走路径，沿新方向继续 (目标 %.2f,%.2f)",
                  turn_target_.x, turn_target_.y);
      return false;
    }
  }

  // A turn timeout is not proof that a whole region is unreachable.
  const bool full_circle = std::fabs(turn_accum_) >= 2.0 * M_PI * turn_solve_max_rev_;
  const bool timed_out = (now() - turn_start_time_).seconds() > turn_solve_timeout_;
  if (full_circle || timed_out) {
    turning_for_solution_ = false;
    unreachable_.push_back(turn_target_);
    plan_pending_ = false;
    last_plan_time_ = now();
    last_replan_reason_ = "turn_timeout_retry";
    has_unreachable_marker_ = true;
    unreachable_pos_ = turn_target_;
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = 0.0;
    RCLCPP_WARN(get_logger(), "本轮转向未完成 → 暂缓观察点 (%.2f,%.2f)，下一规划周期重新选点",
                turn_target_.x, turn_target_.y);
    return true;
  }

  // XY position correction and yaw are one action. Horizontal drift must
  // not pause the turn or relatch its world-frame anchor.
  double desired_rate = 0.0;
  if (turn_heading_valid_) {
    const double predicted = error - gains_.prediction_time * yaw_rate_est_;
    const double limit = std::sqrt(2.0 * gains_.max_yaw_accel * std::abs(predicted));
    const double maximum = std::min(turn_solve_yaw_rate_, gains_.max_yaw_rate);
    desired_rate = std::clamp(std::clamp(gains_.kp_yaw * predicted, -limit, limit) -
                                  gains_.kd_yaw * yaw_rate_est_,
                              -maximum, maximum);
  }
  turn_command_rate_ +=
      std::clamp(desired_rate - turn_command_rate_, -gains_.max_yaw_accel * gains_.dt,
                 gains_.max_yaw_accel * gains_.dt);
  const auto correction = turn_position_correction(turn_hold_, obs);
  cmd.twist.linear.x = correction.x;
  cmd.twist.linear.y = correction.y;
  if (!obstacle_segment_clear(cur, cur, obs, ggcfg_))
    turn_command_rate_ = 0.0;
  cmd.twist.angular.z = turn_command_rate_;
  last_look_ = turn_target_;
  look_valid_ = true;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "原地转身找解中…(已转 %.0f°，朝 %s)",
                       std::fabs(turn_accum_) * 180.0 / M_PI, turn_dir_ > 0 ? "逆时针" : "顺时针");
  return true;
}

void ExplorationNode::replan_locked(const Vec2 &cur) {
  if (!plan_pending_ && !tracker_->has_trajectory() &&
      (now() - last_plan_time_).seconds() < replan_period_)
    return;
  ++replan_checks_;
  bool all_explored = false;
  const Obstacles obs = obs_map_->snapshot();
  if (!plan_pending_ && have_observation_target_ && explore_has_committed_ &&
      (observation_arrived_ ||
       (tracker_->remaining_distance() <= goal_tol_ &&
        std::hypot(cur.x - explore_target_.x, cur.y - explore_target_.y) <= goal_tol_)) &&
      obstacle_segment_clear(cur, cur, obs, ggcfg_)) {
    last_plan_time_ = now();
    return; // Finish the observation at this safe viewpoint before selecting another.
  }
  // Keep a safe reference while holding XY and turning. A new obstacle or an
  // explicit mission request still interrupts the recovery immediately.
  if (!plan_pending_ && tracker_->reorienting() && explore_has_committed_ &&
      path_clear(cur, tracker_->remaining_path(cur.x, cur.y), obs, ggcfg_)) {
    last_plan_time_ = now();
    return;
  }

  // 栅格快照：本次规划期间只读一次地图(一把锁+一次拷贝)，
  //   下面的覆盖率判定与 plan_explore 逐格选点都基于【同一时刻】的视图，不会半新半旧。
  const GridSnapshot gsnap = grid_->snapshot();

  // 黑名单按覆盖率台阶清空：每涨过 unreach_clear_step_(默认5%) 给死路区一次重试机会
  //   (飞机已移动、视角已变，之前 A* 够不到的区可能现在进得去)。换终点也会清(见 on_goal)。
  const double cov_now = gsnap.coverage_ratio();
  if (cov_now - last_unreach_clear_cov_ >= unreach_clear_step_) {
    unreachable_.clear();
    last_unreach_clear_cov_ = cov_now;
  }

  const bool have_commit = tracker_->has_trajectory() && explore_has_committed_ && !explore_failed_;
  const Path2 remaining = have_commit ? tracker_->remaining_path(cur.x, cur.y) : Path2{};
  const bool old_clear = have_commit && path_clear(cur, remaining, obs, ggcfg_);
  const double view_heading = have_observation_target_ ? observation_target_.view_heading : yaw_;
  active_target_gain_ =
      have_commit && have_observation_target_
          ? visible_unknown_count(gsnap, explore_target_, view_heading, obs, fcfg_)
          : -1;
  const bool target_stale =
      have_commit &&
      (have_observation_target_
           ? active_target_gain_ == 0
           : (target_stale_r_ > 0.0 && !gsnap.has_gain_within(explore_target_, target_stale_r_)));
  const PathScore old_score = have_commit ? score_path(cur, remaining, obs, ggcfg_) : PathScore{};
  const bool handoff_due = have_commit && old_score.length < explore_handoff_m_;
  const auto decision_time = now();
  const double candidate_age =
      candidate_time_valid_ ? (decision_time - last_candidate_time_).seconds() : candidate_period_;
  if (!plan_pending_ && old_clear && !target_stale && !handoff_due && candidate_age >= 0.0 &&
      candidate_age < candidate_period_) {
    ++candidate_skips_;
    last_plan_time_ = decision_time;
    return;
  }
  last_candidate_time_ = decision_time;
  candidate_time_valid_ = true;

  // Candidate selection may advance bands, but rejected plans must not
  // change the band followed by the still-active route.
  int candidate_band = cur_band_;
  FrontierSelection observation;
  Path2 wp;
  if (frontier_observation_enabled_) {
    double route_heading = std::numeric_limits<double>::quiet_NaN();
    if (!plan_pending_ && old_clear && tracker_->remaining_distance() > goal_tol_) {
      const double ahead_s = tracker_->progress_distance() + frontier_continuity_lookahead_;
      Vec2 ahead = traj_.back().p;
      for (const auto &point : traj_) {
        if (point.s >= ahead_s) {
          ahead = point.p;
          break;
        }
      }
      if (std::hypot(ahead.x - cur.x, ahead.y - cur.y) > goal_tol_)
        route_heading = std::atan2(ahead.y - cur.y, ahead.x - cur.x);
    }
    observation = select_observation_target(gsnap, fcfg_, cur, yaw_, candidate_band, obs,
                                            &unreachable_, unreach_block_r_, route_heading);
    wp.push_back(cur);
    if (observation.valid)
      wp.push_back(observation.point);
  } else {
    wp = plan_explore(gsnap, fcfg_, cur, yaw_, candidate_band, all_explored, &unreachable_,
                      unreach_block_r_);
  }
  if (wp.size() < 2) {
    // An empty candidate list is not a request to A* back to cur.
    // Retry after a normal planning period as observations update.
    if (!old_clear) {
      traj_.clear();
      tracker_->set_trajectory({});
      explore_has_committed_ = false;
      explore_failed_ = true;
    }
    unreachable_.clear();
    cancel_handoff_brake();
    last_replan_reason_ = "no_observation_candidate";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "[探索选点] 暂无新观察点，覆盖率 %.1f%%，%s；下一规划周期重试",
                         cov_now * 100.0, old_clear ? "继续当前安全路线" : "等待可用路线");
    plan_pending_ = false;
    last_plan_time_ = now();
    return;
  }

  // 挑覆盖路径上第一个距当前 ≥ explore_target_min_dist_ 的点作 A* 终点：
  //   太近绕行无意义、易频繁过期；只需绕近处障碍，到了自然周期重规划接力。
  Vec2 scan_target = wp.empty() ? cur : wp.back();
  for (size_t i = 1; !frontier_observation_enabled_ && i < wp.size(); ++i) {
    if (std::hypot(wp[i].x - cur.x, wp[i].y - cur.y) >= explore_target_min_dist_) {
      scan_target = wp[i];
      break;
    }
  }

  // ★路径承诺/迟滞★：已有绕障折线时，先判旧折线是否仍无碰撞、目标是否漂移。
  //   仅在【目标大幅移动(换区)】或【旧折线被挡(会撞)】时才考虑重算 A*。
  const bool target_moved =
      have_commit && std::hypot(scan_target.x - explore_target_.x,
                                scan_target.y - explore_target_.y) > commit_target_tol_;

  // ★目标失效(2026-08)★：承诺目标【邻域内已无任何未扫大格】→ 这条承诺已无信息可拿。
  //   成因：飞向目标的途中，机载视野(FOV 100°/3m)常常早就把那片扫完了；但承诺/迟滞
  //   只看"目标动没动、路撞不撞"，从不问"这个目标还值不值得去"，于是飞机咬着一个
  //   零收益的点一路飞到跟前 —— 正是"明明已覆盖、还在往前走"的直接原因。
  //   失效即【绕过下面两道迟滞闸】立刻改投新目标(见 stale 在两处的短路)。
  //   ★半径口径★：看邻域而非仅目标格自身。仿真(6 场景, 主指标=达 90% 覆盖用时)：
  //     基线 281.0s | 仅看目标格 283.3s(无改善,3 场景更慢) | 邻域 R=1.0m 242.3s(-13.8%,全面更快)
  //     R=2.0m 反而 293.3s —— 半径过大会过早放弃仍有价值的目标。故取 R=1.0m。
  // 目标几乎没动 且 旧折线仍无碰撞 且 目标仍有收益 → 直接续用，连 A* 都不搜(最省、最稳，绝不翻边)。
  if (have_commit && !target_moved && old_clear && !target_stale) {
    cancel_handoff_brake();
    plan_pending_ = false;
    last_plan_time_ = now(); // 续命，避免下一拍又因 age 触发重搜
    return;
  }
  GlobalResult gr = search_explore_global(cur, scan_target, obs, yaw_);
  if (gr.ok && gr.path.size() >= 2 && carlike_mode_ && cone_hook_deg_ > 0.0) {
    // 钩子 = 起步段需大角度偏离机头【且】实际执行曲线在此处是小半径急弯。
    //   只看角度会把正常侧向转弯也拒掉，旧路安全时又只能"保持旧路"，每个候选周期反复空转
    //   (r05：连拒 13 次直到旧路走完再停车转向)。只看执行曲线的曲率才对应"沿钩子爬行"。
    const Path2 guides = clean_explore_guides(gr.path, obs);
    const double hook = cone_hook_angle(cur, guides);
    if (hook >= cone_hook_deg_ * M_PI / 180.0) {
      const double kappa = start_curvature(blend_explore_transition(guides, obs), obs);
      if (kappa >= cone_hook_kappa_) {
        ++cone_hook_rejections_;
        RCLCPP_INFO(get_logger(),
                    "机头锥内的路起步 %.1fm 内急转 %.0f°、执行曲率 %.1f/m → "
                    "视为锥内无解，改走停车转向/原地找解 (目标 %.2f,%.2f)",
                    cone_hook_window_, hook * 180.0 / M_PI, kappa, scan_target.x, scan_target.y);
        gr.ok = false;
      }
    }
  }
  // ★锥内绕远检测★：锥内路为了不掉头可能先顺着机头走再绕一大圈(r116：机头朝北，去西南角的锥内路
  //   先北后西绕柱，14 s
  //   覆盖率不涨)。只在明显绕远时才补算一次无锥路，按时间比较"原地转向+直达"与"锥内绕行"。
  std::optional<GlobalResult> detour_probe;
  if (gr.ok && gr.path.size() >= 2 && carlike_mode_) {
    const auto length = [](const Path2 &path) {
      double total = 0.0;
      for (size_t i = 1; i < path.size(); ++i)
        total += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      return total;
    };
    const double direct = std::hypot(scan_target.x - cur.x, scan_target.y - cur.y);
    const double cone_length = length(gr.path);
    if (cone_length > 1.5 * direct + 1.0) {
      detour_probe = search_explore_global(cur, scan_target, obs);
      if (detour_probe->ok && detour_probe->path.size() >= 2) {
        const auto &first = detour_probe->path[1];
        const double turn = std::abs(wrap_pi(std::atan2(first.y - cur.y, first.x - cur.x) - yaw_));
        const double cruise = std::max(0.2, 0.6 * gains_.v_max); // 探索段实际平均速度约为上限的 0.6
        const double turn_first = length(detour_probe->path) / cruise +
                                  turn / std::max(0.2, gains_.max_yaw_rate) +
                                  1.5; // 1.5 s = 刹停+起步
        const double detour = cone_length / cruise;
        if (turn_first + 1.0 < detour) {
          ++cone_detour_rejections_;
          RCLCPP_INFO(get_logger(),
                      "机头锥内的路绕远(%.1fm，直线%.1fm)：原地转%.0f°再直达预计%.1fs < 绕行%.1fs "
                      "→ 改走转向 (目标 %.2f,%.2f)",
                      cone_length, direct, turn * 180.0 / M_PI, turn_first, detour, scan_target.x,
                      scan_target.y);
          gr.ok = false;
        }
      }
    }
  }
  if (gr.ok && gr.path.size() >= 2) {
    // ★换路评估(迟滞第二道闸)★：目标漂移触发了重算，但只要旧折线【仍无碰撞】就别急着换——
    //   给旧路/新路各打分(score_path: 剩余弧长=时间效益, 最小障碍边距=安全性)，新路需
    //   【更安全≥safety_gain】且/或【更快≥time_gain】(require_both 控制"且/或")才换，否则续用旧路。
    //   关键：左右横跳的两条路是 near-tie(几乎等长、等安全)→两项都达不到阈值→不换→根除横跳。
    //   例外：旧折线已被挡(old_clear=false 会撞)、旧路快走完(剩余<前瞻)、
    //         或【旧目标已失效(扫完了)】→ 跳过评估直接采纳(安全/进度优先)。
    //   ★target_stale 必须在这里也短路★：否则失效目标虽被识别，却仍可能因
    //   "新路不够安全/不够快"被这道闸打回、继续咬着零收益目标飞 —— 修了等于没修。
    if (old_clear && !target_stale) {
      const PathScore &so = old_score; // 当前真正执行轨迹的剩余段
      const PathScore sn = score_path(cur, gr.path, obs, ggcfg_); // 新路
      const bool exhausted = so.length < explore_handoff_m_; // 旧路快走完→无可咬死，放行换路
      const bool safer = sn.min_clear >= so.min_clear * (1.0 + path_switch_safety_gain_);
      const bool faster = sn.length <= so.length * (1.0 - path_switch_time_gain_);
      const bool worth =
          exhausted || (path_switch_require_both_ ? (safer && faster) : (safer || faster));
      if (!worth) {
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1500,
            "换路评估→保持旧路(旧 长%.2f 边距%.2f | 新 长%.2f 边距%.2f; 更安全=%d 更快=%d)",
            so.length, so.min_clear, sn.length, sn.min_clear, safer ? 1 : 0, faster ? 1 : 0);
        cancel_handoff_brake();
        plan_pending_ = false;
        last_plan_time_ = now();
        return; // 不够好 → 续用旧轨迹，绝不翻边
      }
    }
    if (adopt_explore_path(gr, scan_target, &observation)) {
      last_replan_reason_ =
          !have_commit
              ? "initial_or_recovery"
              : (!old_clear ? "route_blocked"
                            : (target_stale ? "view_exhausted"
                                            : (handoff_due ? "route_end" : "better_route")));
      RCLCPP_INFO(get_logger(), "探索换路[%s] → (%.2f, %.2f)，原目标可见未扫格=%d",
                  last_replan_reason_.c_str(), scan_target.x, scan_target.y, active_target_gain_);
      cur_band_ = candidate_band;
      return;
    }
    if (explore_handoff_deferred_)
      return;
  }
  if (old_clear && !target_stale) {
    // Failure of a speculative candidate does not invalidate the
    // safe, productive trajectory already being flown.
    last_replan_reason_ = "keep_route_candidate_failed";
    cancel_handoff_brake();
    ++kept_after_candidate_failure_;
    plan_pending_ = false;
    last_plan_time_ = now();
    return;
  }
  {
    // ★机头锥内无解★：先用【无锥 probe】判定到底是"任何朝向都没路"还是"有路只是不在机头方向"。
    // 清空前保存运动状态：清空会把上一拍指令归零，直接采纳无锥路径时前进指令一拍跌到 0(r10)。
    const TrajectoryTracker motion = *tracker_;
    explore_failed_ = true;
    explore_has_committed_ = false;
    traj_ = Trajectory{};
    tracker_->set_trajectory(traj_);

    GlobalResult probe =
        detour_probe ? *detour_probe : search_explore_global(cur, scan_target, obs);
    const double probe_turn =
        probe.ok && probe.path.size() >= 2
            ? std::abs(wrap_pi(std::atan2(probe.path[1].y - cur.y, probe.path[1].x - cur.x) - yaw_))
            : M_PI;
    // 首段与机头夹角超过可构造过渡曲线的上限(120°)时仍走原地转身：仿真 r2 里把近掉头
    // 路线直接交给 tracker，它会先朝 Catmull-Rom 折返点以 0.02 m/s 爬 14 s 再停车转向。
    if (probe.ok && probe.path.size() >= 2 && probe_turn <= turn_blend_max_angle_rad_ &&
        adopt_explore_path(probe, scan_target, &observation)) {
      // ★锥外直接采纳(2026-09-23)★：锥内无解但无锥有路时，先把这条路交给 tracker，
      //   由它按航向误差决定"边转边走"(<75°, 起步段经 blend_explore_transition 用机头
      //   方向平滑接入)或"XY定点停车转向"(≥75°)。adopt 内已对过渡曲线做完整验障；
      //   失败才退回下面的原地转身找解。基线仿真里转身找解要等角速度停稳再重搜、
      //   再采纳，三次共停约 5 s，是探索段第二大卡顿源。
      explore_failed_ = false;
      tracker_->inherit_motion(motion);
      last_replan_reason_ = "cone_bypass_adopted";
      ++cone_bypass_adoptions_;
      cur_band_ = candidate_band;
      RCLCPP_INFO(get_logger(), "机头锥内无路但侧向有路 → 直接采纳并由跟踪器转向 (目标 %.2f,%.2f)",
                  scan_target.x, scan_target.y);
      last_plan_time_ = now();
      return;
    }
    if (probe.ok && probe.path.size() >= 2) {
      // 有路，只是不在机头方向 → 原地转身找解(改朝向重搜)，不后退/不跳带。
      enter_turn_for_solution(cur, scan_target, probe, candidate_band, &observation);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "机头方向无路但侧向有路 → 原地转身找解 (目标 %.2f,%.2f)", scan_target.x,
                           scan_target.y);
    } else {
      // 任何朝向都没路 → 走原有 retreat(贴脸脱困) / 跳带(真被围死) 逻辑。
      const double od = nearest_obstacle_dist(obs);
      const bool can_retreat = (od < retreat_trigger_) && retreat_available();
      if (can_retreat) {
        // 贴障且未退够 → 不跳带/不拉黑，交主循环后退脱困，退出去下拍重规划接着绕。
        plan_pending_ = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "探索目标 (%.2f, %.2f) A* 无解且贴障 → 后退脱困重试", scan_target.x,
                             scan_target.y);
      } else {
        // 不是贴脸卡死/已退到极限仍无路 = 真够不到 → 放弃该区：拉黑 + 跳带 + 红叉 + 去别处。
        retreating_ = false;
        unreachable_.push_back(scan_target); // 拉黑够不到的目标：选点不再回头试这条死路
        plan_pending_ = false;
        last_plan_time_ = now();
        has_unreachable_marker_ = true; // ★画红叉★
        unreachable_pos_ = scan_target;
        RCLCPP_WARN(get_logger(),
                    "探索观察点 (%.2f, %.2f) 暂无路径 → 暂缓此点，下一规划周期重试其他观察点",
                    scan_target.x, scan_target.y);
      }
    }
  }
  last_plan_time_ = now();
}

GlobalResult ExplorationNode::search_global(const Vec2 &start, const Vec2 &goal,
                                            const Obstacles &obstacles, const GlobalConfig &config,
                                            double yaw) {
  ++astar_searches_;
  return plan_global_path(start, goal, obstacles, config, yaw);
}

GlobalResult ExplorationNode::search_explore_global(const Vec2 &start, const Vec2 &goal,
                                                    const Obstacles &obstacles, double yaw) {
  GlobalConfig config = ggcfg_;
  config.validate_complete_path = true;
  if (explore_plan_reserve_ > 0.0) {
    config.inflate += explore_plan_reserve_;
    auto reserved = search_global(start, goal, obstacles, config, yaw);
    if (reserved.ok)
      return reserved;
    config.inflate = ggcfg_.inflate;
  }
  return search_global(start, goal, obstacles, config, yaw);
}

} // namespace exploration
