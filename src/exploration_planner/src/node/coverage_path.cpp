#include "exploration_node.hpp"

namespace exploration {

bool ExplorationNode::adopt_explore_path(const GlobalResult &gr, const Vec2 &scan_target,
                                         const FrontierSelection *observation) {
  explore_handoff_deferred_ = false;
  // 用局部相切圆角生成连续轨迹，已构造的起步曲线保留原形状。
  // 再套一遍穿点样条会在曲线接直线处过冲，凭空插入反向曲率。
  const Obstacles transition_obs = obs_map_->snapshot();
  if (gr.path.size() < 2 || !path_inside_safe_field(gr.path) ||
      !path_clear(gr.path.front(), gr.path, transition_obs, ggcfg_)) {
    ++invalid_paths_;
    return false;
  }
  const Path2 blended =
      blend_explore_transition(clean_explore_guides(gr.path, transition_obs), transition_obs);
  // 小间距采样点描述同一条曲线，不作为中间停车点。其余真实折点只在
  // 局部圆角通过曲率、障碍及场界检查后替换；直线段不向外鼓包。
  const Trajectory smooth_traj = rounded_fallback_trajectory(blended, ggcfg_, transition_obs);
  Path2 smooth_points;
  smooth_points.reserve(smooth_traj.size());
  for (const auto &tp : smooth_traj)
    smooth_points.push_back(tp.p);

  // 局部验证之外，最终整条轨迹也必须满足同一安全口径。
  const bool field_safe = smooth_points.size() >= 2 && path_inside_safe_field(smooth_points);
  const bool obstacle_safe =
      field_safe && path_clear(blended.front(), smooth_points, transition_obs, ggcfg_);
  const bool smooth_safe = field_safe && obstacle_safe;
  Trajectory execution = smooth_safe ? smooth_traj : make_polyline_trajectory(blended);
  const bool handoff_moving = v_fwd_est_ > handoff_brake_speed_;
  if (carlike_mode_ && explore_has_committed_ && tracker_->has_trajectory() &&
      (tracker_->remaining_distance() <= explore_handoff_m_ || handoff_moving) &&
      !observation_arrived_ && tracker_->remaining_distance() > goal_tol_ &&
      path_clear({px_, py_}, tracker_->remaining_path(px_, py_), transition_obs, ggcfg_)) {
    // Finish the checked incoming segment before a reverse handoff.
    // Arrival, rather than an unattainable zero-speed state, releases it.
    // Use the actual candidate and tracker gate so flowing curves are
    // unaffected; the old validated endpoint provides its normal brake.
    // ★换路前预减速(2026-09-23)★：不只在旧路快走完时——任何需要停车转向的换路
    //   (如 view_exhausted 掉头)若飞机仍在巡航速，先沿旧的安全路线按斜坡减到
    //   handoff_brake_speed_ 以下再采纳，否则 tracker 一进入 XY 定点转向，惯性会把飞机
    //   带离新路线(seed42 r3 实测偏离 0.27 m)。
    TrajectoryTracker preview(gains_);
    preview.set_trajectory(execution);
    preview.update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_, yaw_rate_est_);
    // 预演只看一拍：机头方向平滑接入后首拍航向误差常落在 60~75°，采纳后一拍才越过
    // 75° 停车线(seed43 实测 64°→76°)。因此巡航中只要新路线前瞻方位误差已超过
    // handoff_brake_angle_，就先减速再采纳，不等它真的触发停车转向。
    // 预演一拍的前瞻点常仍落在沿机头接入的起始段上(seed43 实测 13°→下一拍 64°)，
    // 改为直接量新路线前方 lookahead+0.6 m 内切线与机头的最大夹角。
    double upcoming_turn = std::abs(preview.heading_error());
    for (const auto &point : execution) {
      if (point.s > execution.front().s + gains_.lookahead + 0.6)
        break;
      upcoming_turn = std::max(upcoming_turn, std::abs(wrap_pi(point.theta - yaw_)));
    }
    const bool sharp_handoff =
        preview.reorienting() || (handoff_moving && upcoming_turn >= handoff_brake_angle_);
    if (sharp_handoff) {
      explore_handoff_deferred_ = true;
      handoff_brake_ = handoff_moving;
      last_replan_reason_ =
          handoff_moving ? "brake_before_turn_handoff" : "brake_before_reverse_handoff";
      plan_pending_ = false;
      last_plan_time_ = now();
      return false;
    }
  }
  handoff_brake_ = false;
  ++adoptions_;
  explore_raw_ = blended;
  smooth_failure_reason_ = smooth_safe ? "none" : (field_safe ? "obstacle" : "field");
  if (smooth_safe) {
    traj_ = execution;
  } else {
    ++smooth_fallbacks_;
    traj_ = execution;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                         "平滑轨迹切入安全区 → 退回安全折线跟踪(障碍外表面余量 %.2fm)",
                         ggcfg_.inflate + ggcfg_.robot_radius);
  }
  explore_failed_ = false;
  retreat_exhausted_ = false;
  explore_has_committed_ = true;
  explore_target_ = scan_target;
  explore_goal_projected_ =
      std::hypot(gr.path.back().x - scan_target.x, gr.path.back().y - scan_target.y) > 1e-6;
  have_observation_target_ = observation && observation->valid;
  observation_arrived_ = false;
  if (have_observation_target_) {
    observation_target_ = *observation;
    observation_target_.point = gr.path.back();
    explore_target_ = gr.path.back();
    // Normal viewpoints observe along the executed arrival tangent.
    // Only the explicit cleanup fallback requests a separate look-at turn.
    observation_target_.view_heading =
        observation_target_.requires_turn
            ? std::atan2(observation_target_.look_at.y - explore_target_.y,
                         observation_target_.look_at.x - explore_target_.x)
            : traj_.back().theta;
  }
  observation_yaw_rate_ = 0.0;
  observation_hold_.begin(explore_target_);
  observation_turn_active_ = false;
  tracker_->set_trajectory(traj_);
  plan_pending_ = false;
  last_plan_time_ = now();
  has_unreachable_marker_ = false; // 又有路了 → 撤红叉
  return true;
}

void ExplorationNode::cancel_handoff_brake() {
  explore_handoff_deferred_ = false;
  handoff_brake_ = false;
}

bool ExplorationNode::path_inside_safe_field(const Path2 &path) const {
  return exploration::path_inside_safe_field(path, ggcfg_);
}

GlobalConfig ExplorationNode::explore_exec_config() const {
  GlobalConfig cfg = ggcfg_;
  cfg.inflate = std::min(cfg.inflate, explore_guard_inflate_);
  return cfg;
}

bool ExplorationNode::turn_correction_clear(Vec2 correction, const Obstacles &obstacles) const {
  const Vec2 cur{px_, py_};
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const Vec2 world{c * correction.x - s * correction.y, s * correction.x + c * correction.y};
  const double horizon = std::max(0.5, std::hypot(world.x, world.y) / (2.0 * gains_.max_accel));
  const Vec2 projected{cur.x + world.x * horizon, cur.y + world.y * horizon};
  // 执行口径(见 explore_guard_inflate_)：物理半径仍不可侵入，规划余量允许被跟踪误差吃掉一部分。
  return field_motion_clear(cur, projected, ggcfg_) &&
         obstacle_segment_clear(cur, projected, obstacles, explore_exec_config());
}

void ExplorationNode::begin_hold_from_motion(PositionHold &hold) {
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const Vec2 v{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
  const double speed = std::hypot(v.x, v.y);
  Vec2 anchor{px_, py_};
  if (speed > 1e-3) {
    const double d = std::min(gains_.stop_anchor_max_m,
                              speed * speed / (2.0 * std::max(0.1, gains_.stop_anchor_decel)) +
                                  std::max(0.0, gains_.stop_anchor_latency) * speed);
    anchor = {px_ + v.x / speed * d, py_ + v.y / speed * d};
  }
  hold.begin(anchor, last_command_world_, true, gains_.stop_anchor_max_m);
}

Vec2 ExplorationNode::turn_position_correction(PositionHold &hold, const Obstacles &obstacles) {
  // 实测滑行方向不安全就结束滑行、恢复反推(审查：滑行不反推时，实测漂移原本没有任何检查)。
  if (hold.sliding() && !turn_correction_clear({v_fwd_est_, v_lat_est_}, obstacles))
    hold.end_slide();
  const auto correction = hold.update(
      {px_, py_}, yaw_, v_fwd_est_, v_lat_est_, kp_goal_, kd_goal_, std::min(0.20, v_goal_max_),
      std::max(gains_.max_accel, gains_.stop_anchor_decel), gains_.dt);
  if (!turn_correction_clear(correction, obstacles)) {
    hold.stop();
    return {};
  }
  return correction;
}

void ExplorationNode::finish_observation(VelCmd &command, const Obstacles &obstacles) {
  if (!have_observation_target_ || (!command.at_goal && !observation_arrived_))
    return;
  if (!observation_arrived_)
    begin_hold_from_motion(observation_hold_);
  observation_arrived_ = true;
  const Vec2 cur{px_, py_};
  const auto snapshot = grid_->snapshot();
  const double heading = observation_target_.view_heading;
  const double error = wrap_pi(heading - yaw_);
  const bool need_turn = !observation_turn_active_ &&
                         visible_unknown_count(snapshot, cur, yaw_, obstacles, fcfg_) == 0 &&
                         visible_unknown_count(snapshot, cur, heading, obstacles, fcfg_) > 0 &&
                         std::abs(error) > 0.10;
  if (need_turn)
    observation_turn_active_ = true;
  command = {};
  const auto correction = turn_position_correction(observation_hold_, obstacles);
  command.v_fwd = correction.x;
  command.v_lat = correction.y;
  if (!observation_turn_active_ || std::abs(error) <= 0.10) {
    // This viewpoint has been sampled. Try another viewpoint if its
    // partially occluded cells still need coverage; never mark them here.
    unreachable_.push_back(explore_target_);
    have_observation_target_ = false;
    observation_arrived_ = false;
    explore_has_committed_ = false;
    tracker_->set_trajectory({});
    traj_.clear();
    plan_pending_ = true;
    observation_yaw_rate_ = 0.0;
    observation_turn_active_ = false;
    return;
  }
  if (!obstacle_segment_clear(cur, cur, obstacles, ggcfg_)) {
    observation_yaw_rate_ = 0.0;
    return;
  }
  const double rate = std::clamp(gains_.kp_yaw * error - gains_.kd_yaw * yaw_rate_est_,
                                 -gains_.max_yaw_rate, gains_.max_yaw_rate);
  observation_yaw_rate_ +=
      std::clamp(rate - observation_yaw_rate_, -gains_.max_yaw_accel * gains_.dt,
                 gains_.max_yaw_accel * gains_.dt);
  command.yaw_rate = observation_yaw_rate_;
}

Trajectory ExplorationNode::rounded_fallback_trajectory(const Path2 &path, const GlobalConfig &cfg,
                                                        const Obstacles &obs) const {
  CornerRounding options;
  options.sample_distance = arc_ds_;
  options.min_distance = std::max(1e-3, turn_round_min_m_);
  options.max_curvature =
      turn_round_max_curvature_ > 0.0 ? turn_round_max_curvature_ : options.max_curvature;
  const Path2 rounded = round_path_corners(path, options, [&](const Path2 &piece) {
    return exploration::path_inside_safe_field(piece, cfg) &&
           path_clear(piece.front(), piece, obs, cfg);
  });
  return trajectory_from_polyline(rounded, arc_ds_, gains_.corner_stop_rad);
}

Trajectory ExplorationNode::make_polyline_trajectory(const Path2 &path) const {
  Trajectory out;
  if (path.empty())
    return out;

  Path2 clean;
  clean.reserve(path.size());
  for (const auto &p : path) {
    if (clean.empty() || std::hypot(p.x - clean.back().x, p.y - clean.back().y) > 1e-6)
      clean.push_back(p);
  }
  if (clean.empty())
    return out;

  out.resize(clean.size());
  double s = 0.0;
  for (size_t i = 0; i < clean.size(); ++i) {
    if (i > 0)
      s += std::hypot(clean[i].x - clean[i - 1].x, clean[i].y - clean[i - 1].y);
    out[i].p = clean[i];
    out[i].s = s;
    out[i].kappa = 0.0;
    const size_t ia = (i == 0) ? 0 : i - 1;
    const size_t ib = (i + 1 < clean.size()) ? i + 1 : i;
    out[i].theta = std::atan2(clean[ib].y - clean[ia].y, clean[ib].x - clean[ia].x);
  }
  return out;
}

Path2 ExplorationNode::clean_explore_guides(const Path2 &path, const Obstacles &obs) const {
  Path2 result = path;
  for (size_t i = 1; i + 1 < result.size();) {
    const Vec2 a = result[i - 1], b = result[i], c = result[i + 1];
    const double dx = c.x - a.x, dy = c.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double shorter =
        std::min(std::hypot(b.x - a.x, b.y - a.y), std::hypot(c.x - b.x, c.y - b.y));
    const double u = length2 > 1e-12 ? ((b.x - a.x) * dx + (b.y - a.y) * dy) / length2 : -1.0;
    const double deviation = std::hypot(b.x - a.x - u * dx, b.y - a.y - u * dy);
    const Path2 shortcut{a, c};
    if (shorter < guide_min_segment_ && u >= 0.0 && u <= 1.0 && deviation <= guide_max_deviation_ &&
        path_inside_safe_field(shortcut) && path_clear(a, shortcut, obs, ggcfg_)) {
      result.erase(result.begin() + i);
      if (i > 1)
        --i;
    } else {
      ++i;
    }
  }
  return result;
}

Path2 ExplorationNode::blend_explore_transition(const Path2 &path, const Obstacles &obs) {
  if (!carlike_mode_ || path.size() < 2 || turn_blend_m_ <= 0.0)
    return path;

  const auto safe = [&](const Path2 &candidate) {
    return candidate.size() >= 2 && path_inside_safe_field(candidate) &&
           path_clear(candidate.front(), candidate, obs, ggcfg_);
  };
  // Keep the checked guides when a monotone tangent connection is unavailable.
  const Path2 rounded = path;
  if (rounded.size() < 2)
    return path;

  const Vec2 p0 = rounded.front();
  const Vec2 p3 = rounded[1];
  const double d03 = std::hypot(p3.x - p0.x, p3.y - p0.y);
  if (d03 < 1e-3)
    return rounded;

  // Use measured heading at entry; a following guide may provide an exit
  // tangent only after the complete join passes geometry/curvature checks.
  const Vec2 old_dir{std::cos(yaw_), std::sin(yaw_)};
  const Vec2 new_dir{(p3.x - p0.x) / d03, (p3.y - p0.y) / d03};
  const double dot = std::clamp(old_dir.x * new_dir.x + old_dir.y * new_dir.y, -1.0, 1.0);
  const double angle = std::acos(dot);
  if (angle < turn_blend_min_angle_rad_ || angle > turn_blend_max_angle_rad_)
    return rounded;

  const double handle = std::min(turn_blend_m_, d03 * 0.45);
  if (handle < turn_round_min_m_)
    return rounded;

  const auto connect = [&](Vec2 end_dir, double turn_sign) -> Path2 {
    // 两端切线交点限定各自控制柄，而不是硬套相同长度。短首段中
    // 出场柄越过交点会让控制多边形反折，单向绕障变成 S 形。
    const double cross = old_dir.x * end_dir.y - old_dir.y * end_dir.x;
    if (std::abs(cross) < 1e-9)
      return {};
    const Vec2 chord{p3.x - p0.x, p3.y - p0.y};
    const double incoming = (chord.x * end_dir.y - chord.y * end_dir.x) / cross;
    const double outgoing = (old_dir.x * chord.y - old_dir.y * chord.x) / cross;
    if (incoming <= 1e-6 || outgoing <= 1e-6)
      return {};
    const double first_handle = std::min(handle, incoming);
    const double last_handle = std::min(handle, outgoing);
    const Vec2 c1{p0.x + old_dir.x * first_handle, p0.y + old_dir.y * first_handle};
    const Vec2 c2{p3.x - end_dir.x * last_handle, p3.y - end_dir.y * last_handle};

    const int samples =
        std::max({4, turn_blend_samples_,
                  static_cast<int>(std::ceil((d03 + 2.0 * handle) / std::max(.005, arc_ds_)))});
    Path2 blended;
    blended.reserve(static_cast<size_t>(samples) + rounded.size());
    blended.push_back(p0);
    for (int k = 1; k <= samples; ++k) {
      const double t = static_cast<double>(k) / samples;
      const double u = 1.0 - t;
      const double w0 = u * u * u;
      const double w1 = 3.0 * u * u * t;
      const double w2 = 3.0 * u * t * t;
      const double w3 = t * t * t;
      blended.push_back({w0 * p0.x + w1 * c1.x + w2 * c2.x + w3 * p3.x,
                         w0 * p0.y + w1 * c1.y + w2 * c2.y + w3 * p3.y});
    }
    // Evaluate the cubic derivatives more densely than the executed
    // samples before accepting a start transition. Curvature clipping
    // would conceal a hook rather than make it flyable.
    for (int sample = 0; sample <= samples * 4; ++sample) {
      const double t = static_cast<double>(sample) / (samples * 4), u = 1.0 - t;
      const Vec2 derivative{
          3 * u * u * (c1.x - p0.x) + 6 * u * t * (c2.x - c1.x) + 3 * t * t * (p3.x - c2.x),
          3 * u * u * (c1.y - p0.y) + 6 * u * t * (c2.y - c1.y) + 3 * t * t * (p3.y - c2.y)};
      const Vec2 second{6 * u * (c2.x - 2 * c1.x + p0.x) + 6 * t * (p3.x - 2 * c2.x + c1.x),
                        6 * u * (c2.y - 2 * c1.y + p0.y) + 6 * t * (p3.y - 2 * c2.y + c1.y)};
      const double speed = std::hypot(derivative.x, derivative.y);
      if (speed < 1e-6 || derivative.x * new_dir.x + derivative.y * new_dir.y < -1e-9 ||
          std::abs(derivative.x * second.y - derivative.y * second.x) / (speed * speed * speed) >
              turn_round_max_curvature_)
        return {};
      if (turn_sign * (derivative.x * second.y - derivative.y * second.x) < -1e-9)
        return {};
    }
    for (size_t i = 2; i < rounded.size(); ++i)
      blended.push_back(rounded[i]);

    if (!safe(blended))
      return {};
    return blended;
  };
  // Facing the first chord at the end of a curved join can create an S:
  // turn past that heading, turn back to it, then immediately turn again
  // for the next leg. Prefer one continuous turn tangent to the next leg
  // when its direction brackets the chord and the whole curve is safe.
  if (rounded.size() > 2) {
    const Vec2 next = rounded[2];
    const double length = std::hypot(next.x - p3.x, next.y - p3.y);
    if (length > 1e-6) {
      const Vec2 end_dir{(next.x - p3.x) / length, (next.y - p3.y) / length};
      const double turn = wrap_pi(std::atan2(end_dir.y, end_dir.x) - yaw_);
      const double chord = wrap_pi(std::atan2(new_dir.y, new_dir.x) - yaw_);
      if (std::abs(turn) < turn_blend_max_angle_rad_ && turn * chord > 0 &&
          std::abs(chord) <= std::abs(turn)) {
        const Path2 continuous = connect(end_dir, turn > 0 ? 1 : -1);
        if (!continuous.empty())
          return continuous;
      }
    }
  }
  // 两点直线没有需要衔接的下一段切线。强迫曲线从当前机头出发又
  // 沿首段弦方向结束，必然先转过弦方向再转回。保持已检查的引导线，
  // 由跟踪器处理航向误差和 XY 纠偏；不能回退到允许反向曲率的曲线。
  return rounded;
}

double ExplorationNode::cone_hook_angle(const Vec2 &cur, const Path2 &path) const {
  double s = 0.0, worst = 0.0;
  Vec2 from = cur;
  for (size_t i = 1; i < path.size() && s <= cone_hook_window_; ++i) {
    const double len = std::hypot(path[i].x - from.x, path[i].y - from.y);
    if (len > 0.03)
      worst = std::max(
          worst, std::abs(wrap_pi(std::atan2(path[i].y - from.y, path[i].x - from.x) - yaw_)));
    s += len;
    from = path[i];
  }
  return worst;
}

double ExplorationNode::start_curvature(const Path2 &path, const Obstacles &obs) const {
  if (path.size() < 2)
    return 0.0;
  Trajectory t = rounded_fallback_trajectory(path, ggcfg_, obs);
  Path2 points;
  for (const auto &p : t)
    points.push_back(p.p);
  if (!path_inside_safe_field(points) || !path_clear(path.front(), points, obs, ggcfg_))
    t = make_polyline_trajectory(path);
  double k = 0.0;
  for (const auto &p : t) {
    if (p.s - t.front().s > cone_hook_window_)
      break;
    k = std::max(k, std::abs(p.kappa));
  }
  return k;
}

} // namespace exploration
