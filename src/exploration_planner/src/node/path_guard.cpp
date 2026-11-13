#include "exploration_node.hpp"

namespace exploration {

Vec2 ExplorationNode::pd_core(const Vec2 &heading_pt, double dist_for_speed) {
  const double ex = heading_pt.x - px_;
  const double ey = heading_pt.y - py_;
  const double dir = std::hypot(ex, ey);

  const double v_now = std::hypot(v_fwd_est_, v_lat_est_);
  double v_des = kp_goal_ * dist_for_speed - kd_goal_ * v_now;
  v_des = std::clamp(v_des, 0.0, v_goal_max_);

  double yaw_rate = 0.0, e_yaw = 0.0;
  if (dist_for_speed > goal_tol_ && dir > 1e-6) {
    const double yaw_des = std::atan2(ey, ex);
    e_yaw = yaw_des - yaw_;
    while (e_yaw > M_PI)
      e_yaw -= 2.0 * M_PI;
    while (e_yaw <= -M_PI)
      e_yaw += 2.0 * M_PI;
    const double predicted_error =
        e_yaw - (carlike_mode_ ? gains_.prediction_time * yaw_rate_est_ : 0.0);
    const double damping = carlike_mode_ ? gains_.kd_yaw * yaw_rate_est_ : 0.0;
    yaw_rate = std::clamp(gains_.kp_yaw * predicted_error - damping, -gains_.max_yaw_rate,
                          gains_.max_yaw_rate);
  }

  Vec2 vb{0.0, 0.0};
  if (dir > 1e-6) {
    const double ux = ex / dir, uy = ey / dir;
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    const double fwd = c * ux + s * uy;
    const double lat = -s * ux + c * uy;
    // ★先转再走·朝向门控★：机头偏离目标方向越大,越压住移动(前进+横向都乘),先转够再走。
    //   |e_yaw| > 阈值 → 门=0,本拍只转身不移动(防身后/大角度目标时机体平移甩出去撞柱);
    //   阈值内 cos(e_yaw) 平滑过渡。光压前进不压横向→飞机仍侧移甩出去,故横向也乘。
    double head_gate =
        (std::fabs(e_yaw) > gains_.heading_gate_rad) ? 0.0 : std::max(0.0, std::cos(e_yaw));
    if (carlike_mode_) {
      const double ratio =
          std::clamp((std::fabs(e_yaw) - gains_.heading_gate_rad) /
                         std::max(1e-3, gains_.stop_align_rad - gains_.heading_gate_rad),
                     0.0, 1.0);
      head_gate = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio);
      const double turning_rate = std::max(std::fabs(yaw_rate), std::fabs(yaw_rate_est_));
      if (turning_rate > 1e-3)
        v_des = std::min(v_des, gains_.max_lateral_accel / turning_rate);
    }
    vb.x = v_des * fwd * head_gate;
    vb.y = std::clamp(v_des * lat, -gains_.max_v_lat, gains_.max_v_lat) * head_gate;
  }
  last_yaw_rate_ = yaw_rate;
  return vb;
}

Vec2 ExplorationNode::pd_to_point(const Vec2 &target) {
  return pd_core(target, std::hypot(target.x - px_, target.y - py_));
}

void ExplorationNode::hold_required_path(const std::string &reason) {
  // Holds used to be invisible in the logs; a stall beside an obstacle
  // could only be reconstructed from velocity traces.
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                       "[必达路径] 保持零速: 原因=%s 位置(%.2f,%.2f) 目标(%.2f,%.2f)",
                       reason.c_str(), px_, py_, global_target_.x, global_target_.y);
  required_goal_holding_ = false;
  global_failed_ = true;
  global_has_ = global_at_goal_ = global_braking_blocked_ = false;
  command_projection_blocked_ = measured_projection_blocked_ = false;
  required_blocked_time_valid_ = false;
  global_raw_.clear();
  global_traj_.clear();
  global_tracker_->set_trajectory({});
  global_block_reason_ = reason;
  traj_.clear();
  last_look_ = {px_, py_};
  look_valid_ = true;
  last_yaw_rate_ = 0.0;
  // 不在这里清 retreating_：此函数在路径失效期间每拍都会调用，清零会让后退起点每拍重置，
  //   1.2 m / 2 s 的后退上限永远不触发(审查发现)。后退状态只在路径恢复或 try_retreat 放弃时清。
}

GlobalConfig ExplorationNode::required_config() const {
  GlobalConfig cfg = ggcfg_;
  if (homing_) {
    cfg.required_goal_field_margin = home_goal_wall_margin_;
    cfg.required_direct_path = true;
  }
  return cfg;
}

GlobalConfig ExplorationNode::exec_config() const {
  GlobalConfig cfg = required_config();
  cfg.inflate = std::min(cfg.inflate, required_path_exec_inflate_);
  return cfg;
}

bool ExplorationNode::required_motion_clear(const Vec2 &stop, const Obstacles &obs) const {
  const Vec2 cur{px_, py_};
  GlobalConfig motion_config = required_config();
  motion_config.inflate = required_motion_inflate_;
  const double field_margin = required_field_margin(motion_config);
  const auto allowed_field = [this, field_margin](const Vec2 &p) {
    return p.x >= ggcfg_.min_x + field_margin && p.x <= ggcfg_.max_x - field_margin &&
           p.y >= ggcfg_.min_y + field_margin && p.y <= ggcfg_.max_y - field_margin;
  };
  if (!obstacle_segment_clear(cur, stop, obs, motion_config))
    return false;
  // 按归航任务约定：设定墙线约束路线规划，不再触发执行阶段的墙线保护。
  // 实机漂移或刹停投影暂时越过设定墙线时，仍允许沿核准路线前进并纠偏，
  // 否则每次越线都会清零XY、等风把飞机吹回来才恢复。实体障碍检查仍在上面。
  if (homing_ && global_has_ && !global_failed_ && !global_traj_.empty())
    return true;
  const double low[2]{ggcfg_.min_x + ggcfg_.wall_margin, ggcfg_.min_y + ggcfg_.wall_margin};
  const double high[2]{ggcfg_.max_x - ggcfg_.wall_margin, ggcfg_.max_y - ggcfg_.wall_margin};
  const double a[2]{cur.x, cur.y};
  const auto monotonic_entry = [&](const Vec2 &end) {
    const double b[2]{end.x, end.y};
    for (size_t axis = 0; axis < 2; ++axis) {
      if (a[axis] < low[axis]) {
        if (b[axis] < a[axis] || b[axis] > high[axis])
          return false;
      } else if (a[axis] > high[axis]) {
        if (b[axis] > a[axis] || b[axis] < low[axis])
          return false;
      } else if (b[axis] < low[axis] || b[axis] > high[axis])
        return false;
    }
    return true;
  };
  // Match the planner's initial field-entry policy: an aircraft taking
  // off on a task boundary may move inward before attaining the normal
  // body/wall inset. Otherwise measured speed repeatedly trips this guard
  // Physical obstacle clearance above still applies to the whole segment.
  if (monotonic_entry(stop))
    return true;
  if (!allowed_field(cur) || !allowed_field(stop))
    return false;

  // Only the approved straight terminal connector can leave the full
  // field inset. Its execution capsule uses the existing arrival tolerance.
  const auto connector_distance = [this](const Vec2 &p) {
    const double dx = global_target_.x - global_connector_start_.x;
    const double dy = global_target_.y - global_connector_start_.y;
    const double length2 = dx * dx + dy * dy;
    const double u = length2 > 1e-12 ? std::clamp(((p.x - global_connector_start_.x) * dx +
                                                   (p.y - global_connector_start_.y) * dy) /
                                                      length2,
                                                  0.0, 1.0)
                                     : 0.0;
    return std::hypot(p.x - global_connector_start_.x - u * dx,
                      p.y - global_connector_start_.y - u * dy);
  };
  // A clear home line may run along the virtual boundary inset for more
  // than one metre. If drift leaves its narrow capsule, allow corrections
  // that recover toward the line while keeping the actual field bounds.
  const double connector_tolerance =
      homing_ && global_has_ ? std::max(goal_tol_, connector_distance(cur)) : goal_tol_;
  const auto in_connector = [&](const Vec2 &p) {
    return connector_distance(p) <= connector_tolerance + 1e-9;
  };
  if (in_connector(cur) && in_connector(stop))
    return true;

  // A single stopping segment can span the ordinary inset and the
  // approved terminal connector. Check its exact exit from the inset;
  // requiring the aircraft to be in the terminal capsule already caused
  // full-speed/zero-speed oscillation on short fields.
  const double delta[2]{stop.x - cur.x, stop.y - cur.y};
  double enter = 0.0, leave = 1.0;
  for (size_t axis = 0; axis < 2; ++axis) {
    if (std::abs(delta[axis]) < 1e-12) {
      if (a[axis] < low[axis] || a[axis] > high[axis])
        return false;
      continue;
    }
    double first = (low[axis] - a[axis]) / delta[axis];
    double last = (high[axis] - a[axis]) / delta[axis];
    if (first > last)
      std::swap(first, last);
    enter = std::max(enter, first);
    leave = std::min(leave, last);
  }
  if (enter > leave || leave < 0.0 || leave > 1.0)
    return false;
  const Vec2 exit{std::clamp(cur.x + leave * delta[0], low[0], high[0]),
                  std::clamp(cur.y + leave * delta[1], low[1], high[1])};
  return monotonic_entry(exit) && in_connector(exit) && in_connector(stop);
}

bool ExplorationNode::required_velocity_clear(const Vec2 &world_velocity,
                                              const Obstacles &obs) const {
  const double speed = std::hypot(world_velocity.x, world_velocity.y);
  if (!std::isfinite(speed))
    return false;
  // This is a short command safety projection, not a replacement for
  // the validated A* corridor.  Using the full prediction horizon here
  // made a valid required path look blocked near corners and latched
  // the controller at zero speed.  Keep a bounded look-ahead and let
  // required_path_clear() provide the long-range obstacle guarantee.
  const double horizon = std::min(0.35, std::max(0.0, global_gains_.prediction_time)) +
                         speed / (2.0 * std::max(1e-3, global_gains_.max_accel));
  return required_motion_clear({px_ + world_velocity.x * horizon, py_ + world_velocity.y * horizon},
                               obs);
}

Vec2 ExplorationNode::pd_to_point_avoid(const Vec2 &target, const Obstacles &obs,
                                        double travel_speed_limit) {
  const Vec2 cur{px_, py_};
  const GlobalConfig required_cfg = required_config();
  const double field_margin = required_field_margin(required_cfg);
  global_goal_blocked_ =
      !path_clear(target, Path2{target, target}, obs, ggcfg_) ||
      target.x < ggcfg_.min_x + field_margin || target.x > ggcfg_.max_x - field_margin ||
      target.y < ggcfg_.min_y + field_margin || target.y > ggcfg_.max_y - field_margin;
  if (global_goal_blocked_) {
    global_target_ = target;
    hold_required_path("required_goal_blocked");
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Required goal (%.2f, %.2f) is occupied or outside the safe field; holding", target.x,
        target.y);
    return {};
  }
  const bool target_moved =
      std::hypot(target.x - global_target_.x, target.y - global_target_.y) > 1e-6;
  if (target_moved)
    required_goal_holding_ = false;
  // Recheck visibility from the actual aircraft position during home.
  // Once the goal is safely visible, an old valid detour is no reason to
  // keep visiting its corners. This is independent of the 1 m descent
  // trigger. Adopt once; retain the tracker's velocity/history and run
  // its update exactly once below so the shortcut does not insert a stop.
  if (homing_ && global_has_ && !target_moved && global_raw_.size() > 2 &&
      global_tracker_->has_trajectory() &&
      required_path_clear(cur, {cur, target}, target, obs, required_cfg)) {
    global_connector_start_ = cur;
    global_raw_ = {cur, target};
    global_traj_ = make_polyline_trajectory(global_raw_);
    global_tracker_->set_trajectory(global_traj_);
    required_goal_holding_ = false;
  }
  VelCmd command;
  bool updated_existing = false;
  if (global_has_ && !target_moved && global_tracker_->has_trajectory()) {
    command =
        global_tracker_->update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_, yaw_rate_est_);
    global_tracker_ticked_ = true;
    updated_existing = true;
    if (command.needs_replan)
      global_has_ = false;
    Path2 remaining = global_tracker_->remaining_reference_path();
    if (remaining.size() == 1)
      remaining.push_back(target);
    if (!required_path_clear(cur, remaining, target, obs, exec_config()))
      global_has_ = false;
  }
  if (!global_has_ || target_moved) {
    required_goal_holding_ = false;
    const double age = (now() - last_global_plan_time_).seconds();
    if (global_failed_ && !target_moved && age >= 0.0 && age < replan_period_) {
      hold_required_path("required_path_unavailable");
      return {};
    }
    global_target_ = target;
    last_global_plan_time_ = now();
    ++astar_searches_;
    ++global_path_searches_;
    // 先带预留余量搜(与探索 explore_plan_reserve_ 同理)：最短路的折点紧贴禁入圈边缘，平滑/圆角必然
    //   切进去而被拒，只能退回纯折线逐点停车(r65)。预留后平滑曲线仍按标准余量验收；找不到路再退回标准余量。
    GlobalResult gr;
    if (required_plan_reserve_ > 0.0) {
      GlobalConfig reserved = required_cfg;
      reserved.inflate += required_plan_reserve_;
      gr = plan_required_path(cur, target, obs, reserved, yaw_);
      if (!gr.ok || gr.path.size() < 2) {
        ++astar_searches_;
        ++global_path_searches_;
        gr = plan_required_path(cur, target, obs, reserved);
      }
    }
    if (!gr.ok || gr.path.size() < 2) {
      ++astar_searches_;
      ++global_path_searches_;
      gr = plan_required_path(cur, target, obs, required_cfg, yaw_);
    }
    if (!gr.ok || gr.path.size() < 2) {
      ++astar_searches_;
      ++global_path_searches_;
      gr = plan_required_path(cur, target, obs, required_cfg);
    }
    if (!gr.ok || gr.path.size() < 2 ||
        !required_path_clear(cur, gr.path, target, obs, required_cfg)) {
      hold_required_path("required_path_unavailable");
      return {};
    }
    global_connector_start_ = gr.path[gr.path.size() - 2];
    Path2 prefix(gr.path.begin(), gr.path.end() - 1);
    Trajectory candidate = smooth_catmull_rom(prefix, arc_ds_);
    const double tail_length =
        std::hypot(target.x - global_connector_start_.x, target.y - global_connector_start_.y);
    TrajPoint terminal{};
    terminal.p = target;
    terminal.theta =
        std::atan2(target.y - global_connector_start_.y, target.x - global_connector_start_.x);
    terminal.s = candidate.back().s + tail_length;
    if (candidate.size() == 1)
      candidate.back().theta = terminal.theta;
    candidate.push_back(terminal);
    Path2 executed;
    for (const auto &point : candidate)
      executed.push_back(point.p);
    if (!required_path_clear(cur, executed, target, obs, required_cfg)) {
      ++global_smooth_fallbacks_;
      // 只对末端批准直线之前的部分圆角，connector 起点与直线保持原样(末端胶囊规则依赖它)。
      Path2 rounded_prefix;
      for (const auto &tp : rounded_fallback_trajectory(prefix, required_cfg, obs))
        rounded_prefix.push_back(tp.p);
      Path2 rounded = rounded_prefix;
      rounded.push_back(target);
      candidate = trajectory_from_polyline(rounded, arc_ds_, gains_.corner_stop_rad);
      executed.clear();
      for (const auto &point : candidate)
        executed.push_back(point.p);
      if (candidate.size() < 2 || !required_path_clear(cur, executed, target, obs, required_cfg)) {
        candidate = make_polyline_trajectory(gr.path);
        if (candidate.size() == 1)
          candidate.push_back(candidate.front());
        executed.clear();
        for (const auto &point : candidate)
          executed.push_back(point.p);
      }
    }
    if (!required_path_clear(cur, executed, target, obs, required_cfg)) {
      ++global_invalid_paths_;
      hold_required_path("required_trajectory_invalid");
      return {};
    }
    global_raw_ = executed;
    global_traj_ = candidate;
    global_tracker_->set_trajectory(global_traj_);
    global_failed_ = false;
    global_has_ = true;
    required_blocked_time_valid_ = false;
    if (updated_existing) {
      // A route invalidated this tick must brake before using its
      // replacement; do not advance the tracker twice in one period.
      command.v_fwd = command.v_lat = 0.0;
      command.at_goal = false;
      global_tracker_->constrain_forward_command(0.0);
    } else {
      command =
          global_tracker_->update(px_, py_, yaw_, v_fwd_est_, v_lat_est_, goal_tol_, yaw_rate_est_);
      global_tracker_ticked_ = true;
    }
  }
  traj_ = global_traj_;
  last_look_ = global_tracker_->last_lookahead();
  look_valid_ = true;
  global_at_goal_ = command.at_goal;
  const double remaining = global_tracker_->remaining_distance();
  // Arc progress can reach the last few centimetres while drift keeps the
  // aircraft outside the actual arrival circle. Capture the exact endpoint
  // with XY velocity once its short direct connection has been checked;
  // otherwise the near-zero forward cap can strand it beside the endpoint.
  const bool terminal_capture =
      command.lateral_correction && remaining <= goal_tol_ &&
      std::hypot(target.x - px_, target.y - py_) <= global_gains_.endpoint_slow_r &&
      required_path_clear(cur, Path2{cur, target}, target, obs, required_cfg);
  if (command.at_goal || required_goal_holding_ || terminal_capture) {
    if (!required_goal_holding_) {
      required_goal_hold_.begin(target);
      required_goal_yaw_ = yaw_;
      required_goal_holding_ = true;
    }
    const auto correction =
        required_goal_hold_.update(cur, yaw_, v_fwd_est_, v_lat_est_, kp_goal_, kd_goal_,
                                   std::min(0.20, v_goal_max_), gains_.max_accel, gains_.dt);
    command.v_fwd = correction.x;
    command.v_lat = correction.y;
    command.yaw_rate = std::clamp(gains_.kp_yaw * wrap_pi(required_goal_yaw_ - yaw_) -
                                      gains_.kd_yaw * yaw_rate_est_,
                                  -gains_.max_yaw_rate, gains_.max_yaw_rate);
    command.holding_position = true;
    global_at_goal_ = std::hypot(target.x - px_, target.y - py_) <= goal_tol_;
  }
  const double speed = std::hypot(command.v_fwd, command.v_lat);
  if (speed > 1e-9 && !command.holding_position) {
    // Apply the distance/damping envelope continuously. Switching it
    // that radius even on a straight, completely clear home route.
    if (command.lateral_correction) {
      // Remaining arc length limits forward travel, not the side
      // correction needed to resist drift. Scaling both to zero near
      // the endpoint can leave a moving aircraft unable to arrive.
      const double cap =
          std::min(travel_speed_limit,
                   std::max(0.0, kp_goal_ * remaining - kd_goal_ * std::max(0.0, v_fwd_est_)));
      command.v_fwd = std::min(command.v_fwd, cap);
    } else {
      const double toward_speed =
          std::max(0.0, (v_fwd_est_ * command.v_fwd + v_lat_est_ * command.v_lat) / speed);
      const double cap = std::min(travel_speed_limit,
                                  std::max(0.0, kp_goal_ * remaining - kd_goal_ * toward_speed));
      const double scale = std::min(1.0, cap / speed);
      command.v_fwd *= scale;
      command.v_lat *= scale;
    }
    global_tracker_->constrain_forward_command(command.v_fwd);
  }
  return guard_required_command(command, obs);
}

Vec2 ExplorationNode::guard_required_command(VelCmd command, const Obstacles &obs) {
  const Vec2 cur{px_, py_};
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const Vec2 desired{c * command.v_fwd - s * command.v_lat, s * command.v_fwd + c * command.v_lat};
  const Vec2 measured{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
  const double measured_speed = std::hypot(measured.x, measured.y);
  // A* trajectory.  The generic long stopping projection can intersect
  // a nearby obstacle even when the checked path is clear, causing an
  // endless replan/zero-speed loop.  Keep measured-velocity protection
  // below, but do not reject the path-following command itself here.
  // route; validate their own direction even when that route is clear.
  command_projection_blocked_ = !required_velocity_clear(desired, obs);
  if (command_projection_blocked_ && required_velocity_clear({0.0, 0.0}, obs)) {
    // A stopping projection beyond the permitted corridor may still
    // admit a slower command. Approach that limit continuously instead
    // Every candidate uses the same obstacle and field checks.
    double safe = 0.0, unsafe = 1.0;
    for (int i = 0; i < 16; ++i) {
      const double scale = .5 * (safe + unsafe);
      if (required_velocity_clear({scale * desired.x, scale * desired.y}, obs))
        safe = scale;
      else
        unsafe = scale;
    }
    if (safe * std::hypot(desired.x, desired.y) > 1e-5) {
      command.v_fwd *= safe;
      command.v_lat *= safe;
      global_tracker_->constrain_forward_command(command.v_fwd);
      command_projection_blocked_ = false;
    }
  }
  measured_projection_blocked_ =
      (!std::isfinite(measured_speed) || measured_speed > goal_stop_v_) &&
      !required_velocity_clear(measured, obs);
  const bool blocked = command_projection_blocked_ || measured_projection_blocked_;
  if (blocked) {
    if (!global_braking_blocked_) {
      ++global_velocity_brakes_;
      required_brake_speed_ = 0.0;
    }
    command.v_fwd = command.v_lat = 0.0;
    global_tracker_->constrain_forward_command(0.0);
    // A safe counter-velocity actively brakes drift. Zero velocity alone
    // left the aircraft drifting and prevented this guard from releasing.
    if (std::isfinite(measured_speed) && measured_speed > 1e-9) {
      const double cap =
          std::min({0.20, v_goal_max_, kd_goal_ * measured_speed,
                    required_brake_speed_ +
                        std::max(global_gains_.max_accel, xy_brake_accel_) * global_gains_.dt});
      const Vec2 braking{-measured.x * cap / measured_speed, -measured.y * cap / measured_speed};
      if (required_velocity_clear(braking, obs)) {
        command.v_fwd = c * braking.x + s * braking.y;
        command.v_lat = -s * braking.x + c * braking.y;
        required_brake_speed_ = cap;
      } else {
        required_brake_speed_ = 0.0;
      }
    }
    if (!required_motion_clear(cur, obs))
      command.yaw_rate = 0.0;
  }
  global_braking_blocked_ = blocked;
  global_block_reason_ = blocked ? "required_stopping_projection" : "none";
  if (blocked) {
    const double current_time = steady_seconds();
    if (!required_blocked_time_valid_) {
      required_blocked_since_ = current_time;
      required_blocked_time_valid_ = true;
    }
    if (required_blocked_replan_s_ > 0.0 &&
        current_time - required_blocked_since_ >= required_blocked_replan_s_) {
      ++required_blocked_replans_;
      global_has_ = false;
      required_blocked_time_valid_ = false;
      global_block_reason_ = "required_blocked_replan";
    }
  } else {
    required_blocked_time_valid_ = false;
  }
  last_yaw_rate_ = command.yaw_rate;
  global_tracker_->constrain_lateral_command(command.v_lat);
  return {command.v_fwd, command.v_lat};
}

} // namespace exploration
