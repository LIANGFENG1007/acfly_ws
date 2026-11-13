#include "exploration_node.hpp"

namespace exploration {

void ExplorationNode::remember_yaw_command(double rate) {
  const double dt = std::max(1e-3, gains_.dt);
  const size_t count =
      static_cast<size_t>(std::max(0.0, std::lround(gains_.smith_yaw_time / dt) * 1.0));
  yaw_command_history_.push_back(rate);
  while (yaw_command_history_.size() > count)
    yaw_command_history_.pop_front();
}

double ExplorationNode::pending_yaw_rotation() const {
  if (gains_.smith_yaw_time <= 0)
    return std::max(0.0, gains_.prediction_time) * yaw_rate_est_;
  const double dt = std::max(1e-3, gains_.dt);
  double rotation = 0;
  for (double rate : yaw_command_history_)
    rotation += rate * dt;
  // 新定位恢复时还没有完整指令历史，仅对缺少的部分用实测角速度外推。
  return rotation +
         std::max(0.0, gains_.smith_yaw_time - yaw_command_history_.size() * dt) * yaw_rate_est_;
}

double ExplorationNode::approach_yaw_rate(double heading) const {
  const double error = wrap_pi(heading - yaw_ - pending_yaw_rotation());
  const double accel = std::max(1e-3, gains_.max_yaw_accel);
  const double braking = std::sqrt(2 * accel * std::abs(error));
  const double desired = std::clamp(std::clamp(gains_.kp_yaw * error, -braking, braking) -
                                        gains_.kd_yaw * yaw_rate_est_,
                                    -gains_.max_yaw_rate, gains_.max_yaw_rate);
  return last_yaw_rate_ +
         std::clamp(desired - last_yaw_rate_, -accel * gains_.dt, accel * gains_.dt);
}

bool ExplorationNode::escape_velocity_clear(Vec2 velocity, const Obstacles &obs) const {
  const double speed = std::hypot(velocity.x, velocity.y);
  if (!std::isfinite(speed) || speed <= 1e-9)
    return false;
  const double horizon = std::min(.35, std::max(0.0, global_gains_.prediction_time)) +
                         speed / (2 * std::max(1e-3, global_gains_.max_accel));
  const Vec2 cur{px_, py_}, stop{px_ + velocity.x * horizon, py_ + velocity.y * horizon};
  return obstacle_escape_clear(cur, stop, obs, exec_config()) &&
         field_motion_clear(cur, stop, exec_config());
}

void ExplorationNode::reset_home_approach() {
  home_approach_active_ = false;
  home_curve_active_ = false;
  home_approach_yaw_ = 0.0;
  home_entry_plan_ = {};
  home_entry_goal_ = goal_;
  home_leg_target_.reset();
  home_entry_retry_ = -1;
}

GlobalConfig ExplorationNode::home_transit_config() const {
  auto cfg = required_config();
  // 终点可靠任务边界；实体障碍仍使用完整机体和原规划余量。
  cfg.wall_margin = required_field_margin(cfg);
  cfg.required_goal_field_margin = cfg.wall_margin + goal_tol_;
  cfg.head_cone_half = cfg.head_cone_radius = 0;
  return cfg;
}

void ExplorationNode::straighten_home_entry(Vec2 start, const Obstacles &obs,
                                            const GlobalConfig &transit) {
  auto &plan = home_entry_plan_;
  for (size_t i = 0; i + 1 < plan.path.size(); ++i) {
    const Vec2 anchor = i == 0 ? start : plan.path[i];
    const double distance = std::hypot(anchor.x - goal_.x, anchor.y - goal_.y);
    if (distance <= home_straight_approach_m_)
      continue;
    const double u = home_straight_approach_m_ / distance;
    const Vec2 entry{goal_.x + u * (anchor.x - goal_.x), goal_.y + u * (anchor.y - goal_.y)};
    if (!home_segment_clear(entry, entry, obs, transit) ||
        !field_motion_clear(anchor, entry, transit) ||
        !(i == 0 ? obstacle_escape_clear(anchor, entry, obs, transit)
                 : home_segment_clear(anchor, entry, obs, transit)) ||
        !home_segment_clear(entry, goal_, obs, required_config(), goal_tol_, exec_config().inflate))
      continue;
    plan.path.resize(i + 1);
    plan.path.front() = start;
    plan.path.push_back(entry);
    plan.entry = entry;
    plan.arrival_heading = std::atan2(goal_.y - anchor.y, goal_.x - anchor.x);
    plan.reason = "collinear_home_entry";
    return;
  }
}

bool ExplorationNode::approach_curve_clear(const Path2 &route, const Obstacles &obs,
                                           const GlobalConfig &cfg) const {
  if (route.size() < 2)
    return false;
  for (size_t i = 1; i < route.size(); ++i)
    if (!obstacle_escape_clear(route[i - 1], route[i], obs, cfg) ||
        !field_motion_clear(route[i - 1], route[i], cfg))
      return false;
  return true;
}

void ExplorationNode::prepare_home_curve(const Obstacles &obs) {
  home_curve_active_ = false;
  const auto cfg = home_transit_config();
  CornerRounding options;
  options.max_distance = std::max(turn_round_min_m_, turn_blend_m_);
  options.min_distance = std::max(.001, turn_round_min_m_);
  options.min_angle = .001;
  options.max_curvature = turn_round_max_curvature_;
  options.sample_distance = arc_ds_;
  const auto rounded = round_path_corners(home_entry_plan_.path, options, [&](const Path2 &piece) {
    return approach_curve_clear(piece, obs, cfg);
  });
  if (!approach_curve_clear(rounded, obs, cfg))
    return;
  // 没有足够空间的硬折点不能假装成曲线高速切过去，保留原有逐段安全跟踪。
  for (size_t i = 1; i + 1 < rounded.size(); ++i) {
    const auto a = rounded[i - 1], b = rounded[i], c = rounded[i + 1];
    if (std::abs(wrap_pi(std::atan2(c.y - b.y, c.x - b.x) - std::atan2(b.y - a.y, b.x - a.x))) >=
        gains_.corner_stop_rad)
      return;
  }
  global_traj_ = rounded.size() == 2
                     ? make_polyline_trajectory(rounded)
                     : trajectory_from_polyline(rounded, arc_ds_, gains_.corner_stop_rad);
  if (global_traj_.size() < 2 || global_traj_.back().s <= 1e-6)
    return;
  global_raw_.clear();
  for (const auto &point : global_traj_)
    global_raw_.push_back(point.p);
  if (!approach_curve_clear(global_raw_, obs, cfg))
    return;
  global_tracker_->set_trajectory(global_traj_);
  global_tracker_->advance_reference(px_, py_);
  home_curve_active_ = global_has_ = true;
  global_failed_ = false;
}

Vec2 ExplorationNode::approach_curve_velocity(const Obstacles &obs) {
  const double progress = global_tracker_->progress_distance();
  const auto reference = global_tracker_->reference_at(progress);
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const Vec2 measured{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
  const Vec2 tangent{std::cos(reference.theta), std::sin(reference.theta)};
  const Vec2 normal{-tangent.y, tangent.x};
  const double along = std::max(0.0, measured.x * tangent.x + measured.y * tangent.y);
  const double decel = std::max(.01, gains_.max_decel);
  const double fraction =
      std::clamp(global_tracker_->remaining_distance() / home_slow_radius_, 0.0, 1.0);
  // 到入口连续接上下降速度，斜率在入口归零；不能以巡航速度冲到1米后
  // 再突然套下降限速。保留原减速范围，改变的是连续速度剖面而不是入口半径。
  const double entry_speed = std::min(v_goal_max_, home_approach_speed_);
  double speed = entry_speed + (v_goal_max_ - entry_speed) * fraction * fraction;
  // 曲率和侧向加速度决定弯道限速，提前减速，不在路点处降到零。
  for (const auto &point : global_traj_) {
    if (point.s < progress)
      continue;
    const double curvature = std::abs(point.kappa);
    if (curvature < 1e-6)
      continue;
    const double allowed =
        std::min(std::sqrt(gains_.max_lateral_accel / curvature), gains_.max_yaw_rate / curvature);
    speed = std::min(speed, std::sqrt(allowed * allowed + 2 * decel * (point.s - progress)));
  }
  const auto ahead = global_tracker_->reference_at(progress + along * gains_.prediction_time);
  const double error = (px_ - reference.p.x) * normal.x + (py_ - reference.p.y) * normal.y;
  const double lateral = measured.x * normal.x + measured.y * normal.y;
  const double correction = -kp_goal_ * error - kd_goal_ * lateral;
  Vec2 desired{speed * std::cos(ahead.theta) + normal.x * correction,
               speed * std::sin(ahead.theta) + normal.y * correction};
  // samples are never independent position targets. Keep the last tangent
  // as yaw reference while XY finishes the final centimetres.
  const auto limit = [&](Vec2 &v) {
    const double length = std::hypot(v.x, v.y);
    if (length > v_goal_max_) {
      v.x *= v_goal_max_ / length;
      v.y *= v_goal_max_ / length;
    }
  };
  limit(desired);
  desired = slew_velocity(last_command_world_, desired, measured, gains_.max_accel, xy_brake_accel_,
                          gains_.dt);
  limit(desired);
  const auto &plan = home_entry_plan_;
  global_target_ = plan.entry;
  global_connector_start_ = plan.path[plan.path.size() - 2];
  traj_ = global_traj_;
  last_look_ = ahead.p;
  look_valid_ = true;
  global_has_ = true;
  global_failed_ = global_goal_blocked_ = global_at_goal_ = false;
  VelCmd command;
  command.v_fwd = c * desired.x + s * desired.y;
  command.v_lat = -s * desired.x + c * desired.y;
  command.yaw_rate = approach_yaw_rate(ahead.theta);
  if (!obstacle_segment_clear({px_, py_}, {px_, py_}, obs, exec_config()) &&
      escape_velocity_clear(desired, obs)) {
    global_braking_blocked_ = command_projection_blocked_ = measured_projection_blocked_ = false;
    required_blocked_time_valid_ = false;
    global_block_reason_ = "home_monotonic_escape";
    last_yaw_rate_ = 0;
    return {command.v_fwd, command.v_lat};
  }
  return guard_required_command(command, obs);
}

bool ExplorationNode::select_home_entry(const Obstacles &obs) {
  const Vec2 cur{px_, py_};
  const auto transit = home_transit_config();
  auto execution = transit;
  execution.inflate = exec_config().inflate;
  const auto line_clear = [&](Vec2 a, Vec2 b, const GlobalConfig &cfg) {
    return obstacle_escape_clear(a, b, obs, cfg) && field_motion_clear(a, b, cfg);
  };
  const auto descent_clear = [&](Vec2 entry, double tolerance) {
    return home_segment_clear(entry, goal_, obs, required_config(), tolerance,
                              exec_config().inflate);
  };
  if (home_curve_active_) {
    const auto remaining = global_tracker_->remaining_path(px_, py_);
    const bool reached =
        std::hypot(cur.x - home_entry_plan_.entry.x, cur.y - home_entry_plan_.entry.y) <=
            goal_tol_ &&
        std::hypot(cur.x - goal_.x, cur.y - goal_.y) <= home_straight_approach_m_ + 1e-9 &&
        descent_clear(cur, 0);
    if (global_has_ && (reached || (descent_clear(home_entry_plan_.entry, goal_tol_) &&
                                    approach_curve_clear(remaining, obs, execution)))) {
      global_tracker_->advance_reference(px_, py_);
      return true;
    }
    home_curve_active_ = false;
    home_entry_plan_ = {};
    home_leg_target_.reset();
    home_entry_retry_ = -1;
  }
  const bool had_entry = home_entry_plan_.ok;
  const bool at_entry = had_entry && std::hypot(cur.x - home_entry_plan_.entry.x,
                                                cur.y - home_entry_plan_.entry.y) <= goal_tol_;
  // 一旦进入已选入口容差，按实际位置核准；不因未用到的误差包络补一段横移。
  const bool entry_clear =
      had_entry &&
      (descent_clear(home_entry_plan_.entry, goal_tol_) ||
       (at_entry && descent_clear(home_entry_plan_.entry, 0) && descent_clear(cur, 0)));
  bool missed_leg = false;
  if (home_leg_target_ && had_entry) {
    const double distance = std::hypot(cur.x - home_leg_target_->x, cur.y - home_leg_target_->y);
    const bool intermediate = std::hypot(home_leg_target_->x - home_entry_plan_.entry.x,
                                         home_leg_target_->y - home_entry_plan_.entry.y) > 1e-6;
    if (intermediate && distance <= goal_tol_) {
      auto &route = home_entry_plan_.path;
      auto reached = std::find_if(route.begin() + 1, route.end(), [&](Vec2 p) {
        return std::hypot(p.x - home_leg_target_->x, p.y - home_leg_target_->y) < 1e-6;
      });
      if (reached != route.end()) {
        route.erase(route.begin(), reached + 1);
        route.insert(route.begin(), cur);
      }
      home_leg_target_.reset();
    } else if (distance > goal_tol_) {
      missed_leg = (cur.x - home_leg_target_->x) * std::cos(home_leg_heading_) +
                       (cur.y - home_leg_target_->y) * std::sin(home_leg_heading_) >
                   0;
    }
  }
  Vec2 from = cur;
  bool route_clear = entry_clear && !missed_leg && home_entry_plan_.path.size() > 1;
  for (size_t i = 1; route_clear && i < home_entry_plan_.path.size(); ++i) {
    route_clear = line_clear(from, home_entry_plan_.path[i], execution);
    from = home_entry_plan_.path[i];
  }
  if (route_clear)
    return true;

  // 本航段最初入口的到点容差限制，不能把大幅换路伪装成小修正。
  if (had_entry && !missed_leg && home_leg_target_ &&
      std::hypot(home_leg_target_->x - home_entry_plan_.entry.x,
                 home_leg_target_->y - home_entry_plan_.entry.y) < 1e-6) {
    auto adjusted = plan_home_approach(cur, goal_, obs, execution, required_config(),
                                       home_straight_approach_m_, goal_tol_, exec_config().inflate);
    if (adjusted.ok && adjusted.path.size() == 2 &&
        std::hypot(adjusted.entry.x - home_leg_anchor_.x, adjusted.entry.y - home_leg_anchor_.y) <=
            goal_tol_ &&
        (adjusted.entry.x - cur.x) * std::cos(home_leg_heading_) +
                (adjusted.entry.y - cur.y) * std::sin(home_leg_heading_) >
            0) {
      adjusted.arrival_heading = home_leg_heading_;
      adjusted.reason = "continue_checked_home_final";
      home_entry_plan_ = std::move(adjusted);
      home_leg_target_ = home_entry_plan_.entry;
      return true;
    }
  }
  // 入口因新点云失效时，延续仍安全的中间航段，只重选其后的入口。
  // 旧入口已经失效时不先飞到旧入口，更不回追已越过的路点。
  if (had_entry && !missed_leg && home_leg_target_ &&
      std::hypot(home_leg_target_->x - home_entry_plan_.entry.x,
                 home_leg_target_->y - home_entry_plan_.entry.y) > 1e-6 &&
      line_clear(cur, *home_leg_target_, execution)) {
    auto continued =
        plan_home_approach(*home_leg_target_, goal_, obs, transit, required_config(),
                           home_straight_approach_m_, goal_tol_, exec_config().inflate);
    if (continued.ok) {
      continued.path.insert(continued.path.begin(), cur);
      continued.reason = "continue_checked_home_leg";
      home_entry_plan_ = std::move(continued);
      return true;
    }
  }
  const double t = steady_seconds();
  home_leg_target_.reset();
  home_entry_plan_ = {};
  if (!had_entry && t < home_entry_retry_)
    return false;
  home_entry_retry_ = t + .5;
  // 已在圈内且实际整条接近线安全时不倒退凑半径，也无需为已知终点取样。
  if (std::hypot(cur.x - goal_.x, cur.y - goal_.y) <= home_straight_approach_m_ &&
      descent_clear(cur, 0)) {
    home_entry_plan_ = {true, cur, {cur, cur}, "home_current_point_inside_circle", yaw_};
  } else {
    ++astar_searches_;
    ++global_path_searches_;
    home_entry_plan_ =
        plan_home_approach(cur, goal_, obs, transit, required_config(), home_straight_approach_m_,
                           goal_tol_, exec_config().inflate);
  }
  if (!home_entry_plan_.ok)
    return false;
  straighten_home_entry(cur, obs, transit);
  prepare_home_curve(obs);
  RCLCPP_INFO(get_logger(),
              "[终点接近] 安全入口(%.3f,%.3f)，距终点%.2fm，来源=%s；到入口后直线接近",
              home_entry_plan_.entry.x, home_entry_plan_.entry.y,
              std::hypot(home_entry_plan_.entry.x - goal_.x, home_entry_plan_.entry.y - goal_.y),
              home_entry_plan_.reason);
  return true;
}

Vec2 ExplorationNode::home_target_velocity(const Obstacles &obs) {
  if (home_straight_approach_m_ <= 0)
    return pd_to_point_avoid(goal_, obs);
  if (std::hypot(goal_.x - home_entry_goal_.x, goal_.y - home_entry_goal_.y) > 1e-6)
    reset_home_approach();
  const Vec2 cur{px_, py_};
  Vec2 carrot = goal_;
  if (home_approach_active_ &&
      !home_segment_clear(cur, goal_, obs, required_config(), 0, exec_config().inflate)) {
    reset_home_approach();
    hold_required_path("home_final_segment_blocked");
    return {};
  }
  if (!home_approach_active_) {
    if (!select_home_entry(obs)) {
      auto endpoint = exec_config();
      global_goal_blocked_ = !home_segment_clear(goal_, goal_, obs, endpoint);
      global_target_ = goal_;
      hold_required_path("home_no_safe_entry");
      return {};
    }
    const Vec2 entry = home_entry_plan_.entry;
    const bool entry_reached = home_curve_active_
                                   ? global_tracker_->remaining_distance() <= goal_tol_
                                   : std::hypot(cur.x - entry.x, cur.y - entry.y) <= goal_tol_;
    if (entry_reached &&
        std::hypot(cur.x - goal_.x, cur.y - goal_.y) <= home_straight_approach_m_ + 1e-9 &&
        home_segment_clear(cur, goal_, obs, required_config(), 0, exec_config().inflate)) {
      home_approach_active_ = true;
      home_curve_active_ = false;
      home_approach_yaw_ = yaw_;
      home_leg_target_.reset();
      global_tracker_->set_trajectory({});
      required_goal_holding_ = false;
      required_blocked_time_valid_ = false;
      RCLCPP_INFO(get_logger(), "[终点接近] 已到安全入口，核准实际接近线，保持当前yaw直线前往终点");
    } else if (home_curve_active_) {
      return approach_curve_velocity(obs);
    } else if (entry_reached &&
               home_segment_clear(cur, goal_, obs, required_config(), 0, exec_config().inflate)) {
      // 无法圆角时的低速入口衔接也不能停在1米外无限逼近入口。
      // 整条直线已经核准，继续向终点移动；真正进1米圈才进入最后直线。
      carrot = goal_;
    } else {
      auto &route = home_entry_plan_.path;
      const auto transit = home_transit_config();
      auto execution = transit;
      execution.inflate = exec_config().inflate;
      // 整条当前航段执行完才换下一段，避免逐帧可见性造成左右切换。
      if (!home_leg_target_) {
        for (size_t i = route.size(); i-- > 1;) {
          const auto &cfg = i == 1 ? execution : transit;
          if (!obstacle_escape_clear(cur, route[i], obs, cfg) ||
              !field_motion_clear(cur, route[i], cfg))
            continue;
          home_leg_target_ = route[i];
          home_leg_anchor_ = route[i];
          home_leg_heading_ = std::atan2(route[i].y - cur.y, route[i].x - cur.x);
          route.erase(route.begin(), route.begin() + i);
          route.insert(route.begin(), cur);
          break;
        }
      }
      if (!home_leg_target_) {
        home_entry_plan_ = {};
        home_entry_retry_ = -1;
        global_target_ = goal_;
        hold_required_path("home_entry_route_blocked");
        return {};
      }
      carrot = *home_leg_target_;
    }
  }
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const Vec2 measured{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
  Vec2 desired{kp_goal_ * (carrot.x - px_) - kd_goal_ * measured.x,
               kp_goal_ * (carrot.y - py_) - kd_goal_ * measured.y};
  const double fraction =
      std::clamp(std::hypot(carrot.x - px_, carrot.y - py_) / home_slow_radius_, 0.0, 1.0);
  const double cap =
      home_approach_active_
          ? std::min(v_goal_max_, home_approach_speed_)
          : std::min(v_goal_max_,
                     home_slow_speed_ + (v_goal_max_ - home_slow_speed_) * fraction * fraction);
  const auto limit = [cap](Vec2 &v) {
    const double speed = std::hypot(v.x, v.y);
    if (speed > cap) {
      v.x *= cap / speed;
      v.y *= cap / speed;
    }
  };
  limit(desired);
  desired = slew_velocity(last_command_world_, desired, measured, gains_.max_accel, xy_brake_accel_,
                          gains_.dt);
  limit(desired);
  global_target_ = home_approach_active_ ? goal_ : home_entry_plan_.entry;
  global_raw_ = home_approach_active_ ? Path2{cur, goal_} : home_entry_plan_.path;
  global_connector_start_ = global_raw_[global_raw_.size() - 2];
  global_traj_ = make_polyline_trajectory(global_raw_);
  traj_ = global_traj_;
  last_look_ = carrot;
  look_valid_ = true;
  global_has_ = true;
  global_failed_ = global_goal_blocked_ = false;
  global_at_goal_ = home_approach_active_ && std::hypot(goal_.x - px_, goal_.y - py_) <= goal_tol_;
  VelCmd command;
  command.v_fwd = c * desired.x + s * desired.y;
  command.v_lat = -s * desired.x + c * desired.y;
  command.yaw_rate =
      approach_yaw_rate(home_approach_active_ ? home_approach_yaw_ : home_leg_heading_);
  if (!home_approach_active_) {
    if (!obstacle_segment_clear(cur, cur, obs, exec_config()) &&
        escape_velocity_clear(desired, obs)) {
      global_braking_blocked_ = command_projection_blocked_ = measured_projection_blocked_ = false;
      required_blocked_time_valid_ = false;
      global_block_reason_ = "home_monotonic_escape";
      last_yaw_rate_ = 0;
      return {command.v_fwd, command.v_lat};
    }
    return guard_required_command(command, obs);
  }
  // XY/yaw速度反馈、限幅及上层定位/任务有效期检查继续生效。
  global_braking_blocked_ = command_projection_blocked_ = measured_projection_blocked_ = false;
  required_blocked_time_valid_ = false;
  global_block_reason_.clear();
  last_yaw_rate_ = command.yaw_rate;
  return {command.v_fwd, command.v_lat};
}

} // namespace exploration
