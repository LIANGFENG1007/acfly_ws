#include "exploration_node.hpp"

namespace exploration {

bool ExplorationNode::step_route_rejoin(const Obstacles &obs,
                                        geometry_msgs::msg::TwistStamped &cmd) {
  const bool permitted = has_goal_ && !finished_ && !(homing_ && home_approach_active_);
  const int owner = homing_ ? 1 : 0;
  if (!permitted || (route_rejoin_target_ && owner != route_rejoin_owner_)) {
    route_rejoin_target_.reset();
    route_rejoin_path_.clear();
  }
  if (!permitted)
    return false;
  const Vec2 cur{px_, py_};
  auto cfg = owner == 0 ? explore_exec_config() : exec_config();
  if (homing_)
    cfg.wall_margin = home_goal_wall_margin_;
  const bool inside = !outside_obstacle_rings(cur, obs, cfg);
  if (!route_rejoin_target_ && !inside)
    return false;
  const auto valid = [&](Vec2 target) {
    return outside_obstacle_rings(target, obs, cfg) &&
           obstacle_escape_clear(cur, target, obs, cfg) && field_motion_clear(cur, target, cfg);
  };
  // Arrival means returning to the LINE. Inertia may carry the aircraft
  // along it past the initially selected foot; chasing that old point
  // would pull back, overshoot and circle despite having regained the route.
  if (route_rejoin_target_ && route_distance(cur, route_rejoin_path_) <= .05 && !inside) {
    route_rejoin_target_.reset();
    route_rejoin_path_.clear();
    recovery_holding_ = false;
    recovery_status_ = "route_rejoined";
    // First resume the retained reference, then let periodic planning
    // reassess it. Recovery time alone is not a reason to replace it.
    if (owner == 0 && tracker_->has_trajectory()) {
      tracker_->advance_reference(px_, py_);
      last_plan_time_ = now();
      plan_pending_ = false;
    }
    return false;
  }
  if (!route_rejoin_target_ || !valid(*route_rejoin_target_)) {
    if (route_rejoin_path_.empty()) {
      if (owner == 0 && tracker_->has_trajectory())
        route_rejoin_path_ = tracker_->remaining_reference_path();
      else if ((home_curve_active_) && global_tracker_->has_trajectory())
        route_rejoin_path_ = global_tracker_->remaining_reference_path();
      else if (homing_ && home_entry_plan_.ok)
        route_rejoin_path_ = home_entry_plan_.path;
      else
        route_rejoin_path_ = global_raw_;
    }
    route_rejoin_target_ = nearest_safe_rejoin(cur, route_rejoin_path_, obs, cfg);
    if (!route_rejoin_target_) {
      route_rejoin_path_.clear();
      return false;
    }
    route_rejoin_owner_ = owner;
    route_rejoin_bias_ = 0;
    RCLCPP_WARN(get_logger(),
                "[避障回线] 偏入障碍执行圈，先水平回到安全路线(%.3f,%.3f)，再继续任务",
                route_rejoin_target_->x, route_rejoin_target_->y);
  }
  Vec2 target = *route_rejoin_target_;
  // Keep the selected local route, but permit tangential inertia along
  // it. Correct toward its current perpendicular foot instead of adding
  // a longitudinal reversal just to revisit the old foot. The half-metre
  // neighbourhood prevents a nearby later leg from taking over recovery;
  // the whole new connector still passes the same multi-obstacle check.
  const auto foot = nearest_safe_rejoin(cur, route_rejoin_path_, obs, cfg);
  if (foot && std::hypot(foot->x - target.x, foot->y - target.y) <= .5 &&
      std::hypot(foot->x - cur.x, foot->y - cur.y) < std::hypot(target.x - cur.x, target.y - cur.y))
    target = *foot;
  const double distance = std::hypot(target.x - cur.x, target.y - cur.y);
  const Vec2 direction{(target.x - cur.x) / std::max(1e-6, distance),
                       (target.y - cur.y) / std::max(1e-6, distance)};
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const Vec2 measured{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
  const double cap =
      std::max(0.0, std::min(route_rejoin_v_max_, owner == 0 ? gains_.v_max : v_goal_max_));
  const double toward = measured.x * direction.x + measured.y * direction.y;
  // A short return uses an effective translation command until arrival,
  // not an asymptotically vanishing PD velocity inside the obstacle ring.
  const double minimum = inside ? std::clamp(route_rejoin_min_mps_, 0.0, cap) : 0.0;
  const double predicted =
      std::max(0.0, distance - std::max(0.0, toward) * gains_.stop_anchor_latency);
  // Outside the ring allow braking, but do not strand a dead-zone-limited
  // aircraft just short of the line. Bounded integral compensation grows
  // only while lateral response is small; it is removed during motion and
  // reset at every new recovery, never inherited by normal flight.
  if (!inside && toward < .05)
    route_rejoin_bias_ = std::min(std::min(route_rejoin_min_mps_, cap),
                                  route_rejoin_bias_ + .5 * kp_goal_ * distance * gains_.dt);
  else
    route_rejoin_bias_ = std::max(0.0, route_rejoin_bias_ - .10 * gains_.dt);
  const double speed =
      std::clamp(kp_goal_ * predicted - kd_goal_ * toward + route_rejoin_bias_, minimum, cap);
  const double previous =
      std::max(0.0, last_command_world_.x * direction.x + last_command_world_.y * direction.y);
  const double magnitude = std::min(speed, previous + xy_brake_accel_ * gains_.dt);
  Vec2 velocity{magnitude * direction.x, magnitude * direction.y};
  const double horizon = std::min(.35, std::max(0.0, gains_.prediction_time)) +
                         magnitude / (2 * std::max(.01, xy_brake_accel_));
  // Include stopping distance; a return vector must not run beyond its
  // safe connector into another obstacle or configured wall. The
  // connector itself was validated, so the projection is capped at the
  // target: a faster return must not be abandoned just because its
  // stopping envelope reaches past the line onto the retained route.
  const double reach = std::min(magnitude * horizon, distance);
  const Vec2 projected{cur.x + direction.x * reach, cur.y + direction.y * reach};
  if (!obstacle_escape_clear(cur, projected, obs, cfg) ||
      !field_motion_clear(cur, projected, cfg)) {
    route_rejoin_target_.reset();
    route_rejoin_path_.clear();
    return false;
  }
  cmd.twist.linear.x = c * velocity.x + s * velocity.y;
  cmd.twist.linear.y = -s * velocity.x + c * velocity.y;
  cmd.twist.angular.z = 0;
  last_look_ = target;
  look_valid_ = true;
  recovery_requested_ = true;
  recovery_status_ = "returning_to_safe_route";
  global_block_reason_ = "returning_to_safe_route";
  tracker_->record_external_command(yaw_, cmd.twist.linear.x, cmd.twist.linear.y, 0);
  global_tracker_->record_external_command(yaw_, cmd.twist.linear.x, cmd.twist.linear.y, 0);
  return true;
}

Vec2 ExplorationNode::recovery_velocity(const Obstacles &obs, bool required_geometry) {
  recovery_requested_ = true;
  if (!recovery_holding_) {
    recovery_anchor_ = {px_, py_};
    recovery_holding_ = true;
  }
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  const Vec2 measured{c * v_fwd_est_ - s * v_lat_est_, s * v_fwd_est_ + c * v_lat_est_};
  const double cap = std::min(0.20, v_goal_max_);
  const auto body = [c, s](Vec2 v) { return Vec2{c * v.x + s * v.y, -s * v.x + c * v.y}; };
  const auto safe = [&](Vec2 v) {
    return required_geometry ? required_velocity_clear(v, obs)
                             : turn_correction_clear(body(v), obs);
  };
  const auto limit = [cap](Vec2 v) {
    const double speed = std::hypot(v.x, v.y);
    if (speed > cap) {
      v.x *= cap / speed;
      v.y *= cap / speed;
    }
    return v;
  };
  Vec2 desired{kp_goal_ * (recovery_anchor_.x - px_) - kd_goal_ * measured.x,
               kp_goal_ * (recovery_anchor_.y - py_) - kd_goal_ * measured.y};
  desired = limit(slew_velocity(last_command_world_, limit(desired), measured, gains_.max_accel,
                                xy_brake_accel_, gains_.dt));
  if (safe(desired)) {
    recovery_status_ = "holding_after_path_failure";
    return body(desired);
  }
  // Returning to the anchor can be blocked by newly observed geometry.
  // Keep a separately checked counter-velocity, seeded from the last
  // actual output so a replan/defer cannot restart its ramp at zero.
  const double speed = std::hypot(measured.x, measured.y);
  if (speed > 1e-6) {
    const Vec2 direction{-measured.x / speed, -measured.y / speed};
    const double previous =
        std::max(0.0, direction.x * last_command_world_.x + direction.y * last_command_world_.y);
    const double magnitude =
        std::min({cap, kd_goal_ * speed,
                  previous + std::max(gains_.max_accel, xy_brake_accel_) * gains_.dt});
    const Vec2 brake{direction.x * magnitude, direction.y * magnitude};
    if (safe(brake)) {
      recovery_status_ = "braking_after_path_failure";
      return body(brake);
    }
  }
  // No command may bypass physical obstacle overlap or field limits.
  recovery_status_ = "no_safe_recovery_velocity";
  return {};
}

VelCmd ExplorationNode::guard_explore_velocity(VelCmd command, const Obstacles &obs) {
  if (!command.lateral_correction)
    return command;
  // Lateral compensation changes the velocity from the original reference
  // tangent. Check the full vector AND the aircraft's actual drift.
  // 近零速时实测速度方向只是噪声，不能作为"正在向障碍漂"的证据：2026-09-23 仿真里
  // 飞机被膨胀后的拟合圆吞进禁入圈边缘，此处随机拦截 + 恢复速度同样过不了同一几何
  // 校验，原地抖了 11 s。显著实测速度仍按原口径检查。
  const double measured_speed = std::hypot(v_fwd_est_, v_lat_est_);
  const bool measured_safe =
      measured_speed <= goal_stop_v_ || turn_correction_clear({v_fwd_est_, v_lat_est_}, obs);
  bool desired_safe = turn_correction_clear({command.v_fwd, command.v_lat}, obs);
  if (measured_safe && !desired_safe) {
    // A partial correction may fit when the full correction does not.
    for (double scale : {0.75, 0.50, 0.25, 0.0}) {
      if (turn_correction_clear({command.v_fwd, command.v_lat * scale}, obs)) {
        command.v_lat *= scale;
        desired_safe = true;
        ++explore_guard_lateral_trims_;
        break;
      }
    }
  }
  if (measured_safe && !desired_safe && tracker_->has_trajectory() &&
      std::abs(tracker_->debug().geo_error) <=
          gains_.heading_gate_rad && // 实测机头，不用 Smith 预测航向
      path_clear({px_, py_}, tracker_->remaining_path(px_, py_), obs, explore_exec_config())) {
    // 机头已对准一条刚复核过的安全路线：沿线前进本身是安全的，只有横向补偿不安全。
    // 只去掉补偿，不把整机压成零速(那会在禁入圈边缘形成停滞)。
    command.v_lat = 0.0;
    desired_safe = true;
    ++explore_guard_lateral_trims_;
  }
  if (!measured_safe || !desired_safe) {
    ++explore_guard_rejections_;
    const Vec2 brake = recovery_velocity(obs, false);
    command.v_fwd = brake.x;
    command.v_lat = brake.y;
    command.holding_position = true;
    if (!obstacle_segment_clear({px_, py_}, {px_, py_}, obs, ggcfg_))
      command.yaw_rate = 0.0;
  }
  tracker_->constrain_forward_command(command.v_fwd);
  tracker_->constrain_lateral_command(command.v_lat);
  return command;
}

double ExplorationNode::nearest_obstacle_dist(const Obstacles &obs) const {
  double best = 1e9;
  for (const auto &o : obs) {
    const double d = std::hypot(o.cx - px_, o.cy - py_) - o.r;
    best = std::min(best, d);
  }
  return best;
}

Vec2 ExplorationNode::do_retreat(const Obstacles &obs) {
  // 取最近障碍，后退方向 = 障碍→飞机(把飞机往外推)
  double bd = 1e9;
  Vec2 oc{px_, py_};
  for (const auto &o : obs) {
    const double d = std::hypot(o.cx - px_, o.cy - py_) - o.r;
    if (d < bd) {
      bd = d;
      oc = {o.cx, o.cy};
    }
  }
  double ux = px_ - oc.x, uy = py_ - oc.y;
  const double n = std::hypot(ux, uy);
  if (n < 1e-6) {
    ux = std::cos(yaw_ + M_PI);
    uy = std::sin(yaw_ + M_PI);
  } // 退化:贴圆心,朝身后退
  else {
    ux /= n;
    uy /= n;
  }

  // ★避墙★：纯"远离障碍"方向常常正好朝墙(柱子在墙边时)，会把飞机往墙里推→更贴墙→更无解。
  //   若沿此方向退一步会进墙禁入区/出界，就把【朝墙的分量】翻成朝场内，使后退转为沿墙切向退。
  const double wm = ggcfg_.wall_margin;
  {
    // 朝墙分量【去掉】改沿墙切向退(2026-09-23)。旧做法把它翻成反向：柱子在墙边时"远离柱子"正对墙，
    //   翻向后恰好指回柱子，随即被碰撞检查拒绝，后退失败(r57 卡在柱与东墙之间 30 s)。
    double tx = px_ + ux * retreat_step_, ty = py_ + uy * retreat_step_;
    const double away_x = px_ - oc.x, away_y = py_ - oc.y;
    bool zeroed = false;
    if ((tx > ggcfg_.max_x - wm && ux > 0) || (tx < ggcfg_.min_x + wm && ux < 0)) {
      ux = 0.0;
      zeroed = true;
      if (std::fabs(uy) < 1e-6)
        uy = away_y >= 0.0 ? 1.0 : -1.0;
    }
    ty = py_ + uy * retreat_step_; // 改完方向后重算(审查：旧值会漏掉另一面墙)
    if ((ty > ggcfg_.max_y - wm && uy > 0) || (ty < ggcfg_.min_y + wm && uy < 0)) {
      uy = 0.0;
      zeroed = true;
      if (std::fabs(ux) < 1e-6)
        ux = away_x >= 0.0 ? 1.0 : -1.0;
    }
    // 墙角：沿墙切向被另一面墙挡住时，改走反向切向(只要它仍远离障碍，下面的 escape 检查会把关)。
    if (zeroed && std::hypot(ux, uy) > 1e-6) {
      const double n0 = std::hypot(ux, uy);
      const Vec2 probe{px_ + ux / n0 * retreat_step_, py_ + uy / n0 * retreat_step_};
      if (!field_motion_clear({px_, py_}, probe, ggcfg_)) {
        ux = -ux;
        uy = -uy;
      }
    }
    const double nn = std::hypot(ux, uy);
    if (nn < 1e-6) { // 两墙夹角：朝场地中心退
      const double cxw = 0.5 * (ggcfg_.min_x + ggcfg_.max_x);
      const double cyw = 0.5 * (ggcfg_.min_y + ggcfg_.max_y);
      ux = cxw - px_;
      uy = cyw - py_;
      const double n2 = std::hypot(ux, uy);
      if (n2 < 1e-6) {
        ux = -1.0;
        uy = 0.0;
      } else {
        ux /= n2;
        uy /= n2;
      }
    } else {
      ux /= nn;
      uy /= nn;
    }
  }

  if (!retreating_) {
    retreating_ = true;
    retreat_origin_ = {px_, py_};
    retreat_start_ = now();
  }

  const Vec2 back{px_ + ux * retreat_step_, py_ + uy * retreat_step_};
  traj_ = smooth_catmull_rom(Path2{{px_, py_}, back}, arc_ds_); // 可视化:后退直线
  last_look_ = back;
  look_valid_ = true;

  // ★不走 pd_core(2026-08 修死锁)★
  //   旧实现 pd_core(back, step) 有致命缺陷：后退点在飞机【身后】，e_yaw≈180° →
  //   朝向门控 head_gate=0 → 速度恒为 0，只发 yaw_rate 让飞机原地掉头 180°。
  //   实测要转 72 拍(1.44s)速度才解禁；而这期间飞机没动 → traveled 恒为 0 →
  //   try_retreat 的"已退 0.00m"永远成立 → 无限重试；若期间障碍圆微抖使 back 方向
  //   改变、或 replan_locked 复位 retreat_origin_，则永远转不出去 → 彻底卡死
  //   (日志表现：反复刷"A* 无解且贴障 → 后退脱困(已退 0.00m)"，飞机悬在原地不动)。
  //   ★后退本来就不该先掉头★：飞机是全向的，直接把"世界系后退方向"投影到机体系
  //   即可，机头保持不动(也不该动——机头还朝着障碍/未扫区，转走反而丢失视野)。
  const double c = std::cos(yaw_), s = std::sin(yaw_);
  Vec2 vb{(c * ux + s * uy) * retreat_v_max_,   // 机体前向分量
          (-s * ux + c * uy) * retreat_v_max_}; // 机体横向分量
  last_yaw_rate_ = 0.0;                         // 后退期间不转头，保持视野朝向
  // 合速度限幅（分量已按 retreat_v_max_ 缩放，这里只防数值溢出）
  const double sp = std::hypot(vb.x, vb.y);
  if (sp > retreat_v_max_ && sp > 1e-6) {
    const double k = retreat_v_max_ / sp;
    vb.x *= k;
    vb.y *= k;
  }
  if (!obstacle_escape_clear({px_, py_}, back, obs, ggcfg_) ||
      !field_motion_clear({px_, py_}, back, ggcfg_)) {
    // Moving away from the closest obstacle is not enough: another
    // obstacle or wall can occupy the proposed retreat segment.
    traj_.clear();
    last_look_ = {px_, py_};
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "[后退脱困] 退路受障碍/边界限制，停止后退；本轮超时后改选观察点");
    return {};
  }
  return vb;
}

bool ExplorationNode::retreat_available() {
  const Vec2 cur{px_, py_};
  if (retreat_exhausted_) {
    if (std::hypot(cur.x - retreat_exhausted_at_.x, cur.y - retreat_exhausted_at_.y) <
        std::max(.10, goal_tol_))
      return false;
    retreat_exhausted_ = false;
  }
  if (!retreating_)
    return true;
  const double traveled = std::hypot(px_ - retreat_origin_.x, py_ - retreat_origin_.y);
  const double elapsed = (now() - retreat_start_).seconds();
  if (traveled < retreat_max_dist_ && !(retreat_timeout_ > 0.0 && elapsed >= retreat_timeout_))
    return true;
  // Preserve exhaustion across planning/timer calls. Clearing only the
  // running flag allowed the next tick to start the same failed attempt.
  retreat_exhausted_ = true;
  retreat_exhausted_at_ = cur;
  retreating_ = false;
  RCLCPP_WARN(
      get_logger(),
      "后退脱困结束(%.1fs，已退%.2fm)，暂停原地重复尝试，改选观察点；位置改变或有效新路线后可重试",
      elapsed, traveled);
  return false;
}

bool ExplorationNode::try_retreat(const Obstacles &obs, geometry_msgs::msg::TwistStamped &cmd) {
  if (!retreat_available())
    return false;
  const double od = nearest_obstacle_dist(obs);
  const double traveled =
      retreating_ ? std::hypot(px_ - retreat_origin_.x, py_ - retreat_origin_.y) : 0.0;
  const double elapsed = retreating_ ? (now() - retreat_start_).seconds() : 0.0;
  if (od < retreat_trigger_) {
    const Vec2 vb = do_retreat(obs);
    cmd.twist.linear.x = vb.x;
    cmd.twist.linear.y = vb.y;
    cmd.twist.angular.z = last_yaw_rate_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "A* 无解且贴障(边距 %.2fm) → 后退脱困(已退 %.2fm, %.1fs)", od, traveled,
                         elapsed);
    return true;
  }
  retreating_ = false; // 已不贴障，交正常规划逻辑。
  return false;
}

bool ExplorationNode::required_retreat_allowed(const Obstacles &obs,
                                               geometry_msgs::msg::TwistStamped &cmd) {
  if (required_retreats_ >= max_home_retreats_)
    return false;
  const bool was_retreating = retreating_;
  const bool retreated = try_retreat(obs, cmd);
  if (retreated && !was_retreating && retreating_) {
    ++required_retreats_;
    RCLCPP_WARN(get_logger(), "[必达路径] 贴障无路 → 后退脱困(第 %d/%d 次)", required_retreats_,
                max_home_retreats_);
  }
  return retreated;
}

void ExplorationNode::record_trail() {
  const Vec2 cur{px_, py_};
  if (trail_.empty() ||
      std::hypot(cur.x - trail_.back().x, cur.y - trail_.back().y) >= trail_spacing_) {
    trail_.push_back(cur);
    if (trail_.size() > static_cast<size_t>(trail_capacity_))
      trail_.pop_front();
  }
}

void ExplorationNode::finish_backtrack(const char *reason) {
  // 已退过的面包屑出队：下次再卡住就从更早的来路继续退。
  if (backtrack_next_ > 0) {
    const Vec2 reached = backtrack_path_[backtrack_next_ - 1];
    while (!trail_.empty() &&
           std::hypot(trail_.back().x - reached.x, trail_.back().y - reached.y) > 1e-6)
      trail_.pop_back();
  }
  backtracking_ = false;
  backtrack_path_.clear();
  unreachable_.clear();
  turning_for_solution_ = false;
  plan_pending_ = true;
  no_route_since_ = steady_seconds();
  RCLCPP_INFO(get_logger(), "[探索] 原路后退结束(%s)，清空够不到黑名单重新找路", reason);
}

bool ExplorationNode::update_backtrack(const Obstacles &obs,
                                       geometry_msgs::msg::TwistStamped &cmd) {
  const double t = steady_seconds();
  if (last_explore_tick_ < 0.0 || t - last_explore_tick_ > 0.3 || !has_goal_ || finished_) {
    no_route_since_ = t; // 刚回到探索分支 / 任务还没开始
    if (backtracking_) {
      backtracking_ = false;
      backtrack_path_.clear();
    } // 暂停后从当前位置重新寻路
  }
  last_explore_tick_ = t;
  if (!backtracking_) {
    const bool has_route = explore_has_committed_ && !explore_failed_ && tracker_->has_trajectory();
    if (has_route)
      record_trail(); // 贴障后退的反向轨迹不记入来路(审查发现)
    if (has_route || retreating_) {
      no_route_since_ = t;
      return false;
    }
    if (backtrack_after_s_ <= 0.0 || t - no_route_since_ < backtrack_after_s_)
      return false;
    no_route_since_ = t;
    backtrack_path_.clear();
    Vec2 from{px_, py_};
    double length = 0.0;
    for (auto it = trail_.rbegin(); it != trail_.rend() && length < backtrack_distance_; ++it) {
      const double step = std::hypot(it->x - from.x, it->y - from.y);
      if (step < 0.10)
        continue;
      backtrack_path_.push_back(*it);
      length += step;
      from = *it;
    }
    if (backtrack_path_.empty()) {
      RCLCPP_WARN(get_logger(), "[探索] %.0f 秒没有可走的路，但没有可退的来路；继续原地找路",
                  backtrack_after_s_);
      return false;
    }
    backtracking_ = true;
    backtrack_next_ = 0;
    backtrack_started_ = t;
    backtrack_command_ = {};
    turning_for_solution_ = false;
    ++backtracks_;
    RCLCPP_WARN(get_logger(),
                "[探索] %.0f 秒没有可走的路 → 沿来时轨迹原路后退 %.1fm 再换路(第%llu次)",
                backtrack_after_s_, length, static_cast<unsigned long long>(backtracks_));
  }
  const Vec2 cur{px_, py_};
  while (backtrack_next_ < backtrack_path_.size() &&
         std::hypot(backtrack_path_[backtrack_next_].x - cur.x,
                    backtrack_path_[backtrack_next_].y - cur.y) < backtrack_tolerance_)
    ++backtrack_next_;
  if (backtrack_next_ >= backtrack_path_.size()) {
    finish_backtrack("已退到位");
    return false;
  }
  if (t - backtrack_started_ > backtrack_timeout_) {
    finish_backtrack("超时");
    return false;
  }
  const Vec2 target = backtrack_path_[backtrack_next_];
  if (!obstacle_segment_clear(cur, target, obs, ggcfg_) ||
      !field_motion_clear(cur, target, ggcfg_)) {
    finish_backtrack("来路被障碍/边界挡住");
    return false;
  }
  double remaining = std::hypot(target.x - cur.x, target.y - cur.y);
  for (size_t i = backtrack_next_ + 1; i < backtrack_path_.size(); ++i)
    remaining += std::hypot(backtrack_path_[i].x - backtrack_path_[i - 1].x,
                            backtrack_path_[i].y - backtrack_path_[i - 1].y);
  const double speed = std::min(retreat_v_max_, 0.8 * remaining + 0.05);
  const double d = std::max(1e-6, std::hypot(target.x - cur.x, target.y - cur.y));
  const double c = std::cos(yaw_), sn = std::sin(yaw_);
  const Vec2 measured{c * v_fwd_est_ - sn * v_lat_est_, sn * v_fwd_est_ + c * v_lat_est_};
  backtrack_command_ = slew_velocity(
      backtrack_command_, {(target.x - cur.x) / d * speed, (target.y - cur.y) / d * speed},
      measured, gains_.max_accel, xy_brake_accel_, gains_.dt);
  cmd.twist.linear.x = c * backtrack_command_.x + sn * backtrack_command_.y;
  cmd.twist.linear.y = -sn * backtrack_command_.x + c * backtrack_command_.y;
  cmd.twist.angular.z = 0.0; // 机头不动：倒着退回去，视野仍朝原方向
  traj_ = make_polyline_trajectory([&] {
    Path2 path{cur};
    for (size_t i = backtrack_next_; i < backtrack_path_.size(); ++i)
      path.push_back(backtrack_path_[i]);
    return path;
  }());
  last_look_ = target;
  look_valid_ = true;
  return true;
}

} // namespace exploration
