#include "exploration_planner/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace exploration {
namespace {
double wrap_pi(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

double clamp_abs(double v, double lim)
{
    return std::clamp(v, -std::max(0.0, lim), std::max(0.0, lim));
}

double dist(const Vec2& a, const Vec2& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

double heading(const Vec2& a, const Vec2& b)
{
    return std::atan2(b.y - a.y, b.x - a.x);
}
}  // namespace

// 停车转向的 XY 锚点取"预计滑停点"而非当前点：以 0.5 m/s 进入停车，惯性会再滑约 0.2 m，
// 锚在脚下会先冲出再以 0.15 m/s 倒退拉回 1 s(seed43 实测)。滑停点是物理上必然经过的位置，
// 锚在那里不增加风险；偏移量上限 0.20 m，调用方的定点纠偏输出仍经障碍校验。
Vec2 TrajectoryTracker::stop_anchor(const Vec2& cur, double yaw, double v_fwd_est, double v_lat_est) const
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double vx = c * v_fwd_est - s * v_lat_est, vy = s * v_fwd_est + c * v_lat_est;
    const double speed = std::hypot(vx, vy);
    if (speed < 1e-3) return cur;
    const double brake = std::max(0.1, g_.stop_anchor_decel);
    const double distance = std::min(g_.stop_anchor_max_m,
        speed * speed / (2.0 * brake) + std::max(0.0, g_.stop_anchor_latency) * speed);
    return {cur.x + vx / speed * distance, cur.y + vy / speed * distance};
}

Vec2 TrajectoryTracker::issued_world_velocity(double yaw) const
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * previous_forward_command_ - s * previous_lateral_command_,
            s * previous_forward_command_ + c * previous_lateral_command_};
}

void TrajectoryTracker::inherit_motion(const TrajectoryTracker& other)
{
    previous_forward_command_ = other.previous_forward_command_;
    previous_lateral_command_ = other.previous_lateral_command_;
    previous_yaw_command_ = other.previous_yaw_command_;
    filtered_yaw_rate_ = other.filtered_yaw_rate_;
    filtered_desired_rate_ = other.filtered_desired_rate_;
    filtered_lateral_target_ = other.filtered_lateral_target_;
    filtered_slip_ = other.filtered_slip_;
    prev_yaw_ = other.prev_yaw_;
    last_steering_heading_ = other.last_steering_heading_;
    motion_valid_ = other.motion_valid_;
    history_ = other.history_;
}

void TrajectoryTracker::set_trajectory(const Trajectory& traj)
{
    if (!traj_.empty() && !traj.empty()) {
        // 换路衔接的是当前正在执行的航向，不是旧路线尚未飞到的终点切线。
        // 用终点方向会在重规划后先朝未来弯道摆头，再返回当前直线。
        const double old_heading = motion_valid_ ? last_steering_heading_ : sample_at(progress_s_).theta;
        const double new_heading = traj.front().theta;
        const double delta = wrap_pi(new_heading - old_heading);
        if (std::abs(delta) < 0.7853981633974483) {
            handoff_heading_ = old_heading;
            handoff_remaining_ = 0.30;
            handoff_valid_ = true;
        } else {
            handoff_valid_ = false;
        }
    } else if (traj.empty()) {
        handoff_valid_ = false;
    }
    traj_ = traj;
    progress_idx_ = 0;
    progress_s_ = traj_.empty() ? 0.0 : traj_.front().s;
    passed_corner_s_ = -1.0;
    progress_valid_ = false;
    last_nearest_dist_ = 0.0;
    last_heading_error_ = last_curvature_ = 0.0;
    prev_valid_ = false;
    alignment_heading_valid_ = false;
    alignment_settled_ = 0.0;
    aligning_ = false;
    holonomic_hold_active_ = false;
    turn_direction_ = 0;
    if (traj_.empty()) {
        motion_valid_ = false;
        filtered_yaw_rate_ = previous_yaw_command_ = previous_forward_command_ = previous_lateral_command_ = 0.0;
        filtered_desired_rate_ = filtered_lateral_target_ = filtered_slip_ = 0.0;
        history_.clear();
        aligning_ = false;
        turn_direction_ = 0;
    }
}

TrajPoint TrajectoryTracker::sample_at(double s) const
{
    if (traj_.size() < 2 || s <= traj_.front().s) return traj_.front();
    if (s >= traj_.back().s) return traj_.back();
    const auto upper = std::upper_bound(traj_.begin(), traj_.end(), s,
        [](double value, const TrajPoint& point) { return value < point.s; });
    const auto& b = *upper;
    const auto& a = *(upper - 1);
    const double u = std::clamp((s - a.s) / std::max(1e-9, b.s - a.s), 0.0, 1.0);
    return {{a.p.x + u * (b.p.x - a.p.x), a.p.y + u * (b.p.y - a.p.y)},
            a.theta + u * wrap_pi(b.theta - a.theta),
            a.kappa + u * (b.kappa - a.kappa), s};
}

size_t TrajectoryTracker::next_corner() const
{
    if (!g_.forward_only) return traj_.size();
    for (size_t i = std::max<size_t>(1, progress_idx_); i + 1 < traj_.size(); ++i) {
        if (traj_[i].s <= passed_corner_s_ + 1e-6 || traj_[i].s < progress_s_ - 1e-6)
            continue;
        if (dist(traj_[i - 1].p, traj_[i].p) < 1e-6 ||
            dist(traj_[i].p, traj_[i + 1].p) < 1e-6) continue;
        const double angle = wrap_pi(heading(traj_[i].p, traj_[i + 1].p) -
                                     heading(traj_[i - 1].p, traj_[i].p));
        // Only a discontinuous, sharp polyline vertex requires a stop. Small
        // heading increments on a sampled curve are handled by moving steering.
        if (std::abs(angle) >= std::max(0.01, g_.corner_stop_rad)) return i;
    }
    return traj_.size();
}

double TrajectoryTracker::projected_progress(double px, double py) const
{
    const Vec2 cur{px, py};
    const double traveled = progress_valid_ ? dist(cur, progress_position_) : 0.0;
    // Limit progress in metres, independently of sampling density. Nearby return
    // legs must not replace the current leg merely because they are closer.
    double limit = std::min(traj_.back().s,
        progress_s_ + std::max(0.15, traveled * 2.0 + 0.05));
    if (!progress_valid_) limit = std::min(traj_.back().s,
        progress_s_ + std::max(g_.lookahead, dist(cur, traj_.front().p)));
    const size_t corner = next_corner();
    if (corner < traj_.size()) limit = std::min(limit, traj_[corner].s);

    double best_s = progress_s_;
    double best_distance = dist(cur, sample_at(progress_s_).p);
    for (size_t i = progress_idx_; i + 1 < traj_.size(); ++i) {
        if (traj_[i].s > limit + 1e-9) break;
        const auto& a = traj_[i];
        const auto& b = traj_[i + 1];
        const double ds = b.s - a.s;
        const double dx = b.p.x - a.p.x, dy = b.p.y - a.p.y;
        const double length2 = dx * dx + dy * dy;
        if (ds <= 1e-9 || length2 <= 1e-12 || b.s < progress_s_) continue;
        const double lo = std::clamp((progress_s_ - a.s) / ds, 0.0, 1.0);
        const double hi = std::clamp((limit - a.s) / ds, lo, 1.0);
        const double u = std::clamp(((px - a.p.x) * dx + (py - a.p.y) * dy) / length2,
                                    lo, hi);
        const double d = dist(cur, {a.p.x + u * dx, a.p.y + u * dy});
        if (d + 1e-9 < best_distance) {
            best_distance = d;
            best_s = a.s + u * ds;
        }
    }
    return best_s;
}

void TrajectoryTracker::advance_to_nearest(double px, double py)
{
    const Vec2 cur{px, py};
    progress_s_ = projected_progress(px, py);
    while (progress_idx_ + 1 < traj_.size() &&
           traj_[progress_idx_ + 1].s <= progress_s_ + 1e-9) ++progress_idx_;
    last_nearest_dist_ = dist(cur, sample_at(progress_s_).p);
    progress_position_ = cur;
    progress_valid_ = true;
}

Path2 TrajectoryTracker::remaining_path(double px, double py) const
{
    if (traj_.empty()) return {};
    Path2 result{{px, py}};
    // Planning runs before update() in the node. Project the current pose with
    // the same bounded progress rule instead of reconnecting to last tick's
    // point behind us, which can falsely invalidate a safe obstacle escape.
    const double remaining_s = projected_progress(px, py);
    // Rejoin the next forward reference, not the perpendicular foot of the
    // current pose. In a margin escape that perpendicular connection can point
    // inward even though the commanded forward connection is clear.
    const size_t corner = next_corner();
    if (corner < traj_.size() && traj_[corner].s <= remaining_s + 1e-9 &&
        dist(result.back(), traj_[corner].p) > 1e-6)
        result.push_back(traj_[corner].p);  // Keep every unpassed hard corner.
    size_t first = 0;
    while (first < traj_.size() && traj_[first].s <= remaining_s + 1e-9) ++first;
    // 直线的采样密度不应改变重接方向。贴障脱离时，下一个几厘米远的
    // 采样点可能位于飞机侧后方，造成“路被挡住”的假判断和连续重规划。
    // 只合并同向共线的采样点；每个真实弯点和未经过的硬折点都保留，
    // 从实际位置到该直线末端的整段仍由调用方进行障碍检查。
    if (first > 0 && first < traj_.size()) {
        const Vec2 a = traj_[first-1].p;
        const Vec2 b = traj_[first].p;
        const double dx = b.x-a.x, dy = b.y-a.y;
        const double length = std::hypot(dx,dy);
        while (first+1 < traj_.size() && first != corner && length > 1e-9) {
            const Vec2 from = traj_[first].p, to = traj_[first+1].p;
            const double sx = to.x-from.x, sy = to.y-from.y;
            if (dx*sx+dy*sy <= 0 ||
                std::abs(dx*sy-dy*sx) > 1e-9*length*std::hypot(sx,sy)) break;
            ++first;
        }
    }
    for (size_t i = first; i < traj_.size(); ++i) {
        if (dist(result.back(), traj_[i].p) > 1e-6)
            result.push_back(traj_[i].p);
    }
    if (result.size() == 1) result.push_back(traj_.back().p);
    return result;
}

VelCmd TrajectoryTracker::update(double px, double py, double yaw,
                                 double v_fwd_est, double v_lat_est,
                                 double goal_tol, double measured_yaw_rate)
{
    VelCmd cmd = compute(px, py, yaw, v_fwd_est, v_lat_est, goal_tol, measured_yaw_rate);
    const double keep = std::max(g_.smith_time, g_.smith_yaw_time);
    if (keep > 0.0 && !traj_.empty()) {
        history_.push_back({cmd.v_fwd, cmd.v_lat, cmd.yaw_rate, yaw});
        const size_t limit = static_cast<size_t>(std::ceil(keep / std::max(1e-3, g_.dt))) + 1;
        while (history_.size() > limit) history_.pop_front();
    }
    return cmd;
}

Vec2 TrajectoryTracker::smith_displacement() const
{
    const double dt = g_.dt > 1e-6 ? g_.dt : 0.02;
    const size_t n = std::min(history_.size(), static_cast<size_t>(std::lround(g_.smith_time / dt)));
    Vec2 d{};
    for (size_t i = history_.size() - n; i < history_.size(); ++i) {
        const auto& h = history_[i];
        const double c = std::cos(h.yaw), s = std::sin(h.yaw);
        d.x += (c * h.fwd - s * h.lat) * dt;
        d.y += (s * h.fwd + c * h.lat) * dt;
    }
    return d;
}

double TrajectoryTracker::smith_heading() const
{
    const double dt = g_.dt > 1e-6 ? g_.dt : 0.02;
    const size_t n = std::min(history_.size(), static_cast<size_t>(std::lround(g_.smith_yaw_time / dt)));
    double a = 0.0;
    for (size_t i = history_.size() - n; i < history_.size(); ++i) a += history_[i].rate * dt;
    return a;
}

VelCmd TrajectoryTracker::compute(double px, double py, double yaw,
                                  double v_fwd_est, double v_lat_est,
                                  double goal_tol, double measured_yaw_rate)
{
    debug_.limiter = 0;
    VelCmd cmd;
    if (traj_.empty()) return cmd;
    const double dt = g_.dt > 1e-6 ? g_.dt : 0.02;
    const Vec2 cur{px, py};
    const double d_goal = dist(cur, traj_.back().p);
    advance_to_nearest(px, py);
    const TrajPoint nearest = sample_at(progress_s_);

    if (std::isfinite(measured_yaw_rate)) {
        filtered_yaw_rate_ = measured_yaw_rate;
    } else {
        const double raw_rate = motion_valid_ ? wrap_pi(yaw - prev_yaw_) / dt : 0.0;
        const double alpha = dt / (std::max(0.0, g_.yaw_filter_tau) + dt);
        filtered_yaw_rate_ += alpha * (raw_rate - filtered_yaw_rate_);
    }
    prev_yaw_ = yaw;
    motion_valid_ = true;

    if (d_goal <= goal_tol && (!g_.forward_only ||
        traj_.back().s - progress_s_ <= goal_tol)) {
        previous_forward_command_ = previous_yaw_command_ = previous_lateral_command_ = 0.0;
        aligning_ = false;
        alignment_heading_valid_ = false;
        last_look_ = traj_.back().p;
        cmd.at_goal = true;
        return cmd;
    }

    size_t corner = next_corner();
    if (corner < traj_.size()) {
        const Vec2 before = traj_[corner - 1].p, vertex = traj_[corner].p;
        const double incoming = dist(before, vertex);
        const double passed = incoming > 1e-9
            ? ((cur.x - vertex.x) * (vertex.x - before.x) +
               (cur.y - vertex.y) * (vertex.y - before.y)) / incoming : 0.0;
        if (passed > 0.0 && dist(cur, vertex) > std::max(.08, goal_tol)) {
            // A short stale vertex behind us is not a new destination. Ask
            // the obstacle-aware planner for a connection from here immediately;
            // never skip the corner blindly or circle back to chase its point.
            previous_forward_command_ = previous_yaw_command_ = 0.0;
            aligning_ = alignment_heading_valid_ = false;
            last_look_ = cur;
            cmd.needs_replan = true;
            return cmd;
        }
    }
    if (corner < traj_.size() && dist(cur, traj_[corner].p) <= std::max(0.08, goal_tol) &&
        progress_s_ >= traj_[corner].s - std::min(.02, goal_tol * .25)) {
        // Arrival starts a single XY-hold/yaw action. Persistent drift must
        // not prevent selection of the outgoing segment.
        if (!aligning_) {
            aligning_ = true;
            alignment_hold_.begin(stop_anchor(cur, yaw, v_fwd_est, v_lat_est), issued_world_velocity(yaw),
                                  g_.hold_slide, g_.stop_anchor_max_m);
        }
        // 到点是一次新的航段交接。此前若已因偏移进入转向，也必须替换旧的
        // “朝向折点”参考；保持原XY锚点，不先转完旧方向再转回下一段。
        alignment_heading_ = heading(traj_[corner].p, traj_[corner + 1].p);
        alignment_heading_valid_ = true;
        alignment_settled_ = 0.0;
        turn_direction_ = wrap_pi(alignment_heading_ - yaw) >= 0.0 ? 1 : -1;
        passed_corner_s_ = traj_[corner].s;
        progress_s_ = traj_[corner].s;
        progress_idx_ = corner;
        corner = next_corner();
    }
    double lookahead = g_.lookahead;
    if (g_.lookahead_time > 0.0 && g_.lookahead_max > g_.lookahead)
        lookahead = std::clamp(g_.lookahead_time * previous_forward_command_, g_.lookahead, g_.lookahead_max);
    double look_s = std::min(traj_.back().s, progress_s_ + std::max(0.01, lookahead));
    if (corner < traj_.size()) look_s = std::min(look_s, traj_[corner].s);
    const TrajPoint ref = sample_at(look_s);
    last_look_ = ref.p;
    double desired_theta = ref.theta;
    double path_lateral_velocity = 0.0;
    if (g_.forward_only) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        const double vx = c * v_fwd_est - s * v_lat_est;
        const double vy = s * v_fwd_est + c * v_lat_est;
        const double prediction = std::max(0.0, g_.prediction_time);
        const double nx = -std::sin(nearest.theta), ny = std::cos(nearest.theta);
        path_lateral_velocity = nx * vx + ny * vy;
        const double lateral_lead = std::max(0.0, g_.lateral_prediction_time - prediction);
        Vec2 predicted{px + prediction * vx + lateral_lead * path_lateral_velocity * nx,
                       py + prediction * vy + lateral_lead * path_lateral_velocity * ny};
        if (g_.smith_time > 0.0) {
            const Vec2 d = smith_displacement();
            predicted = {px + d.x, py + d.y};
        }
        const double rx = ref.p.x - px, ry = ref.p.y - py;
        const double distance2 = rx * rx + ry * ry;
        if (distance2 > 1e-12) {
            const double advance = ((predicted.x - px) * rx + (predicted.y - py) * ry) / distance2;
            if (advance > .5) {
                predicted.x -= (advance - .5) * rx;
                predicted.y -= (advance - .5) * ry;
            }
        }
        Vec2 steering_ref = ref.p;
        if (corner < traj_.size() && look_s >= traj_[corner].s - 1e-9) {
            // 折点会截短前瞻距离。几厘米横漂除以趋零的剩余距离会把航向拉向
            // 一侧，随后又被出场方向拉回。仅将航向参考沿入场切线延伸到原有
            // 前瞻长度；位置目标/减速/到点检查仍是原折点，不能切角越障。
            const double incoming = heading(traj_[corner - 1].p, traj_[corner].p);
            const double tx = std::cos(incoming), ty = std::sin(incoming);
            const double ahead = (steering_ref.x-predicted.x)*tx + (steering_ref.y-predicted.y)*ty;
            const double extension = std::max(0.0, lookahead-ahead);
            steering_ref.x += extension*tx;
            steering_ref.y += extension*ty;
        }
        if (dist(predicted, steering_ref) > 1e-6) desired_theta = heading(predicted, steering_ref);
        debug_.geo_error = wrap_pi(heading(cur, ref.p) - yaw);
        debug_.lead_error = wrap_pi(desired_theta - heading(cur, ref.p));
    }
    // Smith 模式下航向误差对比的是"已发出角速度指令生效后"的机头朝向。
    const double yaw_ahead = g_.smith_yaw_time > 0.0 ? wrap_pi(yaw + smith_heading()) : yaw;
    double e_yaw = wrap_pi(desired_theta - yaw_ahead);
    if (handoff_valid_ && g_.forward_only && handoff_remaining_ > 0.0) {
        const double blend = std::clamp(handoff_remaining_ / 0.30, 0.0, 1.0);
        const double delta = wrap_pi(desired_theta - handoff_heading_);
        desired_theta = handoff_heading_ + (1.0 - blend) * delta;
        e_yaw = wrap_pi(desired_theta - yaw_ahead);
        handoff_remaining_ = std::max(0.0, handoff_remaining_ - dt);
        if (handoff_remaining_ <= 0.0) handoff_valid_ = false;
    }
    last_heading_error_ = e_yaw;
    last_steering_heading_ = desired_theta;
    const double e_ct = -std::sin(nearest.theta) * (px - nearest.p.x) +
                         std::cos(nearest.theta) * (py - nearest.p.y);

    if (!g_.forward_only) {
        const double de_yaw = prev_valid_ ? wrap_pi(e_yaw - prev_e_yaw_) / dt : 0.0;
        const double de_ct = prev_valid_ ? (e_ct - prev_e_ct_) / dt : 0.0;
        prev_e_yaw_ = e_yaw;
        prev_e_ct_ = e_ct;
        prev_valid_ = true;
        cmd.yaw_rate = clamp_abs(g_.kp_yaw * e_yaw + g_.kd_yaw * de_yaw, g_.max_yaw_rate);
        const double gate = std::abs(e_yaw) > g_.heading_gate_rad ? 0.0 :
                            std::max(0.0, std::cos(e_yaw));
        if (gate == 0.0) {
            if (!holonomic_hold_active_) alignment_hold_.begin(cur);
            holonomic_hold_active_ = true;
            const auto correction = alignment_hold_.update(cur, yaw, v_fwd_est, v_lat_est,
                g_.turn_hold_kp, g_.turn_hold_kd, std::min(g_.turn_hold_speed, g_.v_max), g_.max_accel, dt);
            cmd.v_fwd = correction.x; cmd.v_lat = correction.y;
            cmd.holding_position = true;
            return cmd;
        }
        holonomic_hold_active_ = false;
        const double lateral_target = clamp_abs(-(g_.kp_lat * e_ct + g_.kd_lat * de_ct), g_.max_v_lat) * gate;
        // Limit lateral command slew in ordinary exploration.  A raw
        // cross-track derivative can change sign between adjacent scans and
        // produce the observed left/right rocking even when the path itself
        // is smooth.  The limit is local to the lateral channel and does not
        // affect heading alignment or corridor control.
        const double lateral_step = std::max(0.01, g_.max_lateral_accel * dt);
        cmd.v_lat = std::clamp(lateral_target,
                               previous_lateral_command_ - lateral_step,
                               previous_lateral_command_ + lateral_step);
        previous_lateral_command_ = cmd.v_lat;
        double v = g_.v_max / (1.0 + g_.k_curv * std::abs(ref.kappa)) * gate;
        if (d_goal < g_.endpoint_slow_r) v *= d_goal / g_.endpoint_slow_r;
        if (d_goal >= g_.endpoint_slow_r && gate > 0.5) v = std::max(v, g_.v_min);
        cmd.v_fwd = std::clamp(v, 0.0, g_.v_max);
        return cmd;
    }

    const double stop_angle = std::clamp(g_.stop_align_rad, 0.35, M_PI * 0.5);
    if (!aligning_ && std::abs(e_yaw) >= stop_angle) {
        aligning_ = true;
        alignment_hold_.begin(stop_anchor(cur, yaw, v_fwd_est, v_lat_est), issued_world_velocity(yaw),
                              g_.hold_slide, g_.stop_anchor_max_m);
        alignment_heading_valid_ = false;
        alignment_settled_ = 0.0;
        // 目标接近正后方(>149°)时最短转向的符号几乎随机：顺着当前正在转的方向继续转，
        //   不先反向刹车(r33：交接时正以 -0.7 rad/s 转，却选了反向，刹停+多转 194° 共 7 s)。
        const double spin = std::abs(previous_yaw_command_) > 0.1 ? previous_yaw_command_ : filtered_yaw_rate_;
        turn_direction_ = std::abs(e_yaw) > 2.6 && std::abs(spin) > 0.1
            ? (spin > 0.0 ? 1 : -1) : (e_yaw >= 0.0 ? 1 : -1);
    }
    if (aligning_ && !alignment_heading_valid_) {
        alignment_heading_ = desired_theta;
        alignment_heading_valid_ = true;
    }
    if (aligning_ && alignment_heading_valid_) {
        // Smith 模式下同样对比预测航向(审查发现：此处用实测航向且又关掉了实测角速度外推，停车转向会过冲约 0.3 rad)。
        e_yaw = wrap_pi(alignment_heading_ - yaw_ahead);
        last_heading_error_ = e_yaw;
        last_steering_heading_ = alignment_heading_;
        // The shortest turn changes sign at pi. Keep the chosen turn through
        // that noisy boundary until there is an unambiguous shorter direction.
        if (std::abs(e_yaw) > 2.6 && turn_direction_ * e_yaw < 0.0)
            e_yaw += turn_direction_ * 2.0 * M_PI;
    }

    const double yaw_accel = std::max(1e-3, g_.max_yaw_accel);
    const double command_weight = std::clamp(g_.yaw_damping_command_weight, 0.0, 1.0);
    const double damping_rate = command_weight * previous_yaw_command_ + (1.0 - command_weight) * filtered_yaw_rate_;
    // Smith 模式的 e_yaw 已对比预测航向，不再按实测角速度重复外推。
    const double yaw_prediction = g_.smith_yaw_time > 0.0 ? 0.0 : std::max(0.0, g_.prediction_time);
    double desired_rate = 0.0;
    if (!aligning_ || alignment_heading_valid_) {
        const double predicted_error = e_yaw - yaw_prediction * damping_rate;
        const double braking_rate = std::sqrt(2.0 * yaw_accel * std::abs(predicted_error));
        desired_rate = clamp_abs(g_.kp_yaw * predicted_error, braking_rate) -
                       g_.kd_yaw * damping_rate;
        debug_.ff_term = 0.0;
        debug_.p_term = clamp_abs(g_.kp_yaw * predicted_error, braking_rate);
        debug_.d_term = -g_.kd_yaw * damping_rate;
        debug_.ff_term = 0.0;
        if (!aligning_) {
            // 曲率前馈(2026-09-23)：弯道中稳态角速度 = v·κ。只靠前瞻点方位误差的 P 项，
            // 必须先积累出航向误差才有角速度，飞机总是滞后切到弯外侧(基线弯道贴线误差
            // 0.10~0.17 m)。用上一拍实际前进指令与最近参考点曲率做前馈，P 项只修残差。
            // 用实测前进速度而非指令：加速段指令领先实际约一个响应时间常数，按指令前馈会转过头，
            // 再由 P 项反向拉回，形成一次多余的角速度反转(单元测试 110° 弯道复现)。
            const double feed_speed = std::clamp(v_fwd_est, 0.0, std::max(0.0, previous_forward_command_));
            desired_rate += g_.curvature_feedforward * feed_speed * nearest.kappa;
            debug_.ff_term = g_.curvature_feedforward * feed_speed * nearest.kappa;
        }
        desired_rate = clamp_abs(desired_rate, g_.max_yaw_rate);
        debug_.desired = desired_rate;
        // 只对反馈部分(P+D)低通：曲率前馈来自参考路径本身、已平滑，滤它只会让弯道滞后切外侧。
        const double feedforward = aligning_ ? 0.0 : debug_.ff_term;
        if (!aligning_ && g_.command_filter_tau > 0.0) {
            filtered_desired_rate_ += dt / (g_.command_filter_tau + dt) *
                (desired_rate - feedforward - filtered_desired_rate_);
            desired_rate = clamp_abs(filtered_desired_rate_ + feedforward, g_.max_yaw_rate);
        } else {
            filtered_desired_rate_ = desired_rate - feedforward;
        }
    }
    cmd.yaw_rate = previous_yaw_command_ + clamp_abs(desired_rate - previous_yaw_command_, yaw_accel * dt);
    cmd.yaw_rate = clamp_abs(cmd.yaw_rate, g_.max_yaw_rate);
    previous_yaw_command_ = cmd.yaw_rate;

    if (aligning_) {
        const double predicted_error = e_yaw - yaw_prediction * damping_rate;
        const double stopping_angle = filtered_yaw_rate_ * filtered_yaw_rate_ /
            (2.0 * yaw_accel);
        const bool turn_is_braking = e_yaw * filtered_yaw_rate_ >= 0.0 &&
            stopping_angle <= std::abs(e_yaw) + g_.align_resume_rad * 0.25;
        const bool settled = alignment_heading_valid_ &&
            std::abs(e_yaw) <= std::min(g_.align_resume_rad, stop_angle * 0.8) &&
            std::abs(predicted_error) <= g_.align_resume_rad &&
            (std::abs(filtered_yaw_rate_) <= g_.align_stop_yaw_rate || turn_is_braking);
        alignment_settled_ = settled ? alignment_settled_ + dt : 0.0;
        if (alignment_settled_ >= std::max(dt, g_.align_settle_s)) {
            aligning_ = false;
            alignment_heading_valid_ = false;
            turn_direction_ = 0;
        }
        previous_forward_command_ = 0.0;
        previous_lateral_command_ = 0.0;
        // 刹车按 stop_anchor_decel(与滑停锚点估计同口径)，定点纠偏本身限速 turn_hold_speed。
        //   用 max_decel(0.6) 时仿真 L3 折返点滑过 0.2~0.4 m(r05)。
        const auto correction = alignment_hold_.update(cur, yaw, v_fwd_est, v_lat_est,
            g_.turn_hold_kp, g_.turn_hold_kd, std::min(g_.turn_hold_speed, g_.v_max),
            std::max(g_.max_accel, g_.stop_anchor_decel), dt);
        cmd.v_fwd = correction.x;
        cmd.v_lat = correction.y;
        cmd.holding_position = true;
        return cmd;
    }

    // Translate only as fast as the remaining correction and lateral inertia
    // allow. A lower speed bound must never override braking or acceleration.
    const double slow_angle = std::clamp(g_.heading_gate_rad, 0.0, stop_angle - 0.01);
    const double angle_ratio = std::clamp((std::abs(e_yaw) - slow_angle) /
        std::max(0.01, stop_angle - slow_angle), 0.0, 1.0);
    const double gate = std::pow(std::max(0.0, std::cos(e_yaw)), 2) *
        (1.0 - angle_ratio * angle_ratio * (3.0 - 2.0 * angle_ratio));
    double curvature = std::max(std::abs(nearest.kappa), std::abs(ref.kappa));
    for (size_t i = progress_idx_; i < traj_.size() && traj_[i].s <= look_s; ++i)
        curvature = std::max(curvature, std::abs(traj_[i].kappa));
    last_curvature_ = curvature;
    double v = g_.v_max / (1.0 + g_.k_curv * curvature);
    int limiter = 1;
    const auto note = [&](int id, double before) { if (v < before - 1e-9) limiter = id; };
    const double lateral_accel = std::max(1e-3, g_.max_lateral_accel);
    const double decel = std::max(1e-3, g_.max_decel);
    // 前瞻速度剖面：前瞻点之外、下一个硬拐点之前的每个采样点都有自己的曲率限速；
    // 以 decel 反推当前允许速度，弯道减速提前、连续开始，而不是进入前瞻窗才骤降。
    {
        const double before = v;
        const double preview_end = std::min(traj_.back().s,
            look_s + g_.v_max * g_.v_max / (2.0 * decel) + std::max(0.0, g_.lookahead));
        const double corner_s = corner < traj_.size() ? traj_[corner].s : preview_end;
        for (size_t i = progress_idx_; i < traj_.size() && traj_[i].s <= std::min(preview_end, corner_s); ++i) {
            if (traj_[i].s <= look_s) continue;
            const double k = std::abs(traj_[i].kappa);
            if (k <= 1e-6) continue;
            const double allowed = std::min(g_.v_max / (1.0 + g_.k_curv * k), std::sqrt(lateral_accel / k));
            v = std::min(v, std::sqrt(allowed * allowed + 2.0 * decel * (traj_[i].s - look_s)));
        }
        note(2, before);
    }
    double stop_distance = d_goal;
    if (corner < traj_.size()) stop_distance = std::min(stop_distance, dist(cur, traj_[corner].p));
    const double slow_radius = std::max(1e-3, g_.endpoint_slow_r);
    // 终点/尖角前的线性停车律保留，但加最低爬行速度：v ∝ d 与曲率/航向门叠加后是指数逼近，
    // 仿真里出现朝尖角以 0.02 m/s 爬 14 s 才进入停车转向。到达判定容差为 max(0.08, goal_tol)。
    // (试过 sqrt(2ad) 律：到达速度约 0.2 m/s，终点过冲、航向来回，单元测试 45°/90° 退化，弃用。)
    {
        const double slow_speed = g_.endpoint_slow_speed > 0.0 ? std::min(g_.endpoint_slow_speed, g_.v_max) : g_.v_max;
        const double before = v;
        if (stop_distance < slow_radius)
            v = std::min(v, std::max(std::min(g_.creep_speed, g_.v_max), slow_speed * stop_distance / slow_radius));
        else
            v = std::min(v, std::sqrt(slow_speed * slow_speed + 2.0 * decel * (stop_distance - slow_radius)));
        note(3, before);
    }
    const double slip = std::max(std::abs(v_lat_est), std::abs(path_lateral_velocity));
    filtered_slip_ = g_.lateral_slow_tau > 0.0
        ? filtered_slip_ + dt / (g_.lateral_slow_tau + dt) * (slip - filtered_slip_) : slip;
    const double lat_factor = 1.0 / (1.0 + std::max(0.0, g_.lateral_slow_gain) *
        std::max(0.0, filtered_slip_ - std::max(0.0, g_.lateral_slow_deadband)));
    if (gate * lat_factor < 0.999) limiter = gate <= lat_factor ? 4 : 7;
    v *= gate * lat_factor;
    if (stop_distance >= slow_radius && gate > 0.5)
        v = std::max(v, std::min(g_.v_min, g_.v_max) * gate);
    if (curvature > 1e-6) { const double before = v; v = std::min(v, std::sqrt(lateral_accel / curvature)); note(5, before); }
    if (std::abs(filtered_yaw_rate_) > 1e-6) {
        const double before = v;
        v = std::min(v, lateral_accel / std::abs(filtered_yaw_rate_));
        note(6, before);
    }
    v = std::clamp(v, 0.0, g_.v_max);
    debug_.limiter = limiter; debug_.gate = gate; debug_.lat_factor = lat_factor; debug_.v_target = v;
    // 加速与减速都走斜坡。真正需要急停的分支(尖角/大角度停车/滑过拐点)在上方已单独返回。
    cmd.v_fwd = std::clamp(v, std::max(0.0, previous_forward_command_ - decel * dt),
                           previous_forward_command_ + std::max(0.0, g_.max_accel) * dt);
    if (g_.compensate_lateral_drift) {
        // Correct displacement and actual velocity normal to the route. Do not
        // differentiate a moving lookahead/segment target to estimate drift.
        const double normal_command = -(g_.kp_lat * e_ct + g_.drift_velocity_gain * path_lateral_velocity);
        double lateral_target = clamp_abs(normal_command * std::cos(nearest.theta - yaw), g_.max_v_lat);
        if (g_.command_filter_tau > 0.0) {
            filtered_lateral_target_ += dt / (g_.command_filter_tau + dt) * (lateral_target - filtered_lateral_target_);
            lateral_target = filtered_lateral_target_;
        }
        const double step = std::max(0.0, g_.max_lateral_accel) * dt;
        cmd.v_lat = std::clamp(lateral_target, previous_lateral_command_ - step,
                                               previous_lateral_command_ + step);
        if (std::abs(cmd.v_lat) < 1e-3 && std::abs(lateral_target) < 1e-3) {
            // 无需补偿：不标记横向纠偏，调用方也就不必对纯沿线前进做额外的短投影
            // 几何校验(该校验在禁入圈边缘、近零速时会被方向噪声随机拦截而停滞)。
            cmd.v_lat = 0.0;
            previous_lateral_command_ = 0.0;
        } else {
            // Forward-only still forbids commanded reverse travel during cruise.
            // Reserve speed for the lateral correction rather than exceeding v_max.
            cmd.v_fwd = std::min(cmd.v_fwd, std::sqrt(std::max(0.0,
                g_.v_max * g_.v_max - cmd.v_lat * cmd.v_lat)));
            previous_lateral_command_ = cmd.v_lat;
            cmd.lateral_correction = true;
        }
    }
    previous_forward_command_ = cmd.v_fwd;
    return cmd;
}

}  // namespace exploration
