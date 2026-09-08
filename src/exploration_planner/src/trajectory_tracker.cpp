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

void TrajectoryTracker::set_trajectory(const Trajectory& traj)
{
    traj_ = traj;
    progress_idx_ = 0;
    progress_s_ = traj_.empty() ? 0.0 : traj_.front().s;
    passed_corner_s_ = -1.0;
    progress_valid_ = false;
    last_nearest_dist_ = 0.0;
    prev_valid_ = false;
    alignment_heading_valid_ = false;
    alignment_settled_ = 0.0;
    if (traj_.empty()) {
        motion_valid_ = false;
        filtered_yaw_rate_ = previous_yaw_command_ = previous_forward_command_ = 0.0;
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

void TrajectoryTracker::advance_to_nearest(double px, double py)
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
    progress_s_ = best_s;
    while (progress_idx_ + 1 < traj_.size() &&
           traj_[progress_idx_ + 1].s <= progress_s_ + 1e-9) ++progress_idx_;
    last_nearest_dist_ = best_distance;
    progress_position_ = cur;
    progress_valid_ = true;
}

Path2 TrajectoryTracker::remaining_path(double px, double py) const
{
    if (traj_.empty()) return {};
    Path2 result{{px, py}};
    const Vec2 projection = sample_at(progress_s_).p;
    if (dist(result.back(), projection) > 1e-6) result.push_back(projection);
    for (const auto& point : traj_) {
        if (point.s > progress_s_ + 1e-9 && dist(result.back(), point.p) > 1e-6)
            result.push_back(point.p);
    }
    return result;
}

VelCmd TrajectoryTracker::update(double px, double py, double yaw,
                                 double v_fwd_est, double v_lat_est,
                                 double goal_tol, double measured_yaw_rate)
{
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
        previous_forward_command_ = previous_yaw_command_ = 0.0;
        aligning_ = false;
        alignment_heading_valid_ = false;
        last_look_ = traj_.back().p;
        cmd.at_goal = true;
        return cmd;
    }

    const double speed = std::hypot(v_fwd_est, v_lat_est);
    size_t corner = next_corner();
    if (corner < traj_.size() && dist(cur, traj_[corner].p) <= std::max(0.08, goal_tol) &&
        speed <= g_.align_stop_speed) {
        passed_corner_s_ = traj_[corner].s;
        progress_s_ = traj_[corner].s;
        progress_idx_ = corner;
        corner = next_corner();
    }
    double look_s = std::min(traj_.back().s, progress_s_ + std::max(0.01, g_.lookahead));
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
        const Vec2 predicted{px + prediction * vx + lateral_lead * path_lateral_velocity * nx,
                             py + prediction * vy + lateral_lead * path_lateral_velocity * ny};
        if (dist(predicted, ref.p) > 1e-6) desired_theta = heading(predicted, ref.p);
    }
    double e_yaw = wrap_pi(desired_theta - yaw);
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
        cmd.v_lat = clamp_abs(-(g_.kp_lat * e_ct + g_.kd_lat * de_ct), g_.max_v_lat) * gate;
        double v = g_.v_max / (1.0 + g_.k_curv * std::abs(ref.kappa)) * gate;
        if (d_goal < g_.endpoint_slow_r) v *= d_goal / g_.endpoint_slow_r;
        if (d_goal >= g_.endpoint_slow_r && gate > 0.5) v = std::max(v, g_.v_min);
        cmd.v_fwd = std::clamp(v, 0.0, g_.v_max);
        return cmd;
    }

    const double stop_angle = std::clamp(g_.stop_align_rad, 0.35, M_PI * 0.5);
    if (!aligning_ && std::abs(e_yaw) >= stop_angle) {
        aligning_ = true;
        alignment_heading_valid_ = false;
        alignment_settled_ = 0.0;
        turn_direction_ = e_yaw >= 0.0 ? 1 : -1;
    }
    if (aligning_ && !alignment_heading_valid_ && speed <= g_.align_stop_speed) {
        alignment_heading_ = desired_theta;
        alignment_heading_valid_ = true;
    }
    if (aligning_ && alignment_heading_valid_) {
        e_yaw = wrap_pi(alignment_heading_ - yaw);
        // The shortest turn changes sign at pi. Keep the chosen turn through
        // that noisy boundary until there is an unambiguous shorter direction.
        if (std::abs(e_yaw) > 2.6 && turn_direction_ * e_yaw < 0.0)
            e_yaw += turn_direction_ * 2.0 * M_PI;
    }

    const double yaw_accel = std::max(1e-3, g_.max_yaw_accel);
    double desired_rate = 0.0;
    if (!aligning_ || alignment_heading_valid_) {
        const double predicted_error = e_yaw - std::max(0.0, g_.prediction_time) * filtered_yaw_rate_;
        const double braking_rate = std::sqrt(2.0 * yaw_accel * std::abs(predicted_error));
        desired_rate = clamp_abs(g_.kp_yaw * predicted_error, braking_rate) -
                       g_.kd_yaw * filtered_yaw_rate_;
        desired_rate = clamp_abs(desired_rate, g_.max_yaw_rate);
    }
    cmd.yaw_rate = previous_yaw_command_ + clamp_abs(desired_rate - previous_yaw_command_, yaw_accel * dt);
    cmd.yaw_rate = clamp_abs(cmd.yaw_rate, g_.max_yaw_rate);
    previous_yaw_command_ = cmd.yaw_rate;

    if (aligning_) {
        const double predicted_error = e_yaw -
            std::max(0.0, g_.prediction_time) * filtered_yaw_rate_;
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
    double v = g_.v_max / (1.0 + g_.k_curv * curvature);
    double stop_distance = d_goal;
    if (corner < traj_.size()) stop_distance = std::min(stop_distance, dist(cur, traj_[corner].p));
    const double slow_radius = std::max(1e-3, g_.endpoint_slow_r);
    v *= std::min(1.0, stop_distance / slow_radius);
    v *= gate / (1.0 + 3.0 * std::max(std::abs(v_lat_est), std::abs(path_lateral_velocity)));
    if (stop_distance >= slow_radius && gate > 0.5)
        v = std::max(v, std::min(g_.v_min, g_.v_max) * gate);
    const double lateral_accel = std::max(1e-3, g_.max_lateral_accel);
    if (curvature > 1e-6) v = std::min(v, std::sqrt(lateral_accel / curvature));
    if (std::abs(filtered_yaw_rate_) > 1e-6)
        v = std::min(v, lateral_accel / std::abs(filtered_yaw_rate_));
    v = std::clamp(v, 0.0, g_.v_max);
    cmd.v_fwd = std::min(v, previous_forward_command_ + std::max(0.0, g_.max_accel) * dt);
    previous_forward_command_ = cmd.v_fwd;
    return cmd;
}

}  // namespace exploration
