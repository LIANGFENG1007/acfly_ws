#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace exploration {
namespace {
double distance(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
bool finite(const Vec2& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
bool positive(std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v) && v > 0.0; });
}
bool nonnegative(std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v) && v >= 0.0; });
}
}

CorridorController::CorridorController(const CorridorConfig& config)
    : cfg_(config), perception_(config.perception)
{
    const auto& p = cfg_.perception;
    if (!positive({cfg_.entry_speed, cfg_.cruise_speed, cfg_.gate_speed, cfg_.align_speed,
                   cfg_.yaw_kp, cfg_.max_yaw_rate, cfg_.yaw_accel, cfg_.acceleration,
                   cfg_.position_kp, cfg_.velocity_kd, cfg_.velocity_filter_tau, cfg_.arrival_hysteresis,
                   cfg_.heading_tolerance, cfg_.heading_stop, cfg_.point_tolerance,
                   cfg_.stop_speed, cfg_.lookahead, cfg_.approach_distance, cfg_.exit_distance,
                   cfg_.moving_lookahead, cfg_.center_prediction_time,
                   cfg_.center_tolerance, cfg_.gate_association, cfg_.cloud_timeout,
                   cfg_.pose_timeout, p.corridor_width, p.robot_width, p.cell_size, p.lookahead,
                   p.wall_min_span, p.max_wall_gap, p.gate_min_span, p.gate_cluster_depth,
                   p.gate_max_depth, p.surface_sample_gap, p.min_observed_ahead}) ||
        !nonnegative({cfg_.settle_time, cfg_.cloud_window, cfg_.continuous_center_reserve,
                      cfg_.continuous_time_margin, p.minimum_gap_extra, p.lookbehind,
                      p.wall_search_tolerance, p.wall_exclusion_band, p.endpoint_exclusion}) ||
        !std::isfinite(cfg_.initial_yaw) || cfg_.confirm_frames < 1 || p.wall_min_points < 2 ||
        p.gate_min_points < 2 || p.gate_min_clear_rays < 1 || p.corridor_width <= p.robot_width ||
        cfg_.heading_stop <= cfg_.heading_tolerance || cfg_.arrival_hysteresis < 1.0) {
        throw std::invalid_argument("Invalid corridor geometry, speed, tolerance, or timing parameter");
    }
}

void CorridorController::reset()
{
    *this = CorridorController(cfg_);
}

bool CorridorController::startedMoving() const
{
    return phase_ != CorridorPhase::Idle && phase_ != CorridorPhase::WaitRoute;
}

bool CorridorController::configure(const Vec2& entry, const Vec2& h)
{
    if (startedMoving() || !finite(entry) || !finite(h) || distance(entry, h) < 0.5)
        return false;
    entry_ = entry;
    h_ = h;
    perception_.setFrame(entry, h);
    configured_ = true;
    if (phase_ == CorridorPhase::WaitRoute) transition(CorridorPhase::Rotate);
    return true;
}

void CorridorController::start(const Vec2& red, double now)
{
    red_ = red;
    last_update_ = now;
    transition(configured_ ? CorridorPhase::Rotate : CorridorPhase::WaitRoute);
}

void CorridorController::observe(const Path2& points, double stamp)
{
    if (!std::isfinite(stamp)) return;
    visibility_points_ = points;
    has_sensor_origin_ = false;
    if (stamp < cloud_time_) clouds_.clear();
    cloud_time_ = stamp;
    ++cloud_sequence_;
    clouds_.push_back({stamp, points});
    while (!clouds_.empty() && stamp - clouds_.front().stamp > cfg_.cloud_window)
        clouds_.pop_front();
    // Retain measured coordinates, not voxel centers: no geometric inflation.
    std::map<std::pair<long long, long long>, Vec2> voxels;
    const double cell = std::max(0.01, cfg_.perception.cell_size * 0.5);
    for (const auto& cloud : clouds_) {
        for (const auto& p : cloud.points) {
            if (!finite(p)) continue;
            voxels[{static_cast<long long>(std::floor(p.x / cell)),
                    static_cast<long long>(std::floor(p.y / cell))}] = p;
        }
    }
    points_.clear();
    if (points.empty()) return;  // An empty current frame is not free-space evidence.
    for (const auto& item : voxels) points_.push_back(item.second);
}

void CorridorController::observe(const Path2& points, double stamp, const Vec2& sensor_origin)
{
    observe(points, stamp);
    sensor_origin_ = sensor_origin;
    has_sensor_origin_ = true;
}

void CorridorController::transition(CorridorPhase phase)
{
    if (phase == phase_) return;
    phase_ = phase;
    settled_since_ = -1.0;
}

bool CorridorController::stable(bool condition, double now)
{
    if (!condition) { settled_since_ = -1.0; return false; }
    if (settled_since_ < 0.0) settled_since_ = now;
    return now - settled_since_ >= cfg_.settle_time;
}

CorridorCommand CorridorController::hold(const Vec2& position, const std::string& reason)
{
    previous_velocity_ = {};
    previous_yaw_rate_ = 0.0;
    CorridorCommand cmd;
    cmd.target = position;
    cmd.status = reason;
    return cmd;
}

CorridorCommand CorridorController::enter(const Vec2& position, double yaw,
                                         double vf, double vl, double dt)
{
    auto cmd = translate(position, yaw, entry_, cfg_.entry_speed, vf, vl, dt);
    if (cmd.status.empty()) cmd.status = "Translating to entry with fixed heading";
    return cmd;
}

CorridorCommand CorridorController::translate(const Vec2& position, double yaw,
                                             const Vec2& target, double speed,
                                             double vf, double vl, double dt,
                                             double longitudinal_limit)
{
    const double error = angle(cfg_.initial_yaw - yaw);
    if (std::fabs(error) > cfg_.heading_stop && std::hypot(vf, vl) > cfg_.stop_speed)
        return hold(position, "Braking to hold corridor heading");
    const double d = distance(position, target);
    const double deadband = std::min(cfg_.point_tolerance, cfg_.center_tolerance) * 0.25;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const Vec2 measured{c * vf - s * vl, s * vf + c * vl};
    const double error_scale = d > deadband ? (d - deadband) / d : 0.0;
    Vec2 desired{cfg_.position_kp * (target.x - position.x) * error_scale - cfg_.velocity_kd * measured.x,
                 cfg_.position_kp * (target.y - position.y) * error_scale - cfg_.velocity_kd * measured.y};
    const bool forward_corridor = phase_ == CorridorPhase::Search ||
        phase_ == CorridorPhase::Approach || phase_ == CorridorPhase::Cross;
    const double route_length = distance(entry_, h_);
    const Vec2 axis{(h_.x - entry_.x) / route_length, (h_.y - entry_.y) / route_length};
    if (forward_corridor && perception_.longitudinal(position) < perception_.length()) {
        const double along = desired.x * axis.x + desired.y * axis.y;
        if (along < 0.0) { desired.x -= along * axis.x; desired.y -= along * axis.y; }
    }
    if (longitudinal_limit >= 0.0) {
        const double along = desired.x * axis.x + desired.y * axis.y;
        const double limited = std::clamp(along, 0.0, longitudinal_limit);
        desired.x += (limited - along) * axis.x;
        desired.y += (limited - along) * axis.y;
    }
    const double desired_speed = std::hypot(desired.x, desired.y);
    if (desired_speed > speed && desired_speed > 1e-9) {
        desired.x *= speed / desired_speed;
        desired.y *= speed / desired_speed;
    }
    // Reduce motion continuously between the alignment and stop thresholds.
    const double ratio = std::clamp((std::fabs(error) - cfg_.heading_tolerance) /
        (cfg_.heading_stop - cfg_.heading_tolerance), 0.0, 1.0);
    const double head_gate = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio);
    desired.x *= head_gate;
    desired.y *= head_gate;
    Vec2 delta{desired.x - previous_velocity_.x, desired.y - previous_velocity_.y};
    const double delta_norm = std::hypot(delta.x, delta.y);
    const double step = cfg_.acceleration * dt;
    if (delta_norm > step) { delta.x *= step / delta_norm; delta.y *= step / delta_norm; }
    Vec2 velocity{previous_velocity_.x + delta.x, previous_velocity_.y + delta.y};
    const double velocity_norm = std::hypot(velocity.x, velocity.y);
    if (velocity_norm > speed && velocity_norm > 1e-9) {
        velocity.x *= speed / velocity_norm;
        velocity.y *= speed / velocity_norm;
    }
    if (head_gate == 0.0) velocity = {};
    const double horizon = std::min(d, std::max(cfg_.lookahead, std::hypot(vf, vl) * 0.5));
    const Vec2 carrot = d > 1e-6 ? Vec2{position.x + (target.x - position.x) * horizon / d,
                                      position.y + (target.y - position.y) * horizon / d} : position;
    if (!CorridorPerception::pathClear(points_, position, carrot, cfg_.perception.robot_width * 0.5))
        return hold(position, "Translation blocked: waiting for a clear centerline");
    const double commanded_speed = std::hypot(velocity.x, velocity.y);
    if (commanded_speed > 1e-6) {
        const double travel = std::max(commanded_speed * 0.5,
                                      commanded_speed * commanded_speed / (2.0 * cfg_.acceleration));
        const Vec2 projected{position.x + velocity.x * travel / commanded_speed,
                             position.y + velocity.y * travel / commanded_speed};
        if (!CorridorPerception::pathClear(points_, position, projected, cfg_.perception.robot_width * 0.5))
            return hold(position, "Braking: commanded motion is blocked");
    }
    CorridorCommand cmd;
    cmd.forward = c * velocity.x + s * velocity.y;
    cmd.lateral = -s * velocity.x + c * velocity.y;
    const double desired_yaw = std::clamp(cfg_.yaw_kp * error, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
    const double yaw_step = cfg_.yaw_accel * dt;
    cmd.yaw_rate = std::clamp(desired_yaw, previous_yaw_rate_ - yaw_step, previous_yaw_rate_ + yaw_step);
    cmd.target = target;
    cmd.path = {position, target};
    previous_velocity_ = velocity;
    previous_yaw_rate_ = cmd.yaw_rate;
    return cmd;
}

double CorridorController::approachForwardLimit(double lateral_error, double lateral_speed,
                                                double remaining_distance) const
{
    if (remaining_distance <= 0.0) return 0.0;
    const double component_speed = cfg_.align_speed / std::sqrt(2.0);
    const double response_gain = cfg_.position_kp / (1.0 + cfg_.velocity_kd);
    const double stop_time = std::fabs(lateral_speed) / cfg_.acceleration;
    const double drift = lateral_speed * lateral_speed / (2.0 * cfg_.acceleration);
    const double remaining_error = std::fabs(lateral_error) + drift;
    const double threshold = cfg_.center_tolerance * 0.75;
    const double saturation_error = component_speed / response_gain;
    // Include both the speed-limited transfer and the slow PD tail. Treat
    // existing lateral momentum as extra work even when it currently points inward.
    double alignment_time = stop_time + cfg_.continuous_time_margin;
    if (remaining_error > saturation_error)
        alignment_time += (remaining_error - saturation_error) / component_speed;
    const double settling_error = std::min(remaining_error, saturation_error);
    if (settling_error > threshold)
        alignment_time += std::log(settling_error / threshold) / response_gain;
    const double time_limit = remaining_distance / std::max(0.02, alignment_time);
    const double braking_limit = std::sqrt(2.0 * cfg_.acceleration * remaining_distance);
    return std::min({component_speed, time_limit, braking_limit});
}

CorridorCommand CorridorController::update(const Vec2& position, double yaw, double vf,
                                           double vl, double now, bool pose_fresh)
{
    const double dt = last_update_ < 0.0 ? 0.02 : std::clamp(now - last_update_, 0.001, 0.10);
    if (now < last_update_) { clouds_.clear(); points_.clear(); cloud_time_ = -1e9; }
    last_update_ = now;
    if (!pose_fresh || !finite(position) || !std::isfinite(yaw) || !std::isfinite(vf) || !std::isfinite(vl) || !std::isfinite(now))
        return hold(position, "Waiting for fresh odometry");
    const double c = std::cos(yaw), sn = std::sin(yaw);
    const Vec2 measured{c * vf - sn * vl, sn * vf + c * vl};
    if (!velocity_valid_) { filtered_velocity_ = measured; velocity_valid_ = true; }
    const double alpha = dt / (cfg_.velocity_filter_tau + dt);
    filtered_velocity_.x += alpha * (measured.x - filtered_velocity_.x);
    filtered_velocity_.y += alpha * (measured.y - filtered_velocity_.y);
    vf = c * filtered_velocity_.x + sn * filtered_velocity_.y;
    vl = -sn * filtered_velocity_.x + c * filtered_velocity_.y;
    if (phase_ == CorridorPhase::Idle) return hold(position, "Exploration");
    if (phase_ == CorridorPhase::WaitRoute) return hold(position, "Waiting for entry / H coordinates");
    if (phase_ == CorridorPhase::Done) {
        auto cmd = hold(h_, "H reached");
        cmd.finished = true;
        return cmd;
    }
    const double speed = std::hypot(vf, vl);
    if (phase_ == CorridorPhase::Rotate) {
        if (speed > cfg_.stop_speed) { settled_since_ = -1.0; return hold(position, "Braking at exploration endpoint"); }
        const double error = angle(cfg_.initial_yaw - yaw);
        if (stable(std::fabs(error) <= cfg_.heading_tolerance, now)) {
            transition(CorridorPhase::Entry);
            return hold(position, "Heading aligned: entering corridor");
        }
        auto cmd = hold(position, "Rotate to entry heading");
        cmd.yaw_rate = std::clamp(cfg_.yaw_kp * error, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
        return cmd;
    }
    if (now - cloud_time_ > cfg_.cloud_timeout || now < cloud_time_ || points_.empty()) {
        confirmations_ = 0;
        settled_since_ = -1.0;
        return hold(position, "Waiting for fresh corridor cloud");
    }
    const double s = perception_.longitudinal(position);
    const double t = perception_.lateral(position);
    const double radius = cfg_.perception.robot_width * 0.5;
    const double resolution = cfg_.perception.cell_size;
    const Vec2 unit = perception_.toWorld(1.0, 0.0);
    const Vec2 axis{unit.x - entry_.x, unit.y - entry_.y};
    const double lateral_speed = -axis.y * filtered_velocity_.x + axis.x * filtered_velocity_.y;
    auto analyze = [&](double minimum) {
        return perception_.analyze(points_, position, minimum, &visibility_points_,
                                    has_sensor_origin_ ? &sensor_origin_ : nullptr);
    };
    auto confirm = [&](const CorridorObservation& observed) {
        if (!observed.gate_passable || observed.gate_s <= passed_s_) return;
        if (candidate_sequence_ == cloud_sequence_) return;
        if (candidate_.gate_passable && distance(candidate_.gate_center, observed.gate_center) <= cfg_.gate_association)
            ++confirmations_;
        else confirmations_ = 1;
        candidate_ = observed;
        candidate_sequence_ = cloud_sequence_;
    };
    const bool new_cloud = processed_sequence_ != cloud_sequence_;
    if (new_cloud) {
        processed_sequence_ = cloud_sequence_;
        observation_ = analyze(passed_s_);
        if (!gate_locked_) {
            if (observation_.gate_passable) confirm(observation_);
            else confirmations_ = 0;
        } else {
            const double back = locked_gate_.gate_s + locked_gate_.gate_depth * 0.5;
            const auto next = analyze(back + std::max(resolution, cfg_.perception.gate_cluster_depth));
            if (next.gate_passable) confirm(next);
            else if (candidate_.gate_s > back) confirmations_ = 0;
        }
    }

    // Intermediate waypoints are passed within this tick; only H has a dwell.
    for (int handoff = 0; handoff < 5; ++handoff) {
        if (phase_ == CorridorPhase::Entry) {
            if (distance(position, entry_) > cfg_.point_tolerance) return enter(position, yaw, vf, vl, dt);
            transition(CorridorPhase::Search);
        }
        if (!gate_locked_ && distance(position, h_) <= cfg_.point_tolerance)
            transition(CorridorPhase::Finish);
        if (phase_ == CorridorPhase::Finish) {
            if (distance(position, h_) > cfg_.point_tolerance * cfg_.arrival_hysteresis)
                transition(CorridorPhase::Search);
            else {
                if (stable(speed <= cfg_.stop_speed, now)) transition(CorridorPhase::Done);
                return hold(position, "Settling at H");
            }
        }
        if (!gate_locked_ && confirmations_ >= cfg_.confirm_frames &&
            candidate_.gate_passable && candidate_.gate_s > passed_s_) {
            locked_gate_ = candidate_;
            gate_locked_ = true;
            transition(CorridorPhase::Approach);
        }

        if (gate_locked_) {
            const double center_t = 0.5 * (locked_gate_.gap_left + locked_gate_.gap_right);
            const double front = locked_gate_.gate_s - locked_gate_.gate_depth * 0.5;
            const double back = locked_gate_.gate_s + locked_gate_.gate_depth * 0.5;
            const double first_clear_s = last_gate_back_ + radius + resolution;
            const double last_clear_s = front - radius - resolution;
            if (first_clear_s > last_clear_s)
                return hold(position, "Insufficient measured space between doors for the aircraft body");
            const double approach = std::clamp(locked_gate_.gate_s - cfg_.approach_distance,
                                               first_clear_s, last_clear_s);
            const double body_clear_s = back + radius + resolution;
            const auto next = analyze(back + std::max(resolution, cfg_.perception.gate_cluster_depth));
            if (phase_ == CorridorPhase::Approach) {
                const bool centered = std::fabs(t - center_t) <= cfg_.center_tolerance &&
                    std::fabs(t + cfg_.center_prediction_time * lateral_speed - center_t) <= cfg_.center_tolerance;
                if (centered && std::fabs(angle(cfg_.initial_yaw - yaw)) <= cfg_.heading_tolerance) {
                    transition(CorridorPhase::Cross);
                    continue;
                }
                if (cfg_.continuous_approach) {
                    const double remaining = last_clear_s - cfg_.continuous_center_reserve - s;
                    const double forward_limit = approachForwardLimit(center_t - t, lateral_speed, remaining);
                    const double target_s = std::min(perception_.length(),
                        std::max(back + cfg_.exit_distance, s + cfg_.moving_lookahead));
                    auto cmd = translate(position, yaw, perception_.toWorld(target_s, center_t),
                                         cfg_.align_speed, vf, vl, dt, forward_limit);
                    if (cmd.status.empty()) cmd.status = "Approaching door center continuously";
                    return cmd;
                }
                // Do not reverse toward a staging point already behind the vehicle.
                const double target_s = std::min(last_clear_s, std::max(s, approach));
                auto cmd = translate(position, yaw, perception_.toWorld(target_s, center_t), cfg_.align_speed, vf, vl, dt);
                if (cmd.status.empty()) cmd.status = "Approaching door center";
                return cmd;
            }
            if (phase_ == CorridorPhase::Cross) {
                double exit_limit = perception_.length();
                if (next.gate_observed) {
                    exit_limit = std::min(exit_limit, next.gate_s - next.gate_depth * 0.5 - radius - resolution);
                    if (exit_limit < body_clear_s)
                        return hold(position, "Insufficient measured space to clear this door before the next");
                } else {
                    exit_limit = std::min(exit_limit, std::max(back + cfg_.exit_distance,
                                                              observation_.observed_until - radius));
                }
                const double requested_exit = back + cfg_.exit_distance;
                // Keep the complete straight exit when space allows. Only a
                // constrained exit needs an early handoff before its stop limit.
                const double exit_handoff = requested_exit < exit_limit - 1e-6
                    ? requested_exit : exit_limit - cfg_.point_tolerance;
                const double pass_s = std::max(body_clear_s, exit_handoff);
                if (s >= pass_s && t >= locked_gate_.gap_right + radius &&
                    t <= locked_gate_.gap_left - radius) {
                    last_gate_back_ = back;
                    passed_s_ = back + resolution;
                    gate_locked_ = false;
                    ++gates_passed_;
                    if (candidate_.gate_s <= passed_s_) confirmations_ = 0;
                    observation_ = analyze(passed_s_);
                    confirm(observation_);
                    transition(CorridorPhase::Search);
                    continue;
                }
                const double target_s = std::min(exit_limit, std::max(back + cfg_.exit_distance, s + cfg_.moving_lookahead));
                auto cmd = translate(position, yaw, perception_.toWorld(target_s, center_t), cfg_.gate_speed, vf, vl, dt);
                if (cmd.status.empty()) cmd.status = "Crossing door";
                return cmd;
            }
        }

        if (!observation_.walls_observed) return hold(position, "Waiting for both corridor walls");
        if (observation_.gate_observed) {
            if (observation_.gate_passable) {
                // Confirmation can run while moving safely toward the pre-door point.
                const double safe_s = observation_.gate_s - observation_.gate_depth * 0.5 - radius - resolution;
                const double target_s = std::min(safe_s, std::max(s, observation_.gate_s - cfg_.approach_distance));
                const double target_t = 0.5 * (observation_.gap_left + observation_.gap_right);
                auto cmd = translate(position, yaw, perception_.toWorld(target_s, target_t), cfg_.align_speed, vf, vl, dt);
                if (cmd.status.empty()) cmd.status = "Confirming door while approaching";
                return cmd;
            }
            return hold(position, observation_.reason == "gate_closed_or_too_narrow"
                ? "Door closed or measured gap narrower than aircraft"
                : "Waiting for door boundary evidence: " + observation_.reason);
        }
        if (!observation_.corridor_clear_observed)
            return hold(position, "Waiting for observed corridor ahead: " + observation_.reason);
        const double supported_end = observation_.observed_until - radius;
        const double safe_end = supported_end + resolution >= perception_.length() ? perception_.length() : supported_end;
        if (safe_end < perception_.length() && safe_end <= s + cfg_.point_tolerance)
            return hold(position, "Waiting for more corridor visibility");
        const Vec2 target = safe_end >= perception_.length() ? h_ : perception_.toWorld(safe_end, 0.0);
        auto cmd = translate(position, yaw, target, cfg_.cruise_speed, vf, vl, dt);
        if (cmd.status.empty()) cmd.status = "Following observed corridor to H";
        return cmd;
    }
    return hold(position, "Waiting for next route segment");
}

}  // namespace exploration
