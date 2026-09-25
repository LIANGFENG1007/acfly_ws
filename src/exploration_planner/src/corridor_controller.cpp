#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
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
// 线段 a→b 不进入任何点的 r 圈；起点已在某点 r 圈内(贴着门框/墙)时，只允许朝远离它的方向动。
// 纯 pathClear 在贴近时会否决一切纠偏，涡流就把飞机一点点推上去。
bool motionClear(const Path2& points, const Vec2& a, const Vec2& b, double r) {
    if (!finite(a) || !finite(b) || !std::isfinite(r) || r <= 0.0) return false;
    const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    bool observed = false;
    for (const auto& p : points) {
        if (!finite(p)) continue;
        observed = true;
        if (std::hypot(a.x - p.x, a.y - p.y) <= r) {
            if ((a.x - p.x) * dx + (a.y - p.y) * dy < 0.0) return false;
            continue;
        }
        const double u = l2 > 1e-12 ? std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
        if (std::hypot(p.x - a.x - u * dx, p.y - a.y - u * dy) <= r) return false;
    }
    return observed;
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
        !positive({cfg_.gate_align_distance, cfg_.center_kp, cfg_.lateral_max_speed,
                   cfg_.lateral_acceleration, cfg_.center_filter_time, cfg_.entry_center_tolerance,
                   cfg_.lateral_latency, cfg_.lateral_brake_accel,
                   cfg_.smith_time + 1.0, cfg_.wall_brake_accel,
                   p.width_tolerance, p.center_search_range}) ||
        !nonnegative({cfg_.center_kd, cfg_.min_forward_speed, cfg_.wall_latency}) ||
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
    if (cfg_.centerline_mode) {
        // entry 只是探索终点：真正的锁定点等 start() 给出到达位置后再定
        if (!setCenterlineFrame(red_known_ ? red_ : entry)) return false;
    } else {
        perception_.setFrame(entry, h);
    }
    configured_ = true;
    if (phase_ == CorridorPhase::WaitRoute) transition(CorridorPhase::Rotate);
    return true;
}

void CorridorController::start(const Vec2& red, double now)
{
    red_ = red;
    red_known_ = true;
    if (cfg_.centerline_mode && configured_) setCenterlineFrame(red);
    rotation_hold_.begin(red);
    heading_recovery_ = false;
    rotation_done_ = false;
    last_update_ = now;
    transition(configured_ ? CorridorPhase::Rotate : CorridorPhase::WaitRoute);
}

void CorridorController::observe(const Path2& points, double stamp)
{
    if (!std::isfinite(stamp)) return;
    if (stamp < cloud_time_) {
        clouds_.clear();
        visibility_points_.clear();
        evidence_time_ = -1e9;
    }
    cloud_time_ = stamp;
    if (!points.empty()) {
        visibility_points_ = points;
        has_sensor_origin_ = false;
        evidence_time_ = stamp;
        ++cloud_sequence_;  // Empty packets are not independent door confirmations.
    }
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
    // Sparse scan packets may alternate between wall hits and no height-slice
    // hits. Keep only the explicitly bounded measured window, never infer free
    // space from an empty packet or extend the age of its last valid rays.
    for (const auto& item : voxels) points_.push_back(item.second);
}

void CorridorController::observe(const Path2& points, double stamp, const Vec2& sensor_origin)
{
    observe(points, stamp);
    if (!points.empty()) {
        sensor_origin_ = sensor_origin;
        has_sensor_origin_ = true;
    }
}

void CorridorController::transition(CorridorPhase phase)
{
    if (phase == phase_) return;
    phase_ = phase;
    settled_since_ = -1.0;
    heading_recovery_ = false;
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
    rotation_hold_.stop();
    heading_hold_.stop();
    CorridorCommand cmd;
    cmd.target = position;
    cmd.status = reason;
    return cmd;
}

CorridorCommand CorridorController::rotateAt(PositionHold& hold, const Vec2& position,
                                              double yaw, double vf, double vl, double dt)
{
    auto correction = hold.update(position, yaw, vf, vl, cfg_.position_kp,
                                  cfg_.velocity_kd, cfg_.align_speed, cfg_.acceleration, dt);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const Vec2 world{c * correction.x - s * correction.y, s * correction.x + c * correction.y};
    const double speed = std::hypot(world.x, world.y);
    const double horizon = std::max(0.5, speed / (2.0 * cfg_.acceleration));
    const Vec2 projected{position.x + world.x * horizon, position.y + world.y * horizon};
    // Rotation predates corridor cloud acquisition. Available returns can veto
    // XY correction without cancelling yaw; empty scans never authorize entry.
    if (!points_.empty() && !motionClear(points_, position, projected, cfg_.perception.robot_width * 0.5)) {
        correction = {};
        hold.stop();
    }
    CorridorCommand cmd;
    cmd.forward = correction.x;
    cmd.lateral = correction.y;
    cmd.target = hold.anchor();
    cmd.path = {position, cmd.target};
    const double desired = std::clamp(cfg_.yaw_kp * angle(cfg_.initial_yaw - yaw),
                                      -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
    const double step = cfg_.yaw_accel * dt;
    cmd.yaw_rate = std::clamp(desired, previous_yaw_rate_ - step, previous_yaw_rate_ + step);
    previous_yaw_rate_ = cmd.yaw_rate;
    previous_velocity_ = {c * cmd.forward - s * cmd.lateral, s * cmd.forward + c * cmd.lateral};
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
    if (!heading_recovery_ && std::fabs(error) > cfg_.heading_stop) {
        heading_hold_.begin(position);
        heading_recovery_ = true;
    }
    if (heading_recovery_) {
        if (std::fabs(error) > cfg_.heading_tolerance) {
            auto cmd = rotateAt(heading_hold_, position, yaw, vf, vl, dt);
            cmd.status = "Restoring corridor heading with XY hold";
            return cmd;
        }
        heading_recovery_ = false;
    }
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
    const double prior_yaw_command = previous_yaw_rate_;
    auto cmd = updateMotion(position, yaw, vf, vl, now, pose_fresh);
    const double evidence_timeout = cfg_.cloud_window > 0.0
        ? std::min(cfg_.cloud_window, cfg_.cloud_timeout) : cfg_.cloud_timeout;
    // With fresh measurements, waiting for wall/door evidence still holds the
    // first waiting position. It must not silently hand XY to the flight controller.
    // Missing pose/cloud data and actual occupied corrections remain stop cases.
    const bool can_hold = pose_fresh && finite(position) && std::isfinite(yaw) &&
        std::isfinite(vf) && std::isfinite(vl) && std::isfinite(now) &&
        now >= evidence_time_ && now - evidence_time_ <= evidence_timeout && !points_.empty() &&
        phase_ != CorridorPhase::Idle && phase_ != CorridorPhase::WaitRoute &&
        !cmd.finished && cmd.path.empty();
    if (can_hold) {
        if (!waiting_for_evidence_) waiting_hold_.begin(position);
        waiting_for_evidence_ = true;
        previous_yaw_rate_ = prior_yaw_command;
        const std::string status = cmd.status;
        cmd = rotateAt(waiting_hold_, position, yaw, vf, vl, dt);
        cmd.status = status;
    } else {
        waiting_for_evidence_ = false;
    }
    if (cfg_.smith_time > 0.0 && std::isfinite(now) && std::isfinite(yaw)) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        if (!command_history_.empty() && now < command_history_.back().time) command_history_.clear();
        command_history_.push_back({now, {c * cmd.forward - s * cmd.lateral, s * cmd.forward + c * cmd.lateral}});
        while (command_history_.size() > 1 && command_history_[1].time <= now - cfg_.smith_time)
            command_history_.pop_front();
    }
    return cmd;
}

Vec2 CorridorController::smithDisplacement(double now) const
{
    Vec2 d{};
    for (std::size_t i = 0; i < command_history_.size(); ++i) {
        const double begin = std::max(command_history_[i].time, now - cfg_.smith_time);
        const double end = i + 1 < command_history_.size() ? command_history_[i + 1].time : now;
        if (end <= begin) continue;
        d.x += command_history_[i].velocity.x * (end - begin);
        d.y += command_history_[i].velocity.y * (end - begin);
    }
    return d;
}

CorridorCommand CorridorController::updateMotion(const Vec2& position, double yaw, double vf,
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
    if (phase_ == CorridorPhase::Rotate) {
        // Show measured walls and openings while yaw is in progress too.
        if (cloud_sequence_ != processed_sequence_ && now >= evidence_time_ &&
            now - evidence_time_ <= cfg_.cloud_timeout && !points_.empty()) {
            processed_sequence_ = cloud_sequence_;
            if (cfg_.centerline_mode) {
                // 转头/降高期间就开始找墙对，横移开始时中线往往已锁定
                observation_ = analyzeCenterline(position, passed_s_);
                updateCenter(observation_, now, position);
            } else {
                observation_ = perception_.analyze(points_, position, passed_s_, &visibility_points_,
                    has_sensor_origin_ ? &sensor_origin_ : nullptr);
            }
        }
        auto cmd = rotateAt(rotation_hold_, position, yaw, vf, vl, dt);
        cmd.status = "Rotate to entry heading with XY hold";
        const double error = angle(cfg_.initial_yaw - yaw);
        if (!rotation_done_ && stable(std::fabs(error) <= cfg_.heading_tolerance, now)) rotation_done_ = true;
        if (rotation_done_) {
            // 必须先转头、再降高：转好后仍在红点 XY 定点并保持航向，等降到走廊高度才横移进入口。
            if (!cfg_.descend_before_entry || altitude_ready_) {
                transition(CorridorPhase::Entry);
                cmd.status = "Heading aligned: entering corridor";
            } else {
                cmd.status = "Heading aligned: descending to corridor height with XY hold";
            }
        }
        return cmd;
    }
    const double evidence_timeout = cfg_.cloud_window > 0.0
        ? std::min(cfg_.cloud_window, cfg_.cloud_timeout) : cfg_.cloud_timeout;
    if (now - evidence_time_ > evidence_timeout || now < evidence_time_ || points_.empty()) {
        confirmations_ = 0;
        settled_since_ = -1.0;
        return hold(position, "Waiting for fresh corridor cloud");
    }
    if (cfg_.centerline_mode) return updateCenterline(position, yaw, vf, vl, now, dt);
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
                auto cmd = translate(position, yaw, h_, cfg_.align_speed, vf, vl, dt);
                if (stable(!cmd.path.empty() && distance(position, h_) <= cfg_.point_tolerance &&
                    std::fabs(angle(cfg_.initial_yaw - yaw)) <= cfg_.heading_tolerance, now))
                    transition(CorridorPhase::Done);
                if (cmd.status.empty()) cmd.status = "Holding position at H";
                return cmd;
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

// ═══════════════ 按宽度对中模式 ═══════════════
// 坐标系：原点 = 锁定点(到达探索终点的位置)，s 轴 = initial_yaw 方向，t = 横向。
// 隧道中线 t 完全由雷达墙对测量；H 只给出隧道在哪一侧(reference_t_)和纵向终点(length)。
bool CorridorController::setCenterlineFrame(const Vec2& reference)
{
    const Vec2 axis{std::cos(cfg_.initial_yaw), std::sin(cfg_.initial_yaw)};
    const double length = (h_.x - reference.x) * axis.x + (h_.y - reference.y) * axis.y;
    if (!finite(reference) || !finite(h_) || length < 0.5) return false;
    entry_ = reference;
    perception_.setFrame(reference, {reference.x + axis.x * length, reference.y + axis.y * length});
    reference_t_ = perception_.lateral(h_);
    center_locked_ = false;
    center_votes_ = center_outliers_ = 0;
    center_t_ = reference_t_;
    door_valid_ = door_passable_ = door_zone_ = false;
    width_est_ = cfg_.perception.corridor_width;
    return true;
}

CorridorObservation CorridorController::analyzeCenterline(const Vec2& position, double minimum_gate_s) const
{
    // 未锁定：自由搜索一对符合宽度的墙(取离 H 线最近的一对)；锁定后在当前中线 ± 半宽附近跟踪两墙
    return perception_.analyze(points_, position, minimum_gate_s, &visibility_points_,
        has_sensor_origin_ ? &sensor_origin_ : nullptr,
        center_locked_ ? center_t_ : reference_t_, !center_locked_);
}

void CorridorController::updateCenter(const CorridorObservation& observed, double now, const Vec2& position)
{
    if (!observed.center_valid) return;
    double measured = observed.center_t;
    if (observed.center_from_pair) {
        // 实测宽度慢低通；只在锁定后且飞机在隧道内时更新(隧道外看到的是墙外表面，会多一个墙厚)
        if (center_locked_ && std::fabs(perception_.lateral(position) - center_t_) < 0.5 * width_est_)
            width_est_ += 0.1 * (observed.measured_width - width_est_);
    } else if (center_locked_ && observed.right_valid && !observed.left_valid) {
        measured = observed.right_wall + 0.5 * width_est_;           // 只见一侧墙：用实测宽度，不用设定值
    } else if (center_locked_ && observed.left_valid && !observed.right_valid) {
        measured = observed.left_wall - 0.5 * width_est_;
    }
    if (!center_locked_) {
        // 锁定必须是一对符合宽度的墙，且连续 confirm_frames 帧落在同一位置
        if (!observed.center_from_pair) return;
        if (center_votes_ > 0 && std::fabs(measured - center_candidate_) <= cfg_.gate_association) ++center_votes_;
        else center_votes_ = 1;
        center_candidate_ = measured;
        if (center_votes_ >= cfg_.confirm_frames) {
            center_locked_ = true;
            center_t_ = measured;
            center_time_ = now;
            center_outliers_ = 0;
        }
        return;
    }
    // 跟踪：单帧跳变过大视为误检；连续 10 帧都对不上说明锁错了，回到自由搜索
    if (std::fabs(measured - center_t_) > std::max(cfg_.perception.width_tolerance, 2.0 * cfg_.gate_association)) {
        if (++center_outliers_ >= 10) {
            center_locked_ = false;
            center_votes_ = center_outliers_ = 0;
        }
        return;
    }
    center_outliers_ = 0;
    const double dt = std::clamp(now - center_time_, 0.0, 0.5);
    center_t_ += dt / (cfg_.center_filter_time + dt) * (measured - center_t_);
    center_time_ = now;
}

void CorridorController::updateDoor(const Vec2& position)
{
    const double s = perception_.longitudinal(position);
    const double radius = cfg_.perception.robot_width * 0.5;
    const double resolution = cfg_.perception.cell_size;
    const auto same_door = [&](const CorridorObservation& o) {
        return o.gate_observed && std::fabs(o.gate_s - door_.gate_s) <= std::max(0.30, door_.gate_depth);
    };
    // 当前门永远是最近的未通过门：隧道外先看到远门、后看到近门时要切过来重新确认
    if (observation_.gate_observed && observation_.gate_s > passed_s_ &&
        (!door_valid_ || observation_.gate_s < door_.gate_s - std::max(0.50, door_.gate_depth))) {
        door_valid_ = true;
        door_passable_ = door_zone_ = false;
        door_ = observation_;
        confirmations_ = 0;
        candidate_ = {};
    }
    if (door_valid_ && same_door(observation_)) {
        if (observation_.gate_passable) {
            if (candidate_.gate_passable &&
                distance(candidate_.gate_center, observation_.gate_center) <= cfg_.gate_association)
                ++confirmations_;
            else confirmations_ = 1;
            candidate_ = observation_;
            // 确认后仍用最新观测更新门中心(涡流下飞机与门的相对位置要实时)
            if (door_passable_ || confirmations_ >= cfg_.confirm_frames) {
                door_passable_ = true;
                door_ = observation_;
            }
        } else if (!door_passable_) {
            door_ = observation_;
        }
    }
    // 机身越过当前门后再看下一扇门：门间距短时，进入下一扇门的对准区就提前交接
    next_door_ = {};
    if (door_valid_) {
        const double back = door_.gate_s + door_.gate_depth * 0.5;
        if (s >= back + radius + resolution)
            next_door_ = analyzeCenterline(position, back + std::max(resolution, cfg_.perception.gate_cluster_depth));
    }
}

double CorridorController::alignForwardLimit(double lateral_error, double lateral_speed,
                                             double remaining) const
{
    if (remaining <= 0.0) return 0.0;
    const double stop_time = std::fabs(lateral_speed) / cfg_.lateral_acceleration;
    const double align_time = std::fabs(lateral_error) / cfg_.lateral_max_speed + stop_time +
        cfg_.continuous_time_margin;
    return std::min(remaining / std::max(0.05, align_time), std::sqrt(2.0 * cfg_.acceleration * remaining));
}

CorridorCommand CorridorController::followLine(const Vec2& position, double yaw, double s_target,
                                              double t_ref, double forward_cap, double lateral_cap,
                                              bool allow_reverse, double forward_floor, double dt)
{
    const double c = std::cos(yaw), sn = std::sin(yaw);
    const double error = angle(cfg_.initial_yaw - yaw);
    if (!heading_recovery_ && std::fabs(error) > cfg_.heading_stop) {
        heading_hold_.begin(position);
        heading_recovery_ = true;
    }
    if (heading_recovery_) {
        if (std::fabs(error) > cfg_.heading_tolerance) {
            const double vf = c * filtered_velocity_.x + sn * filtered_velocity_.y;
            const double vl = -sn * filtered_velocity_.x + c * filtered_velocity_.y;
            auto cmd = rotateAt(heading_hold_, position, yaw, vf, vl, dt);
            cmd.status = "Restoring corridor heading with XY hold";
            return cmd;
        }
        heading_recovery_ = false;
    }
    const Vec2 origin = perception_.toWorld(0.0, 0.0);
    const Vec2 unit = perception_.toWorld(1.0, 0.0);
    const Vec2 axis{unit.x - origin.x, unit.y - origin.y};
    const Vec2 normal{-axis.y, axis.x};
    const double s = perception_.longitudinal(position);
    const double t = perception_.lateral(position);
    const double v_s = axis.x * filtered_velocity_.x + axis.y * filtered_velocity_.y;
    const double v_t = normal.x * filtered_velocity_.x + normal.y * filtered_velocity_.y;
    // 纵向：PD 追前瞻点；离停止点还远时不低于 forward_floor(涡流里飞太慢反而晃)
    const double ds = s_target - s;
    double command_s = std::clamp(cfg_.position_kp * ds - cfg_.velocity_kd * v_s,
                                  allow_reverse ? -forward_cap : 0.0, forward_cap);
    const double floor = std::min(forward_floor, forward_cap);
    if (floor > 0.0 && ds > cfg_.point_tolerance + floor * floor / (2.0 * cfg_.acceleration))
        command_s = std::max(command_s, floor);
    // 横向：对中 PD，无死区、单独限幅，前进速度不挤占纠偏能力。误差用 Smith 预测位置
    //   (实测位置 + 已发出未生效的指令积分)，大延迟下不过冲、涡流里不来回晃
    double t_predicted = t;
    if (cfg_.smith_time > 0.0) {
        const Vec2 ahead = smithDisplacement(last_update_);
        t_predicted += normal.x * ahead.x + normal.y * ahead.y;
    }
    const double lateral_error = t_ref - t_predicted;
    double command_t = std::clamp(cfg_.center_kp * lateral_error - cfg_.center_kd * v_t, -lateral_cap, lateral_cap);
    // 大横移按"响应延迟 + 刹车距离"包络限速：v·T + v²/(2a) ≤ |误差|，提前刹车不过冲
    //   (门洞中心常贴着隧道墙，余量只有几厘米；仿真 r284 横移 0.33 m 时滑过 5 cm 擦墙)。小误差时 PD 更小，不受影响。
    if (command_t * lateral_error > 0.0) {
        const double a = cfg_.lateral_brake_accel, latency = cfg_.lateral_latency;
        const double envelope = a * (-latency + std::sqrt(latency * latency + 2.0 * std::fabs(lateral_error) / a));
        command_t = std::copysign(std::min(std::fabs(command_t), envelope), command_t);
    }
    // 净距包络：朝障碍运动时 v·T + v²/(2a) ≤ 机身到最近点的净距。飞机刹停要 1 s 以上(Gazebo r285
    //   横移滑过 5 cm 擦墙；慢响应下前方 0.25 m 的碰撞检查来不及停在门框前)。
    //   横向：只看机身纵向范围内的点(贴墙门洞只余几厘米时慢慢靠过去；离墙 0.3 m 以上仍够抗涡流)；
    //   纵向：只看机身横向范围内的点(对准门洞时门框不在范围内不减速；偏得过不去就提前减速停在门框前)。
    //   远离障碍的运动不受限。
    if (cfg_.wall_latency > 0.0) {
        const double body = cfg_.perception.robot_width * 0.5;
        const double a = cfg_.wall_brake_accel, latency = cfg_.wall_latency;
        const auto allowed = [&](double free) {
            return a * (-latency + std::sqrt(latency * latency + 2.0 * std::max(0.0, free) / a));
        };
        const double direction = command_t >= 0.0 ? 1.0 : -1.0;
        double side_free = std::numeric_limits<double>::infinity();
        double front_free = std::numeric_limits<double>::infinity();
        for (const auto& p : points_) {
            const double ds = perception_.longitudinal(p) - s;
            const double dt_side = perception_.lateral(p) - t;
            if (std::fabs(ds) < body && dt_side * direction > 0.0)
                side_free = std::min(side_free, dt_side * direction - std::sqrt(body * body - ds * ds));
            if (std::fabs(dt_side) < body && ds > 0.0)
                front_free = std::min(front_free, ds - std::sqrt(body * body - dt_side * dt_side));
        }
        if (std::fabs(command_t) > 1e-6 && std::isfinite(side_free))
            command_t = direction * std::min(std::fabs(command_t), allowed(side_free));
        if (command_s > 0.0 && std::isfinite(front_free))
            command_s = std::min(command_s, allowed(front_free - cfg_.perception.cell_size));
    }
    const double ratio = std::clamp((std::fabs(error) - cfg_.heading_tolerance) /
        (cfg_.heading_stop - cfg_.heading_tolerance), 0.0, 1.0);
    const double head_gate = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio);
    command_s *= head_gate;
    command_t *= head_gate;
    const double previous_s = axis.x * previous_velocity_.x + axis.y * previous_velocity_.y;
    const double previous_t = normal.x * previous_velocity_.x + normal.y * previous_velocity_.y;
    command_s = std::clamp(command_s, previous_s - cfg_.acceleration * dt, previous_s + cfg_.acceleration * dt);
    command_t = std::clamp(command_t, previous_t - cfg_.lateral_acceleration * dt,
                           previous_t + cfg_.lateral_acceleration * dt);
    if (head_gate == 0.0) command_s = command_t = 0.0;
    const double radius = cfg_.perception.robot_width * 0.5;
    const Vec2 target = perception_.toWorld(s_target, t_ref);
    const double d = distance(position, target);
    const double horizon = std::min(d, std::max(cfg_.lookahead, std::hypot(v_s, v_t) * 0.5));
    const Vec2 carrot = d > 1e-6 ? Vec2{position.x + (target.x - position.x) * horizon / d,
                                      position.y + (target.y - position.y) * horizon / d} : position;
    std::string blocked_status;
    if (!motionClear(points_, position, carrot, radius)) {
        // 前方被挡(多半是还没对准的门框)：纵向停住，横向继续往目标线纠偏，不整体悬停
        command_s = std::min(command_s, 0.0);
        blocked_status = "Forward blocked: stopping and correcting sideways";
    }
    const Vec2 velocity{axis.x * command_s + normal.x * command_t, axis.y * command_s + normal.y * command_t};
    const double speed = std::hypot(velocity.x, velocity.y);
    if (speed > 1e-6) {
        const double travel = std::max(speed * 0.5, speed * speed / (2.0 * cfg_.acceleration));
        const Vec2 projected{position.x + velocity.x * travel / speed, position.y + velocity.y * travel / speed};
        if (!motionClear(points_, position, projected, radius))
            return hold(position, "Braking: commanded motion is blocked");
    }
    CorridorCommand cmd;
    cmd.status = blocked_status;
    cmd.forward = c * velocity.x + sn * velocity.y;
    cmd.lateral = -sn * velocity.x + c * velocity.y;
    const double desired_yaw = std::clamp(cfg_.yaw_kp * error, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
    const double yaw_step = cfg_.yaw_accel * dt;
    cmd.yaw_rate = std::clamp(desired_yaw, previous_yaw_rate_ - yaw_step, previous_yaw_rate_ + yaw_step);
    cmd.target = target;
    cmd.path = {position, target};
    previous_velocity_ = velocity;
    previous_yaw_rate_ = cmd.yaw_rate;
    return cmd;
}

CorridorCommand CorridorController::updateCenterline(const Vec2& position, double yaw, double vf,
                                                     double vl, double now, double dt)
{
    const double s = perception_.longitudinal(position);
    const double t = perception_.lateral(position);
    const double radius = cfg_.perception.robot_width * 0.5;
    const double resolution = cfg_.perception.cell_size;
    const Vec2 origin = perception_.toWorld(0.0, 0.0);
    const Vec2 unit = perception_.toWorld(1.0, 0.0);
    const Vec2 normal{-(unit.y - origin.y), unit.x - origin.x};
    const double lateral_speed = normal.x * filtered_velocity_.x + normal.y * filtered_velocity_.y;
    if (processed_sequence_ != cloud_sequence_) {
        processed_sequence_ = cloud_sequence_;
        observation_ = analyzeCenterline(position, passed_s_);
        updateCenter(observation_, now, position);
        updateDoor(position);
    }
    // 门通过：离开门后沿 gate_align_distance，或机身已过门且下一扇门的对准区已开始
    if (door_valid_) {
        const double back = door_.gate_s + door_.gate_depth * 0.5;
        const bool body_clear = s >= back + radius + resolution;
        const bool next_zone = next_door_.gate_observed &&
            s >= next_door_.gate_s - next_door_.gate_depth * 0.5 - cfg_.gate_align_distance;
        const bool near_h = perception_.length() - s <= cfg_.gate_align_distance;
        if (s >= back + cfg_.gate_align_distance || (body_clear && (next_zone || near_h))) {
            last_gate_back_ = back;
            passed_s_ = back + resolution;
            ++gates_passed_;
            door_valid_ = door_passable_ = door_zone_ = false;
            confirmations_ = 0;
            candidate_ = {};
            if (next_door_.gate_observed && next_door_.gate_s > passed_s_) {
                door_valid_ = true;
                door_ = next_door_;
                if (next_door_.gate_passable) { confirmations_ = 1; candidate_ = next_door_; }
            }
            next_door_ = {};
        }
    }
    gate_locked_ = door_valid_ && door_passable_;
    locked_gate_ = door_;

    if (phase_ == CorridorPhase::Entry) {
        // 纵向锁在 s=0(轴沿 SLAM Y 时即锁 Y)，只沿横向平移到雷达测得的中线
        const double lateral_cap = std::min(cfg_.entry_speed, cfg_.lateral_max_speed);
        if (!center_locked_) {
            if (std::fabs(reference_t_ - t) <= cfg_.entry_center_tolerance)
                return hold(position, "Centering: waiting for a wall pair matching the corridor width");
            auto cmd = followLine(position, yaw, 0.0, reference_t_, cfg_.entry_speed, 0.5 * lateral_cap,
                                  true, 0.0, dt);
            if (cmd.status.empty()) cmd.status = "Centering: searching for corridor walls toward H";
            return cmd;
        }
        auto cmd = followLine(position, yaw, 0.0, center_t_, cfg_.entry_speed, lateral_cap, true, 0.0, dt);
        if (cmd.path.empty() || !stable(std::fabs(t - center_t_) <= cfg_.entry_center_tolerance, now)) {
            if (cmd.status.empty()) cmd.status = "Centering: Y locked, translating to corridor center";
            return cmd;
        }
        transition(CorridorPhase::Search);
    }

    if (phase_ == CorridorPhase::Search) {
        if (!center_locked_) return hold(position, "Waiting for a wall pair matching the corridor width");
        double s_target = std::min(perception_.length(), s + cfg_.moving_lookahead);
        double t_ref = center_t_;
        double forward_cap = cfg_.cruise_speed;
        double floor = cfg_.min_forward_speed;
        std::string status = "Holding corridor center";
        if (door_valid_) {
            const double front = door_.gate_s - door_.gate_depth * 0.5;
            const double stop_s = front - radius - resolution;
            if (door_passable_) {
                // 只在离门前沿 gate_align_distance 内对准门中心；过门后到后沿同样距离再回中线。
                //   进入后保持(门位置每帧更新会抖几厘米，边界附近不能在中线/门中心之间来回切)
                if (!door_zone_ && s >= front - cfg_.gate_align_distance) door_zone_ = true;
                if (door_zone_) {
                    t_ref = 0.5 * (door_.gap_left + door_.gap_right);
                    forward_cap = cfg_.gate_speed;
                    status = "Aligning to door center";
                    const double error = t_ref - t;
                    const double margin = 0.5 * (door_.gap_width - cfg_.perception.robot_width);
                    const double remaining = stop_s - s;
                    if (std::fabs(error) > 0.5 * margin) {
                        // 对不准就按剩余距离限速(不保底)；物理上过不去时降到停，只横向纠偏
                        const double limit = remaining > 0.0 ? alignForwardLimit(error, lateral_speed, remaining)
                            : (std::fabs(error) > margin ? 0.0 : forward_cap);
                        forward_cap = std::min(forward_cap, limit);
                        floor = 0.0;
                    }
                }
            } else {
                // 门洞未确认：停在门前等证据，不冲门
                s_target = std::min(s_target, stop_s);
                floor = 0.0;
                status = door_.reason == "gate_closed_or_too_narrow"
                    ? "Door closed or measured gap narrower than aircraft"
                    : "Waiting for door opening evidence: " + door_.reason;
                if (s >= stop_s - cfg_.point_tolerance) return hold(position, status);
            }
        }
        if (door_valid_ || perception_.length() - s > cfg_.point_tolerance) {
            auto cmd = followLine(position, yaw, s_target, t_ref, forward_cap, cfg_.lateral_max_speed,
                                  false, floor, dt);
            if (cmd.status.empty()) cmd.status = status;
            return cmd;
        }
        transition(CorridorPhase::Finish);
    }

    if (phase_ == CorridorPhase::Finish) {
        // H 的纵向 + 雷达中线的横向
        const Vec2 goal = perception_.toWorld(perception_.length(), center_t_);
        auto cmd = translate(position, yaw, goal, cfg_.align_speed, vf, vl, dt);
        if (stable(!cmd.path.empty() && distance(position, goal) <= cfg_.point_tolerance &&
                   std::fabs(angle(cfg_.initial_yaw - yaw)) <= cfg_.heading_tolerance, now))
            transition(CorridorPhase::Done);
        if (cmd.status.empty()) cmd.status = "Holding position at H";
        return cmd;
    }
    return hold(position, "Waiting for next route segment");
}

}  // namespace exploration
