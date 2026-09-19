#include "exploration_planner/bezier.hpp"
#include "exploration_planner/global_planner.hpp"
#include "exploration_planner/params.hpp"
#include "exploration_planner/trajectory_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace exploration;
constexpr double dt = .02;

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

double distance(Vec2 a, Vec2 b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool contains(const Path2& path, Vec2 point)
{
    return std::any_of(path.begin(), path.end(), [&](Vec2 other) { return distance(point, other) < 1e-9; });
}

GlobalConfig field()
{
    return {-1, -1, 8, 6, .04, .05, .05, .2};
}

Path2 round(const Path2& path, const Obstacles& obstacles = {}, const GlobalConfig& config = field())
{
    CornerRounding options;
    return round_path_corners(path, options, [&](const Path2& candidate) {
        return path_inside_safe_field(candidate, config) &&
            path_clear(candidate.front(), candidate, obstacles, config);
    });
}

TrackerGains gains()
{
    namespace p = exploration::params;
    TrackerGains g{};
    g.v_max = p::V_MAX; g.v_min = p::V_MIN;
    g.k_curv = p::CARLIKE_K_CURV; g.lookahead = p::CARLIKE_LOOKAHEAD;
    g.endpoint_slow_r = p::ENDPOINT_SLOW_R;
    g.kp_yaw = p::KP_YAW; g.kd_yaw = p::CARLIKE_KD_YAW;
    g.max_yaw_rate = p::CARLIKE_MAX_YAW_RATE;
    g.heading_gate_rad = p::CARLIKE_HEADING_GATE_DEG * M_PI / 180.0;
    g.forward_only = true; g.dt = dt;
    g.max_accel = p::CARLIKE_MAX_ACCEL; g.max_yaw_accel = p::CARLIKE_MAX_YAW_ACCEL;
    g.prediction_time = p::CARLIKE_PREDICTION_S;
    g.lateral_prediction_time = p::CARLIKE_LATERAL_PREDICTION_S;
    g.stop_align_rad = p::CARLIKE_STOP_ALIGN_DEG * M_PI / 180.0;
    g.corner_stop_rad = p::CARLIKE_CORNER_STOP_DEG * M_PI / 180.0;
    g.max_lateral_accel = p::CARLIKE_MAX_LATERAL_ACCEL;
    g.align_resume_rad = p::CARLIKE_ALIGN_RESUME_DEG * M_PI / 180.0;
    g.align_stop_speed = p::CARLIKE_ALIGN_STOP_SPEED;
    g.align_stop_yaw_rate = p::CARLIKE_ALIGN_STOP_YAW_RATE;
    g.align_settle_s = p::CARLIKE_ALIGN_SETTLE_S;
    return g;
}

// Reconstruct the previous first-segment blend. This is a captured algorithm
// for the comparison, never a second implementation of the new rounder.
Trajectory previous_turn(double angle)
{
    const Vec2 control1{.8, 0}, control2{2 - .8 * std::cos(angle), -.8 * std::sin(angle)};
    Path2 path{{0, 0}};
    for (int i = 1; i <= 12; ++i) {
        const double t = static_cast<double>(i) / 12, u = 1 - t;
        path.push_back({3 * u * u * t * control1.x + 3 * u * t * t * control2.x + 2 * t * t * t,
                        3 * u * u * t * control1.y + 3 * u * t * t * control2.y});
    }
    path.push_back({2 + 2 * std::cos(angle), 2 * std::sin(angle)});
    return smooth_catmull_rom(path, .05);
}

double path_distance(Vec2 point, const Trajectory& path)
{
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < path.size(); ++i) {
        const Vec2 a = path[i - 1].p, b = path[i].p;
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double t = std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) /
            std::max(1e-12, dx * dx + dy * dy), 0.0, 1.0);
        best = std::min(best, distance(point, {a.x + t * dx, a.y + t * dy}));
    }
    return best;
}

struct Result {
    bool reached = false;
    double duration = 0, slow = 0, minimum_speed = 1, max_error = 0;
    int recoveries = 0, yaw_reversals = 0;
};

Result follow(const Trajectory& path, bool noisy = false)
{
    const TrackerGains g = gains();
    TrajectoryTracker tracker(g);
    tracker.set_trajectory(path);
    Vec2 position = path.front().p;
    double yaw = 0, vx = 0, vy = 0, yaw_rate = 0, prior_command = 0, prior_yaw_command = 0;
    double ox = position.x, oy = position.y, oyaw = 0, ofwd = 0, olat = 0;
    int last_sign = 0, sample = 0;
    bool previous_hold = false;
    Result result;
    for (int tick = 0; tick < 4500; ++tick) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        if (!noisy || tick == 0 || tick * 2 / 5 != (tick - 1) * 2 / 5) {
            ++sample;
            ox = position.x + (noisy ? .003 * std::sin(1.41 * sample) : 0);
            oy = position.y + (noisy ? .003 * std::sin(1.73 * sample) : 0);
            oyaw = yaw + (noisy ? .012 * std::sin(1.13 * sample) : 0);
            ofwd = vx * std::cos(oyaw) + vy * std::sin(oyaw);
            olat = -vx * std::sin(oyaw) + vy * std::cos(oyaw);
        }
        const VelCmd command = tracker.update(ox, oy, oyaw, ofwd, olat, .08, yaw_rate);
        require(std::isfinite(command.v_fwd) && std::isfinite(command.yaw_rate) &&
            (command.holding_position
                ? std::hypot(command.v_fwd, command.v_lat) <= g.turn_hold_speed + 1e-9
                : (command.v_fwd >= 0 && command.v_fwd <= g.v_max && command.v_lat == 0)) &&
            std::abs(command.yaw_rate) <= g.max_yaw_rate + 1e-9 &&
            (command.holding_position || previous_hold ||
             command.v_fwd - prior_command <= g.max_accel * dt + 1e-9) &&
            (command.at_goal || std::abs(command.yaw_rate - prior_yaw_command) <= g.max_yaw_accel * dt + 1e-9),
            "turn command violated existing velocity/acceleration limits");
        prior_command = command.v_fwd; prior_yaw_command = command.yaw_rate;
        previous_hold = command.holding_position;
        if (tick * dt > 2 && distance(position, path.back().p) > .6) {
            result.minimum_speed = std::min(result.minimum_speed, command.v_fwd);
            if (command.v_fwd < .05) result.slow += dt;
            if (tracker.reorienting()) ++result.recoveries;
        }
        if (std::abs(command.yaw_rate) > .06) {
            const int sign = command.yaw_rate > 0 ? 1 : -1;
            if (last_sign && sign != last_sign) ++result.yaw_reversals;
            last_sign = sign;
        }
        vx += (c * command.v_fwd - s * command.v_lat - vx) * dt / .4;
        vy += (s * command.v_fwd + c * command.v_lat - vy) * dt / .4;
        yaw_rate += (command.yaw_rate - yaw_rate) * dt / .4;
        position.x += vx * dt; position.y += vy * dt;
        yaw = std::atan2(std::sin(yaw + yaw_rate * dt), std::cos(yaw + yaw_rate * dt));
        result.max_error = std::max(result.max_error, path_distance(position, path));
        result.duration = tick * dt;
        if (command.at_goal && std::hypot(vx, vy) < .025) { result.reached = true; break; }
    }
    return result;
}

void ordinary_turns()
{
    for (double degrees : {45., 90., 110.}) {
        const double angle = degrees * M_PI / 180;
        const Path2 raw{{0, 0}, {2, 0}, {2 + 2 * std::cos(angle), 2 * std::sin(angle)}};
        const Path2 rounded = round(raw);
        require(rounded.size() > raw.size() && !contains(rounded, raw[1]), "ordinary corner was not rounded");
        for (Vec2 p : rounded) require(p.y >= -1e-12, "corner starts by steering opposite the requested turn");
        require(rounded[1].x < raw[1].x && rounded[rounded.size() - 2].y > 0,
                "round must start before and finish after the original vertex");
        const Trajectory path = trajectory_from_polyline(rounded, .05, gains().corner_stop_rad);
        for (const auto& point : path) require(std::abs(point.kappa) <= 5.01, "round contains excessive curvature");
        const Result old = follow(previous_turn(angle)), current = follow(path), noise = follow(path, true);
        std::cout << degrees << "deg old=" << old.duration << "s new=" << current.duration
                  << "s min_speed=" << current.minimum_speed << " slow=" << current.slow
                  << " recovery_ticks=" << current.recoveries << " yaw_reversals=" << current.yaw_reversals
                  << " max_error=" << current.max_error << " noisy_max_error=" << noise.max_error << '\n';
        require(current.reached && noise.reached && current.duration + .5 < old.duration &&
            current.slow == 0 && current.recoveries == 0 && noise.slow == 0 && noise.recoveries == 0 &&
            current.minimum_speed > .10 && noise.minimum_speed > .10 &&
            current.max_error < .15 && noise.max_error < .15 &&
            current.yaw_reversals <= 1 && noise.yaw_reversals <= 2,
            "ordinary turn did not remain continuous with less delay");
    }
}

void constrained_rounds()
{
    const Path2 raw{{0, 0}, {2, 0}, {2, 2}};
    const Obstacles partial{{1.6, .4, .28}};
    require(path_clear(raw.front(), raw, partial, field()), "partial-round fixture must leave the original path safe");
    const Path2 reduced = round(raw, partial);
    require(reduced.size() > raw.size() && reduced[1].x > 1.2 + 1e-6 &&
        path_clear(reduced.front(), reduced, partial, field()), "local round must shrink to validated clearance");
    const Obstacles blocked{{1.8, .2, .095}};
    const Path2 two_corners{{0, 0}, {2, 0}, {2, 2}, {4, 2}};
    const Path2 mixed = round(two_corners, blocked);
    require(contains(mixed, {2, 0}) && !contains(mixed, {2, 2}) &&
        path_clear(mixed.front(), mixed, blocked, field()),
        "one obstructed round must preserve that corner without discarding another safe round");
    const Trajectory fallback = trajectory_from_polyline(mixed, .05, gains().corner_stop_rad);
    require(std::any_of(fallback.begin(), fallback.end(), [](const TrajPoint& p) {
        return distance(p.p, {2, 0}) < 1e-9;
    }), "resampling swallowed the true fallback corner");
    TrajectoryTracker tracker(gains());
    tracker.set_trajectory(fallback);
    require(std::abs(tracker.next_corner_distance() - 2) < 1e-9,
            "fallback must still expose the original corner to conservative stopping logic");

    GlobalConfig walls = field(); walls.min_x = 0; walls.min_y = 0; walls.wall_margin = .6;
    const Path2 wall_path{{.6, .6}, {.6, 3}, {3, 3}, {3, .6}};
    const Path2 inside = round(wall_path, {}, walls);
    require(inside.size() > wall_path.size() && path_inside_safe_field(inside, walls), "wall round escaped the field inset");
    for (Vec2 point : inside) require(point.x >= .6 - 1e-9 && point.y >= .6 - 1e-9,
                                     "wall round leaked outside the field");

    const double reversal = 163. * M_PI / 180;
    const Path2 near_reverse{{0, 0}, {2, 0}, {2 + 2 * std::cos(reversal), 2 * std::sin(reversal)}};
    require(round(near_reverse).size() == near_reverse.size(), "near reversal must keep the true sharp vertex");
    const Path2 tiny{{0, 0}, {.1, 0}, {.1, .1}};
    require(round(tiny).size() == tiny.size(), "tiny high-curvature pseudo-round must be rejected");
    std::cout << "constrained, mixed fallback, wall, reversal and tiny-corner checks passed\n";
}

void preserved_stop()
{
    const TrackerGains g = gains();
    for (double degrees : {45., 90., 110.}) {
        const double angle = degrees * M_PI / 180;
        const Path2 raw{{0, 0}, {2, 0}, {2 + 2 * std::cos(angle), 2 * std::sin(angle)}};
        const Trajectory path = trajectory_from_polyline(raw, .05, g.corner_stop_rad);
        TrajectoryTracker tracker(g);
        tracker.set_trajectory(path);
        Vec2 position{};
        double vx = 0, vy = 0, yaw = 0, yaw_rate = 0;
        bool released = false, reached = false;
        for (int tick = 0; tick < 2500; ++tick) {
            const double c = std::cos(yaw), s = std::sin(yaw);
            const VelCmd command = tracker.update(position.x, position.y, yaw,
                c * vx + s * vy, -s * vx + c * vy, .08, yaw_rate);
            if (!released && tracker.last_lookahead().y > 1e-6) {
                released = true;
                require(command.holding_position &&
                    distance(position, {2, 0}) <= .08 + 1e-9,
                    "sharp fallback must begin XY hold and turn within the original vertex tolerance");
            }
            vx += (c * command.v_fwd - s * command.v_lat - vx) * dt / .4;
            vy += (s * command.v_fwd + c * command.v_lat - vy) * dt / .4;
            yaw_rate += (command.yaw_rate - yaw_rate) * dt / .4;
            position.x += vx * dt; position.y += vy * dt;
            yaw = std::atan2(std::sin(yaw + yaw_rate * dt), std::cos(yaw + yaw_rate * dt));
            if (command.at_goal && std::hypot(vx, vy) < .025) { reached = true; break; }
        }
        const Result result = follow(path);
        require(released && reached && result.duration < 16,
                "sharp fallback must finish without an extra high-curvature crawl");
        std::cout << degrees << "deg sharp fallback preserved stop, duration=" << result.duration << "s\n";
    }
}
}  // namespace

int main()
{
    try { ordinary_turns(); constrained_rounds(); preserved_stop(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    return 0;
}
