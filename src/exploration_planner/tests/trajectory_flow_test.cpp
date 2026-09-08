#include "exploration_planner/trajectory_tracker.hpp"
#include "exploration_planner/params.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 0.02;

double wrap(double value)
{
    return std::atan2(std::sin(value), std::cos(value));
}

exploration::TrackerGains gains(bool production)
{
    namespace p = exploration::params;
    exploration::TrackerGains result{};
    result.v_max = production ? p::V_MAX : 0.4;
    result.v_min = production ? p::V_MIN : 0.08;
    result.k_curv = p::CARLIKE_K_CURV;
    result.lookahead = p::CARLIKE_LOOKAHEAD;
    result.endpoint_slow_r = p::ENDPOINT_SLOW_R;
    result.kp_yaw = p::KP_YAW;
    result.kd_yaw = p::CARLIKE_KD_YAW;
    result.max_yaw_rate = p::CARLIKE_MAX_YAW_RATE;
    result.heading_gate_rad = p::CARLIKE_HEADING_GATE_DEG * kPi / 180.0;
    result.max_accel = p::CARLIKE_MAX_ACCEL;
    result.max_yaw_accel = p::CARLIKE_MAX_YAW_ACCEL;
    result.yaw_filter_tau = p::CARLIKE_YAW_FILTER_S;
    result.prediction_time = p::CARLIKE_PREDICTION_S;
    result.lateral_prediction_time = p::CARLIKE_LATERAL_PREDICTION_S;
    result.stop_align_rad = p::CARLIKE_STOP_ALIGN_DEG * kPi / 180.0;
    result.corner_stop_rad = p::CARLIKE_CORNER_STOP_DEG * kPi / 180.0;
    result.max_lateral_accel = p::CARLIKE_MAX_LATERAL_ACCEL;
    result.align_resume_rad = p::CARLIKE_ALIGN_RESUME_DEG * kPi / 180.0;
    result.align_stop_speed = p::CARLIKE_ALIGN_STOP_SPEED;
    result.align_stop_yaw_rate = p::CARLIKE_ALIGN_STOP_YAW_RATE;
    result.align_settle_s = p::CARLIKE_ALIGN_SETTLE_S;
    result.forward_only = true;
    result.dt = kDt;
    return result;
}

exploration::Trajectory curve(double angle, double radius, bool s_bend)
{
    exploration::Trajectory result{{{0.0, 0.0}, 0.0, 0.0, 0.0}};
    auto append = [&](double length, double curvature) {
        const int count = std::max(1, static_cast<int>(std::ceil(length / 0.02)));
        const double ds = length / count;
        for (int sample = 0; sample < count; ++sample) {
            const auto previous = result.back();
            const double theta = previous.theta + curvature * ds;
            exploration::Vec2 position{};
            if (std::abs(curvature) < 1e-9) {
                position = {previous.p.x + ds * std::cos(theta),
                            previous.p.y + ds * std::sin(theta)};
            } else {
                position = {previous.p.x + (std::sin(theta) - std::sin(previous.theta)) / curvature,
                            previous.p.y - (std::cos(theta) - std::cos(previous.theta)) / curvature};
            }
            result.push_back({position, theta, curvature, previous.s + ds});
        }
    };
    append(1.5, 0.0);
    append(radius * angle, 1.0 / radius);
    if (s_bend) {
        append(0.5, 0.0);
        append(radius * angle, -1.0 / radius);
    }
    append(2.0, 0.0);
    return result;
}

double path_distance(const exploration::Vec2& point, const exploration::Trajectory& path)
{
    double best = std::numeric_limits<double>::infinity();
    for (size_t index = 1; index < path.size(); ++index) {
        const auto& a = path[index - 1].p;
        const auto& b = path[index].p;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double u = std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) /
            std::max(1e-12, dx * dx + dy * dy), 0.0, 1.0);
        best = std::min(best, std::hypot(point.x - a.x - u * dx, point.y - a.y - u * dy));
    }
    return best;
}

struct Scenario {
    const char* name;
    double angle;
    double radius;
    bool s_bend;
    bool smoothed = false;
};

bool run(const Scenario& scenario, bool production)
{
    const auto config = gains(production);
    const auto path = scenario.smoothed ? exploration::smooth_catmull_rom(
        {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {4.0, 2.0}}, 0.05) :
        curve(scenario.angle, scenario.radius, scenario.s_bend);
    exploration::TrajectoryTracker tracker(config);
    tracker.set_trajectory(path);
    double x = 0.0, y = 0.0, yaw = 0.0, vx = 0.0, vy = 0.0, yaw_rate = 0.0;
    double observed_x = 0.0, observed_y = 0.0, observed_yaw = 0.0;
    double observed_fwd = 0.0, observed_lat = 0.0, measured_yaw_rate = 0.0;
    double previous_sample_yaw = 0.0;
    int previous_sample_tick = -1, sample = 0;
    double minimum_speed = config.v_max, max_error = 0.0, previous_command = 0.0;
    double previous_yaw_command = 0.0;
    int stops = 0, reorientations = 0;
    bool bounded = true, reached = false;
    double elapsed = 0.0;
    for (int tick = 0; tick < 5000; ++tick) {
        if (tick == 0 || tick * 2 / 5 != (tick - 1) * 2 / 5) {
            ++sample;
            observed_x = x + 0.003 * std::sin(1.41 * sample);
            observed_y = y + 0.003 * std::sin(1.73 * sample);
            observed_yaw = wrap(yaw + 0.012 * std::sin(1.13 * sample));
            observed_fwd = vx * std::cos(observed_yaw) + vy * std::sin(observed_yaw);
            observed_lat = -vx * std::sin(observed_yaw) + vy * std::cos(observed_yaw);
            if (previous_sample_tick >= 0) {
                const double source_dt = (tick - previous_sample_tick) * kDt;
                const double raw_rate = wrap(observed_yaw - previous_sample_yaw) / source_dt;
                measured_yaw_rate = exploration::params::V_EST_ALPHA * raw_rate +
                    (1.0 - exploration::params::V_EST_ALPHA) * measured_yaw_rate;
            }
            previous_sample_yaw = observed_yaw;
            previous_sample_tick = tick;
        }
        const auto command = tracker.update(observed_x, observed_y, observed_yaw,
            observed_fwd, observed_lat, 0.08, measured_yaw_rate);
        bounded = bounded && std::isfinite(command.v_fwd) && std::isfinite(command.yaw_rate) &&
            command.v_fwd >= 0.0 && command.v_fwd <= config.v_max + 1e-8 &&
            command.v_lat == 0.0 && std::abs(command.yaw_rate) <= config.max_yaw_rate + 1e-8 &&
            command.v_fwd - previous_command <= config.max_accel * kDt + 1e-8 &&
            (command.at_goal || std::abs(command.yaw_rate - previous_yaw_command) <=
                config.max_yaw_accel * kDt + 1e-8);
        const double goal_distance = std::hypot(x - path.back().p.x, y - path.back().p.y);
        // Exclude intentional launch acceleration and final stopping only.
        if (tick * kDt > 2.0 && goal_distance > 0.6) {
            minimum_speed = std::min(minimum_speed, command.v_fwd);
            if (command.v_fwd < 1e-4) ++stops;
            if (tracker.reorienting()) ++reorientations;
        }
        previous_command = command.v_fwd;
        previous_yaw_command = command.yaw_rate;
        vx += (std::cos(yaw) * command.v_fwd - vx) * kDt / 0.40;
        vy += (std::sin(yaw) * command.v_fwd - vy) * kDt / 0.40;
        yaw_rate += (command.yaw_rate - yaw_rate) * kDt / 0.40;
        x += vx * kDt;
        y += vy * kDt;
        yaw = wrap(yaw + yaw_rate * kDt);
        max_error = std::max(max_error, path_distance({x, y}, path));
        elapsed = tick * kDt;
        if (command.at_goal && std::hypot(vx, vy) < 0.025 && goal_distance < 0.12) {
            reached = true;
            break;
        }
    }
    std::cout << scenario.name << ' ' << reached << ' ' << bounded << ' ' << stops << ' '
              << reorientations << ' ' << std::fixed << std::setprecision(3) << minimum_speed << ' '
              << max_error << ' ' << elapsed << '\n';
    return reached && bounded && stops == 0 && reorientations == 0 && minimum_speed > 0.03 &&
        max_error < 0.18;
}

bool sharp_fallback(double angle, bool production)
{
    const auto config = gains(production);
    exploration::TrajectoryTracker tracker(config);
    const exploration::Vec2 vertex{2.0, 0.0};
    const exploration::Vec2 end{2.0 + 2.0 * std::cos(angle), 2.0 * std::sin(angle)};
    tracker.set_trajectory({{{0.0, 0.0}, 0.0, 0.0, 0.0},
                            {vertex, angle, 0.0, 2.0}, {end, angle, 0.0, 4.0}});
    double x = 0.0, y = 0.0, yaw = 0.0, vx = 0.0, vy = 0.0, yaw_rate = 0.0;
    bool released = false, reached = false;
    double release_speed = 0.0, release_distance = 0.0;
    for (int tick = 0; tick < 4000; ++tick) {
        const double forward = vx * std::cos(yaw) + vy * std::sin(yaw);
        const double lateral = -vx * std::sin(yaw) + vy * std::cos(yaw);
        const auto command = tracker.update(x, y, yaw, forward, lateral, 0.08, yaw_rate);
        if (!released && tracker.last_lookahead().y > 1e-5) {
            released = true;
            release_speed = std::hypot(vx, vy);
            release_distance = std::hypot(x - vertex.x, y - vertex.y);
        }
        vx += (std::cos(yaw) * command.v_fwd - vx) * kDt / 0.40;
        vy += (std::sin(yaw) * command.v_fwd - vy) * kDt / 0.40;
        yaw_rate += (command.yaw_rate - yaw_rate) * kDt / 0.40;
        x += vx * kDt;
        y += vy * kDt;
        yaw = wrap(yaw + yaw_rate * kDt);
        if (command.at_goal && std::hypot(vx, vy) < 0.025) {
            reached = true;
            break;
        }
    }
    std::cout << "sharp_fallback_" << angle * 180.0 / kPi << " reached=" << reached
              << " release_speed=" << release_speed << " release_distance=" << release_distance << '\n';
    return reached && released && release_speed <= config.align_stop_speed + 1e-8 &&
        release_distance <= 0.08 + 1e-8;
}
}  // namespace

int main(int argc, char** argv)
{
    const bool production = argc == 2 && std::string(argv[1]) == "--production";
    if (argc > 2 || (argc == 2 && !production)) return 2;
    const std::vector<Scenario> scenarios{
        {"arc_45", kPi / 4.0, 0.8, false},
        {"arc_90", kPi / 2.0, 0.8, false},
        {"tight_arc_90", kPi / 2.0, 0.45, false},
        {"s_bend_45", kPi / 4.0, 0.8, true},
        {"s_bend_90", kPi / 2.0, 0.8, true},
        {"planner_smoothed_corners", kPi / 2.0, 0.8, true, true},
    };
    std::cout << "scenario reached bounded stops reorient_ticks min_speed_mps max_error_m time_s\n";
    bool passed = true;
    for (const auto& scenario : scenarios) passed = run(scenario, production) && passed;
    passed = sharp_fallback(kPi / 4.0, production) && passed;
    passed = sharp_fallback(kPi / 2.0, production) && passed;
    return passed ? 0 : 1;
}
