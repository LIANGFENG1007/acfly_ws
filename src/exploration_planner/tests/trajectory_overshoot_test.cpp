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
    exploration::TrackerGains result{};
    result.v_max = 0.4;
    result.v_min = 0.08;
    result.k_curv = 0.8;
    result.lookahead = 0.6;
    result.endpoint_slow_r = 0.5;
    result.kp_yaw = 1.6;
    result.kd_yaw = 0.2;
    result.max_yaw_rate = 1.8;
    result.kp_lat = 0.8;
    result.kd_lat = 0.1;
    result.max_v_lat = 0.2;
    result.heading_gate_rad = 18.0 * kPi / 180.0;
    result.forward_only = true;
    result.dt = kDt;
    if (production) {
        namespace p = exploration::params;
        result.v_max = p::V_MAX;
        result.v_min = p::V_MIN;
        result.k_curv = p::CARLIKE_K_CURV;
        result.lookahead = p::CARLIKE_LOOKAHEAD;
        result.endpoint_slow_r = p::ENDPOINT_SLOW_R;
        result.kp_yaw = p::KP_YAW;
        result.kd_yaw = p::CARLIKE_KD_YAW;
        result.max_yaw_rate = p::CARLIKE_MAX_YAW_RATE;
        result.kp_lat = p::KP_LAT;
        result.kd_lat = p::KD_LAT;
        result.max_v_lat = p::MAX_V_LAT;
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
    }
    return result;
}

exploration::Trajectory polyline(const exploration::Path2& points, bool dense)
{
    exploration::Trajectory result;
    double distance = 0.0;
    for (size_t index = 1; index < points.size(); ++index) {
        const auto& from = points[index - 1];
        const auto& to = points[index];
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        const double length = std::hypot(dx, dy);
        const int count = dense ? std::max(1, static_cast<int>(std::ceil(length / 0.04))) : 1;
        const double heading = std::atan2(dy, dx);
        for (int sample = 0; sample < count; ++sample) {
            const double fraction = static_cast<double>(sample) / count;
            result.push_back({{from.x + dx * fraction, from.y + dy * fraction},
                              heading, 0.0, distance + length * fraction});
        }
        distance += length;
        if (index + 1 == points.size()) {
            result.push_back({to, heading, 0.0, distance});
        }
    }
    return result;
}

struct Scenario {
    const char* name;
    double yaw;
    double initial_y;
    double velocity_lag;
    double yaw_lag;
    bool dense = true;
    bool replace = false;
    bool corner = false;
};

struct Reversals {
    int sign = 0;
    int count = 0;

    void add(double value, double threshold)
    {
        const int next = value > threshold ? 1 : (value < -threshold ? -1 : 0);
        if (next == 0) return;
        if (sign != 0 && sign != next) ++count;
        sign = next;
    }
};

struct Result {
    bool reached = false;
    bool bounded = true;
    int crossings = 0;
    int yaw_reversals = 0;
    int command_reversals = 0;
    double overshoot = 0.0;
    double max_error = 0.0;
    double final_error = 0.0;
    double time = 0.0;
    double acquisition_distance = 0.0;
};

Result run(const Scenario& scenario, bool production)
{
    auto config = gains(production);
    exploration::TrajectoryTracker tracker(config);
    exploration::Path2 path = scenario.corner ? exploration::Path2{{0.0, 0.0}, {2.0, 0.0}, {2.0, 3.0}}
                                             : exploration::Path2{{0.0, 0.0}, {8.0, 0.0}};
    tracker.set_trajectory(polyline(path, scenario.dense));
    double x = 0.0;
    double y = scenario.initial_y;
    double yaw = scenario.yaw;
    double vx = 0.3 * std::cos(yaw);
    double vy = 0.3 * std::sin(yaw);
    double yaw_rate = 0.0;
    double observed_x = x;
    double observed_y = y;
    double observed_yaw = yaw;
    double observed_fwd = 0.3;
    double observed_lat = 0.0;
    double measured_yaw_rate = std::numeric_limits<double>::quiet_NaN();
    double previous_sample_yaw = yaw;
    int previous_sample_tick = -1;
    bool replaced = false;
    int sample = 0;
    int first_error_side = scenario.initial_y > 0.02 ? 1 : (scenario.initial_y < -0.02 ? -1 : 0);
    Reversals cross;
    Reversals yaw_turns;
    Reversals commands;
    Result result;
    bool acquired = std::abs(scenario.initial_y) <= 0.04;
    double correction_origin_x = x;

    for (int tick = 0; tick < 3500; ++tick) {
        const double time = tick * kDt;
        if (scenario.replace && !replaced && time >= 7.0) {
            path = {{x, 0.45}, {8.0, 0.45}};
            tracker.set_trajectory(polyline(path, scenario.dense));
            replaced = true;
            cross.sign = 0;
            first_error_side = -1;
            acquired = false;
            correction_origin_x = x;
        }

        // A 50 Hz control loop receives 20 Hz sample-and-hold odometry.
        // Noise is deterministic and unrelated to controller internals.
        if (tick == 0 || tick * 2 / 5 != (tick - 1) * 2 / 5) {
            ++sample;
            observed_x = x + 0.003 * std::sin(1.41 * sample);
            observed_y = y + 0.003 * std::sin(1.73 * sample);
            observed_yaw = wrap(yaw + 0.012 * std::sin(1.13 * sample));
            observed_fwd = vx * std::cos(observed_yaw) + vy * std::sin(observed_yaw);
            observed_lat = -vx * std::sin(observed_yaw) + vy * std::cos(observed_yaw);
            if (previous_sample_tick >= 0) {
                const double source_dt = (tick - previous_sample_tick) * kDt;
                const double raw_yaw_rate = wrap(observed_yaw - previous_sample_yaw) / source_dt;
                const double previous_rate = std::isfinite(measured_yaw_rate) ? measured_yaw_rate : 0.0;
                measured_yaw_rate = exploration::params::V_EST_ALPHA * raw_yaw_rate +
                    (1.0 - exploration::params::V_EST_ALPHA) * previous_rate;
            }
            previous_sample_yaw = observed_yaw;
            previous_sample_tick = tick;
        }
        const auto command = production ?
            tracker.update(observed_x, observed_y, observed_yaw, observed_fwd, observed_lat, 0.08,
                           measured_yaw_rate) :
            tracker.update(observed_x, observed_y, observed_yaw, observed_fwd, observed_lat, 0.08);
        if (!std::isfinite(command.v_fwd) || !std::isfinite(command.v_lat) ||
            !std::isfinite(command.yaw_rate) ||
            (command.holding_position
                ? std::hypot(command.v_fwd, command.v_lat) > config.turn_hold_speed + 1e-8
                : (command.v_fwd < -1e-8 || command.v_fwd > config.v_max + 1e-8 ||
                   std::abs(command.v_lat) > 1e-8)) ||
            std::abs(command.yaw_rate) > config.max_yaw_rate + 1e-8) {
            result.bounded = false;
            break;
        }
        commands.add(command.yaw_rate, 0.12);
        yaw_turns.add(yaw_rate, 0.07);

        // Translational inertia acts in the world frame: rotating the body
        // cannot instantly rotate the vehicle's existing velocity vector.
        const double target_vx = std::cos(yaw) * command.v_fwd - std::sin(yaw) * command.v_lat;
        const double target_vy = std::sin(yaw) * command.v_fwd + std::cos(yaw) * command.v_lat;
        vx += (target_vx - vx) * kDt / scenario.velocity_lag;
        vy += (target_vy - vy) * kDt / scenario.velocity_lag;
        yaw_rate += (command.yaw_rate - yaw_rate) * kDt / scenario.yaw_lag;
        x += vx * kDt;
        y += vy * kDt;
        yaw = wrap(yaw + yaw_rate * kDt);

        const double signed_error = scenario.corner && x > 1.7 && y > 0.15 ? 2.0 - x : y - path.front().y;
        result.max_error = std::max(result.max_error, std::abs(signed_error));
        if (first_error_side == 0 && std::abs(signed_error) > 0.02) first_error_side = signed_error > 0.0 ? 1 : -1;
        if (first_error_side != 0 && first_error_side * signed_error < 0.0) {
            result.overshoot = std::max(result.overshoot, std::abs(signed_error));
        }
        cross.add(signed_error, 0.02);
        if (!acquired && !scenario.corner) {
            result.acquisition_distance = std::max(0.0, x - correction_origin_x);
            acquired = std::abs(signed_error) <= 0.04;
        }
        result.final_error = std::hypot(x - path.back().x, y - path.back().y);
        result.time = time;
        if (command.at_goal && std::hypot(vx, vy) < 0.025 && result.final_error < 0.12) {
            result.reached = true;
            break;
        }
    }
    result.crossings = cross.count;
    result.yaw_reversals = yaw_turns.count;
    result.command_reversals = commands.count;
    return result;
}

}  // namespace

int main(int argc, char** argv)
{
    bool baseline = false;
    bool production = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--baseline") baseline = true;
        else if (argument == "--production") production = true;
        else {
            std::cerr << "Usage: trajectory_overshoot_test [--baseline] [--production]\n";
            return 2;
        }
    }
    const std::vector<Scenario> scenarios{
        {"heading_60", 60.0 * kPi / 180.0, 0.0, 0.25, 0.25},
        {"heading_90", 90.0 * kPi / 180.0, 0.0, 0.40, 0.40},
        {"heading_150", 150.0 * kPi / 180.0, 0.0, 0.40, 0.40},
        {"parallel_offset", 0.0, 0.40, 0.40, 0.40},
        {"replacement", 0.0, 0.0, 0.40, 0.40, true, true},
        {"antipodal_positive", kPi - 0.003, 0.0, 0.40, 0.40},
        {"antipodal_negative", -kPi + 0.003, 0.0, 0.40, 0.40},
        {"sparse_offset", 0.0, 0.40, 0.40, 0.40, false},
        {"sparse_right_angle", 0.0, 0.0, 0.40, 0.40, false, false, true},
    };
    bool passed = true;
    std::cout << "scenario reached bounded crossings yaw_reversals command_reversals overshoot_m max_error_m final_error_m acquisition_m time_s\n";
    for (const auto& scenario : scenarios) {
        const Result result = run(scenario, production);
        std::cout << scenario.name << ' ' << result.reached << ' ' << result.bounded << ' '
                  << result.crossings << ' ' << result.yaw_reversals << ' ' << result.command_reversals << ' '
                  << std::fixed << std::setprecision(3) << result.overshoot << ' ' << result.max_error << ' '
                  << result.final_error << ' ' << result.acquisition_distance << ' ' << result.time << '\n';
        const bool smooth = result.crossings <= (scenario.replace || scenario.corner ? 4 : 2) &&
                            result.yaw_reversals <= (scenario.replace || scenario.corner ? 8 : 5) &&
                            result.command_reversals <= (scenario.replace || scenario.corner ? 14 : 10) &&
                            (scenario.corner || result.overshoot <= 0.08) &&
                            result.max_error <= 0.55 && result.acquisition_distance <= 2.5;
        passed = passed && result.reached && result.bounded && smooth;
    }
    return baseline || passed ? 0 : 1;
}
