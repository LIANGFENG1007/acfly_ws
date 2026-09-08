#include "corridor_arena_scene.hpp"
#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {

constexpr double kDt = 0.02;
constexpr Vec2 kEntry{8.25, 4.25};
constexpr Vec2 kGoal{8.25, -4.25};
constexpr Vec2 kRed{7.0, 4.25};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

double distance(const Vec2& a, const Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

struct Scene {
    bool arena;
    Path2 samples;
    std::array<double, 2> planes;
    std::array<double, 2> centers;

    explicit Scene(bool use_arena) : arena(use_arena) {
        if (arena) {
            planes = {0.9, -2.0};
            centers = {7.925, 8.575};
            return;
        }
        planes = {1.25, 0.45};
        centers = {7.90, 8.60};
        // Independent 80 cm openings, only 80 cm apart, with alternating offsets.
        for (int index = 0; index <= 400; ++index) {
            const double y = 5.0 - index * 0.025;
            if (y <= 3.5) samples.push_back({7.50, y});
            samples.push_back({9.00, y});
        }
        for (int index = 0; index <= 28; ++index) {
            samples.push_back({8.30 + index * 0.025, planes[0]});
            samples.push_back({7.50 + index * 0.025, planes[1]});
        }
    }

    Path2 cloud(const Vec2& position) const {
        return arena ? corridor_arena::arenaCloud(position, 0.3) : samples;
    }

    double clearance(const Vec2& position) const {
        if (arena) return corridor_arena::bodyClearance(position);
        double value = std::numeric_limits<double>::infinity();
        for (const auto& point : samples) value = std::min(value, distance(position, point));
        return value;
    }
};

bool safetyWait(const std::string& status) {
    return status.find("Waiting for ") == 0 ||
           status.find("Translation blocked") == 0 ||
           status.find("Braking") == 0 ||
           status.find("Insufficient measured") == 0 ||
           status.find("Door closed") == 0;
}

struct Metrics {
    int planned_zero_frames = 0;
    int planned_zero_intervals = 0;
    int longest_planned_zero = 0;
    int safety_zero_frames = 0;
    int slow_frames = 0;
    double max_speed = 0.0;
    double min_clearance = std::numeric_limits<double>::infinity();
    double max_yaw_error = 0.0;
    double max_yaw_command = 0.0;
    double max_backward_speed = 0.0;
    std::array<double, 2> center_error{-1.0, -1.0};
    std::array<double, 2> crossing_speed{-1.0, -1.0};
    std::map<std::string, int> holds;
};

void runMission(bool arena, bool interrupt_cloud, bool baseline) {
    const std::string name = arena ? (interrupt_cloud ? "arena-cloud-resume" : "arena-flow")
                                   : "nearby-doors-flow";
    Scene scene(arena);
    CorridorConfig config;
    config.perception.corridor_width = 1.5;
    config.perception.robot_width = 0.5;
    CorridorController controller(config);
    require(controller.configure(kEntry, kGoal), name + ": valid route rejected");
    controller.start(kRed, 0.0);
    Vec2 position = kRed, velocity;
    double yaw = config.initial_yaw, yaw_rate = 0.0, now = 0.0;
    double dropout_start = -1.0;
    bool entry_reached = false, finished = false, stopped_for_cloud = false;
    Metrics metrics;
    int current_planned_zero = 0;
    CorridorCommand command;
    const double response = 1.0 - std::exp(-kDt / 0.15);
    for (int step = 0; step < 10000; ++step) {
        if (interrupt_cloud && dropout_start < 0.0 && controller.gatesPassed() == 1)
            dropout_start = now;
        const bool suppress_cloud = dropout_start >= 0.0 && now - dropout_start < 1.2;
        if (step % 5 == 0 && !suppress_cloud) controller.observe(scene.cloud(position), now, position);
        const double vf = std::cos(yaw) * velocity.x + std::sin(yaw) * velocity.y;
        const double vl = -std::sin(yaw) * velocity.x + std::cos(yaw) * velocity.y;
        command = controller.update(position, yaw, vf, vl, now, true);
        const auto phase = controller.phase();
        if (phase == CorridorPhase::Entry && distance(position, kEntry) <= config.point_tolerance)
            entry_reached = true;
        if (phase == CorridorPhase::Search || phase == CorridorPhase::Approach || phase == CorridorPhase::Cross)
            entry_reached = true;
        const double command_speed = std::hypot(command.forward, command.lateral);
        const bool flowing = entry_reached && phase != CorridorPhase::Finish && phase != CorridorPhase::Done;
        if (flowing && command_speed < 1e-8) {
            ++metrics.holds[command.status];
            if (safetyWait(command.status)) {
                ++metrics.safety_zero_frames;
                current_planned_zero = 0;
            } else {
                ++metrics.planned_zero_frames;
                if (current_planned_zero++ == 0) ++metrics.planned_zero_intervals;
                metrics.longest_planned_zero = std::max(metrics.longest_planned_zero, current_planned_zero);
            }
        } else current_planned_zero = 0;
        if (flowing && command_speed < 0.02) ++metrics.slow_frames;
        if (phase != CorridorPhase::Rotate) {
            metrics.max_yaw_command = std::max(metrics.max_yaw_command, std::abs(command.yaw_rate));
            const double error = std::atan2(std::sin(yaw - config.initial_yaw),
                                             std::cos(yaw - config.initial_yaw));
            metrics.max_yaw_error = std::max(metrics.max_yaw_error, std::abs(error));
        }
        metrics.max_speed = std::max(metrics.max_speed, command_speed);
        if (suppress_cloud && now - dropout_start > config.cloud_timeout + 0.1) {
            require(command_speed == 0.0 && !command.finished, name + ": stale cloud did not stop motion");
            stopped_for_cloud = true;
        }
        const Vec2 previous = position;
        const Vec2 target{std::cos(yaw) * command.forward - std::sin(yaw) * command.lateral,
                          std::sin(yaw) * command.forward + std::cos(yaw) * command.lateral};
        if (flowing) metrics.max_backward_speed = std::max(metrics.max_backward_speed, target.y);
        velocity.x += response * (target.x - velocity.x);
        velocity.y += response * (target.y - velocity.y);
        yaw_rate += response * (command.yaw_rate - yaw_rate);
        position.x += velocity.x * kDt;
        position.y += velocity.y * kDt;
        yaw += yaw_rate * kDt;
        metrics.min_clearance = std::min(metrics.min_clearance, scene.clearance(position));
        require(scene.clearance(position) > config.perception.robot_width * 0.5,
                name + ": aircraft intersects physical wall or door at t=" + std::to_string(now) +
                " xy=(" + std::to_string(position.x) + "," + std::to_string(position.y) + ")");
        for (std::size_t gate = 0; gate < scene.planes.size(); ++gate) {
            if (previous.y > scene.planes[gate] && position.y <= scene.planes[gate]) {
                const double fraction = (previous.y - scene.planes[gate]) / (previous.y - position.y);
                const double crossing_x = previous.x + fraction * (position.x - previous.x);
                metrics.center_error[gate] = std::abs(crossing_x - scene.centers[gate]);
                metrics.crossing_speed[gate] = std::hypot(velocity.x, velocity.y);
            }
        }
        now += kDt;
        if (command.finished) { finished = true; break; }
    }
    std::cout << name << " finished=" << finished << " doors=" << controller.gatesPassed()
              << " duration_s=" << now << " planned_zero_frames=" << metrics.planned_zero_frames
              << " planned_zero_intervals=" << metrics.planned_zero_intervals
              << " longest_planned_zero_s=" << metrics.longest_planned_zero * kDt
              << " safety_zero_frames=" << metrics.safety_zero_frames
              << " slow_frames=" << metrics.slow_frames
              << " center_errors_m=" << metrics.center_error[0] << ',' << metrics.center_error[1]
              << " crossing_speeds=" << metrics.crossing_speed[0] << ',' << metrics.crossing_speed[1]
              << " body_clearance_m=" << metrics.min_clearance
              << " max_speed=" << metrics.max_speed << " yaw_error=" << metrics.max_yaw_error
              << " max_backward_speed=" << metrics.max_backward_speed
              << " final_speed=" << std::hypot(velocity.x, velocity.y)
              << " status=" << command.status << '\n';
    for (const auto& hold : metrics.holds)
        std::cout << "  " << name << " zero_status=" << hold.first << " frames=" << hold.second << '\n';
    if (baseline) return;
    require(finished && controller.gatesPassed() == 2, name + ": both doors and H must finish");
    require(metrics.planned_zero_frames == 0, name + ": entry/door transitions must not plan a stop");
    require(metrics.max_speed <= 0.300001, name + ": configured 0.3 m/s limit exceeded");
    require(metrics.max_backward_speed < 0.005,
            name + ": passing an earlier approach point must not command backward flight");
    require(metrics.max_yaw_error < 1e-9 && metrics.max_yaw_command < 1e-9,
            name + ": fixed corridor heading changed");
    require(metrics.center_error[0] >= 0.0 && metrics.center_error[1] >= 0.0 &&
            metrics.center_error[0] < 0.08 && metrics.center_error[1] < 0.08,
            name + ": door-plane center error exceeds 8 cm");
    require(distance(position, kGoal) <= config.point_tolerance * config.arrival_hysteresis &&
            std::hypot(velocity.x, velocity.y) < config.stop_speed,
            name + ": H completion must remain a settled arrival");
    if (interrupt_cloud) require(stopped_for_cloud, name + ": missing-cloud stop was not exercised");
}

}  // namespace

int main(int argc, char** argv) {
    const bool baseline = argc > 1 && std::string(argv[1]) == "--baseline";
    int failures = 0;
    for (const auto& setting : {std::pair<bool, bool>{true, false}, {false, false}, {true, true}}) {
        try { runMission(setting.first, setting.second, baseline); }
        catch (const std::exception& error) { std::cerr << "FAIL: " << error.what() << '\n'; ++failures; }
    }
    if (!failures) std::cout << "corridor_flow_test: " << (baseline ? "baseline recorded" : "all checks passed") << '\n';
    return failures ? 1 : 0;
}
