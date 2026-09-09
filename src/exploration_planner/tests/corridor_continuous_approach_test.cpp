#include "corridor_arena_scene.hpp"
#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {
constexpr Vec2 kEntry{8.1, 4.25};
constexpr Vec2 kGoal{8.1, -4.0};
constexpr double kDt = 0.02;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Result {
    double duration = 0.0;
    double second_approach = 0.0;
    double second_near_zero = 0.0;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    double second_release_y = 0.0;
    bool finished = false;
};

Result run(bool continuous, double lag, bool frozen_lateral = false, bool dropout = false,
           double lateral_kick = 0.0, bool limited_space = false, double align_speed = 0.20) {
    CorridorConfig config;
    config.perception.corridor_width = 1.5;
    config.perception.robot_width = 0.64;
    config.center_tolerance = 0.015;
    config.align_speed = align_speed;
    config.continuous_approach = continuous;
    CorridorController controller(config);
    CorridorPerception frame(config.perception);
    frame.setFrame(kEntry, kGoal);
    require(controller.configure(kEntry, kGoal), "route rejected");
    controller.start(kEntry, 0.0);
    Vec2 position = kEntry, velocity;
    const double response = 1.0 - std::exp(-kDt / lag);
    double now = 0.0, first_back = -1.0, first_center = 0.0, second_started = -1.0;
    double stopped_cloud_at = -1.0;
    bool stopped_for_cloud = false;
    bool applied_fault = false;
    std::string last_status;
    Result result;
    for (int step = 0; step < 6000; ++step) {
        const auto before = controller.phase();
        const int passed_before = controller.gatesPassed();
        if (passed_before == 1 && !applied_fault) {
            velocity.x += lateral_kick;
            if (limited_space) { position.y = -1.35; velocity.y = -0.20; }
            applied_fault = true;
        }
        if (passed_before == 1 && stopped_cloud_at < 0.0) stopped_cloud_at = now;
        const bool suppress = dropout && stopped_cloud_at >= 0.0 && now - stopped_cloud_at < 1.1;
        if (step % 5 == 0 && !suppress)
            controller.observe(corridor_arena::arenaCloud(position), now, position);
        const auto command = controller.update(position, config.initial_yaw,
                                                 -velocity.y, velocity.x, now, true);
        last_status = command.status;
        const double commanded_speed = std::hypot(command.forward, command.lateral);
        require(commanded_speed <= 0.300001 && std::abs(command.yaw_rate) < 1e-12,
                "speed limit or fixed heading changed");
        if (controller.gateLocked() && first_back < 0.0) {
            first_back = controller.gate().gate_s + controller.gate().gate_depth * 0.5;
            first_center = controller.gate().gate_center.x;
        }
        const double s = frame.longitudinal(position);
        if (first_back >= 0.0 && s < first_back + config.exit_distance &&
            s > first_back - config.perception.robot_width * 0.5) {
            require(controller.gatesPassed() == 0, "first exit released before the complete 1 metre");
            require(std::abs(position.x - first_center) <= config.center_tolerance + 0.002,
                    "lateral transition began before the complete straight exit");
        }
        if (controller.gatesPassed() == 1 && controller.phase() == CorridorPhase::Approach) {
            if (second_started < 0.0) second_started = now;
            result.second_approach += kDt;
            if (commanded_speed < 0.035) result.second_near_zero += kDt;
        }
        if (before == CorridorPhase::Approach && controller.phase() == CorridorPhase::Cross) {
            const double error = std::abs(position.x - controller.gate().gate_center.x);
            require(error <= config.center_tolerance + 1e-6, "crossing started before strict center acquisition");
            if (controller.gatesPassed() == 1) result.second_release_y = position.y;
        }
        if (suppress && now - stopped_cloud_at > config.cloud_timeout + 0.1) {
            require(commanded_speed == 0.0, "freshness hold was bypassed by continuous approach");
            stopped_for_cloud = true;
        }
        const Vec2 desired{command.lateral, -command.forward};
        if (frozen_lateral && controller.gatesPassed() == 1)
            velocity.x = 0.0;
        else
            velocity.x += response * (desired.x - velocity.x);
        velocity.y += response * (desired.y - velocity.y);
        position.x += velocity.x * kDt;
        position.y += velocity.y * kDt;
        result.minimum_clearance = std::min(result.minimum_clearance,
                                            corridor_arena::bodyClearance(position));
        require(result.minimum_clearance > config.perception.robot_width * 0.5,
                "aircraft envelope intersects a physical wall or door");
        now += kDt;
        if (frozen_lateral && second_started >= 0.0 && now - second_started > 80.0) {
            std::cout << "frozen gate stop xy=" << position.x << ',' << position.y
                      << " command=" << command.forward << ',' << command.lateral
                      << " velocity=" << velocity.x << ',' << velocity.y << std::endl;
            require(controller.gatesPassed() == 1 && controller.phase() == CorridorPhase::Approach,
                    "missing lateral authority allowed an unaligned gate crossing");
            require(position.y > -1.95 + config.perception.robot_width * 0.5,
                    "unaligned aircraft moved into the second door body envelope");
            require(command.forward < 0.005 && std::abs(velocity.y) < 0.01,
                    "insufficient alignment space did not stop longitudinal motion");
            break;
        }
        if (command.finished) { result.finished = true; break; }
    }
    result.duration = now;
    std::cout << "continuous=" << continuous << " lag=" << lag << " frozen=" << frozen_lateral
              << " align_speed=" << align_speed
              << " dropout=" << dropout << " lateral_kick=" << lateral_kick
              << " limited_space=" << limited_space << " duration=" << result.duration
              << " second_approach=" << result.second_approach
              << " second_near_zero=" << result.second_near_zero
              << " clearance=" << result.minimum_clearance
              << " second_release_y=" << result.second_release_y
              << " xy=" << position.x << ',' << position.y
              << " gates=" << controller.gatesPassed() << " status=" << last_status << std::endl;
    if (!frozen_lateral) {
        require(result.finished && controller.gatesPassed() == 2, "two doors and H did not finish");
        if (continuous && !dropout)
            require(result.second_near_zero < 0.15, "continuous second-door approach still plans a stop");
    }
    if (dropout) require(stopped_for_cloud, "cloud-dropout branch was not exercised");
    return result;
}
}  // namespace

int main() {
    const Result baseline = run(false, 0.35);
    const Result flowing = run(true, 0.35);
    require(baseline.second_near_zero > 1.0, "baseline did not reproduce the near-zero alignment tail");
    require(flowing.duration < baseline.duration, "continuous scheduling did not improve total duration");
    run(true, 0.15);
    run(true, 0.60);
    run(true, 0.35, false, false, 0.0, false, 0.15);
    run(true, 0.35, false, false, -0.15);
    run(true, 0.35, false, false, 0.25);
    run(true, 0.35, true);
    run(true, 0.35, true, false, 0.0, true);
    run(true, 0.35, false, true);
    std::cout << "corridor continuous approach tests passed\n";
}
