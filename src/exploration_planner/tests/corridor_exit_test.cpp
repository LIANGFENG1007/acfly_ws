#include "corridor_arena_scene.hpp"
#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace exploration;

namespace {

constexpr Vec2 kEntry{8.25, 4.25};
constexpr Vec2 kGoal{8.25, -4.25};
constexpr double kDt = 0.02;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

double distance(const Vec2& a, const Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

struct Scene {
    bool arena;
    std::vector<corridor_arena::Box> walls;
    Path2 samples;

    explicit Scene(double second_gate, bool real_arena = false) : arena(real_arena) {
        walls = {{7.45, 7.50, -5.0, 5.0}, {9.0, 9.05, -5.0, 5.0},
                 {8.30, 9.0, 1.20, 1.30},
                 {7.50, 8.20, kEntry.y - second_gate - 0.05, kEntry.y - second_gate + 0.05}};
        for (const auto& wall : walls) {
            const int nx = static_cast<int>(std::ceil((wall.x_max - wall.x_min) / 0.025));
            const int ny = static_cast<int>(std::ceil((wall.y_max - wall.y_min) / 0.025));
            for (int index = 0; index <= nx; ++index) {
                const double x = wall.x_min + (wall.x_max - wall.x_min) * index / nx;
                samples.push_back({x, wall.y_min});
                samples.push_back({x, wall.y_max});
            }
            for (int index = 0; index <= ny; ++index) {
                const double y = wall.y_min + (wall.y_max - wall.y_min) * index / ny;
                samples.push_back({wall.x_min, y});
                samples.push_back({wall.x_max, y});
            }
        }
    }

    Path2 cloud(const Vec2& position) const {
        return arena ? corridor_arena::arenaCloud(position, 0.3) : samples;
    }

    double clearance(const Vec2& position) const {
        if (arena) return corridor_arena::bodyClearance(position);
        double result = std::numeric_limits<double>::infinity();
        for (const auto& wall : walls) {
            const double dx = std::max({wall.x_min - position.x, 0.0, position.x - wall.x_max});
            const double dy = std::max({wall.y_min - position.y, 0.0, position.y - wall.y_max});
            result = std::min(result, std::hypot(dx, dy));
        }
        return result;
    }
};

struct Result {
    double release_s = -1.0;
    double gate_back = 0.0;
    double next_lateral_s = -1.0;
    double minimum_clearance = std::numeric_limits<double>::infinity();
    double max_exit_center_error = 0.0;
};

Result runMission(const std::string& name, double exit_distance, double second_gate,
                  bool strict_first_exit, bool arena = false) {
    CorridorConfig config;
    config.exit_distance = exit_distance;
    Scene scene(second_gate, arena);
    CorridorController controller(config);
    CorridorPerception frame(config.perception);
    frame.setFrame(kEntry, kGoal);
    require(controller.configure(kEntry, kGoal), name + ": invalid route");
    controller.start(kEntry, 0.0);
    Vec2 position = kEntry, velocity;
    const double yaw = config.initial_yaw;
    const double response = 1.0 - std::exp(-kDt / 0.15);
    double now = 0.0, gate_center = 0.0, gate_plane = 0.0;
    bool first_locked = false, finished = false;
    Result result;
    CorridorCommand command;
    for (int step = 0; step < 10000; ++step) {
        if (step % 5 == 0) controller.observe(scene.cloud(position), now, position);
        const double vf = std::cos(yaw) * velocity.x + std::sin(yaw) * velocity.y;
        const double vl = -std::sin(yaw) * velocity.x + std::cos(yaw) * velocity.y;
        const int previous_count = controller.gatesPassed();
        const double s = frame.longitudinal(position);
        command = controller.update(position, yaw, vf, vl, now, true);
        if (!first_locked && controller.gateLocked()) {
            const auto& gate = controller.gate();
            gate_plane = gate.gate_s;
            result.gate_back = gate.gate_s + 0.5 * gate.gate_depth;
            gate_center = 0.5 * (gate.gap_left + gate.gap_right);
            first_locked = true;
        }
        const double speed = std::hypot(command.forward, command.lateral);
        require(speed <= 0.300001 && std::abs(command.yaw_rate) < 1e-9,
                name + ": speed or fixed-heading contract violated");
        if (strict_first_exit && first_locked && s >= gate_plane &&
            s < result.gate_back + exit_distance) {
            require(controller.gatesPassed() == 0 && controller.phase() == CorridorPhase::Cross,
                    name + ": switched to the next segment before the complete exit distance");
            const double center_error = std::abs(frame.lateral(position) - gate_center);
            result.max_exit_center_error = std::max(result.max_exit_center_error, center_error);
            require(center_error <= config.center_tolerance + 0.001,
                    name + ": aircraft left the current door centerline during the exit leg");
            require(speed > 0.02, name + ": a clear exit leg added a planned stop");
        }
        if (previous_count == 0 && controller.gatesPassed() == 1) {
            result.release_s = s;
            require(speed > 0.0, name + ": completing the exit leg inserted a zero command");
        }
        if (controller.gatesPassed() == 1 && command.lateral > 0.03 && result.next_lateral_s < 0.0)
            result.next_lateral_s = s;
        const Vec2 target{std::cos(yaw) * command.forward - std::sin(yaw) * command.lateral,
                          std::sin(yaw) * command.forward + std::cos(yaw) * command.lateral};
        velocity.x += response * (target.x - velocity.x);
        velocity.y += response * (target.y - velocity.y);
        position.x += velocity.x * kDt;
        position.y += velocity.y * kDt;
        result.minimum_clearance = std::min(result.minimum_clearance, scene.clearance(position));
        require(scene.clearance(position) > config.perception.robot_width * 0.5,
                name + ": aircraft collided with a physical wall or door");
        require(frame.longitudinal(position) <= frame.length() + 1e-6,
                name + ": the exit segment overshot H");
        now += kDt;
        if (command.finished) { finished = true; break; }
    }
    std::cout << name << " exit_m=" << exit_distance << " finished=" << finished
              << " gates=" << controller.gatesPassed() << " duration_s=" << now
              << " measured_back=" << result.gate_back << " release_s=" << result.release_s
              << " next_lateral_s=" << result.next_lateral_s
              << " exit_center_error=" << result.max_exit_center_error
              << " minimum_clearance=" << result.minimum_clearance
              << " final_s=" << frame.longitudinal(position) << " status=" << command.status << '\n';
    require(finished && controller.gatesPassed() == 2,
            name + ": the extended exit segment must still complete both doors and H");
    require(distance(position, kGoal) < config.point_tolerance * config.arrival_hysteresis &&
            std::hypot(velocity.x, velocity.y) < config.stop_speed,
            name + ": H completion must remain a settled arrival");
    if (strict_first_exit) {
        require(result.release_s >= result.gate_back + exit_distance &&
                result.release_s <= result.gate_back + exit_distance + 0.02,
                name + ": release coordinate does not follow the configured door-back distance");
    }
    return result;
}

}  // namespace

int main() {
    try {
        const auto short_exit = runMission("wide-short-exit", 0.50, 6.0, true);
        const auto long_exit = runMission("wide-long-exit", 1.20, 6.0, true);
        require(long_exit.release_s - short_exit.release_s > 0.68,
                "increasing exit distance by 0.7 m must delay the door handoff by the same distance");
        require(short_exit.next_lateral_s > 0.0 &&
                long_exit.next_lateral_s - short_exit.next_lateral_s > 0.60,
                "a longer exit leg must delay lateral translation to the next door");
        runMission("nearby-long-exit", 1.20, 3.8, false);
        runMission("near-H-long-exit", 1.20, 7.9, true);
        runMission("arena-long-exit", 1.20, 0.0, true, true);
        std::cout << "corridor_exit_test: all checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
