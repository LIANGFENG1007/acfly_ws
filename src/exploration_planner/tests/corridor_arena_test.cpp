#include "corridor_arena_scene.hpp"
#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

using namespace exploration;

namespace {

constexpr Vec2 kEntry{8.25, 4.25};
constexpr Vec2 kH{8.25, -4.25};
constexpr Vec2 kRed{7.0, 4.25};

bool checkEntry(double voxel_size, const Vec2& position = kEntry) {
    CorridorPerceptionConfig config;
    config.corridor_width = 1.5;
    CorridorPerception perception(config);
    perception.setFrame(kEntry, kH);
    const auto cloud = corridor_arena::arenaCloud(position, voxel_size);
    const auto observed = perception.analyze(cloud, position);
    std::cout << "entry voxel=" << voxel_size << " xy=(" << position.x << ',' << position.y
              << ") points=" << cloud.size()
              << " walls=" << observed.walls_observed << " gate=" << observed.gate_observed
              << " passable=" << observed.gate_passable << " center=("
              << observed.gate_center.x << ',' << observed.gate_center.y << ')'
              << " width=" << observed.gap_width << " reason=" << observed.reason << '\n';
    const bool success = observed.gate_observed && observed.gate_passable &&
        std::abs(observed.gate_center.y - 0.9) < 0.12 &&
        std::abs(observed.gate_center.x - 7.925) < 0.12;
    if (!success) std::cerr << "FAIL: visible first door must be selected despite the rear end wall\n";
    return success;
}

bool checkUnobservedGap() {
    CorridorPerceptionConfig config;
    config.corridor_width = 1.5;
    CorridorPerception perception(config);
    perception.setFrame({0.0, 0.0}, {8.5, 0.0});
    Path2 points;
    for (int index = 0; index <= 75; ++index) {
        points.push_back({index * 0.04, -0.75});
        points.push_back({index * 0.04, 0.75});
    }
    for (double t : {-0.6, -0.35, -0.1, 0.5}) points.push_back({3.0, t});
    const auto observed = perception.analyze(points, {1.0, 0.0});
    std::cout << "missing-door-samples gate=" << observed.gate_observed
              << " passable=" << observed.gate_passable << " reason=" << observed.reason << '\n';
    const bool success = observed.gate_observed && !observed.gate_passable;
    if (!success) std::cerr << "FAIL: a missing column without any returns beyond the door is not free space\n";
    return success;
}

bool runMission(double voxel_size) {
    CorridorConfig config;
    config.perception.corridor_width = 1.5;
    config.perception.robot_width = 0.5;
    CorridorController controller(config);
    if (!controller.configure(kEntry, kH)) return false;
    controller.start(kRed, 0.0);
    Vec2 position = kRed;
    double yaw = 0.0, vx = 0.0, vy = 0.0, yaw_rate = 0.0;
    double now = 0.0, next_cloud = 0.0, last_motion = 0.0;
    double min_clearance = corridor_arena::bodyClearance(position), max_cross_speed = 0.0;
    bool saw_entry_lateral = false;
    bool finished = false;
    CorridorCommand command;
    CorridorPhase prior_phase = controller.phase();
    for (int step = 0; step < 10000; ++step) {
        if (now + 1e-9 >= next_cloud) {
            controller.observe(corridor_arena::arenaCloud(position, voxel_size), now, position);
            next_cloud += 0.1;
        }
        const double vf = std::cos(yaw) * vx + std::sin(yaw) * vy;
        const double vl = -std::sin(yaw) * vx + std::cos(yaw) * vy;
        command = controller.update(position, yaw, vf, vl, now, true);
        if (controller.phase() != prior_phase) {
            std::cout << "mission voxel=" << voxel_size << " t=" << now << " phase="
                      << static_cast<int>(controller.phase()) << " xy=(" << position.x << ','
                      << position.y << ") gates=" << controller.gatesPassed()
                      << " status=" << command.status << '\n';
            prior_phase = controller.phase();
        }
        if (controller.phase() == CorridorPhase::Entry && std::abs(command.lateral) > 0.05) {
            saw_entry_lateral = true;
        }
        if (controller.phase() == CorridorPhase::Cross) {
            max_cross_speed = std::max(max_cross_speed, std::hypot(command.forward, command.lateral));
        }
        const double response = 1.0 - std::exp(-0.02 / 0.15);
        const double target_vx = std::cos(yaw) * command.forward - std::sin(yaw) * command.lateral;
        const double target_vy = std::sin(yaw) * command.forward + std::cos(yaw) * command.lateral;
        vx += response * (target_vx - vx);
        vy += response * (target_vy - vy);
        yaw_rate += response * (command.yaw_rate - yaw_rate);
        position.x += vx * 0.02;
        position.y += vy * 0.02;
        yaw += yaw_rate * 0.02;
        min_clearance = std::min(min_clearance, corridor_arena::bodyClearance(position));
        if (!corridor_arena::bodyClear(position, 0.25)) {
            std::cerr << "FAIL: physical body intersects an arena wall at (" << position.x << ','
                      << position.y << "), phase=" << static_cast<int>(controller.phase()) << '\n';
            return false;
        }
        if (std::hypot(vx, vy) > 0.005 || std::abs(yaw_rate) > 0.005) last_motion = now;
        if (command.finished) { finished = true; break; }
        if (now - last_motion > 10.0) break;
        now += 0.02;
    }
    const double goal_distance = std::hypot(position.x - kH.x, position.y - kH.y);
    std::cout << "mission voxel=" << voxel_size << " finished=" << finished
              << " gates=" << controller.gatesPassed() << " xy=(" << position.x << ',' << position.y
              << ") clearance=" << min_clearance << " cross_speed=" << max_cross_speed
              << " status=" << command.status << '\n';
    if (!finished) {
        const auto& observed = controller.observation();
        std::cout << "last observation points=" << controller.points().size()
                  << " walls=" << observed.walls_observed << " left=" << observed.left_wall
                  << " right=" << observed.right_wall << " observed_until=" << observed.observed_until
                  << " gate=" << observed.gate_observed << " gate_s=" << observed.gate_s
                  << " reason=" << observed.reason << '\n';
    }
    const bool success = finished && controller.gatesPassed() == 2 && saw_entry_lateral &&
        goal_distance < 0.12 && max_cross_speed <= 0.300001 && min_clearance > 0.25;
    if (!success) std::cerr << "FAIL: actual arena visible-cloud mission did not safely complete both doors and H\n";
    return success;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--dump-entry-cloud") {
        std::cout << std::setprecision(17);
        for (const auto& point : corridor_arena::arenaCloud(kEntry, 0.3)) {
            std::cout << point.x << ' ' << point.y << '\n';
        }
        return 0;
    }
    bool success = true;
    success = checkEntry(0.0) && success;
    success = checkEntry(0.3) && success;
    for (double dx : {-0.08, 0.08}) {
        for (double dy : {-0.08, 0.08}) {
            success = checkEntry(0.0, {kEntry.x + dx, kEntry.y + dy}) && success;
            success = checkEntry(0.3, {kEntry.x + dx, kEntry.y + dy}) && success;
        }
    }
    success = checkUnobservedGap() && success;
    if (argc <= 1 || std::string(argv[1]) != "--perception-only") {
        success = runMission(0.0) && success;
        success = runMission(0.3) && success;
    }
    if (success) std::cout << "corridor_arena_test: all checks passed\n";
    return success ? 0 : 1;
}
