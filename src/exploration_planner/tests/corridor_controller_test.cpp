#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace exploration;

namespace {
constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

double distance(const Vec2& a, const Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

struct Scene {
    CorridorConfig config;
    Vec2 entry{7.75, 4.25};
    Vec2 h{7.75, -4.25};
    Vec2 red{7.0, 4.25};
    Path2 cloud;
    CorridorPerception frame;

    explicit Scene(double separation, bool closed = false) {
        config.perception.corridor_width = 1.5;
        config.perception.robot_width = 0.6; // Also verify the larger previous aircraft envelope.
        frame = CorridorPerception(config.perception);
        frame.setFrame(entry, h);
        for (int i = 18; i <= 355; ++i) {
            cloud.push_back(frame.toWorld(i * 0.025, -0.75));
            cloud.push_back(frame.toWorld(i * 0.025, 0.75));
        }
        if (separation > 0.0) {
            for (int i = 0; i <= 28; ++i) {
                cloud.push_back(frame.toWorld(3.0, 0.05 + i * 0.025));
                cloud.push_back(frame.toWorld(3.0 + separation, -0.75 + i * 0.025));
            }
        }
        if (closed) {
            for (int i = 0; i <= 60; ++i) cloud.push_back(frame.toWorld(3.0, -0.75 + i * 0.025));
        }
    }
};

struct Simulation {
    Scene scene;
    CorridorController controller;
    Vec2 position;
    double yaw = 0.0;
    double vx = 0.0, vy = 0.0, yaw_rate = 0.0;
    double now = 0.0;
    bool saw_entry_lateral = false;
    double max_cross_speed = 0.0;
    double minimum_clearance = 100.0;

    explicit Simulation(double separation, bool closed = false)
        : scene(separation, closed), controller(scene.config), position(scene.red) {
        require(controller.configure(scene.entry, scene.h), "valid supplied route is accepted");
        controller.start(scene.red, now);
    }

    CorridorCommand step(bool publish_cloud = true, bool empty = false,
                         bool fresh_pose = true, bool dynamics = true) {
        if (publish_cloud) controller.observe(empty ? Path2{} : scene.cloud, now);
        const double vf = std::cos(yaw) * vx + std::sin(yaw) * vy;
        const double vl = -std::sin(yaw) * vx + std::cos(yaw) * vy;
        auto command = controller.update(position, yaw, vf, vl, now, fresh_pose);
        if (controller.phase() == CorridorPhase::Entry && command.lateral > 0.05) {
            saw_entry_lateral = true;
            require(std::abs(std::atan2(std::sin(yaw + kPi / 2.0), std::cos(yaw + kPi / 2.0))) < 0.10,
                    "entry translation preserves the requested -90 degree heading");
        }
        if (controller.phase() == CorridorPhase::Cross) {
            max_cross_speed = std::max(max_cross_speed, std::hypot(command.forward, command.lateral));
            require(max_cross_speed <= 0.300001, "door crossing speed respects the 0.3 m/s limit");
        }
        if (dynamics) {
            const double dt = 0.02;
            const double response = 1.0 - std::exp(-dt / 0.15);
            const double commanded_x = std::cos(yaw) * command.forward - std::sin(yaw) * command.lateral;
            const double commanded_y = std::sin(yaw) * command.forward + std::cos(yaw) * command.lateral;
            vx += response * (commanded_x - vx);
            vy += response * (commanded_y - vy);
            yaw_rate += response * (command.yaw_rate - yaw_rate);
            position.x += vx * dt;
            position.y += vy * dt;
            yaw += yaw_rate * dt;
            for (const auto& p : scene.cloud) minimum_clearance = std::min(minimum_clearance, distance(position, p));
        }
        now += 0.02;
        return command;
    }
};

void runMission(double separation) {
    Simulation simulation(separation);
    bool finished = false;
    for (int i = 0; i < 7000; ++i) {
        const auto command = simulation.step();
        if (command.finished) { finished = true; break; }
    }
    if (!finished) {
        const auto command = simulation.step();
        std::cerr << "separation=" << separation << " phase=" << static_cast<int>(simulation.controller.phase())
                  << " s=" << simulation.scene.frame.longitudinal(simulation.position)
                  << " t=" << simulation.scene.frame.lateral(simulation.position)
                  << " status=" << command.status << '\n';
    }
    require(finished && simulation.controller.gatesPassed() == 2,
            "alternating doors complete through H with 0.15 s velocity response");
    require(simulation.saw_entry_lateral, "red point to entry uses the required lateral translation");
    require(simulation.max_cross_speed > 0.29, "crossing reaches the configured cruise speed");
    require(simulation.minimum_clearance > 0.299,
            "simulated full aircraft radius remains clear of all raw wall and bar points");
    require(distance(simulation.position, simulation.scene.h) < simulation.scene.config.point_tolerance * 1.5,
            "completion occurs near the supplied H coordinate");
}

void startInside(Simulation& simulation) {
    simulation.position = simulation.scene.entry;
    simulation.yaw = -kPi / 2.0;
    for (int i = 0; i < 45; ++i) simulation.step();
}

}  // namespace

int main() {
    runMission(0.8);
    runMission(1.0);
    runMission(1.5);

    Simulation empty(1.0);
    startInside(empty);
    for (int i = 0; i < 40; ++i) {
        const auto command = empty.step(true, true);
        if (i > 16) require(command.forward == 0.0 && command.lateral == 0.0 && !command.finished,
                "empty packets extended the 0.3-second lifetime of measured free space");
    }
    Simulation stale(1.0);
    startInside(stale);
    for (int i = 0; i < 70; ++i) {
        const auto command = stale.step(false);
        if (i > 30) require(command.forward == 0.0 && command.lateral == 0.0 && !command.finished,
                            "a frozen cloud stream stops motion after the timeout");
    }
    Simulation bad_pose(1.0);
    startInside(bad_pose);
    const auto paused = bad_pose.step(true, false, false);
    require(paused.forward == 0.0 && paused.lateral == 0.0 && !paused.finished,
            "stale odometry stops an active corridor task");

    Simulation closed(1.0, true);
    startInside(closed);
    for (int i = 0; i < 60; ++i) closed.step();
    require(closed.controller.gatesPassed() == 0 &&
            closed.controller.phase() != CorridorPhase::Done &&
            closed.scene.frame.longitudinal(closed.position) < 1.0,
            "closed first door is never completed or traversed");

    Simulation drift(1.0);
    startInside(drift);
    for (int i = 0; i < 1600 && drift.controller.phase() != CorridorPhase::Cross; ++i) drift.step();
    require(drift.controller.phase() == CorridorPhase::Cross, "drift case reaches the first crossing");
    const double gate_s = drift.controller.gate().gate_s;
    drift.position = drift.scene.frame.toWorld(gate_s + 0.8, 0.25);
    drift.vx = drift.vy = 0.0;
    for (int i = 0; i < 30; ++i) {
        const auto command = drift.step(true, false, true, false);
        require(drift.controller.gatesPassed() == 0 && !command.finished,
                "longitudinal progress with an off-center body cannot count as passing a door");
    }

    Simulation cleared(3.0);
    startInside(cleared);
    for (int i = 0; i < 1600 && cleared.controller.phase() != CorridorPhase::Cross; ++i) cleared.step();
    require(cleared.controller.phase() == CorridorPhase::Cross, "cleared-body case reaches the first crossing");
    const auto crossed_gate = cleared.controller.gate();
    const double gap_center = 0.5 * (crossed_gate.gap_left + crossed_gate.gap_right);
    cleared.position = cleared.scene.frame.toWorld(
        crossed_gate.gate_s + crossed_gate.gate_depth * 0.5 + cleared.scene.config.exit_distance + 0.01,
        gap_center + 0.05);
    cleared.vx = 0.0;
    cleared.vy = -0.10;
    const auto handoff = cleared.step(true, false, true, false);
    require(cleared.controller.gatesPassed() == 1 && !handoff.finished,
            "after the straight exit a cleared body must hand off without recentering on the old door");
    require(std::hypot(handoff.forward, handoff.lateral) > 0.0 && handoff.forward >= -1e-9,
            "a door handoff must keep moving and never reverse to the old exit point");

    Simulation endpoint(0.0);
    startInside(endpoint);
    endpoint.position = endpoint.scene.frame.toWorld(8.5 - 0.07965, 0.00922);
    endpoint.vx = endpoint.vy = 0.0;
    bool endpoint_finished = false;
    for (int i = 0; i < 120; ++i) {
        const auto command = endpoint.step();
        if (command.finished) { endpoint_finished = true; break; }
    }
    require(endpoint_finished,
            "radial H tolerance completes even when the remaining axial distance is below point tolerance");

    CorridorConfig invalid;
    invalid.acceleration = 0.0;
    bool rejected = false;
    try { CorridorController controller(invalid); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "zero acceleration is rejected before a mission can run");
    invalid = CorridorConfig{};
    invalid.perception.cell_size = std::numeric_limits<double>::quiet_NaN();
    rejected = false;
    try { CorridorController controller(invalid); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "nonfinite perception resolution is rejected before quantization");

    std::cout << "corridor_controller_test: all checks passed\n";
    return 0;
}
