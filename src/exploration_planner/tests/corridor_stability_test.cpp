#include "corridor_arena_scene.hpp"
#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {

constexpr double kDt = 0.02;

double norm(const Vec2& p) { return std::hypot(p.x, p.y); }
double distance(const Vec2& a, const Vec2& b) { return norm({a.x - b.x, a.y - b.y}); }
double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

Vec2 worldVelocity(const CorridorCommand& cmd, double yaw) {
    return {std::cos(yaw) * cmd.forward - std::sin(yaw) * cmd.lateral,
            std::sin(yaw) * cmd.forward + std::cos(yaw) * cmd.lateral};
}

struct Fixture {
    CorridorConfig config;
    CorridorController controller{config};
    CorridorPerception frame{config.perception};
    Vec2 entry{8.25, 4.25}, goal{8.25, -4.25}, position = entry;
    Vec2 velocity;
    Path2 cloud;
    double yaw = config.initial_yaw;
    double now = 0.0;

    explicit Fixture(bool gates = false, bool closed = false) {
        frame.setFrame(entry, goal);
        for (int i = -28; i <= 356; ++i) {
            cloud.push_back(frame.toWorld(i * 0.025, -0.75));
            cloud.push_back(frame.toWorld(i * 0.025, 0.75));
        }
        if (gates) {
            for (int i = 0; i <= 28; ++i) {
                cloud.push_back(frame.toWorld(3.0, 0.05 + i * 0.025));
                cloud.push_back(frame.toWorld(5.0, -0.75 + i * 0.025));
            }
        }
        if (closed) {
            for (int i = 0; i <= 60; ++i)
                cloud.push_back(frame.toWorld(3.0, -0.75 + i * 0.025));
        }
        require(controller.configure(entry, goal), "fixture route is valid");
        controller.start(position, now);
    }

    CorridorCommand step(bool dynamics = false, bool publish_cloud = true,
                         bool pose_fresh = true, const Vec2& noise = {}) {
        if (publish_cloud) controller.observe(cloud, now);
        const Vec2 measured{velocity.x + noise.x, velocity.y + noise.y};
        const double vf = std::cos(yaw) * measured.x + std::sin(yaw) * measured.y;
        const double vl = -std::sin(yaw) * measured.x + std::cos(yaw) * measured.y;
        auto cmd = controller.update(position, yaw, vf, vl, now, pose_fresh);
        if (dynamics) {
            const auto target = worldVelocity(cmd, yaw);
            const double response = 1.0 - std::exp(-kDt / 0.15);
            velocity.x += response * (target.x - velocity.x);
            velocity.y += response * (target.y - velocity.y);
            position.x += velocity.x * kDt;
            position.y += velocity.y * kDt;
            yaw += cmd.yaw_rate * kDt;
        }
        now += kDt;
        return cmd;
    }

    void enter() {
        for (int i = 0; i < 30 && controller.phase() != CorridorPhase::Entry; ++i) step();
        require(controller.phase() == CorridorPhase::Entry, "initial heading settles into Entry");
    }

    Vec2 staging() const {
        const auto& gate = controller.gate();
        return frame.toWorld(gate.gate_s - config.approach_distance,
                             0.5 * (gate.gap_left + gate.gap_right));
    }

    void approach() {
        for (int i = 0; i < 100 && controller.phase() != CorridorPhase::Approach; ++i) step();
        require(controller.phase() == CorridorPhase::Approach, "visible first door reaches Approach");
    }
};

void checkEntryNoise() {
    Fixture fixture;
    fixture.position.x -= 0.4;
    fixture.enter();
    double max_command = 0.0;
    int departures = 0;
    for (int i = 0; i < 100; ++i) {
        const double noise = (i / 5) % 2 == 0 ? -1.0 : 1.0;
        fixture.position = {fixture.entry.x - 0.078 + 0.012 * noise, fixture.entry.y};
        const auto cmd = fixture.step(false, true, true, {0.06 * noise, 0.0});
        max_command = std::max(max_command, norm(worldVelocity(cmd, fixture.yaw)));
        if (fixture.controller.phase() != CorridorPhase::Entry) { ++departures; break; }
    }
    std::cout << "entry-noise departure=" << departures << " elapsed=" << fixture.now
              << " max_command=" << max_command << '\n';
    require(departures == 1, "10 Hz +/-0.06 m/s noise must not permanently prevent entry handoff");
    require(max_command < 0.06, "entry boundary noise must not cause large alternating corrections");

    Fixture outside;
    outside.position.x -= 0.20;
    outside.enter();
    double final_command = 0.0;
    for (int i = 0; i < 300; ++i) {
        final_command = norm(worldVelocity(outside.step(), outside.yaw));
        require(outside.controller.phase() == CorridorPhase::Entry,
                "time alone must not accept a point that was never reached");
    }
    require(final_command > 0.10, "a genuinely unreached entry must retain corrective motion");

    Fixture escaped;
    escaped.enter();
    escaped.step();
    escaped.position.x -= escaped.config.point_tolerance * escaped.config.arrival_hysteresis + 0.03;
    for (int i = 0; i < 40; ++i) {
        const auto cmd = escaped.step();
        require(escaped.controller.phase() == CorridorPhase::Search,
                "a reached entry must not restart the lateral entry leg after handoff");
        const Vec2 motion = worldVelocity(cmd, escaped.yaw);
        require(motion.x > 0.0 && motion.y < 0.0,
                "entry drift must be corrected while continuing down the observed corridor");
    }
}

void checkDampingAndAcceleration() {
    Fixture damped;
    damped.position.x -= 0.09;
    damped.enter();
    double maximum = 0.0;
    for (int i = 0; i < 60; ++i) {
        maximum = std::max(maximum, norm(worldVelocity(damped.step(), damped.yaw)));
    }
    std::cout << "near-entry error=0.09 maximum=" << maximum << '\n';
    require(maximum < 0.10 && maximum > 0.01,
            "9 cm remaining must command materially less than the former 0.21 m/s");

    Fixture reversing;
    reversing.position = {reversing.entry.x - 0.3, reversing.entry.y - 0.2};
    reversing.enter();
    Vec2 previous;
    double max_acceleration = 0.0;
    bool positive = false, negative = false;
    for (int i = 0; i < 240; ++i) {
        if (i == 100) reversing.position = {reversing.entry.x + 0.3, reversing.entry.y + 0.2};
        const auto cmd = reversing.step();
        const Vec2 current = worldVelocity(cmd, reversing.yaw);
        max_acceleration = std::max(max_acceleration, distance(current, previous) / kDt);
        positive = positive || current.x > 0.10;
        negative = negative || current.x < -0.10;
        require(reversing.controller.phase() == CorridorPhase::Entry,
                "reversal fixture must remain outside arrival tolerance");
        require(distance(current, previous) <= reversing.config.acceleration * kDt + 1e-9,
                "the vector acceleration limit must hold through two-axis direction reversal");
        previous = current;
    }
    std::cout << "vector reversal max_acceleration=" << max_acceleration << '\n';
    require(positive && negative, "reversal test must exercise both commanded directions");
}

void checkMovingDoorAlignment() {
    for (const double scale : {1.25, 2.5}) {
        Fixture fixture(true);
        fixture.approach();
        const Vec2 staging = fixture.staging();
        const double center = fixture.frame.lateral(staging);
        fixture.position = fixture.frame.toWorld(fixture.frame.longitudinal(staging) - 0.30,
            center + fixture.config.center_tolerance * scale);
        fixture.velocity = {0.0, -0.15};
        const auto correction = fixture.step();
        const Vec2 commanded = worldVelocity(correction, fixture.yaw);
        const Vec2 error{staging.x - fixture.position.x, staging.y - fixture.position.y};
        require(commanded.x * error.x + commanded.y * error.y > 0.0,
                "an approaching gate with lateral error must actively correct toward its center");
        require(fixture.controller.phase() == CorridorPhase::Approach,
                "a gate must not be crossed before lateral alignment is restored");
        bool crossed = false;
        for (int i = 0; i < 400; ++i) {
            const auto cmd = fixture.step(true);
            require(norm(worldVelocity(cmd, fixture.yaw)) > 1e-6,
                    "moving door alignment must not add a planned hold");
            if (fixture.controller.phase() == CorridorPhase::Cross) { crossed = true; break; }
        }
        std::cout << "gate-center offset=" << fixture.config.center_tolerance * scale
                  << " crossed=" << crossed << " elapsed=" << fixture.now << '\n';
        require(crossed, "door-center errors must recover while approaching the door");
        require(std::abs(fixture.frame.lateral(fixture.position) - center) <= fixture.config.center_tolerance,
                "crossing begins only after measured lateral alignment is restored");
        require(norm(fixture.velocity) > fixture.config.stop_speed,
                "door crossing must start while already moving");
        require(fixture.controller.gate().gate_s - fixture.frame.longitudinal(fixture.position) > 1.0,
                "door alignment must finish more than one metre ahead in the normal scene");
    }
}

void checkPredictedLateralDrift() {
    Fixture fixture(true);
    fixture.approach();
    const Vec2 staging = fixture.staging();
    const double center = fixture.frame.lateral(staging);
    fixture.position = fixture.frame.toWorld(fixture.frame.longitudinal(staging), center + 0.10);
    fixture.velocity = {0.25, 0.0};
    for (int i = 0; i < 80; ++i) {
        fixture.step();
        require(fixture.controller.phase() == CorridorPhase::Approach,
                "a laterally displaced aircraft cannot enter Cross");
    }
    fixture.position = staging;
    const auto braking = fixture.step();
    require(fixture.controller.phase() == CorridorPhase::Approach,
            "being centered with outward lateral inertia must not immediately start crossing");
    require(worldVelocity(braking, fixture.yaw).x < 0.0,
            "outward lateral inertia must be actively damped");
    bool crossed = false;
    for (int i = 0; i < 400; ++i) {
        fixture.step(true);
        if (fixture.controller.phase() == CorridorPhase::Cross) { crossed = true; break; }
    }
    require(crossed, "lateral inertia must recover without permanently preventing crossing");
    require(std::abs(fixture.frame.lateral(fixture.position) - center) <= fixture.config.center_tolerance,
            "lateral drift recovery must enter Cross within measured center tolerance");
    std::cout << "predicted-lateral-drift recovered=" << crossed << " elapsed=" << fixture.now << '\n';
}

void checkSafetyStops() {
    Fixture stale;
    stale.position.x -= 0.4;
    stale.enter();
    for (int i = 0; i < 30; ++i) stale.step();
    const auto moving = stale.step();
    require(norm(worldVelocity(moving, stale.yaw)) > 0.1, "stale-cloud check begins while moving");
    for (int i = 0; i < 60; ++i) {
        const auto cmd = stale.step(false, false);
        if (i > 30) require(cmd.forward == 0.0 && cmd.lateral == 0.0 && cmd.yaw_rate == 0.0 && !cmd.finished,
                            "expired cloud must stop all corridor commands");
    }
    const auto bad_pose = stale.step(false, true, false);
    require(bad_pose.forward == 0.0 && bad_pose.lateral == 0.0 && !bad_pose.finished,
            "invalid odometry must stop translation");
    stale.cloud.clear();
    const auto empty = stale.step();
    require(empty.forward == 0.0 && empty.lateral == 0.0 && !empty.finished,
            "an empty current cloud cannot reuse accumulated free-space evidence");

    Fixture blocked(false, true);
    for (int i = 0; i < 300; ++i) {
        const auto cmd = blocked.step(true);
        require(blocked.controller.gatesPassed() == 0 && !cmd.finished,
                "closed gates remain blocking despite fixed-heading motion and arrival hysteresis");
    }
    require(blocked.frame.longitudinal(blocked.position) < 0.1,
            "a closed first gate must prevent departure from the entry");
}

void checkFixedHeadingArena() {
    CorridorConfig config;
    CorridorController controller(config);
    // H is to the side of the final door's center, requiring a lateral correction.
    const Vec2 entry{8.25, 4.25}, goal{8.25, -4.25}, red{7.0, 4.25};
    require(controller.configure(entry, goal), "arena route with a laterally offset H is valid");
    controller.start(red, 0.0);
    Vec2 position = red, velocity;
    double yaw = config.initial_yaw, now = 0.0;
    double max_heading_error = 0.0, max_yaw_command = 0.0, min_clearance = 100.0;
    double max_cross_speed = 0.0;
    bool lateral_entry = false, lateral_gate = false, lateral_finish = false;
    bool finished = false;
    CorridorCommand command;
    for (int i = 0; i < 12000; ++i) {
        if (i % 5 == 0) controller.observe(corridor_arena::arenaCloud(position, 0.3), now, position);
        const double vf = std::cos(yaw) * velocity.x + std::sin(yaw) * velocity.y;
        const double vl = -std::sin(yaw) * velocity.x + std::cos(yaw) * velocity.y;
        command = controller.update(position, yaw, vf, vl, now, true);
        const auto phase = controller.phase();
        if (phase != CorridorPhase::Rotate) {
            max_yaw_command = std::max(max_yaw_command, std::abs(command.yaw_rate));
            max_heading_error = std::max(max_heading_error, std::abs(angle(yaw - config.initial_yaw)));
        }
        if (phase == CorridorPhase::Entry && std::abs(command.lateral) > 0.02) lateral_entry = true;
        if (phase == CorridorPhase::Approach && std::abs(command.lateral) > 0.02) lateral_gate = true;
        if (controller.gatesPassed() == 2 && std::abs(command.lateral) > 0.005) lateral_finish = true;
        if (phase == CorridorPhase::Cross)
            max_cross_speed = std::max(max_cross_speed, std::hypot(command.forward, command.lateral));
        const Vec2 target = worldVelocity(command, yaw);
        const double response = 1.0 - std::exp(-kDt / 0.15);
        velocity.x += response * (target.x - velocity.x);
        velocity.y += response * (target.y - velocity.y);
        position.x += velocity.x * kDt;
        position.y += velocity.y * kDt;
        yaw += command.yaw_rate * kDt;
        min_clearance = std::min(min_clearance, corridor_arena::bodyClearance(position));
        require(corridor_arena::bodyClear(position, config.perception.robot_width * 0.5),
                "aircraft body must remain outside thick walls and door bars in the physical arena");
        now += kDt;
        if (command.finished) { finished = true; break; }
    }
    std::cout << "fixed-heading arena finished=" << finished << " gates=" << controller.gatesPassed()
              << " duration=" << now << " xy=(" << position.x << ',' << position.y
              << ") yaw_error=" << max_heading_error << " yaw_command=" << max_yaw_command
              << " clearance=" << min_clearance << " crossing_speed=" << max_cross_speed
              << " status=" << command.status << '\n';
    require(finished && controller.gatesPassed() == 2, "offset-H arena mission must finish both observed doors");
    require(lateral_entry && lateral_gate && lateral_finish,
            "fixed-heading verification must include entry, door and landing-point lateral translations");
    require(max_heading_error < 1e-9 && max_yaw_command < 1e-9,
            "entry, doors and a sideways H must not turn the aircraft toward position targets");
    require(max_cross_speed <= config.gate_speed + 1e-9 && max_cross_speed > 0.20,
            "door traversal must make progress within the configured 0.3 m/s speed limit");
    require(distance(position, goal) <= config.point_tolerance * config.arrival_hysteresis,
            "mission completion must occur at the offset landing point");
}

}  // namespace

int main() {
    int failures = 0;
    const auto run = [&failures](const char* name, const std::function<void()>& test) {
        try { test(); std::cout << "PASS: " << name << '\n'; }
        catch (const std::exception& error) {
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
            ++failures;
        }
    };
    run("entry noise and moving handoff", checkEntryNoise);
    run("near-entry damping and vector reversal", checkDampingAndAcceleration);
    run("early door alignment while moving", checkMovingDoorAlignment);
    run("door-center prediction with lateral inertia", checkPredictedLateralDrift);
    run("stale data and closed-door safety", checkSafetyStops);
    run("fixed heading through physical arena to offset H", checkFixedHeadingArena);
    if (failures == 0) std::cout << "corridor_stability_test: all checks passed\n";
    return failures == 0 ? 0 : 1;
}
