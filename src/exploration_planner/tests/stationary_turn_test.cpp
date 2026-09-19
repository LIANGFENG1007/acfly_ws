#include <cmath>
#include <iostream>
#include <stdexcept>
#include "exploration_planner/corridor_controller.hpp"
#include "exploration_planner/trajectory_tracker.hpp"

using namespace exploration;

void check(bool ok, const char* reason) { if (!ok) throw std::runtime_error(reason); }

void corridor_drift()
{
    CorridorConfig cfg;
    CorridorController controller(cfg);
    check(controller.configure({2, 0}, {2, -3}), "configure corridor");
    const Vec2 anchor{1, 0};
    controller.start(anchor, 0);
    double yaw = 0, rate = 0;
    Vec2 position = anchor, velocity{};
    bool corrected = false, completed = false;
    double last_rate = 0;
    for (int i = 1; i < 600; ++i) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        // Persistent odometry speed bias exceeds the old 0.04 m/s yaw gate.
        const double vx = velocity.x + .08, vy = velocity.y + .03;
        const auto cmd = controller.update(position, yaw, c * vx + s * vy,
                                            -s * vx + c * vy, i * .02, true);
        check(std::hypot(cmd.forward, cmd.lateral) <= cfg.align_speed + 1e-9,
              "rotation XY exceeds hold speed");
        check(std::abs(cmd.yaw_rate - last_rate) <= cfg.yaw_accel * .02 + 1e-8,
              "rotation yaw command jumps");
        last_rate = cmd.yaw_rate;
        check(cmd.target.x == anchor.x && cmd.target.y == anchor.y,
              "rotation anchor followed the drifting aircraft");
        const Vec2 world{c * cmd.forward - s * cmd.lateral, s * cmd.forward + c * cmd.lateral};
        corrected |= world.x < -.01;
        velocity.x += .15 * (world.x + .07 - velocity.x);
        velocity.y += .15 * (world.y + .04 - velocity.y);
        position.x += velocity.x * .02; position.y += velocity.y * .02;
        rate += .15 * (cmd.yaw_rate - rate);
        yaw += rate * .02;
        if (controller.phase() == CorridorPhase::Entry) { completed = true; break; }
    }
    check(corrected && completed, "biased/drifting aircraft paused or failed to finish yaw");
    check(std::hypot(position.x - anchor.x, position.y - anchor.y) < .20,
          "position hold did not contain wind drift");

    controller.reset();
    controller.configure({2, 0}, {2, -3});
    controller.start({1, 1}, 20);
    auto cmd = controller.update({1.2, 1.1}, 0, .1, .1, 20.02, false);
    check(cmd.forward == 0 && cmd.lateral == 0 && cmd.yaw_rate == 0,
          "lost odometry must still stop output");
    cmd = controller.update({1.2, 1.1}, 0, .1, .1, 20.04, true);
    check(cmd.forward < 0 && cmd.lateral < 0 && cmd.yaw_rate < 0 && cmd.target.y == 1,
          "resumed turn lost its new anchor or paused on horizontal speed");
    controller.observe({{1.1, 1.1}}, 20.05);
    cmd = controller.update({1.2, 1.1}, 0, .1, .1, 20.06, true);
    check(cmd.forward == 0 && cmd.lateral == 0 && cmd.yaw_rate < 0,
          "blocked XY correction cancelled yaw or ignored obstacle");
}

void corridor_heading_recovery()
{
    CorridorConfig cfg;
    CorridorController controller(cfg);
    controller.configure({2, 0}, {2, -3}); controller.start({1, 0}, 0);
    double now = 0;
    for (int i = 0; i < 20; ++i) {
        now += .02;
        controller.update({1, 0}, cfg.initial_yaw, .1, 0, now, true);
        if (controller.phase() == CorridorPhase::Entry) break;
    }
    check(controller.phase() == CorridorPhase::Entry, "horizontal bias blocked heading completion");
    controller.observe({{3, -2}, {3, -1}, {3, 0}}, now);
    auto cmd = controller.update({1, 0}, 0, .1, 0, now += .02, true);
    check(cmd.yaw_rate < 0 && cmd.forward < 0, "heading recovery froze on residual speed");
    for (int i = 0; i < 15; ++i) {
        controller.observe({{3, -2}, {3, -1}, {3, 0}}, now);
        cmd = controller.update({1.1, .1}, 0, .1, 0, now += .02, true);
    }
    check(cmd.yaw_rate < 0 && cmd.lateral < 0 && cmd.target.x == 1 && cmd.target.y == 0,
          "heading recovery does not hold a fixed XY anchor");
}

void tracker_drift(bool forward_only)
{
    TrackerGains g{};
    g.v_max = .6; g.lookahead = .5; g.endpoint_slow_r = .5;
    g.kp_yaw = 1.6; g.kd_yaw = .2; g.max_yaw_rate = .6;
    g.heading_gate_rad = .4; g.dt = .02; g.forward_only = forward_only;
    TrajectoryTracker tracker(g);
    tracker.set_trajectory({{{1, 0}, -M_PI / 2, 0, 0}, {{1, -3}, -M_PI / 2, 0, 3}});
    auto cmd = tracker.update(1, 0, 0, .1, .1, .1, 0);
    check(cmd.yaw_rate < 0 && cmd.holding_position, "tracker waits for perfect stop before yaw");
    for (int i = 0; i < 40; ++i) cmd = tracker.update(1.1, .1, 0, .1, .1, .1, 0);
    check(cmd.v_fwd < 0 && cmd.v_lat < 0 && cmd.yaw_rate < 0,
          "tracker did not correct both drift axes while rotating");
    check(std::hypot(cmd.v_fwd, cmd.v_lat) <= g.turn_hold_speed + 1e-9,
          "tracker hold exceeds speed limit");
    tracker.set_trajectory({{{5, 0}, -M_PI / 2, 0, 0}, {{5, -3}, -M_PI / 2, 0, 3}});
    cmd = tracker.update(5, 0, 0, 0, 0, .1, 0);
    check(cmd.v_fwd == 0 && cmd.v_lat == 0, "new route inherited old turn anchor");
}

void sparse_cloud_window()
{
    CorridorConfig cfg;
    cfg.cloud_window = 1.0;
    cfg.perception.corridor_width = 1.5;
    CorridorController controller(cfg);
    controller.configure({0, 0}, {0, -5});
    controller.start({0, 0}, 0);
    Path2 walls;
    for (int i = 0; i <= 100; ++i) {
        walls.push_back({-.75, -i * .05}); walls.push_back({.75, -i * .05});
    }
    controller.observe(walls, .01, {0, 0});
    controller.update({0, 0}, 0, 0, 0, .02, true);
    check(controller.observation().walls_observed, "walls are invisible throughout the yaw stage");
    controller.observe({}, .06);
    check(!controller.points().empty(), "one empty packet erased accumulated wall geometry");
    // Repeated empty packets cannot extend validity or confirm a door.
    for (int i = 1; i <= 30; ++i) {
        const double now = .06 + i * .02;
        controller.observe({}, now);
        controller.update({0, 0}, cfg.initial_yaw, .1, 0, now, true);
    }
    check(!controller.gateLocked(), "empty packets created a confirmed gate");
    const auto stopped = controller.update({0, 0}, cfg.initial_yaw, .1, 0, .70, true);
    check(stopped.forward == 0 && stopped.lateral == 0,
          "one-second geometry cache bypassed the independent data timeout");
    controller.observe({}, 1.10);
    check(controller.points().empty(), "expired one-second wall points stayed in the window");
}

void waiting_drift()
{
    CorridorConfig cfg;
    CorridorController controller(cfg);
    controller.configure({0, 0}, {0, -4}); controller.start({0, 0}, 0);
    const Path2 incomplete_wall{{1, -.5}, {1, -1}, {1, -1.5}};
    double now = 0;
    for (int i = 0; i < 25; ++i) {
        now += .02;
        controller.observe(incomplete_wall, now);
        controller.update({0, 0}, cfg.initial_yaw, 0, 0, now, true);
    }
    check(controller.phase() == CorridorPhase::Search, "wait fixture did not reach corridor search");
    CorridorCommand cmd;
    for (int i = 0; i < 30; ++i) {
        now += .02;
        controller.observe(incomplete_wall, now);
        cmd = controller.update({.1, .1}, cfg.initial_yaw + .2, 0, .1, now, true);
    }
    const double yaw = cfg.initial_yaw + .2;
    const Vec2 world{std::cos(yaw) * cmd.forward - std::sin(yaw) * cmd.lateral,
                     std::sin(yaw) * cmd.forward + std::cos(yaw) * cmd.lateral};
    check(world.x < 0 && world.y < 0 && cmd.target.x == 0 && cmd.target.y == 0,
          "waiting for door evidence did not retain its fixed XY position");
    check(cmd.yaw_rate < -.10, "evidence-wait branch resets yaw acceleration every tick");
}

int main()
{
    corridor_drift(); corridor_heading_recovery();
    tracker_drift(true); tracker_drift(false);
    sparse_cloud_window();
    waiting_drift();
    std::cout << "stationary yaw with XY correction, drift, recovery, limits and reset passed\n";
}
