// Run with: ctest --test-dir build/exploration_planner -R '^required_node_test$' --output-on-failure
// The test calls node methods directly and never spins an executor or a simulator.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <limits>
#include <thread>
#include <unordered_set>
#include <iostream>
#include <stdexcept>
#include "exploration_planner/params.hpp"
#include "exploration_planner/types.hpp"
#include "exploration_planner/grid_map.hpp"
#include "exploration_planner/coverage_planner.hpp"
#include "exploration_planner/bezier.hpp"
#include "exploration_planner/trajectory_tracker.hpp"
#include "exploration_planner/visualizer.hpp"
#include "exploration_planner/obstacle_map.hpp"
#include "exploration_planner/global_planner.hpp"
#include "exploration_planner/corridor_controller.hpp"
#include "exploration_planner/sensor_freshness.hpp"

// Dependencies are already included; this access override affects only the node.
#define private public
#define main exploration_original_main
#include "../src/exploration_planner_node.cpp"
#undef main
#undef private

namespace {

void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}

void reset(ExplorationNode& node, Vec2 start)
{
    node.px_ = start.x; node.py_ = start.y; node.pz_ = .8; node.yaw_ = 0;
    node.v_fwd_est_ = node.v_lat_est_ = node.yaw_rate_est_ = 0;
    node.global_tracker_->set_trajectory({});
    node.global_has_ = node.global_failed_ = node.global_goal_blocked_ = false;
    node.global_at_goal_ = node.global_braking_blocked_ = false;
    node.command_projection_blocked_ = node.measured_projection_blocked_ = false;
    node.required_blocked_time_valid_ = false;
    node.global_traj_.clear(); node.global_raw_.clear();
    node.retreating_ = false;
}

void closed_loop(ExplorationNode& node, const std::string& name, Vec2 start, Vec2 goal,
                 const Obstacles& obstacles)
{
    reset(node, start);
    double vx = 0, vy = 0, rate = 0;
    constexpr double dt = .02;
    const double response = 1.0 - std::exp(-dt / .15);
    const uint64_t first_search = node.global_path_searches_;
    const uint64_t first_brake = node.global_velocity_brakes_;
    double minimum = 1000;
    for (int i = 0; i < 12000; ++i) {
        const double c = std::cos(node.yaw_), s = std::sin(node.yaw_);
        node.v_fwd_est_ = c * vx + s * vy;
        node.v_lat_est_ = -s * vx + c * vy;
        node.yaw_rate_est_ = rate;
        const Vec2 cmd = node.pd_to_point_avoid(goal, obstacles);
        require(!node.global_failed_, name + ": valid mission lost its route: " + node.global_block_reason_);
        require(required_path_clear({node.px_, node.py_}, node.global_raw_, goal, obstacles, node.ggcfg_),
                name + ": accepted execution path invalid");
        require(std::hypot(cmd.x, cmd.y) <= node.v_goal_max_ + 1e-9, name + ": speed exceeds cap");
        if (node.global_at_goal_ && std::hypot(vx, vy) <= node.goal_stop_v_ && !node.global_braking_blocked_) {
            std::cout << name << ": reached in " << i * dt << "s, clearance=" << minimum
                      << ", searches=" << node.global_path_searches_ - first_search
                      << ", brakes=" << node.global_velocity_brakes_ - first_brake << '\n';
            return;
        }
        vx += response * (c * cmd.x - s * cmd.y - vx);
        vy += response * (s * cmd.x + c * cmd.y - vy);
        rate += response * (node.last_yaw_rate_ - rate);
        node.px_ += vx * dt; node.py_ += vy * dt;
        node.yaw_ = std::atan2(std::sin(node.yaw_ + rate * dt), std::cos(node.yaw_ + rate * dt));
        for (const auto& obstacle : obstacles) {
            const double clearance = std::hypot(node.px_ - obstacle.cx, node.py_ - obstacle.cy) - obstacle.r;
            minimum = std::min(minimum, clearance);
            require(clearance >= node.ggcfg_.robot_radius, name + ": physical collision");
        }
    }
    throw std::runtime_error(name + ": timeout at " + std::to_string(node.px_) + "," +
        std::to_string(node.py_) + " remaining=" + std::to_string(node.global_tracker_->remaining_distance()) +
        " reason=" + node.global_block_reason_);
}

void seed26_reference(ExplorationNode& node)
{
    reset(node, {4.459, 1.632});
    node.yaw_ = .59625;
    const Vec2 center{4.086, 2.536};
    const Vec2 goal{7, 4.25};
    const Obstacles obstacles{{center.x, center.y, .40}};
    const double angle = std::atan2(node.py_ - center.y, node.px_ - center.x);
    Path2 path;
    for (int i = 0; i <= 24; ++i) {
        const double theta = angle * (1.0 - i / 24.0);
        path.push_back({center.x + 1.03 * std::cos(theta), center.y + 1.03 * std::sin(theta)});
    }
    node.global_connector_start_ = {6.2, 3.75};
    path.push_back(node.global_connector_start_);
    path.push_back(goal);
    require(required_path_clear({node.px_, node.py_}, path, goal, obstacles, node.ggcfg_),
            "seed26 regression requires a valid outer-margin reference");
    node.global_raw_ = path;
    node.global_traj_ = node.make_polyline_trajectory(path);
    node.global_tracker_->set_trajectory(node.global_traj_);
    node.global_target_ = goal;
    node.global_has_ = true;
}

void seed26_motion_regression(ExplorationNode& node)
{
    const Obstacles obstacles{{4.086, 2.536, .40}};
    const Vec2 goal{7, 4.25};
    const double motion_margin = node.required_motion_inflate_;
    const double planning_margin = node.ggcfg_.inflate;
    seed26_reference(node);
    node.required_motion_inflate_ = planning_margin;
    const Vec2 old_command = node.pd_to_point_avoid(goal, obstacles);
    require(old_command.x == 0 && old_command.y == 0 && node.command_projection_blocked_,
            "seed26 reproduction must block the inward pursuit chord with the planning margin");
    require(!node.required_velocity_clear({.001 * std::cos(node.yaw_), .001 * std::sin(node.yaw_)}, obstacles),
            "low-speed desired motion must still be checked");
    node.required_motion_inflate_ = motion_margin;
    const auto searches = node.global_path_searches_;
    const Vec2 resumed = node.pd_to_point_avoid(goal, obstacles);
    require(resumed.x > 0 && !node.global_braking_blocked_ && node.global_path_searches_ == searches,
            "inner motion margin must release the valid seed26 reference");
    require(node.ggcfg_.inflate == planning_margin, "motion guard must not change the planning margin");

    seed26_reference(node);
    node.required_motion_inflate_ = planning_margin;
    node.yaw_ = 3.0;
    const Vec2 noise{0, .001};
    node.v_fwd_est_ = std::sin(node.yaw_) * noise.y;
    node.v_lat_est_ = std::cos(node.yaw_) * noise.y;
    require(!node.required_velocity_clear(noise, obstacles), "test noise must point into the old margin");
    node.pd_to_point_avoid(goal, obstacles);
    require(!node.command_projection_blocked_ && !node.measured_projection_blocked_,
            "stationary measured noise must not block safe in-place alignment");

    seed26_reference(node);
    node.pd_to_point_avoid(goal, obstacles);
    require(node.global_braking_blocked_ && node.required_blocked_time_valid_,
            "blocked and stopped motion starts recovery timer");
    const auto replans = node.required_blocked_replans_;
    node.required_blocked_since_ = ExplorationNode::steady_seconds() - node.required_blocked_replan_s_ - .1;
    const Vec2 held = node.pd_to_point_avoid(goal, obstacles);
    require(held.x == 0 && held.y == 0 && !node.global_has_ && !node.retreating_ &&
            node.required_blocked_replans_ == replans + 1,
            "persistent stopped blockage must invalidate only the required cache and hold");
    const auto before_search = node.global_path_searches_;
    node.required_motion_inflate_ = motion_margin;
    node.pd_to_point_avoid(goal, obstacles);
    require(node.global_path_searches_ > before_search && !node.global_failed_ &&
            required_path_clear({node.px_, node.py_}, node.global_raw_, goal, obstacles, node.ggcfg_),
            "blocked recovery must produce another fully validated path");
    std::cout << "seed26 motion-margin, stopped-noise and blocked-replan regressions passed\n";
}

struct TurnResult {
    double duration = 0.0;
    double slow_command_s = 0.0;
    double stationary_s = 0.0;
    bool finished = false;
    bool replan_requested = false;
};

TurnResult follow_captured_transition(const Trajectory& path, const TrackerGains& gains)
{
    TrajectoryTracker tracker(gains);
    tracker.set_trajectory(path);
    Vec2 position = path.front().p;
    double yaw = 2.751996734184181;
    double vx = -.5673749426755605, vy = .30758299067099315;
    double yaw_rate = .07261856861717221;
    const double dt = gains.dt;
    // The same lag model is used for both geometries. Seed21's initial braking
    // decayed over roughly .4-.5s; yaw uses the existing .15s test response.
    const double translation_response = 1.0 - std::exp(-dt / .45);
    const double yaw_response = 1.0 - std::exp(-dt / .15);
    TurnResult result;
    for (int tick = 0; tick < static_cast<int>(90.0 / dt); ++tick) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        const VelCmd command = tracker.update(position.x, position.y, yaw,
            c * vx + s * vy, -s * vx + c * vy, .10, yaw_rate);
        if (tick == 0 && path.size() == 110)
            require(!tracker.reorienting(),
                    "prediction must not jump beyond the short reference and trigger a false reversal");
        result.duration = tick * dt;
        if (command.needs_replan) {
            result.replan_requested = true;
            return result;
        }
        if (command.at_goal && std::hypot(vx, vy) <= .05) {
            result.finished = true;
            return result;
        }
        if (std::hypot(command.v_fwd, command.v_lat) < .05) result.slow_command_s += dt;
        if (std::hypot(vx, vy) < .05) result.stationary_s += dt;
        vx += translation_response * (c * command.v_fwd - s * command.v_lat - vx);
        vy += translation_response * (s * command.v_fwd + c * command.v_lat - vy);
        yaw_rate += yaw_response * (command.yaw_rate - yaw_rate);
        position.x += vx * dt; position.y += vy * dt;
        yaw = std::atan2(std::sin(yaw + yaw_rate * dt), std::cos(yaw + yaw_rate * dt));
    }
    return result;
}

void near_reversal_transition(ExplorationNode& node)
{
    // Explanatory reconstruction of seed21 adoption 11 from turn_geometry_audit.json.
    // The fitted old tangent reproduces all 110 published smoothed points.
    const Vec2 start{2.6954501308234753, 3.125888715831432};
    const Vec2 goal{6.25, -.75};
    const double old_heading = 149.86828135849757 * M_PI / 180.0;
    const Path2 raw{start, goal};
    const double maximum_angle = node.turn_blend_max_angle_rad_;
    reset(node, start);
    node.yaw_ = 2.751996734184181;
    node.explore_has_committed_ = true;
    node.explore_raw_ = {{start.x - std::cos(old_heading), start.y - std::sin(old_heading)}, start};
    require(path_clear(start, raw, {}, node.ggcfg_) && node.path_inside_safe_field(raw),
            "seed21 raw fixture must be safe before transition replacement");
    // Preserve the captured old algorithm as an explicit historical fixture;
    // the current transition must no longer reconstruct that unsafe hook.
    const Vec2 control1{start.x + .8 * std::cos(old_heading), start.y + .8 * std::sin(old_heading)};
    const double chord = std::hypot(goal.x - start.x, goal.y - start.y);
    const Vec2 control2{goal.x - .8 * (goal.x - start.x) / chord,
                        goal.y - .8 * (goal.y - start.y) / chord};
    Path2 old_blend{start};
    for (int sample = 1; sample <= 12; ++sample) {
        const double t = static_cast<double>(sample) / 12, u = 1 - t;
        old_blend.push_back({u * u * u * start.x + 3 * u * u * t * control1.x +
            3 * u * t * t * control2.x + t * t * t * goal.x,
            u * u * u * start.y + 3 * u * u * t * control1.y +
            3 * u * t * t * control2.y + t * t * t * goal.y});
    }
    const Trajectory old_path = smooth_catmull_rom(old_blend, node.arc_ds_);
    require(old_blend.size() == 13 && old_path.size() == 110, "old seed21 hook reconstruction changed");
    require(std::hypot(old_path[1].p.x - 2.6478114366321184,
                       old_path[1].p.y - 3.1410200863503817) < 1e-9,
            "seed21 hook must match the captured first corner");
    const double dx = goal.x - start.x, dy = goal.y - start.y;
    const double length = std::hypot(dx, dy);
    double backward = 0.0;
    for (const auto& point : old_blend)
        backward = std::max(backward, -((point.x - start.x) * dx + (point.y - start.y) * dy) / length);
    // Twelve control samples capture 7.22cm; the continuous cubic reaches
    // 7.60cm between them. Compare the same sampling used by this fixture.
    require(std::abs(backward - .07221553668007051) < 1e-8,
            "old sampled transition must reproduce its backward hook");

    node.turn_blend_max_angle_rad_ = maximum_angle;
    const Path2 replacement = node.blend_explore_transition(raw, {});
    require(replacement.size() == 2 && replacement.front().x == start.x && replacement.back().x == goal.x,
            "near reversal must retain the validated original path");
    const Trajectory new_path = smooth_catmull_rom(replacement, node.arc_ds_);
    for (const auto& point : new_path) {
        require(((point.p.x - start.x) * dx + (point.p.y - start.y) * dy) >= -1e-9,
                "replacement must not have a backward hook");
        require(std::abs(point.kappa) < 1e-8, "two-point replacement should not have a curvature spike");
    }
    const TurnResult old_result = follow_captured_transition(old_path, node.gains_);
    const TurnResult new_result = follow_captured_transition(new_path, node.gains_);
    std::cout << "seed21 matched transition: old time=" << old_result.duration
              << "s slow_command=" << old_result.slow_command_s << "s stationary=" << old_result.stationary_s
              << "s; new time=" << new_result.duration << "s slow_command=" << new_result.slow_command_s
              << "s stationary=" << new_result.stationary_s << "s\n";
    require(new_result.finished, "near-reversal replacement must finish");
    require(!new_result.replan_requested, "valid continuous replacement must not miss a corner");
    require(old_result.finished || old_result.replan_requested,
            "historical hook must either finish or stop for a checked replan, never circle indefinitely");

    for (double degrees : {45.0, 90.0}) {
        reset(node, {2, 0});
        node.explore_has_committed_ = true;
        node.explore_raw_ = {{1, 0}, {2, 0}};
        const double angle = degrees * M_PI / 180.0;
        const Path2 corner{{2, 0}, {4, 0}, {4 + 2 * std::cos(angle), 2 * std::sin(angle)}};
        node.turn_blend_max_angle_rad_ = maximum_angle;
        const Path2 after = node.blend_explore_transition(corner, {});
        require(after.size() >= corner.size(),
            "ordinary 45/90 degree transitions must retain their continuous route");
        for (size_t i = 0; i < after.size(); ++i)
            require(after[i].y >= -1e-12,
                    "ordinary corner must not begin with an opposite-direction detour");
        require(path_clear(corner.front(), after, {}, node.ggcfg_) && node.path_inside_safe_field(after),
                "ordinary transition must remain safe");
    }
    const Path2 corner{{2, 0}, {4, 0}, {4, 2}};
    const Obstacles obstacle{{3.4, .6, 0.0}};
    require(path_clear(corner.front(), corner, obstacle, node.ggcfg_), "obstacle fixture must leave raw path safe");
    const Path2 safe_route = node.blend_explore_transition(corner, obstacle);
    require(path_clear(corner.front(), safe_route, obstacle, node.ggcfg_) &&
            node.path_inside_safe_field(safe_route),
            "obstacle clearance must retain a validated route before execution");
    reset(node, {2, 0});
    node.explore_has_committed_ = true;
    node.explore_raw_ = {{3, 0}, {2, 0}};
    require(node.blend_explore_transition({{2, 0}, {4, 0}}, {}).size() == 2,
            "stale opposite raw tangent must not override the current measured heading");
    reset(node, {.605, 0});
    node.yaw_ = M_PI;
    node.explore_has_committed_ = true;
    node.explore_raw_ = {{1.605, 0}, {.605, 0}};
    const Path2 wall{{.605, 0}, {1, 0}, {1, 2}};
    require(node.blend_explore_transition(wall, {}).size() == wall.size(),
            "wall-side reversal must retain the validated original path");
    node.explore_has_committed_ = false;
}

void continuous_route_and_missed_corner(ExplorationNode& node)
{
    reset(node, {2, 0});
    GlobalResult wall_turn;
    wall_turn.ok = true;
    wall_turn.path = {{2, 0}, {6.8, 0}, {6.8, 3}};
    Path2 unsafe_curve;
    for (const auto& p : smooth_catmull_rom(wall_turn.path, node.arc_ds_)) unsafe_curve.push_back(p.p);
    require(!node.path_inside_safe_field(unsafe_curve), "fixture must have an unsafe smoothed curve");
    const auto fallback_before = node.smooth_fallbacks_;
    require(node.adopt_explore_path(wall_turn, wall_turn.path.back()), "safe raw wall turn must be adoptable");
    Path2 executed;
    for (const auto& p : node.traj_) executed.push_back(p.p);
    require(node.smooth_fallbacks_ == fallback_before + 1 && node.path_inside_safe_field(executed) &&
            path_clear(executed.front(), executed, {}, node.ggcfg_),
            "rejected smooth curve must not be executed again as its own fallback");

    reset(node, {2, 0});
    GlobalResult normal;
    normal.ok = true;
    normal.path = {{2, 0}, {4, 0}, {4, 2}};
    require(node.adopt_explore_path(normal, normal.path.back()), "ordinary bend must be adoptable");
    require(node.traj_.size() > 20, "ordinary bend needs continuous curve samples");
    require(node.tracker_->next_corner_distance() < 0,
            "ordinary continuous bend must not create intermediate stop vertices");

    reset(node, {2, 2});
    const auto departure = plan_global_path({2, 2}, {5.5, -2.5}, {}, node.ggcfg_, 0.0);
    require(departure.ok && node.adopt_explore_path(departure, departure.path.back()),
            "departure curve must be constructible from actual current heading");
    require(node.traj_.size() > 2 && node.traj_[1].p.x > 2.0 &&
            std::abs(node.traj_.front().theta) < 0.10,
            "executed start curve must begin along measured heading rather than the raw goal bearing");
    require(node.tracker_->next_corner_distance() < 0,
            "heading connection must not inject a small stop vertex");

    reset(node, {2, 0});
    GlobalResult small_bend;
    small_bend.ok = true;
    small_bend.path = {{2, 0}, {2.04, .002}, {4, .06}};
    require(node.adopt_explore_path(small_bend, small_bend.path.back()),
            "centimetre-spaced guides with a small turn must remain smoothable");
    require(node.tracker_->next_corner_distance() < 0,
            "a small guide kink must not become a mandatory stop vertex");
    const auto small_command = node.tracker_->update(2.20, .006, 0, .4, 0, .10, 0);
    require(!small_command.needs_replan && !node.tracker_->reorienting() && small_command.v_fwd > 0,
            "passing a small guide kink on a continuous curve must keep progressing");

    const Path2 tiny{{2, 0}, {2.04, 0}, {4, .9}};
    const auto tiny_trajectory = node.make_polyline_trajectory(tiny);
    TrajectoryTracker tracker(node.gains_);
    tracker.set_trajectory(tiny_trajectory);
    auto command = tracker.update(2, 0, 0, .8, 0, .10, 0);
    require(!tracker.reorienting() && std::abs(command.yaw_rate) < 1e-9,
            "short forward vertex must not become a rearward carrot through prediction");
    command = tracker.update(2.25, 0, 0, .4, 0, .10, 0);
    require(command.v_fwd == 0 && command.yaw_rate == 0 && !command.needs_replan,
            "after overshooting the vertex, brake before planning or yawing back");
    command = tracker.update(2.25, 0, 0, 0, 0, .10, 0);
    require(command.needs_replan && command.v_fwd == 0 && command.yaw_rate == 0,
            "stopped aircraft must request a fresh checked path instead of chasing the passed vertex");
    auto route = plan_global_path({2.25, 0}, tiny.back(), {}, node.ggcfg_);
    require(route.ok && path_clear({2.25, 0}, route.path, {}, node.ggcfg_),
            "missed-vertex recovery needs a safe route from the actual stopped pose");
    tracker.set_trajectory(smooth_catmull_rom(route.path, node.arc_ds_));
    command = tracker.update(2.25, 0, 0, 0, 0, .10, 0);
    require(!command.needs_replan && !tracker.reorienting() && command.v_fwd > 0,
            "checked missed-vertex recovery should resume forward motion");
    std::cout << "continuous bend, safe fallback and missed-corner regressions passed\n";
}

void settled_turn_search_handoff(ExplorationNode& node)
{
    for (const auto& angles : std::vector<std::pair<double, double>>{
             {0, M_PI / 2}, {179 * M_PI / 180, -179 * M_PI / 180}, {0, M_PI}}) {
        reset(node, {3, 0});
        node.yaw_ = angles.first;
        const Vec2 target{3 + 1.5 * std::cos(angles.second), 1.5 * std::sin(angles.second)};
        const auto probe = plan_global_path({3, 0}, target, {}, node.ggcfg_);
        require(probe.ok, "turn-search fixture must have a safe unconstrained path");
        node.enter_turn_for_solution({3, 0}, target, probe, 0);
        double rate = 0, drift = .35, rotation = 0;
        bool handed_off = false;
        for (int i = 0; i < 1200; ++i) {
            node.v_fwd_est_ = drift;
            node.v_lat_est_ = 0;
            node.yaw_rate_est_ = rate;
            geometry_msgs::msg::TwistStamped command;
            const bool turning = node.step_turn_for_solution({}, command);
            if (!turning) {
                require(drift <= node.gains_.align_stop_speed &&
                        std::abs(rate) <= node.gains_.align_stop_yaw_rate,
                        "turn-search must not hand off a path while translation or yaw is still fast");
                handed_off = true;
                break;
            }
            require(command.twist.linear.x == 0 && command.twist.linear.y == 0,
                    "turn-search must brake translation");
            if (drift > node.gains_.align_stop_speed)
                require(std::abs(command.twist.angular.z) < 1e-9,
                        "turn-search must brake before rotating");
            const double dt = node.gains_.dt;
            drift *= std::exp(-dt / .35);
            rate += (1 - std::exp(-dt / .15)) * (command.twist.angular.z - rate);
            rotation += rate * dt;
            node.yaw_ = std::atan2(std::sin(node.yaw_ + rate * dt), std::cos(node.yaw_ + rate * dt));
        }
        const double requested = std::abs(std::atan2(std::sin(angles.second - angles.first),
                                                    std::cos(angles.second - angles.first)));
        require(handed_off && std::abs(rotation) <= requested + .20,
                "feedback turn-search must finish without spinning a complete extra revolution");
    }
    std::cout << "settled turn-search handoff and angle-wrap regressions passed\n";
}

void observation_heading_and_failed_candidate_preserve_the_active_route(ExplorationNode& node)
{
    reset(node, {2, 0});
    node.turning_for_solution_ = false;
    auto original_grid = std::move(node.grid_);
    node.grid_ = std::make_unique<GridMap>(original_grid->config());
    const auto original_config = node.ggcfg_;
    const auto original_frontier = node.fcfg_;
    FrontierSelection observation;
    observation.valid = true;
    observation.point = {4, 0};
    observation.look_at = {4, 1};
    GlobalResult route;
    route.ok = true;
    route.path = {{2, 0}, {4, 0}};
    require(node.adopt_explore_path(route, observation.point, &observation),
            "normal observation route fixture was rejected");
    auto snapshot = node.grid_->snapshot();
    std::fill(snapshot.big.begin(), snapshot.big.end(), 1);
    snapshot.big[static_cast<size_t>(10) * snapshot.bny + 10] = 0;
    snapshot.small.clear(); snapshot.snx = 0;
    require(visible_unknown_count(snapshot, {4, 0}, node.observation_target_.view_heading,
                                  {}, node.fcfg_) == 1 &&
            visible_unknown_count(snapshot, {4, 0}, M_PI / 2, {}, node.fcfg_) == 0,
            "normal arrival FOV became stale only because look_at used a different direction");

    // Scan the look-at sector completely, leaving real unfinished cells in
    // the arrival sector. Exercise the actual stale-target decision in node.
    node.grid_->mark_scan(4, 0, M_PI / 2);
    const auto observed = node.grid_->snapshot();
    require(visible_unknown_count(observed, {4, 0}, M_PI / 2, {}, node.fcfg_) == 0 &&
            visible_unknown_count(observed, {4, 0}, 0, {}, node.fcfg_) > 0,
            "real-scan stale-target fixture did not separate the two FOVs");
    const double previous_period = node.candidate_period_;
    const bool previous_time_valid = node.candidate_time_valid_;
    const auto previous_candidate_time = node.last_candidate_time_;
    node.candidate_period_ = 1000;
    node.candidate_time_valid_ = true;
    node.last_candidate_time_ = node.now();
    const auto previous_skips = node.candidate_skips_;
    node.replan_locked({2, 0});
    require(node.candidate_skips_ == previous_skips + 1 && node.active_target_gain_ > 0,
            "a safe normal route was prematurely replaced after only its look-at sector was scanned");
    node.candidate_period_ = previous_period;
    node.candidate_time_valid_ = previous_time_valid;
    node.last_candidate_time_ = previous_candidate_time;

    observation.requires_turn = true;
    require(node.adopt_explore_path(route, observation.point, &observation) &&
            std::abs(node.observation_target_.view_heading - M_PI / 2) < 1e-9,
            "explicit cleanup observation lost its requested final turn");

    // All first grid steps miss this narrow cone, so a speculative candidate
    // search fails. The already validated route still contains forward gain.
    node.grid_ = std::make_unique<GridMap>(original_grid->config());
    observation.requires_turn = false;
    observation.point = {6.5, 3.5}; observation.look_at = {7, 4};
    route.path = {{2, 0}, observation.point};
    require(node.adopt_explore_path(route, observation.point, &observation),
            "productive active-route fixture was rejected");
    node.yaw_ = M_PI / 8;
    node.ggcfg_.head_cone_half = .001;
    node.ggcfg_.head_cone_radius = 100;
    node.cur_band_ = 1;
    node.unreachable_.clear();
    node.plan_pending_ = true;
    const auto before_adoptions = node.adoptions_;
    const auto before_kept = node.kept_after_candidate_failure_;
    node.replan_locked({2, 0});
    require(node.kept_after_candidate_failure_ == before_kept + 1 &&
            node.adoptions_ == before_adoptions && node.tracker_->has_trajectory() &&
            node.explore_has_committed_ && !node.explore_failed_ && !node.turning_for_solution_,
            "a failed replacement discarded the safe productive route and started a needless turn");

    // A real obstruction must still cancel the old route immediately.
    auto original_obstacles = std::move(node.obs_map_);
    node.obs_map_ = std::make_unique<ObstacleMap>(node.ocfg_);
    for (int i = 0; i < 3; ++i) node.obs_map_->integrate({{2, 0}}, 0, 0, 0, true);
    node.plan_pending_ = true;
    node.replan_locked({2, 0});
    require(!node.tracker_->has_trajectory() &&
            node.kept_after_candidate_failure_ == before_kept + 1,
            "active-route preference overrode a newly observed physical obstruction");
    node.obs_map_ = std::move(original_obstacles);
    node.grid_ = std::move(original_grid);
    node.ggcfg_ = original_config; node.fcfg_ = original_frontier;
    node.explore_failed_ = node.explore_has_committed_ = node.have_observation_target_ = false;
    node.turning_for_solution_ = false; node.unreachable_.clear();
    std::cout << "observation heading and safe-route commitment regressions passed\n";
}

void reverse_handoff_brakes_on_the_existing_endpoint(ExplorationNode& node)
{
    reset(node, {2, 0});
    node.explore_has_committed_ = false;
    node.tracker_->set_trajectory({});
    FrontierSelection observation;
    observation.valid = true; observation.point = {2.4, 0}; observation.look_at = {4, 0};
    GlobalResult incoming; incoming.ok = true; incoming.path = {{2, 0}, {2.4, 0}};
    require(node.adopt_explore_path(incoming, observation.point, &observation),
            "incoming endpoint fixture was rejected");
    node.v_fwd_est_ = .6;
    GlobalResult reversal; reversal.ok = true; reversal.path = {{2, 0}, {1, 1}};
    const auto before = node.adoptions_;
    require(!node.adopt_explore_path(reversal, {1, 1}, &observation) &&
            node.explore_handoff_deferred_ && node.adoptions_ == before &&
            node.traj_.back().p.x == 2.4 && node.tracker_->has_trajectory(),
            "cruise-speed endpoint handoff adopted a reverse reference before braking");
    node.px_ = 2.35; node.v_fwd_est_ = .2;
    VelCmd command; command.at_goal = true;
    node.finish_observation(command, {});
    require(node.have_observation_target_ && node.tracker_->has_trajectory() &&
            command.v_fwd == 0 && command.yaw_rate == 0,
            "arrival released the old endpoint while the aircraft was still sliding");
    node.px_ = 2.55; node.v_fwd_est_ = .1;
    command = {}; command.v_fwd = .3; command.yaw_rate = .2;
    node.finish_observation(command, {});
    require(node.observation_arrived_ && node.have_observation_target_ &&
            command.v_fwd == 0 && command.yaw_rate == 0,
            "sliding outside the endpoint tolerance reactivated a turn back toward the old point");
    node.v_fwd_est_ = 0;
    node.finish_observation(command, {});
    require(!node.observation_arrived_ && !node.have_observation_target_ && node.plan_pending_,
            "latched observation arrival did not release after stopping");

    reset(node, {2, 0}); node.v_fwd_est_ = .04;
    require(node.adopt_explore_path(reversal, {1, 1}, &observation) &&
            !node.explore_handoff_deferred_, "settled reverse handoff deadlocked");
    node.explore_has_committed_ = false; node.tracker_->set_trajectory({});
    node.v_fwd_est_ = 0;
    require(node.adopt_explore_path(incoming, observation.point, &observation),
            "flowing handoff fixture was rejected");
    node.v_fwd_est_ = .6;
    GlobalResult bend; bend.ok = true; bend.path = {{2, 0}, {3, 0}, {3, 2}};
    require(node.adopt_explore_path(bend, {3, 2}, &observation) &&
            !node.explore_handoff_deferred_, "a continuous ordinary bend was forced to stop");
    node.explore_has_committed_ = node.have_observation_target_ = false;
    node.tracker_->set_trajectory({}); node.traj_.clear();
    std::cout << "reverse handoff braking and continuous forward handoff regressions passed\n";
}

void current_pose_validation_and_new_reference_state(ExplorationNode& node)
{
    TrajectoryTracker tracker(node.gains_);
    const auto route = smooth_catmull_rom({{2, 0}, {5, 0}}, .05);
    const Obstacles passed{{1.22, 0, .2}};
    tracker.set_trajectory(route);
    tracker.update(2, 0, 0, .4, 0, .1, 0);
    const Vec2 current{2.03, 0};
    const auto remaining = tracker.remaining_path(current.x, current.y);
    require(path_clear(current, remaining, passed, node.ggcfg_),
            "one-tick stale progress created a backward connection into a passed obstacle margin");
    require(!path_clear(current, remaining, {{3, 0, .2}}, node.ggcfg_),
            "fresh validation projection skipped a real obstacle ahead");
    require(tracker.progress_distance() == 0,
            "read-only route validation unexpectedly advanced the controller state");
    tracker.update(current.x, current.y, 0, .4, 0, .1, 0);
    require(tracker.progress_distance() > 0 && !tracker.reorienting(),
            "the actual controller did not follow the same forward projection");

    tracker.set_trajectory(node.make_polyline_trajectory({{2.01, 0}, {2.01, 2}}));
    tracker.update(2.01, 0, M_PI / 2, .1, 0, .1, 0);
    const Vec2 displaced{2, .03};
    const auto outward = tracker.remaining_path(displaced.x, displaced.y);
    require(path_clear(displaced, outward, {{2.90, -.10, .21}}, node.ggcfg_),
            "perpendicular reacquisition invalidated a clear forward escape from the extra margin");
    require(!path_clear(displaced, outward, {{2, 1, .1}}, node.ggcfg_),
            "forward reacquisition ignored an obstacle on its actual connection");
    tracker.set_trajectory(node.make_polyline_trajectory({{2, 0}, {2, .04}, {4, .04}}));
    const auto corner_path = tracker.remaining_path(2.02, .05);
    require(corner_path.size() >= 3 && std::hypot(corner_path[1].x - 2, corner_path[1].y - .04) < 1e-9,
            "forward validation skipped a mandatory unpassed corner");

    tracker.set_trajectory(smooth_catmull_rom({{2, 0}, {1, 0}}, .05));
    tracker.update(2, 0, 0, .2, 0, .1, 0);
    require(tracker.reorienting(), "reverse-reference fixture never entered alignment");
    tracker.set_trajectory(route);
    const auto command = tracker.update(2, 0, 0, .2, 0, .1, 0);
    require(!tracker.reorienting() && command.v_fwd > 0,
            "an aligned new reference inherited the previous route's braking/turn state");
    std::cout << "current-pose validation and route-state reset regressions passed\n";
}

void small_guides_are_removed_only_with_clear_shortcuts(ExplorationNode& node)
{
    reset(node, {2, 0});
    node.explore_has_committed_ = false;
    node.tracker_->set_trajectory({});
    const Path2 tiny{{2, 0}, {2.04, 0}, {4, .9}};
    require(node.clean_explore_guides(tiny, {}).size() == 2,
            "a redundant four-centimetre guide was kept as a new corner");
    GlobalResult route; route.ok = true; route.path = tiny;
    require(node.adopt_explore_path(route, tiny.back()), "cleaned small-guide route was rejected");
    auto command = node.tracker_->update(2.15, .01, 0, .4, 0, .1, 0);
    require(!command.needs_replan && !node.tracker_->reorienting() && command.v_fwd > 0 &&
            node.tracker_->next_corner_distance() < 0,
            "a small safe guide still caused a stop or a turn back");
    const Path2 detour{{2, 0}, {2.05, .02}, {4, .02}};
    const Obstacles obstacle{{2.05, -.79, .2}};
    require(path_clear(detour.front(), detour, obstacle, node.ggcfg_) &&
            !path_clear(detour.front(), {detour.front(), detour.back()}, obstacle, node.ggcfg_),
            "short-corner obstacle fixture did not require its intermediate guide");
    require(node.clean_explore_guides(detour, obstacle).size() == detour.size(),
            "short-guide cleanup cut through a real obstacle margin");
    std::cout << "small-guide cleanup and obstacle retention regressions passed\n";
}

void turn_search_uses_the_stopped_pose_and_bounded_retry(ExplorationNode& node)
{
    reset(node, {3, 0});
    node.tracker_->set_trajectory({});
    node.explore_has_committed_ = false;
    node.cur_band_ = 1;
    const Vec2 target{3, 1};
    auto probe = plan_global_path({3, 0}, target, {}, node.ggcfg_);
    node.enter_turn_for_solution({3, 0}, target, probe, 1);
    node.v_fwd_est_ = .3;
    geometry_msgs::msg::TwistStamped command;
    node.step_turn_for_solution({}, command);
    require(!node.turn_heading_valid_ && command.twist.angular.z == 0,
            "turn search committed its heading while still coasting");
    node.px_ = 4; node.v_fwd_est_ = 0;
    node.step_turn_for_solution({}, command);
    require(node.turn_heading_valid_ && std::abs(node.turn_heading_ - 3 * M_PI / 4) < 1e-6,
            "turn search kept its pre-braking bearing after the actual position changed");

    node.enter_turn_for_solution({4, 0}, target, probe, 1);
    node.v_fwd_est_ = .3;
    node.turn_start_time_ = node.now() - rclcpp::Duration::from_seconds(node.turn_solve_timeout_ + 1);
    node.step_turn_for_solution({}, command);
    require(!node.turning_for_solution_ && !node.plan_pending_ && node.cur_band_ == 1,
            "a turn timeout skipped a whole band or requested another immediate planning burst");
    const auto checks = node.replan_checks_;
    node.replan_locked({4, 0});
    require(node.replan_checks_ == checks,
            "a failed turn retried before a normal planning period elapsed");
    node.unreachable_.clear(); node.turning_for_solution_ = false;
    std::cout << "stopped-pose turn search and bounded retry regressions passed\n";
}

void retain_turn_probe_and_reserve_path_clearance(ExplorationNode& node)
{
    reset(node, {3, 0}); node.tracker_->set_trajectory({}); node.explore_has_committed_ = false;
    GlobalResult probe; probe.ok = true; probe.path = {{3, 0}, {4, 0}, {5, 1}};
    node.enter_turn_for_solution({3, 0}, {5, 1}, probe, 1);
    node.turn_heading_valid_ = true; node.turn_heading_ = 0; node.turn_probe_path_ = probe.path;
    node.turn_settled_s_ = node.gains_.align_settle_s;
    node.turn_research_tick_ = node.turn_solve_research_every_;
    const auto searches = node.astar_searches_;
    geometry_msgs::msg::TwistStamped command;
    require(!node.step_turn_for_solution({}, command) && !node.turning_for_solution_ &&
            node.astar_searches_ == searches && node.tracker_->has_trajectory(),
            "settled turn replaced a still-safe probe with another competing search result");

    const auto original = node.ggcfg_;
    const Vec2 start{2, 0}, goal{6, 0};
    const Obstacles obstacle{{4, 0, .2}};
    const auto spacious = node.search_explore_global(start, goal, obstacle);
    require(spacious.ok && score_path(start, spacious.path, obstacle, node.ggcfg_).min_clear >=
        node.ggcfg_.robot_radius + node.ggcfg_.inflate + node.explore_plan_reserve_ - 1e-8,
        "new exploration guides did not reserve space for small obstacle-boundary updates");
    node.ggcfg_.min_y = -1.4; node.ggcfg_.max_y = 1.4;
    const Obstacles narrow{{4, -.83, .2}, {4, .83, .2}};
    const auto fallback = node.search_explore_global(start, goal, narrow);
    require(fallback.ok && path_clear(start, fallback.path, narrow, node.ggcfg_) &&
            score_path(start, fallback.path, narrow, node.ggcfg_).min_clear <
                node.ggcfg_.robot_radius + node.ggcfg_.inflate + node.explore_plan_reserve_,
            "optional planning reserve closed a passage that meets the original safety margin");
    node.ggcfg_ = original;
    node.explore_has_committed_ = false; node.tracker_->set_trajectory({}); node.traj_.clear();
    std::cout << "stable turn probe and optional planning clearance regressions passed\n";
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::InitOptions options;
    options.set_domain_id(203);
    rclcpp::init(argc, argv, options);
    int result = 0;
    try {
        ExplorationNode node;
        closed_loop(node, "straight required connector", {6, 4.25}, {7, 4.25}, {});
        closed_loop(node, "required obstacle detour", {2, 0}, {6, 0}, {{4, 0, .2}});
        seed26_motion_regression(node);
        near_reversal_transition(node);
        continuous_route_and_missed_corner(node);
        settled_turn_search_handoff(node);
        observation_heading_and_failed_candidate_preserve_the_active_route(node);
        reverse_handoff_brakes_on_the_existing_endpoint(node);
        current_pose_validation_and_new_reference_state(node);
        small_guides_are_removed_only_with_clear_shortcuts(node);
        turn_search_uses_the_stopped_pose_and_bounded_retry(node);
        retain_turn_probe_and_reserve_path_clearance(node);

        reset(node, {6, 4.25});
        node.pd_to_point_avoid({7, 4.25}, {});
        const auto searches = node.global_path_searches_;
        node.px_ = 6.95; node.py_ = 4.29;
        for (int i = 0; i < 8; ++i) node.pd_to_point_avoid({7, 4.25}, {});
        require(node.global_path_searches_ == searches, "off-reference terminal pose caused spurious replan");
        require(node.required_motion_clear({7.04, 4.29}, {}), "valid terminal stopping capsule rejected");
        require(!node.required_motion_clear({7.2, 4.29}, {}), "overshoot beyond capsule accepted");
        require(!node.required_motion_clear({6.96, 4.4}, {}), "lateral capsule violation accepted");
        node.px_ = .58; node.py_ = 0;
        require(node.required_motion_clear({.59, 0}, {}), "short monotonic field entry deadlocked");
        require(!node.required_motion_clear({.57, 0}, {}), "field margin retreat accepted");

        reset(node, {2, 0});
        node.pd_to_point_avoid({6, 0}, {{4, 0, .2}});
        node.v_fwd_est_ = 2;
        const Vec2 stopped = node.pd_to_point_avoid({6, 0}, {{4, 0, .2}});
        require(node.global_braking_blocked_ && node.measured_projection_blocked_ && stopped.x == 0 && stopped.y == 0,
                "unsafe measured stopping projection did not brake");
        node.v_fwd_est_ = 0;
        const Vec2 resumed = node.pd_to_point_avoid({6, 0}, {{4, 0, .2}});
        require(resumed.x <= node.global_gains_.max_accel * node.global_gains_.dt + 1e-9,
                "safety cap did not synchronize forward acceleration state");

        reset(node, {2, 0});
        const Vec2 no_route = node.pd_to_point_avoid({6, 0}, {{2, 0, .2}});
        require(node.global_failed_ && !node.global_goal_blocked_ && !node.retreating_ &&
                no_route.x == 0 && no_route.y == 0 && node.global_traj_.empty(),
                "invalid required route used unsafe fallback or retreat");

        reset(node, {7, 4.25});
        const Vec2 occupied = node.pd_to_point_avoid({7, 4.25}, {{6.990, 4.097, .2}});
        require(node.global_goal_blocked_ && !node.global_at_goal_ && occupied.x == 0 && occupied.y == 0,
                "occupied seed22 required goal was accepted");
        node.obs_map_ = std::make_unique<ObstacleMap>(node.ocfg_);
        for (int i = 0; i < 3; ++i) node.obs_map_->integrate({{7, 4.25}}, 7, 4.25, 0, true);
        node.has_pose_ = node.has_goal_ = true;
        node.poi_mode_ = ExplorationNode::PoiMode::GOTO_POI;
        node.poi_target_ = {7, 4.25};
        node.on_timer();
        require(node.poi_mode_ == ExplorationNode::PoiMode::GOTO_POI && !node.finished_,
                "occupied POI entered arrived state");
        node.poi_mode_ = ExplorationNode::PoiMode::EXPLORE;
        node.homing_ = true; node.goal_ = {7, 4.25};
        node.on_timer();
        require(!node.finished_ && !node.corridor_->active() && !node.retreating_,
                "occupied home goal completed, entered corridor or retreated");
        std::cout << "required node: closed-loop and safety regressions passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
