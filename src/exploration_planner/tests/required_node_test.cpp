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
#include <filesystem>
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
#include "exploration_planner/vision_shm.hpp"
#include "exploration_planner/vision_candidates.hpp"
#include "exploration_planner/vision_target_queue.hpp"
#include "exploration_planner/vision_near_confirmation.hpp"
#include "exploration_planner/cloud_xyz.hpp"
#include "exploration_planner/velocity_slew.hpp"
#include "vision_shm_test_writer.hpp"
#include "exploration_planner/drop_approach.hpp"

// Dependencies are already included; this access override affects only the node.
#define private public
#define main exploration_original_main
#include "../src/exploration_planner_node.cpp"
#undef main
#undef private

geometry_msgs::msg::TwistStamped last_published_command;
size_t published_commands = 0;
acfly_interfaces::msg::ExplorationCommand last_control;
size_t published_controls = 0;
std_msgs::msg::Header last_completed_goal;
size_t published_completions = 0;

// Inspect the final command after all node-level limiters; keep publications
// inside this test process instead of sending any flight topic over DDS.
extern "C" rcl_ret_t __wrap_rcl_publish(const rcl_publisher_t* publisher,
                                       const void* message, rmw_publisher_allocation_t*)
{
    if (std::string(rcl_publisher_get_topic_name(publisher)) == "/exploration/cmd_vel") {
        last_published_command = *static_cast<const geometry_msgs::msg::TwistStamped*>(message);
        ++published_commands;
    }
    if (std::string(rcl_publisher_get_topic_name(publisher)) == "/exploration/finished_goal") {
        last_completed_goal = *static_cast<const std_msgs::msg::Header*>(message);
        ++published_completions;
    }
    if (std::string(rcl_publisher_get_topic_name(publisher)) == "/exploration/control") {
        last_control = *static_cast<const acfly_interfaces::msg::ExplorationCommand*>(message);
        ++published_controls;
    }
    return RCL_RET_OK;
}

namespace {

void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}

void reset(ExplorationNode& node, Vec2 start)
{
    node.pose_freshness_ = SensorFreshness{};
    node.pose_freshness_.observe(0, node.steady_seconds());
    node.px_ = start.x; node.py_ = start.y; node.pz_ = .8; node.yaw_ = 0;
    node.v_fwd_est_ = node.v_lat_est_ = node.yaw_rate_est_ = 0;
    node.global_tracker_->set_trajectory({});
    node.global_has_ = node.global_failed_ = node.global_goal_blocked_ = false;
    node.global_at_goal_ = node.global_braking_blocked_ = false;
    node.required_goal_holding_ = false;
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
        require(required_path_clear({node.px_, node.py_}, node.global_raw_, goal, obstacles, node.required_config()),
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
    // Preserve the captured forward-only pursuit chord to exercise the original
    // margin/replan defect. Drift compensation can turn that chord outward and
    // legitimately avoid the block; check that enabled output separately below.
    auto compensated_tracker = std::move(node.global_tracker_);
    auto captured_gains = node.global_gains_;
    captured_gains.compensate_lateral_drift = false;
    node.global_tracker_ = std::make_unique<TrajectoryTracker>(captured_gains);
    seed26_reference(node);
    node.required_motion_inflate_ = planning_margin;
    const Vec2 old_command = node.pd_to_point_avoid(goal, obstacles);
    // 196b3cd 起，被投影拦截的命令不再整机清零，而是二分缩到刚好通过同一投影校验的比例
    // (避免满速/零速反复)。测试意图不变：不安全的 pursuit chord 不得原样放行——
    // 要么拦截为零，要么输出的是经 required_velocity_clear 校验的缩放命令。
    {
        const double c = std::cos(node.yaw_), s = std::sin(node.yaw_);
        const Vec2 world{c * old_command.x - s * old_command.y, s * old_command.x + c * old_command.y};
        const bool blocked_zero = old_command.x == 0 && old_command.y == 0 && node.command_projection_blocked_;
        const bool scaled_safe = std::hypot(old_command.x, old_command.y) > 0 &&
            std::hypot(old_command.x, old_command.y) < node.v_goal_max_ * .5 &&
            node.required_velocity_clear(world, obstacles);
        require(blocked_zero || scaled_safe,
                "seed26 reproduction must block or safely scale the inward pursuit chord with the planning margin");
    }
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
    node.global_tracker_ = std::move(compensated_tracker);
    seed26_reference(node);
    node.required_motion_inflate_ = planning_margin;
    const Vec2 compensated = node.pd_to_point_avoid(goal, obstacles);
    const double c = std::cos(node.yaw_), s = std::sin(node.yaw_);
    require(node.required_velocity_clear({c * compensated.x - s * compensated.y,
                                         s * compensated.x + c * compensated.y}, obstacles),
            "compensated seed26 command must obey the same planning margin");
    node.required_motion_inflate_ = motion_margin;
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
    require(node.adopt_explore_path(wall_turn, wall_turn.path.back()), "safe raw wall turn must be adoptable");
    Path2 executed;
    for (const auto& p : node.traj_) executed.push_back(p.p);
    require(node.path_inside_safe_field(executed) &&
            path_clear(executed.front(), executed, {}, node.ggcfg_),
            "final execution must remain inside the field when an interpolating spline would overshoot");

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
    const double direct_heading = std::atan2(-4.5, 3.5);
    require(node.traj_.size() > 2 && node.traj_[1].p.x > 2.0 &&
            std::all_of(node.traj_.begin(), node.traj_.end(), [&](const TrajPoint& p) {
                return std::abs(std::remainder(p.theta - direct_heading, 2*M_PI)) < 1e-6;
            }), "clear departure must preserve its straight route rather than bend to match current yaw");
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
    require(command.v_fwd == 0 && command.yaw_rate == 0 && command.needs_replan,
            "a missed vertex must request checked replanning even while drifting");
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
                require(std::abs(rate) <= node.gains_.align_stop_yaw_rate,
                        "turn-search must settle yaw before path handoff");
                handed_off = true;
                break;
            }
            require(std::hypot(command.twist.linear.x, command.twist.linear.y) <= .20 + 1e-9,
                    "turn-search XY correction exceeds hold speed");
            if (i == 0 && std::abs(angles.second - angles.first) > .1)
                require(std::abs(command.twist.angular.z) > 0,
                        "turn-search paused yaw on residual horizontal speed");
            const double dt = node.gains_.dt;
            drift = .10 + (drift - .10) * std::exp(-dt / .35);
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
    require(!node.have_observation_target_ && node.plan_pending_ &&
            command.v_fwd < 0 && command.yaw_rate == 0,
            "arrival must release the endpoint with active braking despite horizontal drift");

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
    require(node.turn_heading_valid_ && command.twist.angular.z > 0 && command.twist.linear.x < 0,
            "turn search did not start yaw and XY correction together");
    node.px_ = 4; node.v_fwd_est_ = 0;
    node.step_turn_for_solution({}, command);
    require(node.turn_heading_valid_ && std::abs(node.turn_heading_ - M_PI / 2) < 1e-6 &&
            node.turn_hold_.anchor().x == 3 && command.twist.linear.x < 0,
            "drift changed the latched turn heading/anchor or lost XY correction");

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

void exploration_switch_regression()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    node.exploration_enabled_ = false;
    auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
    goal->header.frame_id = "camera_init";
    goal->point.x = 3.0;
    node.on_goal(goal);
    node.on_timer();
    require(node.global_path_searches_ == 0 && !node.finished_,
            "skip exploration moved or finished before odometry was available");

    reset(node, {2, 0});
    node.has_pose_ = true;
    const auto exploration_plans = node.replan_checks_;
    node.on_poi(goal);
    node.on_timer();
    require(node.homing_ && !node.plan_pending_ && !node.global_failed_ &&
            node.global_has_ && !node.global_traj_.empty() &&
            std::hypot(node.global_traj_.back().p.x - 3.0, node.global_traj_.back().p.y) < 1e-9,
            "disabled exploration did not produce an exact path directly to the mission endpoint");
    require(node.replan_checks_ == exploration_plans && node.grid_->coverage_ratio() == 0 &&
            node.poi_queue_.empty() && !node.vision_collection_active_,
            "direct endpoint mode collected coverage or allowed an exploration detour");
    require(!node.finished_ && !node.corridor_->active(),
            "direct endpoint mode skipped arrival and entered the corridor early");

    node.corridor_enabled_ = true;
    require(node.corridor_->configure({3, 1}, {3, -2}), "configure corridor fixture");
    node.px_ = 3.0;
    for (int i = 0; i < 20 && !node.corridor_->active(); ++i) node.on_timer();
    require(node.corridor_->active() && !node.finished_,
            "direct endpoint arrival failed to hand control to the corridor");

    // A new normal mission restores exploration, without inheriting direct mode.
    node.exploration_enabled_ = true;
    reset(node, {2, 0});
    node.on_goal(goal);
    require(!node.homing_ && node.plan_pending_ && !node.corridor_->active(),
            "enabled exploration retained the previous direct endpoint mission");
    node.on_timer();
    require(node.replan_checks_ > exploration_plans && !node.homing_ &&
            node.tracker_->has_trajectory() && node.grid_->coverage_ratio() > 0,
            "enabled exploration did not resume coverage planning");

    // The switch must not weaken boundary safety or inherit an active POI.
    node.exploration_enabled_ = false;
    node.poi_mode_ = ExplorationNode::PoiMode::WAIT_RELEASE;
    goal->point.x = node.ggcfg_.max_x + .02;
    node.on_goal(goal);
    node.on_timer();
    require(node.homing_ && node.global_failed_ && node.global_goal_blocked_ &&
            node.poi_mode_ == ExplorationNode::PoiMode::EXPLORE &&
            !node.corridor_->active() && !node.finished_,
            "skip exploration bypassed endpoint safety or retained an old POI hold");
    std::cout << "exploration switch: direct endpoint, normal exploration, corridor handoff and safety passed\n";
}

void boundary_home_endpoint_regression()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    node.ggcfg_.min_x = 0; node.ggcfg_.max_x = 1.57;
    node.ggcfg_.min_y = -5.35; node.ggcfg_.max_y = 1.15;
    node.home_goal_wall_margin_ = 0;
    node.homing_ = true;
    closed_loop(node, "boundary home goal from takeoff origin", {0, 0}, {1.55, .60}, {});
    node.ggcfg_.min_y = -.35;
    const auto searches = node.global_path_searches_;
    closed_loop(node, "small field home goal from takeoff origin", {0, 0}, {1.55, .60}, {});
    require(node.global_path_searches_ - searches == 1,
            "small field entry repeatedly replanned and reset forward acceleration");
    require(node.px_ <= 1.57 && node.px_ >= 0 && node.py_ <= 1.15,
            "boundary goal tracking left the configured field");
    node.px_ = 1.50; node.py_ = .60;
    require(node.required_motion_clear({1.55, .60}, {}),
            "terminal motion still enforced body-radius field inset");
    require(!node.required_motion_clear({1.58, .60}, {}),
            "terminal stopping projection escaped the field");
    require(!node.required_motion_clear({1.55, .60}, {{1.55, .60, .1}}),
            "terminal motion waived physical obstacle clearance");

    reset(node, {.75, 0});
    node.homing_ = false;
    node.pd_to_point_avoid({1.55, .60}, {});
    require(node.global_goal_blocked_ && node.global_failed_,
            "visual/POI target inherited the home endpoint exception");
    closed_loop(node, "required visual approach from boundary", {0, 0}, {.75, .4}, {});
    reset(node, {.05, .01});
    require(node.required_motion_clear({.08, .03}, {}),
            "inward required-point motion was blocked before reaching body clearance");
    require(!node.required_motion_clear({.04, .03}, {}),
            "boundary takeoff recovery allowed outward motion");
    require(!node.required_motion_clear({.08, .03}, {{.08, .03, .1}}),
            "boundary takeoff recovery waived a physical obstacle");

    // Exercise the actual visual-target flag: it bypasses the command guard,
    // but used to trip the measured-speed guard above goal_stop_v_ at the edge.
    node.ggcfg_.min_y = -5.35;
    const Vec2 visual_goal{.9, 0};
    node.vision_queue_->enqueue({VisionCandidate{1, visual_goal, 10, true, 1, 1}}, {0, 0});
    require(node.vision_queue_->activate_next(1), "activate nearby front visual target");
    closed_loop(node, "active front visual target from boundary", {0, 0}, visual_goal, {});
    reset(node, {.05, 0});
    node.v_fwd_est_ = .08;  // Above the stationary threshold, still inside body inset.
    node.pd_to_point_avoid(visual_goal, {});
    require(!node.global_failed_ && !node.measured_projection_blocked_ && !node.global_braking_blocked_,
            "visual target acceleration was repeatedly clamped by boundary body clearance");
    std::cout << "boundary home endpoint closed-loop and target isolation passed\n";
}

void small_field_stability_regression()
{
    for (double response_time : {.15, .40, .60}) {
        ExplorationNode node;
        node.vision_reader_.reset();
        node.ggcfg_.min_x = 0; node.ggcfg_.max_x = 1.57;
        node.ggcfg_.min_y = -.35; node.ggcfg_.max_y = 1.15;
        node.home_goal_wall_margin_ = 0;
        node.homing_ = true;
        reset(node, {0, 0});
        double x = 0, y = 0, yaw = 0, vx = 0, vy = 0, rate = 0;
        double previous_command = 0, max_drop = 0, max_error = 0;
        int sample = 0, stops = 0;
        bool reached = false;
        const Vec2 goal{1.55, .60};
        constexpr double dt = .02;
        const double response = 1.0 - std::exp(-dt / response_time);
        for (int i = 0; i < 750; ++i) {
            // 20 Hz noisy pose samples through the production velocity
            // estimator, with 50 Hz control and inertial world-frame motion.
            if (i == 0 || i * 2 / 5 != (i - 1) * 2 / 5) {
                ++sample;
                auto odom = std::make_shared<nav_msgs::msg::Odometry>();
                odom->header.stamp = rclcpp::Time(1000000000LL + i * 20000000LL);
                odom->pose.pose.position.x = x + .001 * std::sin(1.41 * sample);
                odom->pose.pose.position.y = y + .001 * std::sin(1.73 * sample);
                odom->pose.pose.position.z = .8;
                const double observed_yaw = yaw + .003 * std::sin(1.13 * sample);
                odom->pose.pose.orientation.z = std::sin(observed_yaw * .5);
                odom->pose.pose.orientation.w = std::cos(observed_yaw * .5);
                node.on_odom(odom);
            }
            const Vec2 cmd = node.pd_to_point_avoid(goal, {});
            require(!node.global_failed_, "small-field stability lost the required route");
            // Exclude launch noise and the intentional final arrival stop.
            if (i * dt > 1.25 && std::hypot(x - goal.x, y - goal.y) > .20) {
                max_drop = std::max(max_drop, previous_command - cmd.x);
                if (previous_command > .02 && cmd.x < 1e-8) ++stops;
            }
            previous_command = cmd.x;
            const double c = std::cos(yaw), s = std::sin(yaw);
            vx += response * (c * cmd.x - s * cmd.y - vx);
            vy += response * (s * cmd.x + c * cmd.y - vy);
            rate += response * (node.last_yaw_rate_ - rate);
            x += dt * vx; y += dt * vy; yaw += dt * rate;
            max_error = std::max(max_error, std::abs(goal.y * x - goal.x * y) /
                std::hypot(goal.x, goal.y));
            require(x >= 0 && x <= 1.57 && y >= -.35 && y <= 1.15,
                    "small-field inertial tracking escaped the field");
            if (node.global_at_goal_ && std::hypot(vx, vy) < node.goal_stop_v_) {
                reached = true;
                break;
            }
        }
        std::cout << "small-field stability lag=" << response_time
                  << " searches=" << node.global_path_searches_ << " stops=" << stops
                  << " max_drop=" << max_drop << " max_error=" << max_error << '\n';
        require(reached && node.global_path_searches_ == 1 && stops == 0 &&
                max_drop < .10 && max_error < .10,
                "small-field home tracking replanned, pulsed its speed or drifted off line");

        node.global_connector_start_ = {.72, .28};
        node.px_ = .50; node.py_ = .15;
        require(node.required_motion_clear({1.02, .36}, {}),
                "safe stopping segment across inset/terminal junction was rejected");
        require(!node.required_motion_clear({1.02, .10}, {}),
                "junction permission escaped through an unrelated field edge");
        require(!node.required_motion_clear({1.60, .60}, {}),
                "junction permission escaped the absolute field boundary");
        require(!node.required_motion_clear({1.02, .36}, {{.8, .28, .01}}),
                "junction permission bypassed an obstacle on the stopping segment");
    }
}

void visual_straight_approach_regression()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    node.homing_ = false;
    node.vision_straight_approach_m_ = 1.0;
    VisionVisitTarget target;
    target.track_id = 42;
    target.position = {3, 0};
    reset(node, {1.99, 0});
    node.yaw_ = 1.2;
    node.visual_target_velocity(target, {});
    require(node.vision_straight_track_ == 0 && std::abs(node.last_yaw_rate_) > 1e-6,
            "far visual approach stopped using the original yaw controller");
    node.px_ = 2;
    const Vec2 lateral = node.visual_target_velocity(target, {});
    require(node.vision_straight_track_ == 42 && node.last_yaw_rate_ == 0 && lateral.y < 0,
            "one-metre approach failed to replace yaw steering with lateral translation");
    node.px_ = 1.98;
    node.visual_target_velocity(target, {});
    require(node.vision_straight_track_ == 42 && node.last_yaw_rate_ == 0,
            "threshold noise released the latched straight approach");
    node.vision_straight_approach_m_ = 0;
    node.visual_target_velocity(target, {});
    require(node.vision_straight_track_ == 0, "zero radius did not disable straight approach");
    node.vision_straight_approach_m_ = 1;

    reset(node, {2.02, 0});
    const Obstacles blocked_chord{{2.51, .4, .01}};
    node.visual_target_velocity(target, blocked_chord);
    require(node.vision_straight_track_ == 0 && !node.global_failed_ &&
            required_path_clear({node.px_, node.py_}, node.global_raw_, target.position,
                                blocked_chord, node.required_config()),
            "near visual approach cut through a blocked straight chord");
    node.visual_target_velocity(target, {});
    require(node.vision_straight_track_ == 42, "clear chord did not enable straight approach");
    const Vec2 blocked = node.visual_target_velocity(target, {{3, 0, .1}});
    require(node.vision_straight_track_ == 0 && node.global_failed_ &&
            blocked.x == 0 && blocked.y == 0,
            "newly occupied target did not stop a latched straight approach");

    // Use the real timer branch for arrival, the full hover interval and FIFO
    // handoff. Flight dynamics stay offline; no executor or aircraft is run.
    node.reset_visual_approach();
    reset(node, {2, 0});
    node.yaw_ = 1.2;
    node.has_pose_ = node.has_goal_ = true;
    node.goal_ = {7, 4.25};
    const Vec2 goal{2.8, .4};
    node.vision_queue_->enqueue({VisionCandidate{1, goal, 10, true, 1, 1},
        VisionCandidate{2, {4.5, 2}, 10, true, 1, 1}}, {2, 0});
    node.activate_visual_target(node.steady_seconds());
    const auto uid = node.vision_queue_->active()->track_id;
    require(node.vision_queue_->refine_active({uid, goal, 3, 10, 2}), "refine test visual target");
    double vx = 0, vy = 0;
    bool reached = false;
    for (int tick = 0; tick < 1000; ++tick) {
        const double c = std::cos(node.yaw_), s = std::sin(node.yaw_);
        node.v_fwd_est_ = c * vx + s * vy;
        node.v_lat_est_ = -s * vx + c * vy;
        const Vec2 cmd = node.visual_target_velocity(*node.vision_queue_->active(), {});
        require(node.last_yaw_rate_ == 0 && !node.global_failed_ &&
                std::hypot(cmd.x, cmd.y) <= node.vision_near_->speed_cap(node.gains_.v_max) + 1e-9,
                "straight visual approach rotated, lost its route or exceeded its speed cap");
        vx += (1 - std::exp(-.02 / .4)) * (c * cmd.x - s * cmd.y - vx);
        vy += (1 - std::exp(-.02 / .4)) * (s * cmd.x + c * cmd.y - vy);
        node.px_ += .02 * vx; node.py_ += .02 * vy;
        if (node.global_at_goal_ && std::hypot(vx, vy) < node.vision_target_stop_speed_) {
            reached = true;
            break;
        }
    }
    require(reached, "fixed-yaw XY approach did not settle at its target");
    node.px_ = goal.x + node.vision_target_tol_ + .01; node.py_ = goal.y;
    node.v_fwd_est_ = .20; node.v_lat_est_ = .10;
    node.vision_straight_velocity_ = {};
    node.on_timer();
    require(node.vision_hover_track_ == 0 && node.vision_hover_since_ < 0,
            "hover began outside the target position tolerance");
    node.px_ = goal.x + node.vision_target_tol_ - .01;
    node.on_timer();
    require(node.vision_hover_track_ == uid && node.vision_hover_since_ >= 0 &&
            node.last_yaw_rate_ == 0, "moving entry into the target radius failed to begin hover");
    node.vision_hover_since_ = node.steady_seconds() - 2.9;
    node.on_timer();
    require(node.vision_queue_->visited().empty() && node.vision_straight_track_ == uid &&
            node.last_yaw_rate_ == 0, "visual task completed before three seconds");
    node.px_ += .15;
    node.vision_hover_since_ = node.steady_seconds() - 1.5;
    const double hover_started = node.vision_hover_since_;
    node.v_fwd_est_ = node.vision_target_stop_speed_ + .05;
    node.on_timer();
    require(node.vision_queue_->visited().empty() && node.vision_hover_track_ == uid &&
            node.vision_hover_since_ == hover_started && node.last_yaw_rate_ == 0,
            "position/speed drift reset the dwell timer, completed early or resumed yaw steering");
    require(node.vision_straight_velocity_.x < 0,
            "hover drift did not command correction toward the target");
    node.px_ = goal.x;
    node.v_fwd_est_ = 0;
    node.on_timer();
    require(node.vision_hover_since_ == hover_started && node.vision_queue_->visited().empty(),
            "returning to the target restarted the dwell timer");
    node.px_ += .15;
    node.v_fwd_est_ = node.vision_target_stop_speed_ + .05;
    node.vision_hover_since_ = node.steady_seconds() - 3.1;
    node.on_timer();
    require(node.vision_queue_->visited().size() == 1 && node.vision_straight_track_ == 0 &&
            node.vision_queue_->active() && node.vision_queue_->active()->track_id != uid,
            "elapsed dwell while drifting failed to complete or hand off to the next target");
    require(node.vision_hover_track_ == 0 && node.vision_hover_since_ < 0,
            "next target inherited the previous dwell timer");
    node.on_timer();
    require(node.vision_straight_track_ == 0 && std::abs(node.last_yaw_rate_) > 1e-6,
            "next distant target failed to restore original yaw steering");
    std::cout << "visual one-metre XY approach, obstacle fallback, three-second hover and yaw restore passed\n";
}
void visual_hover_restores_exploration_display()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.homing_ = false;
    node.goal_ = {7, 4.25};
    FrontierSelection observation;
    observation.valid = true;
    observation.point = {6.5, 3.5};
    observation.look_at = {7, 4};
    GlobalResult route;
    route.ok = true;
    route.path = {{2, 0}, observation.point};
    require(node.adopt_explore_path(route, observation.point, &observation),
            "display regression needs a productive committed exploration route");
    const Trajectory executed = node.traj_;
    const auto adoptions = node.adoptions_;
    const Vec2 visual_goal{2.8, .4};
    node.vision_queue_->enqueue({VisionCandidate{1, visual_goal, 10, true, 1, 1}}, {2, 0});
    node.activate_visual_target(node.steady_seconds());
    const auto uid = node.vision_queue_->active()->track_id;
    require(node.vision_queue_->refine_active({uid, visual_goal, 3, 10, 2}),
            "refine display regression target");
    node.px_ = visual_goal.x; node.py_ = visual_goal.y;
    node.on_timer();
    require(node.vision_hover_track_ == uid && node.traj_.size() <= 2 &&
            node.tracker_->has_trajectory(),
            "hover must replace the display while retaining the exploration reference");
    node.vision_hover_since_ = node.steady_seconds() - 3.1;
    node.on_timer();
    require(!node.vision_queue_->active() && node.vision_queue_->visited().size() == 1,
            "visual hover did not finish before display restoration");

    // A speculative replacement fails, so the first resumed route is the
    // already-valid reference. No adoption will rewrite the display cache.
    node.yaw_ = M_PI / 8;
    node.ggcfg_.head_cone_half = .001;
    node.ggcfg_.head_cone_radius = 100;
    node.cur_band_ = 1;
    const auto kept = node.kept_after_candidate_failure_;
    node.on_timer();
    require(node.kept_after_candidate_failure_ == kept + 1 && node.adoptions_ == adoptions &&
            node.tracker_->has_trajectory(), "resumption did not retain its existing flight route");
    require(node.traj_.size() == executed.size(),
            "first exploration route after visual hover is flown but absent from the display");
    for (size_t i = 0; i < executed.size(); ++i)
        require(std::hypot(node.traj_[i].p.x - executed[i].p.x, node.traj_[i].p.y - executed[i].p.y) < 1e-9,
                "resumed display geometry differs from the route being flown");
    std::cout << "first exploration route after visual hover restored without adopting a second route\n";
}
void recorded_xy_handoffs()
{
    struct Sample { Vec2 position, target, velocity, previous; double yaw; };
    const Sample samples[]{
        {{2.582, .809}, {2.544786, .436249}, {.321, -.275}, {.027, -.096}, -74.58 * M_PI / 180},
        {{1.742, -1.889}, {1.652182, -1.514532}, {-.328, .167}, {-.082, .113}, 124.71 * M_PI / 180}};
    for (const auto& sample : samples) {
        for (double response : {.4, .9}) {
            ExplorationNode node;
            node.vision_reader_.reset();
            reset(node, sample.position);
            node.vision_straight_approach_m_ = .4;
            node.homing_ = false;
            node.yaw_ = sample.yaw;
            node.last_command_world_ = sample.previous;
            VisionVisitTarget target;
            target.track_id = 91;
            target.position = sample.target;
            const double c = std::cos(sample.yaw), s = std::sin(sample.yaw);
            Vec2 velocity = sample.velocity;
            bool reached = false;
            double furthest = 0;
            for (int tick = 0; tick < 600; ++tick) {
                node.v_fwd_est_ = c * velocity.x + s * velocity.y;
                node.v_lat_est_ = -s * velocity.x + c * velocity.y;
                const Vec2 body = node.visual_target_velocity(target, {});
                const Vec2 world{c * body.x - s * body.y, s * body.x + c * body.y};
                if (tick == 0) {
                    require(std::hypot(world.x - sample.previous.x, world.y - sample.previous.y) <=
                            node.xy_brake_accel_ * node.gains_.dt + 1e-9,
                            "recorded XY handoff jumped from the previous output to measured inertia");
                    require((world.x - sample.previous.x) * velocity.x < 0,
                            "recorded sideways inertia was reinforced at XY handoff");
                }
                require(node.last_yaw_rate_ == 0 && !node.global_failed_,
                        "fixed-yaw approach lost its valid route");
                const double alpha = 1 - std::exp(-.02 / response);
                velocity.x += alpha * (world.x - velocity.x);
                velocity.y += alpha * (world.y - velocity.y);
                node.px_ += velocity.x * .02; node.py_ += velocity.y * .02;
                const double distance = std::hypot(node.px_ - sample.target.x, node.py_ - sample.target.y);
                furthest = std::max(furthest, distance);
                if (distance < .10 && std::hypot(velocity.x, velocity.y) < .05) { reached = true; break; }
            }
            std::cout << "recorded handoff lag=" << response << " maximum radius=" << furthest << '\n';
            require(reached && furthest < .46, "recorded near-goal state still made a large orbit or failed to settle");
        }
    }
}

void cruise_drift_compensation_is_checked()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2.3, .08});
    const auto route = node.make_polyline_trajectory({{2, 0}, {5, 0}});
    node.tracker_->set_trajectory(route);
    node.v_fwd_est_ = .2; node.v_lat_est_ = .12;
    VelCmd cmd;
    for (int tick = 0; tick < 25; ++tick)
        cmd = node.tracker_->update(node.px_, node.py_, 0, .2, .12, .1, 0);
    require(cmd.v_fwd > 0 && cmd.v_lat < -.05 && cmd.lateral_correction,
            "ordinary forward cruise ignored real sideways drift");
    const auto free = node.guard_explore_velocity(cmd, {});
    require(free.v_lat < 0 && free.v_fwd > 0, "clear drift correction was suppressed");
    // A second obstacle below the aircraft must still veto the correction.
    const Obstacles below{{node.px_, node.py_ - .73, .1}};
    const auto constrained = node.guard_explore_velocity(cmd, below);
    require(node.turn_correction_clear({constrained.v_fwd, constrained.v_lat}, below),
            "lateral drift correction moved toward a second obstacle without validation");
    const auto overlap = node.guard_explore_velocity(cmd, {{node.px_, node.py_, .1}});
    require(overlap.v_fwd == 0 && overlap.v_lat == 0 && overlap.yaw_rate == 0,
            "cruise recovery bypassed physical overlap");
}

void cruise_with_continuous_drift()
{
    // A bounded constant lateral disturbance plus actuator lag exercises the
    // combined yaw/XY controller and obstacle guard, not only correction signs.
    for (bool required : {false, true}) {
        for (double lag : {.4, .9}) {
            for (double wind : {-.10, .10}) {
                ExplorationNode node;
                node.vision_reader_.reset();
                reset(node, {2, 0});
                const Vec2 goal{5, 0};
                const Obstacles trees{{3.5, .9, .10}, {3.5, -.9, .10}};
                node.tracker_->set_trajectory(node.make_polyline_trajectory({{2, 0}, goal}));
                Vec2 velocity{0, wind};
                double yaw_rate = 0, max_error = 0, min_clearance = 100;
                bool reached = false;
                for (int tick = 0; tick < 2500; ++tick) {
                    const double c = std::cos(node.yaw_), s = std::sin(node.yaw_);
                    node.v_fwd_est_ = c * velocity.x + s * velocity.y;
                    node.v_lat_est_ = -s * velocity.x + c * velocity.y;
                    node.yaw_rate_est_ = yaw_rate;
                    VelCmd cmd;
                    if (required) {
                        const Vec2 body = node.pd_to_point_avoid(goal, trees);
                        require(!node.global_failed_, "drifting required flight lost its clear path");
                        cmd.v_fwd = body.x; cmd.v_lat = body.y;
                        cmd.yaw_rate = node.last_yaw_rate_;
                    } else {
                        cmd = node.tracker_->update(node.px_, node.py_, node.yaw_,
                            node.v_fwd_est_, node.v_lat_est_, .1, yaw_rate);
                        require(!cmd.needs_replan, "straight cruise with drift unexpectedly needs replan");
                        cmd = node.guard_explore_velocity(cmd, trees);
                    }
                    const Vec2 world{c * cmd.v_fwd - s * cmd.v_lat, s * cmd.v_fwd + c * cmd.v_lat};
                    node.last_command_world_ = world;
                    const double response = 1 - std::exp(-.02 / lag);
                    velocity.x += response * (world.x - velocity.x);
                    velocity.y += response * (world.y + wind - velocity.y);
                    yaw_rate += (1 - std::exp(-.02 / .15)) * (cmd.yaw_rate - yaw_rate);
                    node.px_ += velocity.x * .02; node.py_ += velocity.y * .02;
                    node.yaw_ = std::atan2(std::sin(node.yaw_ + yaw_rate * .02),
                                           std::cos(node.yaw_ + yaw_rate * .02));
                    max_error = std::max(max_error, std::abs(node.py_));
                    for (const auto& tree : trees)
                        min_clearance = std::min(min_clearance,
                            std::hypot(node.px_ - tree.cx, node.py_ - tree.cy) - tree.r);
                    if (std::hypot(node.px_ - goal.x, node.py_ - goal.y) < .10) {
                        reached = true; break;
                    }
                }
                std::cout << "continuous drift required=" << required << " lag=" << lag
                          << " wind=" << wind << " max_error=" << max_error
                          << " minimum tree clearance=" << min_clearance
                          << " reached=" << reached << " end=" << node.px_ << ',' << node.py_
                          << " yaw=" << node.yaw_ << " remaining=" << node.global_tracker_->remaining_distance()
                          << " holding=" << node.required_goal_holding_ << " blocked=" << node.global_block_reason_ << '\n';
                require(reached && max_error < .20 && min_clearance >= node.ggcfg_.robot_radius + .15,
                        "continuous drift caused a stall, excessive deviation or tree approach");
            }
        }
    }
}

void path_failure_keeps_safe_braking()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.homing_ = true;
    node.goal_ = {6, 0};
    node.v_fwd_est_ = .12; node.v_lat_est_ = .10;
    node.last_command_world_ = {-.07, -.06};
    // Occupied destination forces the actual on_timer failure branch while
    // preserving a clear braking area around the drifting aircraft.
    for (int i = 0; i < 3; ++i) node.obs_map_->integrate({{6, 0}}, 2, 0, 0, true);
    node.on_timer();
    const auto first = last_published_command.twist;
    require(node.global_failed_ && first.linear.x < -.05 && first.linear.y < -.04 &&
            node.recovery_holding_, "path failure discarded the existing safe counter-velocity");
    const Vec2 anchor = node.recovery_anchor_;
    node.px_ += .08; node.py_ += .05;
    node.on_timer();
    require(node.recovery_anchor_.x == anchor.x && node.recovery_anchor_.y == anchor.y &&
            last_published_command.twist.linear.x < 0 && last_published_command.twist.linear.y < 0,
            "failed-path hold followed the drifting aircraft instead of preserving its anchor");

    // Full retreat is blocked by a different obstacle, but a small checked
    // counter-velocity is feasible. The fallback must not require 0.5 m of room.
    reset(node, {2, 0});
    node.recovery_holding_ = false;
    node.last_command_world_ = {-.04, 0};
    node.v_fwd_est_ = .1;
    const Obstacles narrow{{2.81, 0, .1}, {1.17, 0, .1}};
    require(node.do_retreat(narrow).x == 0, "fixture needs a rejected full retreat");
    const Vec2 short_brake = node.recovery_velocity(narrow, false);
    require(short_brake.x < 0 && node.turn_correction_clear(short_brake, narrow),
            "rejected retreat also discarded a feasible short braking correction");
    const Vec2 stopped = node.recovery_velocity({{2, 0, .1}}, false);
    require(stopped.x == 0 && stopped.y == 0 && node.recovery_status_ == "no_safe_recovery_velocity",
            "unavailable recovery must diagnose and reject physical overlap");
}

void malformed_cloud_does_not_terminate_node()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.corridor_enabled_ = true;
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = "camera_init";
    cloud->width = cloud->height = 1;
    cloud->point_step = cloud->row_step = 12;
    cloud->data.resize(12);  // A malformed packet has no x/y/z field definitions.
    node.on_cloud(cloud);
    node.on_corridor_cloud(cloud);
    require(node.obs_map_->snapshot().empty(), "malformed cloud changed the obstacle map");
    require(node.corridor_cloud_filter_reason_ == "invalid_xyz_layout", "missing malformed-cloud reason");

    // Two organized rows with unaligned XYZ fields and padding that is not a point.
    sensor_msgs::msg::PointCloud2 padded;
    padded.header.frame_id = "camera_init";
    padded.header.stamp.sec = 1;
    padded.width = 1; padded.height = 2;
    padded.point_step = 13; padded.row_step = 28;
    padded.data.resize(56, 0xff);
    const std::array<const char*, 3> names{"x", "y", "z"};
    for (size_t axis = 0; axis < 3; ++axis) {
        sensor_msgs::msg::PointField field;
        field.name = names[axis]; field.offset = 1 + 4 * axis;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32; field.count = 1;
        padded.fields.push_back(field);
    }
    const float samples[2][3] = {{1.25f, -.5f, .8f}, {2.75f, -.5f, .8f}};
    for (bool big_endian : {false, true}) {
        padded.is_bigendian = big_endian;
        for (size_t row = 0; row < 2; ++row) {
            for (size_t axis = 0; axis < 3; ++axis) {
                uint32_t bits;
                std::memcpy(&bits, &samples[row][axis], 4);
                for (size_t byte = 0; byte < 4; ++byte)
                    padded.data[row * 28 + 1 + axis * 4 + byte] =
                        static_cast<uint8_t>(bits >> (8 * (big_endian ? 3 - byte : byte)));
            }
        }
        CloudXYZ xyz(padded);
        require(xyz.valid(), "valid padded XYZ layout rejected");
        size_t count = 0;
        xyz.for_each([&](float x, float y, float z) {
            require(count < 2 && x == samples[count][0] && y == samples[count][1] && z == samples[count][2],
                    "row padding or byte order corrupted coordinates");
            ++count;
        });
        require(count == 2, "organized cloud lost points");
        *cloud = padded;
        node.on_corridor_cloud(cloud);
        require(node.corridor_cloud_kept_points_ == 2, "valid cloud failed to recover after bad packet");
        ++padded.header.stamp.sec;
    }
    for (int defect = 0; defect < 5; ++defect) {
        *cloud = padded;
        if (defect == 0) cloud->data.pop_back();
        if (defect == 1) cloud->fields[2].offset = cloud->point_step - 1;
        if (defect == 2) cloud->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
        if (defect == 3) cloud->point_step = 0;
        if (defect == 4) cloud->row_step = 12;
        require(!CloudXYZ(*cloud).valid(), "invalid XYZ payload accepted");
        node.on_cloud(cloud);
        node.on_corridor_cloud(cloud);
        require(node.corridor_cloud_filter_reason_ == "invalid_xyz_layout", "invalid packet accepted by callback");
    }
    *cloud = padded;
    cloud->header.stamp.sec = -1;
    node.on_corridor_cloud(cloud);
    require(node.corridor_cloud_filter_reason_ == "invalid_stamp", "invalid cloud timestamp accepted");
    *cloud = padded;
    cloud->header.frame_id = "body";
    node.on_cloud(cloud);
    require(node.obs_map_->snapshot().empty(), "wrong-frame cloud entered exploration map");
    *cloud = sensor_msgs::msg::PointCloud2();
    cloud->header.stamp.sec = padded.header.stamp.sec;
    node.on_cloud(cloud);
    require(node.corridor_cloud_filter_reason_ == "empty_input", "empty input was not diagnosed");
    require(node.corridor_->points().size() == 2, "empty packet erased real wall evidence");
    std::cout << "malformed, empty, padded and endian XYZ cloud regressions passed\n";
}

void new_goal_resets_execution()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.exploration_enabled_ = true;
    GlobalResult route;
    route.ok = true;
    route.path = {{2, 0}, {3, 0}};
    require(node.adopt_explore_path(route, {3, 0}), "seed old exploration route");
    node.poi_queue_.push_back({2, 0});
    node.poi_mode_ = ExplorationNode::PoiMode::WAIT_RELEASE;
    auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
    goal->header.frame_id = "camera_init";
    goal->header.stamp.sec = 99;
    goal->point.x = 6;
    node.on_goal(goal);
    require(node.poi_mode_ == ExplorationNode::PoiMode::EXPLORE && node.poi_queue_.empty() &&
            !node.tracker_->has_trajectory() && !node.explore_has_committed_ && node.traj_.empty(),
            "new exploration mission inherited old POI wait or old route");
}

void invalid_odom_stamp_preserves_pose()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->pose.pose.orientation.w = 1;
    odom->pose.pose.position.x = 3;
    odom->header.stamp.sec = -1;
    node.on_odom(odom);
    require(node.px_ == 2, "negative odometry stamp changed the current pose");
    odom->header.stamp.sec = 1;
    odom->header.stamp.nanosec = 1000000000u;
    node.on_odom(odom);
    require(node.px_ == 2, "noncanonical odometry stamp changed the current pose");
    odom->header.stamp.nanosec = 0;
    node.on_odom(odom);
    require(node.px_ == 3, "valid odometry failed to recover after invalid timestamp");
}

void duplicate_goal_preserves_progress()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
    goal->header.frame_id = "camera_init";
    goal->header.stamp.sec = 99;
    goal->point.x = 6;
    node.on_goal(goal);
    require(node.corridor_->configure({6, 0}, {6, -3}), "configure repeated-goal fixture");
    node.corridor_->start({6, 0}, node.steady_seconds());
    node.on_goal(goal);
    require(node.corridor_->phase() == CorridorPhase::Rotate,
            "repeated delivery of the same goal restarted the mission during corridor execution");
    goal->point.x = 5;
    node.on_goal(goal);
    require(node.goal_.x == 6 && node.corridor_->phase() == CorridorPhase::Rotate,
            "conflicting coordinates reused the same completion identity");
    node.finished_ = true;
    goal->point.x = 6;
    node.on_goal(goal);
    require(node.finished_, "duplicate goal reopened an already completed mission");
}

void invalid_goal_preserves_current_mission()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
    goal->header.frame_id = "camera_init";
    goal->header.stamp.sec = 99;
    goal->point.x = 6;
    node.on_goal(goal);
    goal->header.stamp.sec = 100;
    goal->point.x = std::numeric_limits<double>::quiet_NaN();
    node.on_goal(goal);
    require(node.goal_.x == 6 && node.goal_stamp_.sec == 99,
            "invalid new goal destroyed the current valid mission");
    goal->point.x = 5;
    goal->header.frame_id = "untransformed_camera_frame";
    node.on_goal(goal);
    require(node.goal_.x == 6 && node.goal_stamp_.sec == 99, "wrong-frame goal replaced the current mission");
    node.on_poi(goal);
    goal->header.frame_id = "camera_init";
    goal->header.stamp.sec = -1;
    node.on_goal(goal);
    require(node.goal_stamp_.sec == 99, "negative-stamp goal replaced current mission");
    goal->header.stamp.sec = 100;
    goal->header.stamp.nanosec = 1000000000u;
    node.on_goal(goal);
    require(node.goal_stamp_.sec == 99, "malformed-stamp goal replaced current mission");
    goal->header.stamp.nanosec = 0;
    goal->point.x = std::numeric_limits<double>::quiet_NaN();
    node.on_poi(goal);
    goal->point.x = 1e100;
    node.on_poi(goal);
    require(node.poi_queue_.empty() && node.poi_seen_.empty(), "invalid POI entered the flight queue or integer key");
}

void missing_visual_frames_do_not_hold_forever()
{
    ExplorationNode node;
    reset(node, {2, 0});
    node.vision_reader_ = std::make_unique<VisionShmReader>("/dev/null", .5, .02);
    node.has_pose_ = node.has_goal_ = true;
    node.homing_ = false;
    node.goal_ = {7, 4.25};
    node.vision_queue_->enqueue({VisionCandidate{1, {3, 0}, 10, true, 1, 1}}, {2, 0});
    node.activate_visual_target(node.steady_seconds() - 30);
    require(node.vision_queue_->active() && node.vision_near_->near_entered(), "enter near visual confirmation");
    node.poll_vision_shm();
    require(node.vision_queue_->active() && node.vision_queue_->active()->track_id == 1 &&
            node.vision_queue_->visited().empty() &&
            node.vision_last_event_ == "near_timeout_no_refinement" &&
            node.vision_near_->track_id() == 1,
            "missing visual frames switched targets or falsely marked the active target completed");
}

void blocked_visual_target_keeps_fifo_order()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.goal_ = {4.5, 2};
    node.pose_freshness_.observe(1, node.steady_seconds());
    node.vision_queue_->enqueue({
        VisionCandidate{11, {4.0, 0.0}, 10, true, 1, 1},
        VisionCandidate{12, {4.5, 1.0}, 10, true, 1, 1}}, {2, 0});
    node.activate_visual_target(node.steady_seconds());
    require(node.vision_queue_->active() && node.vision_queue_->active()->track_id == 11 &&
            node.vision_queue_->pending().size() == 1, "blocked visual FIFO fixture was not initialized");
    // Make the active target itself occupied in the same obstacle model used by
    // the required planner. It must be retried, not replaced by track 12.
    for (int i = 0; i < 4; ++i)
        node.obs_map_->integrate({{4.0, 0.0}}, node.px_, node.py_, node.yaw_, true);
    node.on_timer();
    require(node.vision_queue_->active() && node.vision_queue_->active()->track_id == 11 &&
            node.vision_queue_->pending().size() == 1 && node.vision_queue_->visited().empty() &&
            node.vision_last_event_ == "visual_path_unavailable",
            "blocked visual target was replaced before completion");
}

void motion_pose_dropout_liveness()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.homing_ = true;
    node.goal_ = {4, 0};
    node.pose_freshness_.observe(1, node.steady_seconds() - node.ccfg_.pose_timeout - .1);
    const double coverage = node.grid_->coverage_ratio();
    node.on_timer();
    const auto& cmd = last_published_command.twist;
    require(cmd.linear.x == 0 && cmd.linear.y == 0 && cmd.angular.z == 0 &&
            node.grid_->coverage_ratio() == coverage,
            "stale ordinary-flight odometry still produced motion or new coverage");
    node.pose_freshness_.observe(2, node.steady_seconds());
    node.on_timer();
    require(std::hypot(cmd.linear.x, cmd.linear.y) > 0 && !node.finished_,
            "fresh odometry failed to resume the original mission after a dropout");
}

void completion_publish_liveness()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = true;
    node.exploration_enabled_ = false;
    node.corridor_enabled_ = false;
    auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
    goal->header.frame_id = "camera_init";
    goal->header.stamp.sec = 42;
    goal->header.stamp.nanosec = 123;
    goal->point.x = 2;
    const auto before = published_completions;
    node.on_goal(goal);
    node.on_timer();
    require(node.finished_ && published_completions == before + 1 && last_completed_goal == goal->header,
            "completion did not echo the exact original goal identity");
    ++goal->header.stamp.sec;
    node.on_goal(goal);
    node.on_timer();
    require(node.finished_ && published_completions == before + 2 && last_completed_goal == goal->header,
            "new mission at the same endpoint reused an old completion identity");
}

void retreat_timeout_liveness()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    const Obstacles obstacles{{2.4, 0, .1}};
    geometry_msgs::msg::TwistStamped cmd;
    require(node.try_retreat(obstacles, cmd) && node.retreating_, "start near-obstacle recovery");
    node.retreat_start_ = node.now() - rclcpp::Duration::from_seconds(node.retreat_timeout_ + .1);
    require(!node.try_retreat(obstacles, cmd), "recovery must expire when aircraft cannot move");
    for (int i = 0; i < 20; ++i)
        require(!node.try_retreat(obstacles, cmd), "expired recovery restarted at the same position on the next tick");
    node.py_ += .25;
    require(node.try_retreat(obstacles, cmd), "a materially changed position must allow a fresh recovery attempt");
}

void retreat_checks_the_other_obstacle()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    const double d = node.ggcfg_.robot_radius + node.ggcfg_.inflate + .11;
    const Obstacles obstacles{{2 + d, 0, .1}, {2 - d - .02, 0, .1}};
    require(obstacle_segment_clear({2, 0}, {2, 0}, obstacles, node.ggcfg_), "retreat fixture must start clear");
    const auto cmd = node.do_retreat(obstacles);
    // A full retreat toward the slightly farther obstacle is unsafe even
    // though it increases clearance from the nearest obstacle.
    require(std::hypot(cmd.x, cmd.y) < 1e-9,
            "retreat moved away from the nearest obstacle straight into the other obstacle");
}

void exhausted_exploration_route()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.homing_ = false;
    node.done_coverage_ = 1.1;
    node.goal_ = {7, 4.25};
    GlobalResult route;
    route.ok = true;
    route.path = {{2, 0}, {3, 0}};
    require(node.adopt_explore_path(route, {3, 0}), "adopt ordinary exploration route");
    node.px_ = 3;
    node.on_timer();
    require(node.plan_pending_ && !node.explore_has_committed_ && !node.tracker_->has_trajectory(),
            "ordinary exploration endpoint retained its exhausted route and zero command");
}

void visual_turn_final_output()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.homing_ = false;
    node.goal_ = {7, 4.25};
    node.yaw_ = M_PI;
    node.v_fwd_est_ = .15;
    node.vision_queue_->enqueue({VisionCandidate{1, {5, 0}, 10, true, 1, 1}}, {2, 0});
    node.activate_visual_target(node.steady_seconds());
    const auto before = published_commands;
    node.on_timer();
    const auto& cmd = last_published_command.twist;
    const double speed = std::hypot(cmd.linear.x, cmd.linear.y);
    require(published_commands == before + 1 && std::abs(cmd.angular.z) > 0 && speed > 0,
            "visual turn failed to publish simultaneous XY correction and yaw");
    require(speed <= node.global_gains_.max_accel * node.global_gains_.dt + 1e-9,
            "visual postprocessor boosted the validated small hold correction beyond its acceleration limit");

    reset(node, {2, 0});
    VelCmd correction;
    correction.holding_position = true;
    correction.v_fwd = .2;
    const Obstacles nearby{{2 + node.ggcfg_.robot_radius + node.required_motion_inflate_ + .12, 0, .1}};
    require(!node.required_velocity_clear({.2, 0}, nearby), "obstacle must block the proposed hold correction");
    const Vec2 safe = node.guard_required_command(correction, nearby, true);
    require(node.required_velocity_clear(safe, nearby) && safe.x < .2,
            "checked visual route incorrectly exempted an off-route hold correction from obstacle checks");
}

void observation_turn_with_drift()
{
    ExplorationNode node;
    reset(node, {2, 0});
    node.have_observation_target_ = true;
    node.observation_target_.view_heading = 0;
    node.explore_target_ = {2, 0};
    node.yaw_ = M_PI;
    node.v_fwd_est_ = .15;
    node.grid_->mark_scan(2, 0, M_PI, {});
    VelCmd command;
    command.at_goal = true;
    node.finish_observation(command, {});
    require(node.observation_turn_active_ && std::abs(command.yaw_rate) > 0 && command.v_fwd < 0,
            "observation yaw did not start with XY correction at nonzero speed");
    // wrap_pi chooses +pi for this half-turn, so advance in that direction.
    node.px_ += .15; node.py_ += .10; node.yaw_ = -M_PI / 2;
    node.grid_->mark_scan(node.px_, node.py_, 0, {});
    node.grid_->mark_scan(node.px_, node.py_, -M_PI / 2, {});
    node.finish_observation(command, {});
    require(node.observation_turn_active_ && command.yaw_rate != 0 &&
            node.observation_hold_.anchor().x == 2 && node.observation_hold_.anchor().y == 0,
            "new visibility or drift interrupted the committed observation turn: active=" +
            std::to_string(node.observation_turn_active_) + " yaw=" + std::to_string(command.yaw_rate) +
            " anchor=" + std::to_string(node.observation_hold_.anchor().x) + "," +
            std::to_string(node.observation_hold_.anchor().y));
    node.yaw_ = 0;
    node.finish_observation(command, {});
    require(!node.observation_turn_active_ && !node.have_observation_target_ && node.plan_pending_,
            "observation yaw completion still waits for low horizontal speed");
}

void required_drift_regressions()
{
    ExplorationNode node;
    reset(node, {.03, 0});
    node.v_fwd_est_ = -.1;
    PositionHold boundary_hold;
    boundary_hold.begin({.05, 0});
    const auto inward = node.turn_position_correction(boundary_hold, {});
    require(inward.x > 0 && node.turn_correction_clear(inward, {}) &&
            !node.turn_correction_clear({-.1, 0}, {}),
            "short inward XY correction was required to cross the entire boundary margin in one tick");
    reset(node, {-.08, -.15});
    node.yaw_ = M_PI;
    node.v_fwd_est_ = .12;
    const auto backed_up = node.pd_to_point_avoid({2, 0}, {});
    require(!node.global_failed_ && std::abs(node.last_yaw_rate_) > 0 && backed_up.x < 0,
            "backing behind the takeoff origin prevents a valid inward turn/velocity command");
    reset(node, {.08, -.15});
    node.yaw_ = M_PI;
    node.v_fwd_est_ = .12;  // Drifting out of the field after backing away at takeoff.
    const auto turn = node.pd_to_point_avoid({2, 0}, {});
    require(!node.global_failed_ && node.global_tracker_->reorienting() &&
            std::abs(node.last_yaw_rate_) > 0 && turn.x < 0,
            "offset boundary takeoff cannot turn and correct outward drift simultaneously");
    node.required_blocked_replan_s_ = .1;
    node.required_blocked_time_valid_ = true;
    node.required_blocked_since_ = node.steady_seconds() - 1;
    const auto replans = node.required_blocked_replans_;
    node.pd_to_point_avoid({2, 0}, {});
    require(node.required_blocked_replans_ > replans,
            "persistent drift prevents retrying a blocked required path");

    reset(node, {1.95, 0});
    node.v_fwd_est_ = .15;
    auto correction = node.pd_to_point_avoid({2, 0}, {});
    require(node.global_at_goal_ && node.required_goal_holding_ && correction.x < 0,
            "arrival tolerance or distance envelope erased active endpoint braking");
    node.py_ = .15;
    for (int i = 0; i < 20; ++i) correction = node.pd_to_point_avoid({2, 0}, {});
    require(correction.y < 0 && std::abs(node.last_yaw_rate_) < 1e-9 &&
            node.required_goal_hold_.anchor().x == 2 && node.required_goal_hold_.anchor().y == 0,
            "endpoint drift relatches the anchor or turns to chase the goal bearing");
}

void corridor_registered_cloud_regression()
{
    ExplorationNode node;
    require(node.corridor_cloud_topic_ == "/cloud_registered",
            "hardware default does not consume Point-LIO registered scans");
    reset(node, {2, 0});
    node.has_pose_ = node.corridor_enabled_ = true;
    require(node.corridor_->configure({2, 0}, {2, -3}), "configure cloud fixture");
    node.corridor_->start({2, 0}, node.steady_seconds());

    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = "camera_init";
    cloud->header.stamp.sec = 1;
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(4);
    sensor_msgs::PointCloud2Iterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
    // Two visible wall returns, one outside the height slice, one beyond range.
    const float samples[4][3] = {{1.25f, -.5f, .8f}, {2.75f, -.5f, .8f},
                                {1.25f, -.5f, 2.f}, {20.f, 0.f, .8f}};
    for (const auto& p : samples) {
        *x = p[0]; *y = p[1]; *z = p[2]; ++x; ++y; ++z;
    }
    node.on_cloud(cloud);
    require(node.corridor_->points().size() == 2 &&
            node.corridor_cloud_raw_points_ == 4 && node.corridor_cloud_height_points_ == 3 &&
            node.corridor_cloud_kept_points_ == 2 && node.corridor_cloud_filter_reason_ == "accepted",
            "active corridor failed to receive the shared registered topic or account for filters");

    ++cloud->header.stamp.sec;
    node.pz_ = 4;
    node.on_cloud(cloud);
    require(!node.corridor_->points().empty() &&
            node.corridor_cloud_filter_reason_ == "height_slice_empty",
            "empty height packets erased the bounded measured window");
    node.pz_ = .8;
    node.yaw_rate_est_ = node.corridor_cloud_max_yaw_rate_ + .1;
    node.on_cloud(cloud);
    require(node.corridor_cloud_filter_reason_ == "yaw_rate_exceeded", "missing yaw rejection reason");
    node.yaw_rate_est_ = 0;
    cloud->header.frame_id = "body";
    node.on_cloud(cloud);
    require(node.corridor_cloud_filter_reason_ == "wrong_frame", "missing cloud frame rejection reason");

    // An explicitly selected dense input must remain separate from exploration scans.
    cloud->header.frame_id = "camera_init";
    ++cloud->header.stamp.sec;
    node.corridor_cloud_topic_ = "/corridor/cloud_registered_dense";
    node.on_cloud(cloud);
    require(node.corridor_->points().size() == 2, "registered scans changed the selected dense input");
    node.on_corridor_cloud(cloud);
    require(node.corridor_->points().size() == 2 && node.corridor_cloud_filter_reason_ == "accepted",
            "explicit dense callback failed after changing hardware default");
    node.on_corridor_cloud(cloud);
    require(node.corridor_cloud_filter_reason_ == "nonadvancing_stamp",
            "repeated cloud timestamps were not diagnosed");
    std::cout << "hardware corridor cloud routing and empty/rejected scan diagnostics passed\n";
}
void visual_sampling_brake_regression()
{
    using Phase = VisualDrop::Phase;
    struct Scenario { const char* name; Vec2 start, object, wind; double lag, delay, yaw; };
    const std::vector<Scenario> scenarios{
        {"logged first target", {0,0}, {1.369,-.161}, {}, .65, .26, 0},
        {"long straight", {1,0}, {4,0}, {}, .65, .26, 0},
        {"slow response", {1,0}, {4,0}, {}, .80, .30, 0},
        {"headwind", {1,0}, {4,0}, {-.04,0}, .65, .26, 0},
        {"crosswind and turn", {1,-1}, {4,1}, {.015,.04}, .65, .26, -.4},
        {"tailwind", {1,0}, {4,0}, {.04,0}, .65, .26, 0},
    };
    bool smooth = true;
    for (const auto& scenario : scenarios) {
        ExplorationNode node;
        node.visual_drop_enabled_ = true;
        node.vision_reader_.reset();
        VisualDropConfig cfg; cfg.sample_seconds = 2.0;
        node.visual_drop_ = VisualDrop(cfg);
        auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
        goal->header.frame_id = "camera_init"; goal->header.stamp = node.now();
        goal->point.x = 7; goal->point.y = 4;
        node.on_goal(goal);
        acfly_interfaces::msg::ExplorationContext context;
        context.mission = goal->header; context.home_z = .2; context.cruise_z = 1.4; context.height_tolerance = .1;
        node.drop_context_ = context;
        reset(node, scenario.start); node.pz_ = 1.4; node.yaw_ = scenario.yaw;
        node.has_pose_ = node.has_goal_ = true;
        const double base = node.steady_seconds();
        node.vision_queue_->enqueue({{1, scenario.object,10,true,base-1,base}},scenario.start);
        node.activate_visual_target(base);
        constexpr double dt = .02;
        std::deque<Vec2> delayed(static_cast<size_t>(std::lround(scenario.delay/dt)), Vec2{});
        Vec2 velocity{}, anchor{}, direction{};
        double yaw_rate=0, entered=-1, descending=-1, entry_speed=0, peak=0, retreat=0;
        for (int tick=0; tick<3000; ++tick) {
            const double elapsed=tick*dt;
            const auto before=node.visual_drop_.phase();
            const Vec2 body=node.visual_drop_velocity({},base+elapsed);
            const auto phase=node.visual_drop_.phase();
            if ((before==Phase::Approach || before==Phase::Idle) && phase==Phase::Sample) {
                entered=elapsed; anchor=node.visual_drop_.lidar_target(node.yaw_);
                direction={scenario.object.x-anchor.x,scenario.object.y-anchor.y};
                const double distance=std::hypot(direction.x,direction.y);
                require(node.drop_entry_ && std::hypot(node.px_-node.drop_entry_->x,node.py_-node.drop_entry_->y)<=node.goal_tol_+1e-9,
                        "sampling began away from the selected safe entry");
                direction.x/=distance; direction.y/=distance;
                peak=(node.px_-anchor.x)*direction.x+(node.py_-anchor.y)*direction.y;
                entry_speed=std::hypot(velocity.x,velocity.y);
            }
            if (phase==Phase::Descend && descending<0) descending=elapsed;
            if (entered>=0) {
                const double progress=(node.px_-anchor.x)*direction.x+(node.py_-anchor.y)*direction.y;
                if (phase==Phase::Sample) peak=std::max(peak,progress);
                retreat=std::max(retreat,peak-progress);
            }
            if (descending>=0 && elapsed>=descending+1.5) break;
            const double c=std::cos(node.yaw_),s=std::sin(node.yaw_);
            const Vec2 world{c*body.x-s*body.y,s*body.x+c*body.y};
            delayed.push_back(world); const auto applied=delayed.front(); delayed.pop_front();
            const double response=1.0-std::exp(-dt/scenario.lag);
            velocity.x+=response*(applied.x+scenario.wind.x-velocity.x);
            velocity.y+=response*(applied.y+scenario.wind.y-velocity.y);
            yaw_rate+=(node.last_yaw_rate_-yaw_rate)*dt/.20;
            node.px_+=velocity.x*dt; node.py_+=velocity.y*dt; node.yaw_+=yaw_rate*dt;
            node.v_fwd_est_=std::cos(node.yaw_)*velocity.x+std::sin(node.yaw_)*velocity.y;
            node.v_lat_est_=-std::sin(node.yaw_)*velocity.x+std::cos(node.yaw_)*velocity.y;
            node.yaw_rate_est_=yaw_rate; node.last_command_world_=world;
        }
        require(entered>=0 && descending>entered && entered<50,
                std::string(scenario.name)+": early braking stranded the target before its safe entry");
        require(std::abs(descending-entered-cfg.sample_seconds)<.021,
                "approach braking changed the full sampling interval");
        std::cout<<"sample braking "<<scenario.name<<": entry="<<entered<<"s speed="<<entry_speed
                 <<" overshoot="<<peak<<" retreat="<<retreat<<'\n';
        smooth = smooth && entry_speed<=.18 && peak<=.10 && retreat<=.10;
    }
    require(smooth, "visual approach overshot or pulled back across its actual sampling anchor");
}

// An object sitting inside an obstacle inflation (about one metre from a
// pillar is common) must still be approached: the tracker aims at a sampling
// entry point on a validated route instead of the blocked object itself.
void visual_drop_entry_point_avoids_blocked_object()
{
    ExplorationNode node;
    node.visual_drop_enabled_ = true;
    node.vision_reader_.reset();
    reset(node, {1, 0});
    node.has_pose_ = node.has_goal_ = true;
    node.goal_ = {7, 4};
    acfly_interfaces::msg::ExplorationContext context;
    context.mission = node.goal_header_; context.home_z = .2; context.cruise_z = 1.4; context.height_tolerance = .1;
    node.drop_context_ = context;
    node.pz_ = 1.4;
    const Vec2 object{3.0, 0.0};
    const Obstacles pillar{{3.45, 0.0, .15}};   // object 0.45m from the pillar: inside r+robot+inflate
    const double base = node.steady_seconds();
    node.vision_queue_->enqueue({{1, object, 10, true, base-1, base}}, {1, 0});
    node.activate_visual_target(base);
    double forward = 0;   // commands ramp from rest under the acceleration slew
    for (int tick = 0; tick < 50; ++tick) {
        const Vec2 body = node.visual_drop_velocity(pillar, base + tick * .02);
        node.last_command_world_ = {std::cos(node.yaw_)*body.x - std::sin(node.yaw_)*body.y,
                                    std::sin(node.yaw_)*body.x + std::cos(node.yaw_)*body.y};
        forward = std::max(forward, body.x);
    }
    require(node.visual_drop_.phase() == VisualDrop::Phase::Approach && forward > .01,
            "clear approach to the compensated endpoint stalled or sampled too early");
    require(node.drop_entry_ && !node.global_goal_blocked_ && !node.global_failed_,
            "blocked object still stalled the approach instead of using an entry point");
    require(std::hypot(node.drop_entry_->x-2,node.drop_entry_->y)<1e-9 && !node.drop_obstructed_approach_ &&
            drop_descent_clear(*node.drop_entry_,node.visual_drop_.alignment_target(0),pillar,node.required_config()),
            "obstacle behind the compensated endpoint unnecessarily triggered special selection");
    (void)forward;
    // Straight, unobstructed case keeps the entry point on the direct route.
    ExplorationNode plain;
    plain.visual_drop_enabled_ = true; plain.vision_reader_.reset();
    reset(plain, {1, 0}); plain.has_pose_ = plain.has_goal_ = true; plain.goal_ = {7, 4};
    plain.drop_context_ = context; plain.pz_ = 1.4;
    plain.vision_queue_->enqueue({{1, object, 10, true, base-1, base}}, {1, 0});
    plain.activate_visual_target(base);
    plain.visual_drop_velocity({}, base);
    require(plain.drop_entry_ && std::abs(plain.drop_entry_->y) < 1e-6 &&
            std::abs(plain.drop_entry_->x - 2) < .02 && !plain.drop_obstructed_approach_,
            "entry point left the straight route in an empty field");
}

// A route being flown is revalidated with the inner execution margin: cutting
// a corner a few centimetres into the planning margin keeps the route, while
// entering the inner ring still discards it.
void executing_route_tolerates_turn_deviation()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2, 0});
    node.ggcfg_.inflate = .30; node.required_path_exec_inflate_ = .15;   // fixed rings for the numbers below
    const Vec2 goal{4, 0};
    const Obstacles pillar{{3.5, 0.85, .10}};   // no-go ring 0.70m, inner execution ring 0.55m
    node.pd_to_point_avoid(goal, pillar);
    require(node.global_has_ && !node.global_failed_, "straight required route beside the pillar was not accepted");
    // 0.63m from the pillar centre and rejoining towards it: inside the planning
    // ring, outside the execution ring. The old full-margin check rejected this.
    node.px_ = 3.2; node.py_ = 0.3;
    auto searches = node.global_path_searches_;
    node.pd_to_point_avoid(goal, pillar);
    require(node.global_has_ && !node.global_failed_ && node.global_path_searches_ == searches,
            "turn deviation inside the planning margin discarded the executing route");
    // 0.50m from the centre: inside the execution ring, the old route is
    // discarded and a fresh one is searched from the current position.
    node.py_ = 0.45;
    searches = node.global_path_searches_;
    node.pd_to_point_avoid(goal, pillar);
    require(node.global_path_searches_ > searches,
            "route inside the inner execution ring was kept without a new search");
}

void visual_sampling_brake_preserves_correction()
{
    ExplorationNode node;
    node.vision_reader_.reset();
    reset(node, {2,0});
    const Vec2 goal{4,0};
    node.pd_to_point_avoid(goal, {}); // Commit the straight path before lateral drift.
    node.py_=.12; node.v_fwd_est_=.08; node.v_lat_est_=.12;
    Vec2 command;
    for (int tick=0; tick<30; ++tick) command=node.pd_to_point_avoid(goal, {}, .08);
    require(command.x>0 && command.x<=.08+1e-9 && command.y<-.08,
            "approach cap removed lateral drift correction or stopped forward progress");
    const Obstacles overlap{{node.px_,node.py_,.1}};
    const auto blocked=node.pd_to_point_avoid(goal, overlap, .08);
    require(blocked.x==0 && blocked.y==0,
            "positive sampling entry speed bypassed physical obstacle checks");

    reset(node, {2,0});
    node.yaw_=M_PI; node.v_fwd_est_=.2;
    for (int tick=0; tick<30; ++tick) command=node.pd_to_point_avoid(goal, {}, .01);
    require(std::abs(node.last_yaw_rate_)>0 && command.x<-.01,
            "approach travel cap restricted XY correction while turning");
}

void home_visibility_shortcut_regression()
{
    ExplorationNode node;
    node.timer_->cancel();node.vision_reader_.reset();node.visual_drop_enabled_=false;
    const Vec2 goal{6.7,0};
    for (double distance : {1.3,2.0,4.0,5.8}) {
        const Vec2 start{goal.x-distance,0};
        reset(node,start);node.goal_=goal;node.reset_home_approach();
        node.homing_=node.has_goal_=node.has_pose_=true;
        node.yaw_=M_PI/2;
        const auto command=node.home_target_velocity({});
        require(node.home_entry_plan_.ok && node.global_raw_.size()==2 && !node.global_failed_ && !node.home_approach_active_,
                "clear home approach must go directly to the near circle entry");
        require(std::hypot(node.home_entry_plan_.entry.x-(goal.x-1),node.home_entry_plan_.entry.y)<1e-9,
                "clear home approach selected an unnecessary far-side entry");
        require(command.y<0 && node.last_yaw_rate_<0,
                "home approach locked yaw early or gated XY while heading was misaligned");
        const auto original=node.home_entry_plan_.path;
        const auto searches=node.global_path_searches_;
        node.px_+=.01;node.home_target_velocity({});
        require(node.home_entry_plan_.path.size()==original.size() &&
            std::hypot(node.home_entry_plan_.path.front().x-original.front().x,
                       node.home_entry_plan_.path.front().y-original.front().y)<1e-9 &&
            node.global_path_searches_==searches,"safe home entry was replanned on normal movement");
    }
    for (double side : {-1.0,1.0}) {
        reset(node,{2,side*.2});node.goal_={5,0};node.reset_home_approach();node.homing_=true;
        const Obstacles obstacles{{4.2,0,.15},{3.1,-side*1.1,.2}};
        node.home_target_velocity(obstacles);
        require(node.home_entry_plan_.ok && !node.home_approach_active_ && !node.global_failed_,
                "obstructed home approach did not find an alternate entry");
        const auto entry=node.home_entry_plan_.entry;
        require(std::abs(std::hypot(entry.x-5,entry.y)-1)<1e-8 && std::abs(entry.y)>.5,
                "obstructed home entry is not on the one-metre circle");
        require(drop_descent_clear(entry,node.goal_,obstacles,node.required_config(),node.goal_tol_,node.exec_config().inflate),
                "home entry checked only the point, not the whole descent envelope");
        double length=0;
        for(size_t i=1;i<node.home_entry_plan_.path.size();++i) {
            auto a=node.home_entry_plan_.path[i-1],b=node.home_entry_plan_.path[i];
            length+=std::hypot(b.x-a.x,b.y-a.y);
            require(obstacle_escape_clear(a,b,obstacles,node.home_transit_config()),"home entry route crosses an obstacle");
        }
        require(length<4,"home entry took an unnecessary far-side detour");
        node.px_=entry.x;node.py_=entry.y;node.yaw_=side*.6;
        node.home_target_velocity(obstacles);
        require(node.home_approach_active_ && node.home_approach_yaw_==node.yaw_,
                "reaching safe home entry failed to capture actual yaw");
    }
    reset(node,{2,0});node.goal_={5,0};node.reset_home_approach();node.homing_=true;
    node.home_target_velocity({{5,0,.2}});
    require(node.global_failed_ && node.global_goal_blocked_ && !node.home_approach_active_,
            "occupied home endpoint fabricated an entry or began descent");
    // The boundary is a task limit, not a physical wall; the whole descent must
    // still stay inside it, and the intermediate point must have arrival room.
    reset(node,{3,3});node.goal_={node.ggcfg_.max_x-.02,node.ggcfg_.max_y-.02};
    node.reset_home_approach();node.homing_=true;node.home_target_velocity({});
    require(node.home_entry_plan_.ok && !node.global_failed_ &&
        drop_descent_clear(node.home_entry_plan_.entry,node.goal_,{},node.required_config(),node.goal_tol_,node.exec_config().inflate),
        "home circle search rejected a valid near-boundary endpoint");
    node.home_straight_approach_m_=0;
    reset(node,{2,0});node.goal_={5,0};node.reset_home_approach();node.homing_=true;
    node.home_target_velocity({});
    require(!node.home_entry_plan_.ok && !node.home_approach_active_ && !node.global_failed_ &&
        std::hypot(node.global_raw_.back().x-5,node.global_raw_.back().y)<1e-9,
        "disabling home entry broke the legacy checked endpoint path");
    std::cout << "home safe entry: direct/blocked/mirrored/boundary routes, nearest entry, yaw and XY continuity passed\n";
}

void home_recorded_return_corner_regression()
{
    ExplorationNode node;
    node.timer_->cancel();node.vision_reader_.reset();node.visual_drop_enabled_=false;
    // 2026-10-06 editor scene: the obstacle requires a detour, but the fixed
    // circle sample added another turn from 3.3 to 10.6 degrees at descent.
    for (double side : {-1.0,1.0}) {
        node.ggcfg_={-.5,side>0 ? -3.8 : -4.6,9,side>0 ? 4.6 : 3.8,.04,.4,.1,.6};
        const Vec2 start{3.3217917410421633,side*3.2837077396187264};
        const Vec2 goal{7.75,side*4.3};
        const Obstacles obs{{2.29375,side*3.19375,.33936536824085956},
            {3.7,-side*2.175,.39316150714175185},
            {4.8625,side*2.9875,.33936536824085956},
            {8.81875,side*1.39375,.5}};
        reset(node,start);node.goal_=goal;node.homing_=true;node.reset_home_approach();
        node.home_target_velocity(obs);
        const auto check_final_line=[&] {
            const auto& plan=node.home_entry_plan_;
            require(plan.ok && plan.path.size()>=2,"recorded home detour has no route");
            const auto a=plan.path[plan.path.size()-2],e=plan.entry;
            const double cross=(e.x-a.x)*(goal.y-e.y)-(e.y-a.y)*(goal.x-e.x);
            require(std::abs(cross)<1e-9,
                    "home safe entry inserts an avoidable corner before the descent");
            require(std::abs(std::hypot(e.x-goal.x,e.y-goal.y)-1)<1e-9,
                    "straightening moved the home descent outside the one-metre circle");
            require(drop_descent_clear(e,goal,obs,node.required_config(),node.goal_tol_,node.exec_config().inflate),
                    "straightened home descent did not validate the complete envelope");
            for(size_t i=1;i<plan.path.size();++i)
                require(obstacle_escape_clear(plan.path[i-1],plan.path[i],obs,node.home_transit_config()),
                        "straightened home route cut through the obstacle");
        };
        check_final_line();
        require(node.home_entry_plan_.path.size()==3,
                "recorded obstacle detour was erased or kept redundant corners");
        double length=0;
        for(size_t i=1;i<node.home_entry_plan_.path.size();++i) {
            const auto a=node.home_entry_plan_.path[i-1],b=node.home_entry_plan_.path[i];
            length+=std::hypot(b.x-a.x,b.y-a.y);
        }
        require(length<3.60,"home straightening selected a farther circle entry");
        require(node.home_curve_active_ && node.global_traj_.size()>10,
                "home return still follows separate PD waypoints instead of a curve");
        const auto trajectory=node.global_traj_;
        const auto join=node.home_entry_plan_.path[1];
        for(size_t i=1;i+1<trajectory.size();++i) {
            require(std::hypot(trajectory[i].p.x-join.x,trajectory[i].p.y-join.y)>.001,
                    "rounded home curve still passes through the sharp waypoint");
            const auto a=trajectory[i-1].p,b=trajectory[i].p,c=trajectory[i+1].p;
            require(std::abs(std::remainder(std::atan2(c.y-b.y,c.x-b.x)-std::atan2(b.y-a.y,b.x-a.x),2*M_PI))<.03,
                    "home reference still contains a heading discontinuity");
        }
        const auto entry=node.home_entry_plan_.entry;
        const auto searches=node.global_path_searches_;
        bool checked_curve=false;
        for(const auto& point:trajectory) {
            node.px_=point.p.x;node.py_=point.p.y;node.yaw_=point.theta;
            node.v_fwd_est_=.35;node.v_lat_est_=0;
            node.last_command_world_={.35*std::cos(point.theta),.35*std::sin(point.theta)};
            const auto command=node.home_target_velocity(obs);
            if(std::abs(point.kappa)>.01) {
                checked_curve=true;
                require(std::hypot(command.x,command.y)>.1,
                        "home curve brakes to zero at an intermediate waypoint");
            }
            if(std::hypot(point.p.x-goal.x,point.p.y-goal.y)>1+1e-9)
                require(!node.home_approach_active_,"home lowering started outside one metre");
            require(std::hypot(node.home_entry_plan_.entry.x-entry.x,node.home_entry_plan_.entry.y-entry.y)<1e-12 &&
                    node.global_path_searches_==searches,"valid home curve was replanned during normal progress");
        }
        require(checked_curve && node.home_approach_active_,
                "home curve failed to transition to straight descent at one metre");
    }
    std::cout << "recorded home return: mirrored obstacle detours and collinear descent handoff passed\n";
}

void home_virtual_boundary_drift_regression()
{
    ExplorationNode node;
    node.timer_->cancel();node.vision_reader_.reset();
    const Vec2 goal{5,node.ggcfg_.max_y-.3};
    reset(node,{2,goal.y});node.goal_=goal;node.homing_=true;node.reset_home_approach();
    node.home_target_velocity({});
    require(node.home_curve_active_,"boundary drift fixture has no checked home curve");
    node.px_+=.05;node.py_=node.ggcfg_.max_y+.03;
    node.yaw_=0;node.v_fwd_est_=.2;node.v_lat_est_=.1;
    node.last_command_world_={.2,-.05};
    const auto command=node.home_target_velocity({});
    require(command.x>0 && command.y<0 && !node.global_braking_blocked_ && !node.global_failed_,
            "virtual boundary drift cleared XY instead of continuing home with inward correction");
    const Vec2 stop{node.px_+.4,node.py_+.02};
    require(node.required_motion_clear(stop,{}),"home stopping projection is still blocked by the virtual boundary");
    require(!node.required_motion_clear(stop,{{stop.x,stop.y,.05}}),
            "home virtual-boundary exception disabled real obstacle protection");
    node.global_has_=false;
    require(!node.required_motion_clear(stop,{}),"home boundary exception admitted an unplanned motion");
    node.global_has_=true;node.homing_=false;
    require(!node.required_motion_clear(stop,{}),"home boundary exception leaked into normal exploration");
    node.homing_=true;
    require(node.corridor_->configure({6,goal.y},{6,goal.y-3}),"boundary test corridor configuration failed");
    node.corridor_->start(goal,node.steady_seconds());
    require(!node.required_motion_clear(stop,{}),"home boundary exception leaked into corridor control");
    std::cout << "home virtual boundary: drift correction, real obstacles and phase scope passed\n";
}

void home_straight_descent_regression()
{
    ExplorationNode node;
    node.timer_->cancel();
    node.corridor_enabled_ = node.corridor_lower_enabled_ = true;
    node.home_straight_approach_m_ = 1.0;
    node.home_approach_speed_ = .4;
    node.corridor_flight_z_ = .05;
    node.corridor_z_tol_ = .05;
    node.ccfg_.descend_before_entry = true;
    node.ccfg_.settle_time = 0.0;
    node.corridor_ = std::make_unique<CorridorController>(node.ccfg_);
    const Vec2 goal{(node.ggcfg_.min_x+node.ggcfg_.max_x)/2, node.ggcfg_.max_y-.3};
    auto message = std::make_shared<geometry_msgs::msg::PointStamped>();
    message->header.frame_id = "camera_init"; message->header.stamp.sec = 700;
    message->point.x = goal.x; message->point.y = goal.y;
    const auto mission = [&] {
        ++message->header.stamp.sec; node.on_goal(message);
        node.homing_ = node.has_goal_ = node.has_pose_ = true;
        node.visual_drop_enabled_ = false;
        reset(node, {goal.x, goal.y-.8});
        node.pz_=1.2; node.yaw_=0;
        node.corridor_disabled_for_goal_=false;
        require(node.corridor_->configure({goal.x+1,goal.y},{goal.x+1,goal.y-4}), "configure home corridor");
    };
    mission();
    // Even a 90-degree heading error must not gate world-Y motion near home.
    auto command=node.home_target_velocity({});
    node.update_home_descent();
    require(node.home_approach_active_ && command.y>.001 && std::abs(node.last_yaw_rate_)<1e-9,
            "home alignment chased goal yaw or lost lateral XY correction");
    require(node.home_descent_started_ && node.drop_command_altitude_==.05 &&
            node.drop_command_speed_==node.corridor_descend_speed_, "home approach did not start descent");
    require(node.global_raw_.size()==2 && node.global_raw_.back().x==goal.x &&
            node.global_raw_.back().y==goal.y, "home shortcut retained the terminal A* bend");
    // Crossing outward again must not unlock yaw or return to cruise altitude.
    node.py_=goal.y-1.04;
    node.home_target_velocity({});node.update_home_descent();
    require(node.home_approach_active_ && node.drop_command_altitude_==.05,
            "distance jitter reset the latched approach/descent");
    // Once the whole actual descent is approved, map changes do not interrupt it.
    node.py_=goal.y-.8;node.pz_=.72;
    command=node.home_target_velocity({{goal.x,goal.y-.4,.05}});node.update_home_descent();
    require(node.home_approach_active_ && !node.global_failed_ && !node.global_braking_blocked_ &&
            node.drop_command_altitude_==.05 && command.y>0,
            "committed home descent was interrupted by a changed obstacle fit");
    // Remote endpoint and disabled corridor may not descend early.
    mission();node.py_=goal.y-1.2;node.home_target_velocity({});node.update_home_descent();
    require(!node.home_approach_active_ && !node.home_descent_started_,"descent started outside configured distance");
    require(node.global_raw_.size()==2 && !node.global_failed_,
            "remote home planning retained an artificial boundary connector");
    const Vec2 line_start=node.global_raw_.front();
    for (const auto& p : node.global_traj_)
        require(std::abs((p.p.x-line_start.x)*(goal.y-line_start.y)-
                         (p.p.y-line_start.y)*(goal.x-line_start.x))<1e-9,
                "trajectory smoothing curved a clear home straight line");
    mission();node.py_=goal.y-1.11;node.home_target_velocity({});node.update_home_descent();
    require(!node.home_descent_started_,"home descent started before reaching the selected entry tolerance");
    node.py_=goal.y-.99;node.home_target_velocity({});node.update_home_descent();
    require(node.home_descent_started_,"safe home approach failed to start descent inside one metre");
    // Whole-route straight flight near an edge, with drift correction outside
    // the normal wall inset, must retain nonzero output before the last metre.
    mission();node.px_=goal.x-2;node.py_=goal.y;
    command=node.home_target_velocity({});
    require(node.global_raw_.size()==2 && command.x>0 && !node.global_braking_blocked_,
            "clear long home connection along the boundary lost motion output");
    node.py_-=.15;
    require(node.required_motion_clear({node.px_+.03,node.py_+.03},{}),
            "home line drift correction was rejected outside its narrow capsule");
    require(node.required_motion_clear({node.px_,node.ggcfg_.max_y+.01},{}),
            "checked home correction is blocked by a virtual field boundary");
    mission();node.corridor_disabled_for_goal_=true;
    node.home_target_velocity({});node.update_home_descent();
    require(!node.home_descent_started_,"disabled corridor triggered low flight");
    mission();node.corridor_lower_enabled_=false;
    node.home_target_velocity({});node.update_home_descent();
    require(!node.home_descent_started_,"disabled corridor lowering still descended");
    node.corridor_lower_enabled_=true;
    mission();node.corridor_enabled_=false;
    node.home_target_velocity({});node.update_home_descent();
    require(!node.home_descent_started_,"home-only mission descended for a nonexistent corridor");
    node.corridor_enabled_=true;
    // Actual timer envelope: XY arrival transfers while still moving and high.
    mission();node.py_=goal.y-.08;node.pz_=.9;node.v_lat_est_=.08;
    node.on_timer();
    require(node.corridor_->active() && node.corridor_->phase()==CorridorPhase::Rotate &&
            std::abs(last_control.altitude-.05)<1e-12,
            "home arrival waited for Z or zero speed, or lost descent during handoff");
    node.px_=goal.x+.07;node.py_=goal.y;node.yaw_=0;node.pz_=.8;
    node.v_fwd_est_=.03;node.v_lat_est_=0;
    node.on_timer();
    require(last_control.velocity.angular.z<0 && last_control.velocity.linear.x<0 &&
            last_control.altitude==.05 && last_control.vertical_speed_limit==node.corridor_descend_speed_,
            "rotation lost simultaneous XY correction / altitude descent");
    require(!node.corridor_->rotationDone(),"test must exercise descent before yaw completion");
    node.yaw_=node.ccfg_.initial_yaw;node.on_timer();
    require(node.corridor_->rotationDone() && node.corridor_->phase()==CorridorPhase::Rotate,
            "entry translation began before Z was ready");
    node.pz_=node.corridor_flight_z_;node.yaw_+=2*node.ccfg_.heading_tolerance;node.on_timer();
    require(node.corridor_->phase()==CorridorPhase::Rotate && last_control.velocity.angular.z<0,
            "entry used a stale aligned heading after yaw drift during descent");
    node.yaw_=node.ccfg_.initial_yaw;node.on_timer();
    require(node.corridor_->phase()==CorridorPhase::Entry,"aligned heading and height failed to release entry");
    ++message->header.stamp.sec;node.on_goal(message);
    require(!node.home_approach_active_ && !node.home_descent_started_ && !node.home_entry_plan_.ok && !node.home_leg_target_ && !node.home_curve_active_,
            "new mission retained old home approach / descent state");
    std::cout << "home approach: direct XY, yaw lock, descent, committed straight motion, moving XY handoff and corridor gates passed\n";
}

void visual_safe_entry_regressions()
{
    using Phase = VisualDrop::Phase;
    ExplorationNode node;
    node.timer_->cancel(); node.vision_reader_.reset(); node.visual_drop_enabled_=true;
    reset(node,{1,0}); node.has_pose_=node.has_goal_=true; node.homing_=false; node.goal_={7,4};
    node.pz_=1.4; node.yaw_=.2;
    acfly_interfaces::msg::ExplorationContext context;
    context.mission=node.goal_header_; context.home_z=.2; context.cruise_z=1.4; context.height_tolerance=.1;
    node.drop_context_=context;
    const double base=node.steady_seconds();
    node.vision_queue_->enqueue({{1,{4,0},10,true,base-1,base}},{1,0});
    node.activate_visual_target(base);
    require(std::abs(node.visual_drop_.yaw()-.2)<1e-9,"target activation reset the held heading to zero");
    const Obstacles obstacles{{2.5,0,.35}};
    node.visual_drop_velocity(obstacles,base);
    require(node.visual_drop_.phase()==Phase::Approach && node.drop_entry_,"hidden target skipped its safe entry");
    require(drop_descent_clear(*node.drop_entry_,node.visual_drop_.alignment_target(node.yaw_),obstacles,node.required_config()),
            "selected point cannot descend straight to compensated package position");
    const Vec2 entry=*node.drop_entry_;
    // 进入原1米圈但仍位于障碍旁，不得按旧半径触发采样。
    node.px_=3.2; node.py_=0;
    node.visual_drop_velocity(obstacles,base+.02);
    require(node.visual_drop_.phase()==Phase::Approach,"old 1m trigger bypassed chosen entry");
    node.px_=entry.x;node.py_=entry.y;node.last_command_world_={};
    node.visual_drop_velocity(obstacles,base+1);
    require(node.visual_drop_.phase()==Phase::Sample,"arrival at safe entry did not start sampling");
    node.visual_drop_velocity(obstacles,base+2.01);
    require(node.visual_drop_.phase()==Phase::Descend,"safe descent was not authorized after sampling");
    // 下降期间拟合圈抖动/扩张不得重新走A*或把速度置零；偏航时仍锁原yaw。
    node.py_+=.15;node.yaw_+=.12;node.v_lat_est_=.10;node.last_command_world_={};
    const auto searches=node.global_path_searches_;
    const Obstacles changed{{node.px_,node.py_,.2},{3.5,1,.3}};
    Vec2 command;
    for(int i=0;i<20;++i) {
        command=node.visual_drop_velocity(changed,base+2.03+i*.02);
        node.last_command_world_={std::cos(node.yaw_)*command.x-std::sin(node.yaw_)*command.y,
                                 std::sin(node.yaw_)*command.x+std::cos(node.yaw_)*command.y};
    }
    require(!node.global_failed_ && !node.global_braking_blocked_ &&
            std::hypot(command.x,command.y)>.05 && node.global_path_searches_==searches && node.drop_path_.size()==2 &&
            node.last_yaw_rate_<0,"committed descent replanned, lost XY correction or stopped holding yaw");
    require(node.drop_command_altitude_<node.pz_,"generic obstacle guard paused committed descent height");

    // 精确目标发生位移：必须先检查，再下降，不能先让tick触发释放时钟。
    node.visual_drop_=VisualDrop();node.vision_queue_->reset();node.drop_entry_.reset();
    reset(node,{3,0});node.pz_=1.4;node.has_goal_=node.has_pose_=true;
    node.vision_queue_->enqueue({{2,{4,0},10,true,base,base}},{1,0});node.activate_visual_target(base+10);
    const Obstacles far{{4.75,0,.1}}; // refined lidar point 4.27 overlaps the fitted circle + body, not just the outer margin
    node.visual_drop_velocity(far,base+10);
    require(node.visual_drop_.phase()==Phase::Sample,"clear coarse corridor failed fixture");
    VisionFrame frame;frame.seq=2;frame.stamp=base+10.2;frame.count=1;frame.targets[0]={1,4.5,0};
    node.visual_drop_.sample(frame,{1,0},base+10.2,{});
    node.visual_drop_velocity(far,base+11.01);
    require(node.visual_drop_.phase()==Phase::Approach && node.visual_drop_.xy_since()<0 &&
            node.drop_command_altitude_==1.4 && node.vision_queue_->active()->track_id==2,
            "unsafe refined point descended, released or changed target identity");
    node.visual_drop_=VisualDrop();node.vision_queue_->reset();node.drop_cruise_z_=1.4;
    node.px_=2;node.py_=0;node.pz_=.91;node.vision_queue_->enqueue({{3,{3,0},10,true,base,base}},{2,0});
    node.activate_visual_target(base+20);node.visual_drop_velocity({},base+20);
    require(node.visual_drop_.phase()==Phase::Sample && node.visual_drop_.return_z()==1.4 &&
            node.drop_command_altitude_==.91,"next target replaced cruise height with incomplete climb height");
    std::cout<<"safe-entry trigger, precise revalidation, locked yaw and committed XY/height passed\n";
}

void visual_normal_and_obstructed_entry_regressions()
{
    using Phase=VisualDrop::Phase;
    const auto prepare=[](ExplorationNode& node,Vec2 object) {
        node.timer_->cancel();node.vision_reader_.reset();node.visual_drop_enabled_=true;
        reset(node,{1,0});node.pz_=1.4;node.has_goal_=node.has_pose_=true;node.homing_=false;
        acfly_interfaces::msg::ExplorationContext context;
        context.mission=node.goal_header_;context.home_z=.2;context.cruise_z=1.4;context.height_tolerance=.1;
        node.drop_context_=context;
        const double t=node.steady_seconds();
        node.vision_queue_->enqueue({{1,object,10,true,t-1,t}},{1,0});node.activate_visual_target(t);
        return t;
    };
    ExplorationNode clear;
    const double t=prepare(clear,{4,0});
    clear.visual_drop_velocity({{2,3,.3}},t);
    require(clear.visual_drop_.phase()==Phase::Approach && !clear.drop_obstructed_approach_ &&
            clear.drop_entry_ && std::abs(clear.drop_entry_->x-3)<1e-9,
            "irrelevant obstacle triggered special selection or early sampling");
    clear.px_=2.0;clear.py_=.02;
    clear.visual_drop_velocity({{2,3,.3}},t+.1);
    require(std::hypot(clear.drop_entry_->x-3,clear.drop_entry_->y)<1e-9,
            "normal entry moved with tracking drift");
    clear.px_=3;clear.py_=0;
    clear.visual_drop_velocity({{2,3,.3}},t+.2);
    require(clear.visual_drop_.phase()==Phase::Sample && !clear.drop_obstructed_approach_,
            "unobstructed target did not sample at the ordinary 1m entry");

    ExplorationNode near;
    const double nt=prepare(near,{3,0});
    // Normal entry sits 0.76m from the obstacle centre: body+obstacle+margin+arrival
    // need 0.70m. An additional 0.30m planning margin would reject it unnecessarily.
    near.visual_drop_velocity({{2,.76,.1}},nt);
    require(!near.drop_obstructed_approach_ && near.drop_entry_ &&
            std::abs(near.drop_entry_->x-2)<1e-9 && near.visual_drop_.phase()==Phase::Approach &&
            std::abs(near.drop_transit_config().inflate-near.ggcfg_.inflate)<1e-9,
            "nearby adequate-clearance point required an extra obstacle standoff");

    ExplorationNode changed;
    const double ct=prepare(changed,{4,0});
    changed.visual_drop_velocity({},ct);
    require(!changed.drop_obstructed_approach_,"empty scene was obstructed");
    const Obstacles blocked{{2.5,0,.35}};
    changed.visual_drop_velocity(blocked,ct+.02);
    require(changed.drop_obstructed_approach_ && changed.drop_entry_ &&
            drop_descent_clear(*changed.drop_entry_,changed.visual_drop_.alignment_target(0),blocked,changed.required_config()),
            "newly blocked normal approach did not select a valid alternative");
    const Vec2 entry=*changed.drop_entry_;
    changed.px_=entry.x;changed.py_=entry.y;
    changed.visual_drop_velocity(blocked,ct+1);
    require(changed.drop_obstructed_approach_ && changed.visual_drop_.phase()==Phase::Sample &&
            std::abs(std::hypot(changed.visual_drop_.object().x-entry.x,changed.visual_drop_.object().y-entry.y)-1)<1e-9,
            "obstructed approach did not sample at its selected 1m circle entry");
    std::cout<<"normal 1m approach, irrelevant/nearby obstacles and conditional latched safe selection passed\n";
}

// 接近时转头与XY同时输出，到点才锁yaw；瞬时yaw不能改变所选入口。
void visual_entry_commitment_regression()
{
    using Phase=VisualDrop::Phase;
    ExplorationNode node;node.timer_->cancel();node.vision_reader_.reset();node.visual_drop_enabled_=true;
    reset(node,{1,0});node.pz_=1.4;node.yaw_=-1;node.has_goal_=node.has_pose_=true;
    acfly_interfaces::msg::ExplorationContext context;
    context.mission=node.goal_header_;context.home_z=.2;context.cruise_z=1.4;context.height_tolerance=.1;
    node.drop_context_=context;
    const double t=node.steady_seconds();
    node.vision_queue_->enqueue({{1,{4,0},10,true,t-1,t}},{1,0});node.activate_visual_target(t);
    auto command=node.visual_drop_velocity({},t);
    require(node.visual_drop_.phase()==Phase::Approach && node.last_yaw_rate_>0 &&
        std::hypot(command.x,command.y)>0,"approach locked activation yaw or gated XY during turning");
    const Vec2 entry=*node.drop_entry_;
    for(double yaw:{-.7,-.2,.1,.3}) {
        node.yaw_=yaw;node.px_+=.1;
        node.visual_drop_velocity({},t+.02);
        require(node.drop_entry_ && std::hypot(node.drop_entry_->x-entry.x,node.drop_entry_->y-entry.y)<1e-9,
            "turning moved the safe entry");
    }
    node.px_=entry.x;node.py_=entry.y;node.yaw_=.08;
    node.visual_drop_velocity({},t+.2);
    require(node.visual_drop_.phase()==Phase::Sample && std::abs(node.visual_drop_.yaw()-.08)<1e-9,
        "arrival did not capture actual heading");
    node.yaw_=.18;node.py_+=.05;node.last_command_world_={};
    command=node.visual_drop_velocity({},t+.22);
    require(node.last_yaw_rate_<0 && std::hypot(command.x,command.y)>0,
        "sampling lost locked heading or drift correction");

    // 小幅拟合变化未侵入到点包络时保持入口，不能因瞬时yaw改变而重选。
    node.visual_drop_=VisualDrop();node.vision_queue_->reset();reset(node,{1,0});node.yaw_=0;node.pz_=1.4;
    node.has_goal_=node.has_pose_=true;
    node.vision_queue_->enqueue({{2,{4.5,0},10,true,t,t}},{1,0});node.activate_visual_target(t+2);
    const Obstacles obs{{2.7,0,.3},{4,1.5,.25}};
    node.visual_drop_velocity(obs,t+2);require(node.drop_entry_.has_value(),"commitment fixture lacks entry");
    const Vec2 chosen=*node.drop_entry_;
    auto changed=obs;changed[0].r+=.005;
    node.yaw_=.7;
    node.visual_drop_velocity(changed,t+2.02);
    require(node.drop_entry_ && std::hypot(node.drop_entry_->x-chosen.x,node.drop_entry_->y-chosen.y)<1e-9 &&
        !node.global_failed_,"minor fitting/heading change replaced a still-safe entry or inserted recovery");
    std::cout<<"approach yaw/XY, arrival yaw capture and stable safe entry passed\n";
}

void visual_exit_shortcut_regression()
{
    // A shortcut changes the final connector. It must change the checked
    // payload heading with it, rather than turn toward the discarded segment.
    for(const int side:{-1,1}) {
        ExplorationNode node;node.timer_->cancel();node.vision_reader_.reset();
        node.ggcfg_={-5,-5,8,5,.04,.4,.1,.6};
        reset(node,{0,0});node.yaw_=0;node.homing_=false;
        VisionVisitTarget visit;visit.track_id=1;visit.position={4,0};
        node.visual_drop_.begin(visit,0);
        const Vec2 entry{3,0};
        node.drop_approach_={true,entry,{{0,0},{1,1.0*side},entry},"exit_fixture",
                            std::atan2(-1.0*side,2.0)};
        auto cmd=node.drop_xy_velocity(entry,{},0,false);
        require(node.drop_approach_.path.size()==2 && std::abs(node.drop_approach_.arrival_heading)<1e-9 &&
                std::abs(node.last_yaw_rate_)<1e-9 && std::hypot(cmd.x,cmd.y)>0,
                "safe shortcut kept discarded final heading or lost XY motion");
        node.px_=.2;node.py_=.03*side;
        node.drop_xy_velocity(entry,{},.02,false);
        require(std::abs(node.drop_approach_.arrival_heading)<1e-9 && std::abs(node.drop_leg_heading_)<1e-9,
                "drift moved the accepted shortcut heading");

        reset(node,{0,0});node.drop_leg_target_.reset();
        node.drop_approach_={true,entry,{{0,0},{1.5,-.4*side},entry},"margin_fixture",
                            std::atan2(.4*side,1.5)};
        const Obstacles obs{{1.5,.65*side,.2}};
        require(obstacle_escape_clear({0,0},entry,obs,node.exec_config()) &&
                !obstacle_escape_clear({0,0},entry,obs,node.drop_transit_config()),
                "shortcut fixture must distinguish planning from execution margin");
        node.drop_xy_velocity(entry,obs,.04,false);
        require(std::hypot(node.last_look_.x-entry.x,node.last_look_.y-entry.y)>.1,
                "new shortcut used relaxed execution margin to cut the obstacle corner");

        // Once accepted, an avoidance leg does not disappear just because the
        // next vertex becomes visible. The handoff occurs at the same waypoint.
        const Vec2 kept=node.last_look_;
        const double kept_heading=node.drop_leg_heading_;
        node.px_=.3;node.py_=-.08*side;
        node.drop_xy_velocity(entry,{},.05,false);
        require(std::hypot(node.last_look_.x-kept.x,node.last_look_.y-kept.y)<1e-9 &&
                std::abs(node.drop_leg_heading_-kept_heading)<1e-9,
                "new visibility abandoned the active avoidance leg before arrival");

        reset(node,{0,0});node.drop_leg_target_.reset();
        node.drop_approach_={true,{0,3},{{0,0},{0,3}},"turn_fixture",M_PI/2};
        // An abrupt new leg must respect the same yaw acceleration limit, and
        // the yaw transition may not suppress the independent XY command.
        const double prior=node.last_yaw_rate_;
        cmd=node.drop_xy_velocity({0,3},{},.06,false);
        require(std::abs(node.last_yaw_rate_-prior)<=node.gains_.max_yaw_accel*node.gains_.dt+1e-9 &&
                std::hypot(cmd.x,cmd.y)>0,
                "approach leg change jumped yaw rate or gated XY");

        // Fitted obstacle growth/drift can put the start inside the execution
        // ring. The checked outward segment must survive the final guard.
        reset(node,{3.267,side*1.12});node.yaw_=0;node.last_command_world_={};
        node.drop_leg_target_.reset();
        const Vec2 exit{3.64,side*1.26};
        const Obstacles overlap{{2.93125,side*1.71875,.339365368}};
        node.v_fwd_est_=-.03;node.v_lat_est_=side*.02;
        node.drop_approach_={true,entry,{{node.px_,node.py_},exit,entry},"escape_fixture",0};
        cmd=node.drop_xy_velocity(entry,overlap,.08,false);
        require(std::hypot(cmd.x,cmd.y)>1e-6 &&
                node.drop_escape_velocity_clear(cmd,overlap) && node.last_yaw_rate_==0,
                "overlapping fit zeroed an independently checked outward command");
        require(!node.drop_escape_velocity_clear({-.1,side*.1},overlap),
                "escape admitted inward movement toward the overlapping obstacle");
        auto blocked_escape=overlap;blocked_escape.push_back({3.90,side*1.12,.2});
        require(!node.drop_escape_velocity_clear({.6,0},blocked_escape),
                "escape from one obstacle bypassed another obstacle");
    }
    std::cout<<"mirrored exit connector/heading consistency, stable reference and checked shortcut passed\n";
}

void visual_entry_continuity_regression()
{
    for(const int side:{-1,1}) {
        ExplorationNode node;node.timer_->cancel();node.vision_reader_.reset();
        node.ggcfg_={-.5,-4.6,9,4.6,.04,.4,.1,.6};
        // Mirror the payload as well as the geometry: right package becomes left.
        const unsigned completed=side>0 ? 2 : 1;
        for(unsigned i=1;i<=completed;++i) {
            const double t=10*i;
            VisionVisitTarget visit;visit.track_id=i;visit.position={2,0};
            node.visual_drop_.begin(visit,0);node.visual_drop_.start_sample({1,0},1.2,0,t);
            node.visual_drop_.tick({1,0},1.2,0,t+1.1,0,.1);node.visual_drop_.authorize_descent();
            const auto aim=node.visual_drop_.alignment_target(0);
            node.visual_drop_.tick(aim,0,0,t+2,0,.1);node.visual_drop_.tick(aim,0,0,t+6.1,0,.1);
            require(node.visual_drop_.serial_finished(i,i,aim,t+6.2),"continuity fixture release failed");
            node.visual_drop_.tick(aim,0,0,t+9,0,.1);
            node.visual_drop_.tick(aim,1.2,0,t+9.1,0,.1);node.visual_drop_.finish_visit();
        }
        const Vec2 object{3.75,side*2.56};
        VisionVisitTarget visit;visit.track_id=10;visit.position=object;node.visual_drop_.begin(visit,0);
        // A legal near-side descent must beat a longer full-padding route.
        reset(node,{3.289294140841054,side*1.3229144165042386});node.homing_=false;
        const Obstacles near_obs{{2.93456500137,side*2.468140002744,.301101379846},
                                  {5.04910976136,side*2.783991738636,.36873248658}};
        auto entry=node.drop_entry_point(10,object,near_obs,40);
        require(entry && entry->x>node.px_ && side*entry->y<side*object.y &&
                node.drop_approach_.path.size()==2,
                "near-side legal entry lost to a farther entry with extra padding");

        // Recorded final-leg map update: a small entry change must not add a
        // waypoint or change yaw when the same descent heading is still safe.
        const Vec2 cur{4.749282791291578,side*3.5801495865815767};
        reset(node,cur);node.homing_=false;
        const Vec2 old_entry{4.296034474933682,side*3.3977627063697082};
        const double heading=side*(-160.24318669656148*M_PI/180);
        node.drop_entry_=old_entry;node.drop_entry_track_=10;node.drop_entry_object_=object;
        node.drop_obstructed_approach_=true;
        node.drop_approach_={true,old_entry,{cur,old_entry},"final_fixture",heading};
        node.drop_leg_target_=old_entry;node.drop_leg_anchor_=old_entry;node.drop_leg_track_=10;node.drop_leg_heading_=heading;
        // The left obstacle grows just enough to invalidate the old envelope.
        const Obstacles updated{{3.08280157440665,side*2.633885329598634,.290182729706742},
                                 {5.076351948781655,side*2.794817736491895,.377542695447008},
                                 {2.725,side*2.425,.18462843753216346}};
        require(!node.drop_descent_line_clear(old_entry,node.visual_drop_.alignment_target(heading),updated,node.goal_tol_),
                "recorded map update must invalidate the old arrival envelope");
        entry=node.drop_entry_point(10,object,updated,41);
        require(entry && node.drop_approach_.path.size()==2 &&
                std::hypot(entry->x-old_entry.x,entry->y-old_entry.y)>1e-4 &&
                std::abs(std::remainder(node.drop_approach_.arrival_heading-heading,2*M_PI))<1e-9,
                "small final-entry correction reintroduced a corner or reversed the yaw reference");
        require(node.drop_descent_line_clear(*entry,node.visual_drop_.alignment_target(heading),updated,node.goal_tol_),
                "preserved heading bypassed the updated descent envelope");

        node.yaw_=.7*side;
        const Vec2 near{node.px_+.001,node.py_-.001*side};
        const auto actual_landing=node.visual_drop_.alignment_target(node.yaw_);
        const auto checked_landing=node.drop_landing_from_entry(near,{node.px_,node.py_});
        require(std::hypot(actual_landing.x-checked_landing.x,actual_landing.y-checked_landing.y)<1e-9,
                "millimetre-long entry fabricated a new payload heading inside arrival tolerance");

        // Newly blocked entry, still-clear intermediate leg: retain its target
        // and heading, but replace the unsafe rest of the path.
        reset(node,{0,0});node.homing_=false;
        const Vec2 join{1,side*1.0};
        const Vec2 blocked{3,side*2.0};
        node.drop_entry_=blocked;node.drop_entry_object_=object;node.drop_entry_track_=10;
        node.drop_approach_={true,blocked,{{0,0},join,blocked},"blocked_fixture",.4*side};
        node.drop_leg_target_=join;node.drop_leg_track_=10;node.drop_leg_heading_=side*M_PI/4;
        entry=node.drop_entry_point(10,object,{{blocked.x,blocked.y,.05}},42);
        require(entry && std::string(node.drop_approach_.reason)=="continue_checked_leg" &&
                std::hypot(node.drop_approach_.path[1].x-join.x,node.drop_approach_.path[1].y-join.y)<1e-9 &&
                std::abs(node.drop_leg_heading_-side*M_PI/4)<1e-9,
                "entry invalidation discarded the still-clear active leg");
        node.drop_xy_velocity(*entry,{{blocked.x,blocked.y,.05}},42.02,false);
        require(std::hypot(node.last_look_.x-join.x,node.last_look_.y-join.y)<1e-9,
                "execution immediately skipped the leg retained by replanning");

        node.px_=join.x+.3;node.py_=join.y+side*.3;
        node.drop_entry_=blocked;node.drop_approach_.entry=blocked;
        entry=node.drop_entry_point(10,object,{{blocked.x,blocked.y,.05}},43);
        require(entry && std::string(node.drop_approach_.reason)!="continue_checked_leg",
                "replanning retained an already-passed avoidance waypoint");
    }
    std::cout<<"mirrored nearest-entry, active-leg retention and final-heading continuity passed\n";
}

void visual_delayed_yaw_regression()
{
    // Odom-derived rate has filtering delay in addition to actuator delay.
    // Supplying the instantaneous model rate would miss the original oscillation.
    for(int side:{-1,1}) for(double response:{.22,.40}) {
        ExplorationNode node;node.timer_->cancel();node.vision_reader_.reset();
        reset(node,{0,0});node.homing_=false;node.yaw_=-side*.6;
        node.gains_.smith_yaw_time=.4;
        VisionVisitTarget visit;visit.track_id=1;visit.position={5,0};node.visual_drop_.begin(visit,node.yaw_);
        node.drop_approach_={true,{4,0},{{0,0},{4,0}},"delayed_yaw",0};
        std::deque<double> delay(13,0.0);
        double rate=0,last_pose_yaw=node.yaw_,reverse=0;
        for(int tick=0;tick<700;++tick) {
            if(tick%3==0) {
                node.yaw_rate_est_=.3*(node.yaw_-last_pose_yaw)/.06+.7*node.yaw_rate_est_;
                last_pose_yaw=node.yaw_;
            }
            const auto velocity=node.drop_xy_velocity({4,0},{},tick*.02,false);
            require(std::hypot(velocity.x,velocity.y)>0,"delayed yaw alignment gated XY");
            node.remember_yaw_command(node.last_yaw_rate_);
            delay.push_back(node.last_yaw_rate_);const double applied=delay.front();delay.pop_front();
            rate+=(1-std::exp(-.02/response))*(applied-rate);
            reverse+=std::max(0.0,-side*rate*.02);
            node.yaw_+=rate*.02;
        }
        require(std::abs(node.yaw_)<.005 && reverse<.001,
                "filtered odometry and command delay made the approach yaw turn back");
        // Safety-clipped zeros must replace previous motion in the prediction.
        for(int i=0;i<25;++i) node.remember_yaw_command(0);
        require(std::abs(node.pending_yaw_rotation())<1e-9,"yaw prediction retained already-clipped commands");
    }
    std::cout<<"mirrored yaw with filtered odometry, actuator delay and final-command history passed\n";
}

void explore_corner_transition_regression()
{
    for (const int side : {-1, 1}) {
        ExplorationNode node; node.timer_->cancel(); node.vision_reader_.reset();
        node.ggcfg_={-.5,-4.6,9,4.6,.04,.4,.1,.6};
        node.yaw_=side*1.4081882827125536;
        Path2 guides{{6.872774956,2.388920592},{5.512774956,3.628920592},
                     {5.032774956,3.708920592},{4.25,3.6}};
        Obstacles obs{{2.974996509,2.558321697,.353877203},{4.675,-.35,.367618617},
                      {4.975,-1.625,.339365368},{5.064907622,2.739792310,.367428476}};
        for (auto& p:guides) p.y*=side;
        for (auto& o:obs) o.cy*=side;
        const Path2 joined=node.blend_explore_transition(guides,obs);
        require(joined.size()>guides.size() && path_clear(joined.front(),joined,obs,node.ggcfg_),
                "recorded exploration turn must have a safe continuous join");
        double prior=node.yaw_;
        for(size_t i=1;i<joined.size();++i) {
            const double heading=std::atan2(joined[i].y-joined[i-1].y,joined[i].x-joined[i-1].x);
            require(side*std::atan2(std::sin(heading-prior),std::cos(heading-prior))>=-1e-5,
                    "safe exploration join turned back before the next guide");
            prior=heading;
        }
        // Put another obstacle on the proposed curved join, while retaining a
        // clear original polyline. The tangent preference must yield to safety.
        bool blocked_fixture=false;
        for(size_t i=1;i+2<joined.size() && !blocked_fixture;++i) for(int k=0;k<32;++k) {
            const double angle=2*M_PI*k/32;
            auto blocked=obs;
            blocked.push_back({joined[i].x+.50*std::cos(angle),joined[i].y+.50*std::sin(angle),.05});
            if(!path_clear(guides.front(),guides,blocked,node.ggcfg_) ||
               path_clear(joined.front(),joined,blocked,node.ggcfg_)) continue;
            const Path2 fallback=node.blend_explore_transition(guides,blocked);
            require(path_clear(fallback.front(),fallback,blocked,node.ggcfg_),
                    "continuous-turn preference cut across an obstacle");
            blocked_fixture=true;break;
        }
        require(blocked_fixture,"could not construct obstructed smooth-turn regression");

        // 39 s after the second delivery: the outgoing tangent intersects the
        // incoming tangent only 0.286 m before the join. The old shared 0.315 m
        // handle overshot that intersection and fell back to an S-shaped join.
        node.yaw_=side*.6849192941308272;
        Path2 short_leg{{5.681492124782662,-2.0263238315914482},
                        {6.041492124782662,-1.4263238315914482},
                        {5.93528336157258,-.47443587513248775}};
        Obstacles short_obs{{4.6582848686429195,-.37502907881424824,.35385153477530523},
                            {4.974950150604145,-1.6250498493958545,.33932940456676336}};
        for(auto& p:short_leg) p.y*=side;
        for(auto& o:short_obs) o.cy*=side;
        const Path2 short_join=node.blend_explore_transition(short_leg,short_obs);
        require(short_join.size()>short_leg.size() &&
                path_clear(short_join.front(),short_join,short_obs,node.ggcfg_),
                "short post-delivery turn lost its feasible safe tangent connection");
        prior=node.yaw_;
        for(size_t i=1;i<short_join.size();++i) {
            const double heading=std::atan2(short_join[i].y-short_join[i-1].y,
                                           short_join[i].x-short_join[i-1].x);
            require(side*std::remainder(heading-prior,2*M_PI)>=-1e-5,
                    "short post-delivery connector contains a reverse bend");
            prior=heading;
        }
        node.px_=short_leg.front().x;node.py_=short_leg.front().y;
        node.explore_has_committed_=false;
        GlobalResult planned;planned.ok=true;planned.path=short_leg;
        require(node.adopt_explore_path(planned,short_leg.back()),"post-delivery path was not adopted");
        prior=node.yaw_;
        Path2 final_path;
        for(const auto& p:node.traj_) {
            require(side*std::remainder(p.theta-prior,2*M_PI)>=-1e-5,
                    "final smoothing reintroduced a reverse post-delivery turn");
            prior=p.theta;final_path.push_back(p.p);
        }
        require(path_clear(final_path.front(),final_path,short_obs,node.ggcfg_),
                "final post-delivery curve cut into the obstacle");

        // With an unobstructed two-point route, adding an S merely to match
        // the current yaw is unnecessary. The heading controller owns alignment.
        const Path2 straight{{2,0},{4,side*1.0}};
        node.yaw_=0;
        const auto unbent=node.blend_explore_transition(straight,{});
        require(unbent.size()==2,"clear straight departure was turned into an S curve");
    }
    std::cout<<"recorded/mirrored exploration tangent and blocked-join fallback passed\n";
}

// 2026-10-03 回放：容差包络已经失效时仍距旧入口0.95/0.63m，必须提前调整。
// 不能等到旧入口中心也不可用，才停下并横移第二段。
void visual_entry_envelope_regression()
{
    for(unsigned package:{2u,3u}) {
        ExplorationNode node;node.timer_->cancel();node.vision_reader_.reset();node.visual_drop_enabled_=true;
        node.ggcfg_={-.5,-3.8,9,4.6,.04,.4,.1,.6};
        const double t=node.steady_seconds();
        for(unsigned i=1;i<package;++i) {
            VisionVisitTarget target;target.track_id=i;target.position={2,0};
            node.visual_drop_.begin(target,0);node.visual_drop_.start_sample({1,0},1.2,0,t+10*i);
            node.visual_drop_.tick({1,0},1.2,0,t+10*i+1.1,0,.1);node.visual_drop_.authorize_descent();
            const Vec2 aim=node.visual_drop_.alignment_target(0);
            node.visual_drop_.tick(aim,0,0,t+10*i+2,0,.1);
            node.visual_drop_.tick(aim,0,0,t+10*i+6.1,0,.1);
            require(node.visual_drop_.serial_finished(i,i,aim,t+10*i+6.2),"entry fixture release failed");
            node.visual_drop_.tick(aim,0,0,t+10*i+7.3,0,.1);
            node.visual_drop_.tick(aim,1.2,0,t+10*i+7.4,0,.1);node.visual_drop_.finish_visit();
        }
        const Vec2 object=package==2?Vec2{5.77,-1.81}:Vec2{3.75,2.56};
        const Vec2 cur=package==2?Vec2{4.152739964,-2.448550727}:Vec2{4.904116123,3.662779303};
        const Vec2 entry=package==2?Vec2{5.098193574,-2.550726755}:Vec2{4.344157331,3.364348846};
        const Obstacles obs=package==2?Obstacles{{4.9054,-1.63775,.298446329922}}:
            Obstacles{{2.975,2.558333333333,.353885312596},{5.11600030233,2.79416648052,.358005068906}};
        reset(node,cur);node.pz_=1.2;node.has_goal_=node.has_pose_=true;
        const double heading=std::atan2(object.y-entry.y,object.x-entry.x);
        node.yaw_=heading;
        acfly_interfaces::msg::ExplorationContext context;
        context.mission=node.goal_header_;context.home_z=0;context.cruise_z=1.2;context.height_tolerance=.1;
        node.drop_context_=context;
        node.vision_queue_->enqueue({{package,object,10,true,t+30,t+30}},cur);node.activate_visual_target(t+30);
        node.drop_entry_=entry;node.drop_entry_track_=package;node.drop_entry_object_=object;
        node.drop_approach_={true,entry,{cur,entry},"recorded_entry",heading};
        node.drop_obstructed_approach_=true;node.drop_entry_time_=t+30;
        const Vec2 landing=node.visual_drop_.alignment_target(heading);
        require(node.drop_descent_line_clear(entry,landing,obs) &&
            !node.drop_descent_line_clear(entry,landing,obs,node.goal_tol_),"recorded envelope fixture no longer separates center from arrival area");
        node.visual_drop_velocity(obs,t+30.02);
        require(node.drop_entry_ && !node.global_failed_ && node.visual_drop_.phase()==VisualDrop::Phase::Approach &&
            std::hypot(node.drop_entry_->x-entry.x,node.drop_entry_->y-entry.y)>.005,
            "lost arrival clearance was retained until stopping at old entry or inserted a retry hover");
        require(std::hypot(node.drop_entry_->x-cur.x,node.drop_entry_->y-cur.y)>.3,
            "recorded adjustment was delayed until the last few centimetres");
    }
    std::cout<<"recorded second/third entries reselect before decelerating at an invalid arrival area passed\n";
}

void visual_recorded_third_drop_regression()
{
    using Phase=VisualDrop::Phase;
    ExplorationNode node;node.timer_->cancel();node.vision_reader_.reset();node.visual_drop_enabled_=true;
    node.ggcfg_={-.5,-3.8,9,4.6,.04,.4,.1,.6};
    reset(node,{5.097,3.7299});node.yaw_=3.1997;node.pz_=1.2;node.has_goal_=node.has_pose_=true;
    acfly_interfaces::msg::ExplorationContext context;
    context.mission=node.goal_header_;context.home_z=0;context.cruise_z=1.2;context.height_tolerance=.1;
    node.drop_context_=context;
    const double base=node.steady_seconds();
    // 先让状态机完成两次真实投放/回升，再验证第三包裹偏移，不能拿第一包裹代替。
    for(unsigned i=1;i<=2;++i) {
        VisionVisitTarget target;target.track_id=i;target.position={2,0};
        node.visual_drop_.begin(target,0);node.visual_drop_.start_sample({1,0},1.2,0,base+10*i);
        node.visual_drop_.tick({1,0},1.2,0,base+10*i+1.1,0,.1);node.visual_drop_.authorize_descent();
        const Vec2 aim=node.visual_drop_.alignment_target(0);
        node.visual_drop_.tick(aim,0,0,base+10*i+2,0,.1);
        node.visual_drop_.tick(aim,0,0,base+10*i+6.1,0,.1);
        require(node.visual_drop_.phase()==Phase::Send,"third-drop setup did not request release");
        node.visual_drop_.serial_finished(i,i,aim,base+10*i+6.2);
        node.visual_drop_.tick(aim,0,0,base+10*i+7.3,0,.1);
        node.visual_drop_.tick(aim,1.2,0,base+10*i+7.4,0,.1);
        node.visual_drop_.finish_visit();
    }
    require(node.visual_drop_.completed()==2,"third-drop fixture has wrong package index");
    node.vision_queue_->enqueue({{3,{3.75,2.56},10,true,base+29,base+30}},{node.px_,node.py_});
    node.activate_visual_target(base+30);
    const Obstacles obs{{2.975,2.55833333333,.353885312596},{5.11600252960,2.79416524759,.358003347095}};
    node.visual_drop_velocity(obs,base+30);
    require(node.drop_entry_ && node.visual_drop_.phase()==Phase::Approach && node.drop_obstructed_approach_,
            "recorded third target remained stuck on the outer planning margin");
    require(node.drop_entry_->x<node.px_ && node.drop_entry_->y>node.visual_drop_.object().y,
            "recorded third target did not approach from the clear upper side");
    node.px_=node.drop_entry_->x;node.py_=node.drop_entry_->y;
    node.visual_drop_velocity(obs,base+31);
    require(node.visual_drop_.phase()==Phase::Sample,"recorded third target never entered sampling");
    node.visual_drop_velocity(obs,base+32.01);
    require(node.visual_drop_.phase()==Phase::Descend,"recorded third target failed final descent validation");
    std::cout<<"recorded third package, upper-side entry and terminal planning-ring regression passed\n";
}

// 有限响应/时延 + 向障碍方向的持续扰动。仅是压力测试，不声称是实际涡流模型。
void visual_obstacle_disturbance_regression()
{
    using Phase=VisualDrop::Phase;
    for(double push:{.03,.06}) {
        ExplorationNode node;node.timer_->cancel();node.vision_reader_.reset();node.visual_drop_enabled_=true;
        reset(node,{1,0});node.has_goal_=node.has_pose_=true;node.homing_=false;node.pz_=1.4;node.goal_={7,4};
        acfly_interfaces::msg::ExplorationContext context;
        context.mission=node.goal_header_;context.home_z=.2;context.cruise_z=1.4;context.height_tolerance=.1;
        node.drop_context_=context;
        const double base=node.steady_seconds();
        node.vision_queue_->enqueue({{1,{4.5,0},10,true,base-1,base}},{1,0});node.activate_visual_target(base);
        const Obstacles obstacles{{2.7,0,.3},{4,1.5,.25}};
        std::deque<Vec2> delay(13,Vec2{});Vec2 velocity{};
        double yaw_rate=0,zspeed=0,min_clear=1000;bool sent=false;
        for(int tick=0;tick<6000;++tick) {
            const double t=base+tick*.02;
            const auto cmd=node.visual_drop_velocity(obstacles,t);
            require(std::isfinite(cmd.x)&&std::isfinite(cmd.y),"disturbance produced invalid command");
            if(node.visual_drop_.phase()==Phase::Send) {sent=true;break;}
            const double c=std::cos(node.yaw_),s=std::sin(node.yaw_);
            const Vec2 world{c*cmd.x-s*cmd.y,s*cmd.x+c*cmd.y};
            delay.push_back(world);const Vec2 applied=delay.front();delay.pop_front();
            Vec2 disturbance{};
            for(const auto& o:obstacles) {
                const double d=std::hypot(o.cx-node.px_,o.cy-node.py_);
                const double weight=std::clamp((2-d)/1.3,0.0,1.0);
                disturbance.x+=push*weight*(o.cx-node.px_)/d;
                disturbance.y+=push*weight*(o.cy-node.py_)/d;
                min_clear=std::min(min_clear,d-o.r-node.robot_radius_);
            }
            velocity.x+=(applied.x+disturbance.x-velocity.x)*(1-std::exp(-.02/.65));
            velocity.y+=(applied.y+disturbance.y-velocity.y)*(1-std::exp(-.02/.65));
            const double vz=std::clamp(.6*(node.drop_command_altitude_-node.pz_)-.2*zspeed,
                -node.drop_command_speed_,node.drop_command_speed_);
            zspeed+=(vz-zspeed)*.02/.2;yaw_rate+=(node.last_yaw_rate_-yaw_rate)*.02/.2;
            node.px_+=velocity.x*.02;node.py_+=velocity.y*.02;node.pz_+=zspeed*.02;node.yaw_+=yaw_rate*.02;
            node.v_fwd_est_=std::cos(node.yaw_)*velocity.x+std::sin(node.yaw_)*velocity.y;
            node.v_lat_est_=-std::sin(node.yaw_)*velocity.x+std::cos(node.yaw_)*velocity.y;
            node.yaw_rate_est_=yaw_rate;node.last_command_world_=world;
        }
        if(!sent) std::cerr<<"stalled phase="<<VisualDrop::name(node.visual_drop_.phase())
            <<" position="<<node.px_<<','<<node.py_<<" entry="
            <<(node.drop_entry_?node.drop_entry_->x:-99)<<','<<(node.drop_entry_?node.drop_entry_->y:-99)
            <<" blocked="<<node.global_block_reason_<<" clearance="<<min_clear<<'\n';
        require(sent,"obstacle-directed disturbance stranded visual approach/descent");
        require(min_clear>=0,"disturbance caused body/obstacle intersection");
        std::cout<<"obstacle-directed disturbance "<<push<<" m/s: minimum body clearance="<<min_clear<<" m\n";
    }
}

void visual_delivery_integration()
{
    ExplorationNode node;
    node.timer_->cancel();

            using Phase = VisualDrop::Phase;
            node.visual_drop_enabled_ = true;
            // These deterministic timestamps use a one-second fixture, separate
            // from the operator's current sampling parameter (now two seconds).
            node.visual_drop_ = VisualDrop();
            node.vision_reader_.reset();
            node.exploration_enabled_ = true;
            node.obs_map_ = std::make_unique<ObstacleMap>(node.ocfg_);
            node.poi_mode_ = ExplorationNode::PoiMode::EXPLORE;
            auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
            goal->header.frame_id = "camera_init"; goal->header.stamp = node.now();
            goal->point.x = 7; goal->point.y = 4;
            node.on_goal(goal);
            reset(node,{2,0}); node.has_pose_ = true;
            const double base = node.steady_seconds();
            std::vector<VisionCandidate> candidates;
            for(unsigned i=1;i<=4;++i) candidates.push_back({i,{2.0+double(i),0},10,true,base-1,base});
            node.vision_queue_->enqueue(candidates,{2,0}); node.activate_visual_target(base);
            node.visual_drop_velocity({},base);
            require(node.visual_drop_.phase()==Phase::Approach && node.vision_last_event_=="drop_waiting_for_takeoff_context",
                    "missing takeoff context began descent");
            auto context = std::make_shared<acfly_interfaces::msg::ExplorationContext>();
            context->mission=goal->header; context->home_z=.2;context->cruise_z=1.4;context->height_tolerance=.1;
            std::shared_ptr<void> context_message=context;
            node.context_sub_->handle_message(context_message,rclcpp::MessageInfo{});
            node.pz_=1.4;
            for(unsigned i=1;i<=3;++i) {
                const double t=base+20*i;
                const Vec2 object{2.0+double(i),0};
                node.px_=object.x-1;node.py_=0;node.pz_=1.4;node.yaw_=0;
                node.last_command_world_={};node.v_fwd_est_=node.v_lat_est_=0;
                node.activate_visual_target(t);
                require(node.vision_queue_->active()->track_id==i,"queue switched visit order");
                node.visual_drop_velocity({},t);
                require(node.visual_drop_.phase()==Phase::Sample,"node missed 1m sample hold");
                node.px_+=.1;
                auto correction=node.visual_drop_velocity({},t+.2);
                require(correction.x<0,"sample hold let XY drift with zero correction");
                node.visual_drop_velocity({},t+1);
                require(node.visual_drop_.phase()==Phase::Descend && node.visual_drop_.used_coarse(),
                        "node stalled when no precise observations arrived");
                const auto aim=node.visual_drop_.lidar_target(0);
                require(node.drop_command_altitude_==.7 && node.drop_command_speed_>.1,
                        "node lost takeoff-relative drop height or kept level speed cap");
                node.px_=aim.x;node.py_=aim.y;node.last_command_world_={};
                node.visual_drop_velocity({},t+2); // Z still above target, XY alone starts deadline.
                node.px_+=.2;
                node.visual_drop_velocity({},t+6);
                require(node.visual_drop_.phase()==Phase::Send && node.visual_drop_.release_index()==i,
                        "node restarted deadline after XY drift or gated it on Z");
                auto wrong=std::make_shared<acfly_interfaces::msg::DropStatus>();
                wrong->mission=goal->header;wrong->release_index=i;wrong->target_id=99;
                std::shared_ptr<void> status_message=wrong;
                node.drop_status_sub_->handle_message(status_message,rclcpp::MessageInfo{});
                require(!node.drop_status_,"another target's acknowledgement accepted");
                wrong->target_id=i;wrong->success=i!=2; // Send failure must also wait, climb and count once.
                node.drop_status_sub_->handle_message(status_message,rclcpp::MessageInfo{});
                node.pz_=.7;
                node.visual_drop_velocity({},t+6.02);
                require(node.visual_drop_.phase()==Phase::Wait && node.visual_drop_.completed()==i &&
                        node.drop_command_altitude_==.7 && node.drop_command_speed_==.1 &&
                        node.visual_drop_.release_index()==0,
                        "serial outcome did not hold the drop altitude for the release wait");
                node.drop_status_sub_->handle_message(status_message,rclcpp::MessageInfo{});
                require(!node.drop_status_,"duplicate acknowledgement relatched wait XY");
                node.last_command_world_={};node.px_+=.1;node.py_+=.05;
                correction=node.visual_drop_velocity({},t+6.04);
                require(correction.x<0&&correction.y<0,"release wait did not correct both axes to fixed anchor");
                node.visual_drop_velocity({},t+8.01);
                require(node.visual_drop_.phase()==Phase::Wait && node.drop_command_altitude_==.7,
                        "climb began before the configured two-second release wait");
                node.visual_drop_velocity({},t+8.03);
                require(node.visual_drop_.phase()==Phase::Climb &&
                        node.drop_command_altitude_==1.4 && node.drop_command_speed_==.6,
                        "release wait did not hand over to climb");
                node.last_command_world_={};node.px_+=.1;node.py_+=.05;
                correction=node.visual_drop_velocity({},t+8.05);
                require(correction.x<0&&correction.y<0,"climb did not correct both axes to fixed anchor");
                if(i==2) node.coverage_closed_=true; // Coverage finished, third already queued.
                if(i==3) {
                    auto late_poi=std::make_shared<geometry_msgs::msg::PointStamped>();
                    late_poi->header.frame_id="camera_init";
                    late_poi->point.x=6;late_poi->point.y=1;
                    node.on_poi(late_poi); // Arrives after the third release, during climb.
                }
                node.pz_=1.31;node.visual_drop_velocity({},t+9);
                require(node.vision_queue_->visited().size()==i && node.visual_drop_.phase()==Phase::Idle,
                        "coarse delivery/failure never completed visit after climb");
            }
            require(node.vision_queue_->pending().empty(),"third release kept unfinished targets");
            node.pose_freshness_=SensorFreshness{};node.pose_freshness_.observe(1,node.steady_seconds());
            node.on_timer();
            require(node.homing_&&!node.vision_queue_->active(),"three deliveries did not bypass unfinished exploration");
            require(published_controls>0 && last_control.mission==goal->header && last_control.altitude==1.4 &&
                    last_control.release_index==0,"unified output lost restored altitude/mission or replayed release");
            const auto publications=published_controls;
            node.pose_freshness_=SensorFreshness{};node.on_timer();
            require(published_controls==publications,"stale odometry refreshed a descent/release command");
            // A blocked target must retain identity and altitude, not fly through an obstacle.
            node.on_goal(std::make_shared<geometry_msgs::msg::PointStamped>());
            node.drop_context_->mission=node.goal_header_;
            reset(node,{2,0});node.pz_=1.4;node.has_pose_=true;node.has_goal_=true;node.homing_=false;
            node.vision_queue_->enqueue({{8,{3,0},10,true,base-1,base}},{2,0});node.activate_visual_target(base);
            node.visual_drop_velocity({},base);node.visual_drop_velocity({},base+1);
            node.visual_drop_velocity({{2.77,0,.3}},base+1.02);
            require(!node.global_failed_ && node.vision_queue_->active()->track_id==8 && node.visual_drop_.phase()==Phase::Descend,
                    "committed straight descent was replaced by generic avoidance or another target");
            // Coverage may finish after only two drops, but an existing third
            // target must retain ownership; without that target, go to the end.
            node.visual_drop_=VisualDrop();node.vision_queue_->reset();node.coverage_closed_=true;
            node.homing_=false;node.has_goal_=true;node.has_pose_=true;
            node.pose_freshness_=SensorFreshness{};node.pose_freshness_.observe(1,node.steady_seconds());
            node.vision_queue_->enqueue({{9,{3,0},10,true,base-1,base}},{2,0});
            node.on_timer();
            require(!node.homing_ && node.vision_queue_->active() && node.vision_queue_->active()->track_id==9,
                    "coverage completion skipped already queued target");
            node.visual_drop_=VisualDrop();node.vision_queue_->reset();node.coverage_closed_=true;
            node.on_timer();require(node.homing_,"empty visual queue blocked coverage-complete home handoff");
            // Closed loop with finite response lag and continuous XY disturbance.
            // Exercise actual output directions rather than teleporting between states.
            goal->header.stamp.sec += 1;node.on_goal(goal);
            context->mission=goal->header;node.drop_context_=*context;
            reset(node,{1.8,0});node.pz_=1.4;node.has_pose_=node.has_goal_=true;
            node.homing_=false;node.poi_mode_=ExplorationNode::PoiMode::EXPLORE;
            node.vision_queue_->enqueue({{1,{3,0},10,true,base-1,base},
                {2,{4,1},10,true,base-.9,base},{3,{5,0},10,true,base-.8,base}},{1.8,0});
            node.activate_visual_target(base);
            double vx=0,vy=0,vz=0,wz=0;
            unsigned sent=0; bool finished=false;
            for(int tick=0;tick<9000;++tick) {
                const double t=base+tick*.02;
                if(!node.vision_queue_->active()) node.activate_visual_target(t);
                if(!node.vision_queue_->active()) {
                    finished=node.visual_drop_.completed()==3;break;
                }
                const auto v=node.visual_drop_velocity({},t);
                require(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(node.last_yaw_rate_),"closed loop produced NaN");
                if(node.visual_drop_.phase()==Phase::Send) {
                    ++sent;auto ack=std::make_shared<acfly_interfaces::msg::DropStatus>();
                    ack->mission=goal->header;ack->release_index=sent;
                    ack->target_id=node.visual_drop_.target_id();ack->success=true;
                    std::shared_ptr<void> ack_msg=ack;node.drop_status_sub_->handle_message(ack_msg,rclcpp::MessageInfo{});
                }
                const double c=std::cos(node.yaw_),ss=std::sin(node.yaw_);
                const Vec2 world{c*v.x-ss*v.y,ss*v.x+c*v.y};
                const double zcmd=std::clamp(.6*(node.drop_command_altitude_-node.pz_)-.2*vz,
                    -node.drop_command_speed_,node.drop_command_speed_);
                vx+=(world.x+.018-vx)*.02/.25;vy+=(world.y-.013-vy)*.02/.25;
                vz+=(zcmd-vz)*.02/.20;wz+=(node.last_yaw_rate_-wz)*.02/.20;
                node.px_+=vx*.02;node.py_+=vy*.02;node.pz_+=vz*.02;node.yaw_+=wz*.02;
                node.v_fwd_est_=std::cos(node.yaw_)*vx+std::sin(node.yaw_)*vy;
                node.v_lat_est_=-std::sin(node.yaw_)*vx+std::cos(node.yaw_)*vy;
                node.yaw_rate_est_=wz;node.last_command_world_=world;
            }
            require(finished&&sent==3&&node.vision_queue_->visited().size()==3,
                    "closed-loop drift/response lag stalled, orbited, or repeated a release");
            // Read the real SHM wire format into the active 1-second window.
            goal->header.stamp.sec += 1;node.on_goal(goal);context->mission=goal->header;node.drop_context_=*context;
            reset(node,{2,0});node.pz_=1.4;node.has_pose_=node.has_goal_=true;node.homing_=false;
            const double sample_start=node.steady_seconds();
            node.vision_queue_->enqueue({{1,{3,0},10,true,sample_start-1,sample_start}},{2,0});
            node.activate_visual_target(sample_start);node.visual_drop_velocity({},sample_start);
            const std::string sample_path="/tmp/acfly-drop-window-"+std::to_string(getpid())+".shm";
            {
                vision_shm::Writer writer(sample_path);
                node.vision_reader_=std::make_unique<VisionShmReader>(sample_path,.5,.02);
                writer.publish(vision_shm::monotonic_now(),{{1,3.1,0}});node.poll_vision_shm();
                writer.publish(vision_shm::monotonic_now(),{});node.poll_vision_shm();
                writer.publish(vision_shm::monotonic_now(),{{2,2.9,0}});node.poll_vision_shm();
                require(node.visual_drop_.samples()==2,"SHM wiring dropped partial samples across empty frame");
                node.visual_drop_velocity({},sample_start+1.001);
                require(node.vision_queue_->active()->refined && node.vision_queue_->active()->class_id==0 &&
                        std::abs(node.vision_queue_->active()->position.x-node.visual_drop_.object().x)<1e-12 &&
                        node.visual_drop_.object().x>2.9 && node.visual_drop_.object().x<3,
                        "SHM weighted coordinates failed to update queue when class votes tied");
                node.vision_reader_.reset();
            }
            std::filesystem::remove(sample_path);
            std::cout << "delivery node: context, three visits, fallback, offsets, holds, deadline, serial status and home handoff passed\n";
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::InitOptions options;
    options.set_domain_id(203);
    rclcpp::init(argc, argv, options);
    int result = 0;
    try {
        if (std::getenv("ACFLY_CORNER_TRANSITION_ONLY")) {
            explore_corner_transition_regression();
            ExplorationNode node;
            node.timer_->cancel(); node.vision_reader_.reset();
            node.ggcfg_ = {0,-5,7.5,5,.04,.4,.1,.6};
            continuous_route_and_missed_corner(node);
            current_pose_validation_and_new_reference_state(node);
            rclcpp::shutdown(); return 0;
        }
        if (std::getenv("ACFLY_VISUAL_DROP_ONLY")) {
            visual_exit_shortcut_regression();
            visual_entry_continuity_regression();
            visual_delayed_yaw_regression();
            visual_safe_entry_regressions();
            visual_normal_and_obstructed_entry_regressions();
            visual_entry_commitment_regression();
            visual_entry_envelope_regression();
            visual_recorded_third_drop_regression();
            visual_obstacle_disturbance_regression();
            visual_delivery_integration();
            visual_sampling_brake_regression();
            visual_drop_entry_point_avoids_blocked_object();
            rclcpp::shutdown(); return 0;
        }
        home_visibility_shortcut_regression();
        home_recorded_return_corner_regression();
        home_virtual_boundary_drift_regression();
        home_straight_descent_regression();
        if (std::getenv("ACFLY_HOME_APPROACH_ONLY")) { rclcpp::shutdown(); return 0; }
        malformed_cloud_does_not_terminate_node();
        recorded_xy_handoffs();
        cruise_drift_compensation_is_checked();
        cruise_with_continuous_drift();
        path_failure_keeps_safe_braking();
        invalid_odom_stamp_preserves_pose();
        std::string goal_errors;
        for (auto test : {new_goal_resets_execution, duplicate_goal_preserves_progress,
                          invalid_goal_preserves_current_mission}) {
            try { test(); }
            catch (const std::exception& error) { goal_errors += std::string(error.what()) + "\n"; }
        }
        require(goal_errors.empty(), goal_errors);
        missing_visual_frames_do_not_hold_forever();
        blocked_visual_target_keeps_fifo_order();
        motion_pose_dropout_liveness();
        retreat_checks_the_other_obstacle();
        completion_publish_liveness();
        retreat_timeout_liveness();
        visual_turn_final_output();
        exhausted_exploration_route();
        observation_turn_with_drift();
        required_drift_regressions();
        corridor_registered_cloud_regression();
        exploration_switch_regression();
        boundary_home_endpoint_regression();
        small_field_stability_regression();
        visual_straight_approach_regression();
        visual_hover_restores_exploration_display();
        ExplorationNode node;
        {
            char directory[] = "/tmp/acfly-vision-node-XXXXXX";
            require(::mkdtemp(directory) != nullptr, "create isolated vision fixture");
            struct RemoveFixture {
                std::string path;
                ~RemoveFixture() { std::filesystem::remove_all(path); }
            } cleanup{directory};
            const std::string path = std::string(directory) + "/targets.bin";
            auto original_reader = std::move(node.vision_reader_);
            node.vision_reader_ = std::make_unique<VisionShmReader>(path, .5, .02);
            vision_shm::Writer writer(path);
            writer.publish(vision_shm::monotonic_now(), {{1, 2.5, -1}, {5, 4, 2}});
            const auto searches = node.astar_searches_;
            node.on_timer();  // Receives SHM even before odometry is available.
            require(node.vision_new_frame_ && node.vision_last_frame_.count == 2 &&
                    node.vision_last_frame_.targets[1].id == 5 &&
                    node.vision_last_frame_.targets[0].x == 2.5,
                    "exploration timer did not consume the supplied SHM writer");
            node.on_timer();
            require(!node.vision_new_frame_ && node.vision_reader_->statistics().frames == 1,
                    "exploration timer counted the same image twice");
            writer.publish(vision_shm::monotonic_now(), {});
            node.on_timer();
            require(node.vision_new_frame_ && node.vision_last_frame_.count == 0 &&
                    node.vision_reader_->statistics().empty_frames == 1,
                    "exploration timer did not deliver a real empty image");
            require(node.poi_queue_.empty() && !node.tracker_->has_trajectory() &&
                    !node.has_goal_ && node.astar_searches_ == searches,
                    "stage-one SHM reception unexpectedly changed flight decisions");
            require(node.vision_candidates_->size() == 0,
                    "preflight frames were counted as mission observations");

            node.has_goal_ = node.has_pose_ = true;
            node.goal_ = {7, 4.25};
            node.px_ = 1; node.py_ = 0;
            const auto original_goal = node.goal_;
            node.pose_freshness_.observe(1, node.steady_seconds());
            for (int i = 1; i <= 10; ++i) {
                writer.publish(vision_shm::monotonic_now(), {{1 + i % 6, 2.5, -1}, {1 + (i + 1) % 6, 4, 2}});
                node.poll_vision_shm();
                require(node.vision_collection_active_ && node.vision_candidates_->size() == 2,
                        "active exploration did not collect spatial candidates from SHM");
                require(node.vision_new_confirmations_.size() == static_cast<size_t>(i == 10 ? 2 : 0),
                        "SHM-to-node integration did not confirm exactly on the tenth new frame");
                node.poll_vision_shm();
                require(node.vision_new_confirmations_.empty(), "same SHM frame emitted confirmation twice");
            }
            require(node.vision_candidates_->statistics().confirmations == 2 &&
                    node.poi_queue_.empty() && !node.tracker_->has_trajectory() &&
                    node.goal_.x == original_goal.x && node.goal_.y == original_goal.y &&
                    node.astar_searches_ == searches,
                    "stage-two confirmation inserted a flight target or triggered a path search");
            require(node.vision_queue_->active() && node.vision_queue_->pending().size() == 1 &&
                    node.vision_queue_->active()->position.x == 2.5 &&
                    node.vision_queue_->active()->class_id == 0 && node.vision_near_->count() == 0,
                    "confirmed batch failed queue admission or reused far IDs in near vote");
            const auto active_track = node.vision_queue_->active()->track_id;
            node.pose_freshness_.observe(2, node.steady_seconds());
            for (int i = 1; i <= 10; ++i) {
                writer.publish(vision_shm::monotonic_now(),
                    {{i <= 6 ? 4 : 2, 2.65, -.95}, {6, 4, 2}, {3, 1, -2}});
                node.poll_vision_shm();
                require(node.vision_queue_->active()->track_id == active_track &&
                        node.vision_near_->count() == i,
                        "queued/detected neighbour stole the active near confirmation");
                require(node.vision_queue_->active()->refined == (i == 10),
                        "near confirmation happened before/after ten fresh frames");
            }
            require(node.vision_queue_->active()->class_id == 4 &&
                    std::abs(node.vision_queue_->active()->position.x - 2.65) < 1e-9 &&
                    node.vision_queue_->pending().size() == 2 && node.vision_queue_->visited().empty() &&
                    node.vision_near_->speed_cap(.6) == .4,
                    "SHM-to-queue near result/limit was incorrect or prematurely marked completed");
            require(node.poi_queue_.empty() && !node.tracker_->has_trajectory() &&
                    node.goal_.x == original_goal.x && node.goal_.y == original_goal.y &&
                    node.astar_searches_ == searches,
                    "data-only queue/refinement changed navigation commands");

            // Simulate the future executor's completion callback. There is no
            // automatic visit-completion call in the live node at this stage.
            require(node.vision_queue_->complete_active(active_track, node.steady_seconds()),
                    "completion callback fixture failed");
            writer.publish(vision_shm::monotonic_now(), {{1, 2.65, -.95}, {3, 1, -2}});
            node.poll_vision_shm();
            require(node.vision_queue_->visited().size() == 1 && node.vision_queue_->active() &&
                    node.vision_queue_->active()->position.x == 4 &&
                    node.vision_queue_->statistics().blacklisted_detections == 1 &&
                    node.vision_candidates_->size() == 2 && node.vision_near_->count() == 0,
                    "completed target re-entered candidates or next target lost FIFO/clean near state");
            auto goal = std::make_shared<geometry_msgs::msg::PointStamped>();
            goal->point.x = 7; goal->point.y = 4.25;
            node.on_goal(goal);
            require(node.vision_candidates_->size() == 0, "new mission retained old visual confirmations");
            require(!node.vision_queue_->active() && node.vision_queue_->pending().empty() &&
                    node.vision_queue_->visited().empty() && node.vision_near_->track_id() == 0,
                    "new mission retained its queue, blacklist or near vote");
            // A stale pose must prevent a new frame from entering the weighted accumulator.
            node.pose_freshness_ = SensorFreshness{};
            writer.publish(vision_shm::monotonic_now(), {{2, 2.5, -1}});
            node.poll_vision_shm();
            require(!node.vision_collection_active_ && node.vision_candidates_->size() == 0,
                    "unavailable odometry entered the visual coordinate average");
            node.pose_freshness_.observe(1, node.steady_seconds());
            node.homing_ = true;
            writer.publish(vision_shm::monotonic_now(), {{2, 2.5, -1}});
            node.poll_vision_shm();
            require(!node.vision_collection_active_ && node.vision_candidates_->size() == 0,
                    "homing accepted new visual candidates");
            node.homing_ = node.has_goal_ = node.has_pose_ = false;
            node.plan_pending_ = false;
            node.vision_collection_active_ = false;
            node.vision_reader_ = std::move(original_reader);
            node.vision_new_frame_.reset(); node.vision_last_frame_ = {};
            std::cout << "exploration SHM far/near confirmation, FIFO/blacklist and no-flight boundary passed\n";
        }
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
        require(node.global_braking_blocked_ && node.measured_projection_blocked_ && stopped.x < 0 &&
                std::hypot(stopped.x, stopped.y) <= .20 + 1e-9 && node.required_velocity_clear(stopped, {{4, 0, .2}}),
                "unsafe measured stopping projection did not apply a checked counter-velocity");
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
        visual_delivery_integration();
        visual_sampling_brake_regression();
        visual_sampling_brake_preserves_correction();
        visual_drop_entry_point_avoids_blocked_object();
        executing_route_tolerates_turn_deviation();
        std::cout << "required node: closed-loop and safety regressions passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
