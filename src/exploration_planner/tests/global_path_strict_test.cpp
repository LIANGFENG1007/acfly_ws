#include "exploration_planner/global_planner.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {
void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

GlobalConfig config()
{
    GlobalConfig cfg{-5, -5, 5, 5, .04, .30, .30, .60};
    cfg.validate_complete_path = true;
    return cfg;
}

void validate(const GlobalResult& result, const Vec2& start,
              const Obstacles& obstacles, const GlobalConfig& cfg)
{
    require(result.ok, "expected valid strict path");
    require(result.path.size() >= 2, "strict path includes complete segment");
    require(path_clear(start, result.path, obstacles, cfg), "strict path obeys obstacle contract");
    require(path_inside_safe_field(result.path, cfg), "strict path obeys field contract");
}

void blocked_goal_proxy()
{
    const auto cfg = config();
    const Obstacles obstacles{{0, 0, .2}};
    const Vec2 start{-2, 0}, goal{0, 0};
    const auto result = plan_global_path(start, goal, obstacles, cfg);
    validate(result, start, obstacles, cfg);
    require(result.goal_blocked, "occupied exact goal is reported");
    const Vec2 endpoint = result.path.back();
    const double radius = std::hypot(endpoint.x, endpoint.y);
    require(radius >= .8 && radius < .86, "occupied goal stops at nearest safe proxy");
}

void exact_points_and_close_goals()
{
    auto cfg = config();
    cfg.min_x = -5.02;
    const Obstacles obstacles{{0, 0, .2}};
    // Exact goal is inside the margin although its grid-cell center is outside.
    const Vec2 start{1.2, 0}, goal{.799, 0};
    const double cell_x = cfg.min_x + (std::floor((goal.x - cfg.min_x) / cfg.cell) + .5) * cfg.cell;
    const double cell_y = cfg.min_y + (std::floor((goal.y - cfg.min_y) / cfg.cell) + .5) * cfg.cell;
    require(std::hypot(cell_x, cell_y) >= .8, "regression requires a safe goal-cell center");
    const auto blocked = plan_global_path(start, goal, obstacles, cfg);
    validate(blocked, start, obstacles, cfg);
    require(blocked.goal_blocked, "exact goal must be checked independently of cell center");
    const auto close_proxy = plan_global_path(blocked.path.back(), goal, obstacles, cfg);
    validate(close_proxy, blocked.path.back(), obstacles, cfg);
    require(close_proxy.path.size() == 2, "already at blocked-goal proxy still returns complete path");
    const auto same_cell = plan_global_path({1.001, 1.001}, {1.002, 1.002}, {}, cfg);
    validate(same_cell, {1.001, 1.001}, {}, cfg);
    require(std::abs(same_cell.path.front().x - 1.001) < 1e-12, "same-cell path preserves exact start");
    require(std::abs(same_cell.path.back().x - 1.002) < 1e-12, "same-cell path preserves exact goal");
    validate(plan_global_path({1, 1}, {1, 1}, {}, cfg), {1, 1}, {}, cfg);
}

void body_clearance_and_escape()
{
    const auto cfg = config();
    const Obstacles obstacles{{0, 0, .2}};
    require(!plan_global_path({.49, 0}, {2, 0}, obstacles, cfg).ok,
        "physical aircraft overlap cannot be relaxed");
    const Vec2 start{.78, 0};
    const auto escape = plan_global_path(start, {2, 0}, obstacles, cfg);
    validate(escape, start, obstacles, cfg);
    require(escape.path.size() == 2, "safe escape retains no tiny stub");
    // A goal behind the obstacle must route outward before going around it.
    const auto around = plan_global_path(start, {-2, 0}, obstacles, cfg);
    validate(around, start, obstacles, cfg);
    require(around.path[1].x >= start.x, "escape cannot initially approach the obstacle");
}

void analytic_edges_and_normal_detour()
{
    auto cfg = config();
    const Obstacles obstacle{{0, 0, .2}};
    validate(plan_global_path({-2, 0}, {2, 0}, obstacle, cfg), {-2, 0}, obstacle, cfg);
    // This obstacle lies between the exact start and the neighboring grid center.
    cfg.cell = .2;
    cfg.robot_radius = .02;
    cfg.inflate = .01;
    const Vec2 start{.19, .10};
    const Obstacles tiny{{.245, .1, .01}};
    const auto result = plan_global_path(start, {.7, .1}, tiny, cfg);
    validate(result, start, tiny, cfg);
    require(result.path.size() > 2, "unsafe exact initial edge must be routed around");
}

void field_entry_and_default_goal()
{
    auto cfg = config();
    cfg.min_x = 0;
    cfg.max_x = 7.5;
    const Vec2 start{0, 0}, goal{3, 0};
    validate(plan_global_path(start, goal, {}, cfg), start, {}, cfg);
    const auto strict_goal = plan_global_path({6, 4.25}, {7, 4.25}, {}, cfg);
    validate(strict_goal, {6, 4.25}, {}, cfg);
    require(strict_goal.goal_blocked && strict_goal.path.back().x <= 6.9,
        "strict wall goal stays at safe proxy");
    cfg.validate_complete_path = false;
    const auto original = plan_global_path({6, 4.25}, {7, 4.25}, {}, cfg);
    require(original.ok && std::abs(original.path.back().x - 7) < 1e-12,
        "default required goal remains exact");
}

void heading_cone()
{
    auto cfg = config();
    cfg.head_cone_half = 22.0 * M_PI / 180.0;
    cfg.head_cone_radius = .8;
    const Vec2 start{.78, 0};
    const Obstacles obstacles{{0, 0, .2}};
    validate(plan_global_path(start, {2, 0}, obstacles, cfg, 0.0), start, obstacles, cfg);
    require(!plan_global_path(start, {2, 0}, obstacles, cfg, M_PI).ok,
        "unsafe forward cone must not silently bypass its direction constraint");
}

void extended_field_entry()
{
    auto cfg = config();
    cfg.min_x = 0;
    cfg.max_x = 7.5;
    const Vec2 start{0, 0}, goal{4, 0};
    // Seed28's entrance requires moving along the initial virtual inset before
    // entering. The old .6m relaxation box truncated this otherwise safe route.
    const Obstacles obstacles{{1, -.836, .40}, {1.257, .709, .40}, {.655, 3.329, .40}};
    const auto result = plan_global_path(start, goal, obstacles, cfg);
    validate(result, start, obstacles, cfg);
    require(result.path.size() >= 3, "entry detour must not cut between inflated obstacles");
    require(!path_clear(start, {start, goal}, obstacles, cfg), "fixture must need a detour");
    const Vec2 exact_start{.001, 0};
    const Obstacles touching_margin{{1, 0, .4}};
    const auto tangent = plan_global_path(exact_start, goal, touching_margin, cfg);
    validate(tangent, exact_start, touching_margin, cfg);
    require(std::abs(tangent.path[1].x - exact_start.x) < 1e-9,
            "exact-pose grid must allow tangent departure without an inward half-cell jump");
}
}  // namespace

int main()
{
    try {
        blocked_goal_proxy();
        exact_points_and_close_goals();
        body_clearance_and_escape();
        analytic_edges_and_normal_detour();
        field_entry_and_default_goal();
        heading_cone();
        extended_field_entry();
        std::cout << "global_path_strict_test: all 7 scenarios passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "global_path_strict_test: " << error.what() << '\n';
        return 1;
    }
}
