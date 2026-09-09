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
    return GlobalConfig{0, -5, 7.5, 5, 0.04, 0.30, 0.30, 0.60};
}

double distance(const Vec2& a, const Vec2& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

void validate(const GlobalResult& result, const Vec2& start, const Vec2& goal,
              const Obstacles& obstacles, const GlobalConfig& cfg)
{
    require(result.ok && !result.goal_blocked, "required route must reach its exact free goal");
    require(result.path.size() >= 2, "required route must include exact start and goal");
    require(distance(result.path.front(), start) < 1e-9, "exact start was lost");
    require(distance(result.path.back(), goal) < 1e-9, "exact required goal was lost");
    require(required_path_clear(start, result.path, goal, obstacles, cfg), "required route failed revalidation");
    for (size_t i = 1; i < result.path.size(); ++i)
        require(obstacle_segment_clear(result.path[i - 1], result.path[i], obstacles, cfg),
                "an exact output edge violates the obstacle contract");
}

void legal_red_and_blocked_seed22()
{
    const auto cfg = config();
    const Vec2 start{4.0, 1.0}, goal{7.0, 4.25};
    const auto route = plan_required_path(start, goal, {}, cfg);
    validate(route, start, goal, {}, cfg);
    require(route.path.size() >= 3, "required connector anchor must survive string pulling");
    const Vec2 anchor = route.path[route.path.size() - 2];
    require(anchor.x <= 6.9 && distance(anchor, goal) <= cfg.required_connector_length + 1e-9,
            "last connector exceeds its local field exception");
    require(path_inside_safe_field(Path2(route.path.begin(), route.path.end() - 1), cfg),
            "normal prefix relaxed the global wall inset");
    require(!path_inside_safe_field(route.path, cfg), "red regression must cross the virtual inset");
    const Obstacles actual_seed22{{6.990, 4.097, 0.20}};
    const auto blocked = plan_required_path(start, goal, actual_seed22, cfg);
    require(!blocked.ok && blocked.goal_blocked && blocked.path.empty(),
            "seed22 obstacle must make the exact goal fail, not return a proxy");
    require(!plan_required_path(start, goal, {{6.25, 4.25, 0.20}}, cfg).ok,
            "required goal cannot consume the configured extra obstacle margin");
    require(!plan_required_path(start, {7.3, 4.25}, {}, cfg).ok,
            "required goal violates the physical field body margin");
    auto short_connector = cfg;
    short_connector.required_connector_length = 0.05;
    require(!plan_required_path(start, goal, {}, short_connector).ok,
            "connector-length budget may not be bypassed by an unconditional final append");
}

void remaining_terminal_validation()
{
    const auto cfg = config();
    const Vec2 goal{7.0, 4.25};
    const Path2 route{{5.0, 4.25}, {6.1, 4.25}, goal};
    require(required_path_clear(route.front(), route, goal, {}, cfg), "legal reference connector rejected");
    require(!required_path_clear(route.front(), {{5, 4.25}, {6.9, 4.25}}, goal, {}, cfg),
            "a safe proxy cannot masquerade as the exact required endpoint");
    require(required_path_clear({6.95, 4.25}, route, goal, {}, cfg),
            "current position already outside the virtual inset cannot continue its approved tail");
    require(required_path_clear({6.95, 4.26}, route, goal, {}, cfg),
            "small current tracking error must be checked as a physical rejoin, not a new reference curve");
    const Path2 sampled{{5.0, 4.25}, {6.1, 4.25}, {6.5, 4.25}, {6.9, 4.25}, {6.95, 4.25}, goal};
    require(required_path_clear({6.95, 4.25}, sampled, goal, {}, cfg),
            "collinear resampling lost the terminal-connector permission");
    require(!required_path_clear({6.95, 4.25}, sampled, goal, {{7.01, 4.25, .02}}, cfg),
            "a newly occupied terminal goal must stop the remaining tail");
    const Obstacles tail_obstacle{{6.15, 4.25, .02}};
    require(!required_path_clear(route.front(), route, goal, tail_obstacle, cfg),
            "new obstruction on the terminal connector was ignored");
    require(required_path_clear({6.95, 4.25}, route, goal, tail_obstacle, cfg),
            "new obstacle behind completed progress should not invalidate the remaining clear tail");
    require(!required_path_clear({7.25, 4.25}, route, goal, {}, cfg),
            "terminal rejoin outside physical field body bounds was accepted");
    require(!required_path_clear({5, 4.25}, {{5, 4.25}, {5.5, 4.25}, goal}, goal, {}, cfg),
            "an oversized terminal edge relaxed too much of the virtual boundary");
    require(!required_path_clear({5, 4.25}, {{5, 4.25}, {6.9, 4.25}, {6.96, 4.30}, goal}, goal, {}, cfg),
            "an unvalidated curved tail cannot inherit a straight connector permission");
    require(!required_path_clear({5, 0}, {{5, 0}, {7, 0}, {6.8, 4.25}, goal}, goal, {}, cfg),
            "terminal permission may not relax unrelated prefix boundary crossings");
    validate(plan_required_path({6.95, 4.25}, goal, {}, cfg), {6.95, 4.25}, goal, {}, cfg);
    validate(plan_required_path(goal, goal, {}, cfg), goal, goal, {}, cfg);
}

void exact_edges_and_detours()
{
    auto cfg = config();
    const Vec2 same_start{1.001, 1.001}, same_goal{1.002, 1.002};
    validate(plan_required_path(same_start, same_goal, {}, cfg), same_start, same_goal, {}, cfg);
    validate(plan_required_path({1, 1}, {1, 1}, {}, cfg), {1, 1}, {1, 1}, {}, cfg);
    validate(plan_required_path({0, 0}, {4, 0}, {}, cfg), {0, 0}, {4, 0}, {}, cfg);

    cfg = GlobalConfig{-1, -1, 1, 1, .2, .02, .01, .06};
    const Vec2 start{.19, .10}, goal{.7, .1};
    const Obstacles between_exact_and_cell{{.245, .1, .01}};
    const auto exact = plan_required_path(start, goal, between_exact_and_cell, cfg);
    validate(exact, start, goal, between_exact_and_cell, cfg);
    require(exact.path.size() > 2, "unsafe exact initial edge was replaced by an unchecked shortcut");

    cfg.cell = .02;
    const Vec2 near_start{.28, .09}, near_goal{.52, .09};
    const Obstacles near_obstacle{{.4, 0, .1}};
    const auto near = plan_required_path(near_start, near_goal, near_obstacle, cfg);
    validate(near, near_start, near_goal, near_obstacle, cfg);
    require(!required_path_clear(near_start, {near_start, near_goal}, near_goal, near_obstacle, cfg),
            "a goal closer than lookahead still cannot be reached through an obstacle");

    cfg = config();
    const Vec2 far_start{1, 0}, far_goal{6, 0};
    const Obstacles several{{3, 0, .3}, {4.6, .7, .3}};
    validate(plan_required_path(far_start, far_goal, several, cfg), far_start, far_goal, several, cfg);
    require(!required_path_clear(far_start, {far_start, far_goal}, far_goal, several, cfg),
            "multi-obstacle detour was reduced to a colliding direct final segment");
}

void short_motion_escape()
{
    const auto cfg = config();
    const Obstacles obstacle{{2, 0, .2}};
    require(obstacle_segment_clear({2.7, 0}, {2.72, 0}, obstacle, cfg),
            "short monotone recovery may remain within extra margin");
    require(!obstacle_segment_clear({2.7, 0}, {2.68, 0}, obstacle, cfg),
            "short recovery cannot move further into an obstacle margin");
    require(!obstacle_segment_clear({2.49, 0}, {2.7, 0}, obstacle, cfg),
            "physical body overlap cannot be waived by recovery");
    require(!obstacle_segment_clear({2.7, 0}, {2.9, 0}, {{2, 0, .2}, {3.4, 0, .2}}, cfg),
            "escaping one obstacle cannot approach another occupied margin");
}

void required_heading_cone()
{
    auto cfg = config();
    cfg.head_cone_half = 22.0 * std::acos(-1.0) / 180.0;
    cfg.head_cone_radius = 0.8;
    const Vec2 start{3, 0}, front{3.6, 0}, behind{2.4, 0};
    validate(plan_required_path(start, front, {}, cfg, 0.0), start, front, {}, cfg);
    require(!plan_required_path(start, behind, {}, cfg, 0.0).ok,
            "virtual terminal edge bypassed the requested local heading cone");
    validate(plan_required_path(start, behind, {}, cfg), start, behind, {}, cfg);
}
}  // namespace

int main()
{
    legal_red_and_blocked_seed22();
    remaining_terminal_validation();
    exact_edges_and_detours();
    short_motion_escape();
    required_heading_cone();
    std::cout << "global_required_path_test: all checks passed\n";
}
