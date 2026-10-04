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

void boundary_endpoint_permission()
{
    auto cfg = config();
    cfg.min_y = -5.35; cfg.max_x = 1.57; cfg.max_y = 1.15;
    const Vec2 start{0, 0}, goal{1.55, .60};
    require(!plan_required_path(start, goal, {}, cfg).ok,
            "default required targets must retain body clearance");
    cfg.required_goal_field_margin = 0.0;
    const auto route = plan_required_path(start, goal, {}, cfg);
    validate(route, start, goal, {}, cfg);
    require(path_inside_safe_field(Path2(route.path.begin(), route.path.end() - 1), cfg),
            "boundary endpoint permission relaxed the ordinary path");
    validate(plan_required_path(start, {1.57, .6}, {}, cfg), start, {1.57, .6}, {}, cfg);
    validate(plan_required_path(start, {1.57, 1.15}, {}, cfg), start, {1.57, 1.15}, {}, cfg);
    require(required_path_clear({1.50, .60}, route.path, goal, {}, cfg),
            "near-boundary execution lost the approved connector");
    validate(plan_required_path({1.50, .60}, goal, {}, cfg), {1.50, .60}, goal, {}, cfg);
    require(!plan_required_path(start, {1.58, .6}, {}, cfg).ok,
            "boundary endpoint permission accepted a goal outside the field");
    require(!required_path_clear({1.58, .60}, route.path, goal, {}, cfg),
            "terminal rejoin outside the field was accepted");
    const Obstacles blocked{{1.55, .60, .1}};
    require(!plan_required_path(start, goal, blocked, cfg).ok &&
            !required_path_clear(start, route.path, goal, blocked, cfg),
            "boundary endpoint permission waived obstacle avoidance");
    cfg.required_goal_field_margin = .1;
    require(!plan_required_path(start, goal, {}, cfg).ok,
            "configured endpoint margin was ignored");
}

void forward_field_entry_revalidation()
{
    auto cfg = config();
    cfg.min_y = -.35; cfg.max_x = 1.57; cfg.max_y = 1.15;
    cfg.required_goal_field_margin = 0;
    const Vec2 goal{1.55, .6}, current{.001, 0};
    const Path2 route{{0, 0}, {.72, .28}, goal};
    require(required_path_clear(current, route, goal, {}, cfg),
            "microscopic backward projection invalidated a safe forward field entry");
    require(!required_path_clear(current, route, goal, {{.35, .15, .01}}, cfg),
            "forward entry rejoin ignored an obstacle on the connection");
    require(!required_path_clear({.041, 0}, {{0, 0}, {.04, .016}, {.72, .28}, goal}, goal, {}, cfg),
            "forward entry rejoin allowed retreat toward a violated boundary");
    require(!required_path_clear(current, {{0, 0}, {.5, .2}, goal}, goal, {}, cfg),
            "forward entry permission bypassed the required inner connector anchor");

    // The next sample can still be behind the aircraft's x coordinate. Adding
    // collinear samples must not turn a legal inward rejoin into a reverse move.
    for (int samples : {1, 16, 100}) {
        Path2 sampled;
        for (int i = 0; i <= samples; ++i)
            sampled.push_back({.72 * i / samples, .28 * i / samples});
        sampled.push_back(goal);
        const Vec2 off_reference{.04852, .00268};
        require(required_path_clear(off_reference, sampled, goal, {}, cfg),
                "collinear entry resampling caused a spurious reverse-boundary replan");
        require(!required_path_clear(off_reference, sampled, goal, {{.35, .15, .01}}, cfg),
                "resampled forward rejoin skipped a new obstacle");
    }
    require(required_path_clear({.553968, .204559}, {{.620423, .278632}, goal}, goal, {}, cfg),
            "inward rejoin before the terminal anchor consumed the connector length budget");
}

void interior_required_routes_have_no_artificial_connector_bend()
{
    auto cfg = config();
    cfg.head_cone_half = 22.0 * std::acos(-1.0) / 180.0;
    cfg.head_cone_radius = .8;
    // Consecutive visual targets from the reported empty-field flight.
    const Vec2 starts[]{{2.445, 1.141}, {3.923, -.048}, {5.894, 1.222}, {2, 0}};
    const Vec2 goals[]{{3.923, -.048}, {5.894, 1.222}, {6.515, -1.190}, {4, 1}};
    for (size_t i = 0; i < 4; ++i) {
        const double bearing = std::atan2(goals[i].y - starts[i].y, goals[i].x - starts[i].x);
        for (double heading_offset : {0.0, .6, 1.5, 3.0}) {
            const auto route = plan_required_path(starts[i], goals[i], {}, cfg, bearing + heading_offset);
            validate(route, starts[i], goals[i], {}, cfg);
            require(route.path.size() == 2,
                    "free interior target retained an unnecessary terminal connector bend");
        }
    }
    const Obstacles obstacle{{3, .5, .2}};
    const auto detour = plan_required_path({2, 0}, {4, 1}, obstacle, cfg);
    validate(detour, {2, 0}, {4, 1}, obstacle, cfg);
    require(detour.path.size() > 2,
            "interior connector simplification removed a necessary obstacle detour");
}
void home_routes_prefer_the_complete_straight_line()
{
    GlobalConfig cfg{-.5, -3.8, 9, 4.6, .04, .4, .10, .60};
    cfg.required_goal_field_margin = 0;
    cfg.required_direct_path = true;
    cfg.head_cone_half = .4;
    cfg.head_cone_radius = .8;
    const Vec2 goal{7.75, 4.3};
    for (Vec2 start : {Vec2{0,0}, {5.72,-1.39}, {3.24,-2.07}, {5.8,4}, {7.1,3.8}, goal}) {
        for (double yaw : {0.0, 1.5, 3.0, -1.5}) {
            const auto route = plan_required_path(start, goal, {}, cfg, yaw);
            validate(route, start, goal, {}, cfg);
            require(route.path.size() == 2, "empty home route retained a grid/heading connector bend");
        }
    }
    // A line parallel to the boundary cannot intersect the ordinary inset.
    // It remains legal for home, including collinear trajectory resampling.
    const Vec2 start{1,4.6}, edge_goal{8,4.6};
    auto route = plan_required_path(start, edge_goal, {}, cfg);
    validate(route, start, edge_goal, {}, cfg);
    require(route.path.size()==2, "clear boundary-parallel home route was bent inward");
    require(required_path_clear({3,4.45}, {start,{3,4.6},{5,4.6},edge_goal}, edge_goal, {}, cfg),
            "resampling or tracking error lost home straight-route permission");
    require(!required_path_clear({3,4.61}, route.path, edge_goal, {}, cfg),
            "home direct rejoin escaped the actual field");
    require(!required_path_clear(start, {start,{3,4.3},edge_goal}, edge_goal, {}, cfg),
            "a curved boundary detour inherited permission for a straight route");
    require(!plan_required_path({0,0}, {9.01,4.3}, {}, cfg).ok,
            "home direct path accepted an out-of-field endpoint");
    cfg.required_goal_field_margin=.1;
    require(!plan_required_path({0,0}, edge_goal, {}, cfg).ok, "home margin setting was ignored");
    cfg.required_goal_field_margin=0;
    // Real obstacles, including ones observed after adopting a line, retain
    // full aircraft clearance. Off-route obstacles must not manufacture bends.
    const Vec2 interior_start{1,0};
    const Obstacles blocking{{4.375,2.15,.25}};
    route=plan_required_path(interior_start,goal,blocking,cfg);
    validate(route,interior_start,goal,blocking,cfg);
    require(route.path.size()>2, "home shortcut crossed a blocking obstacle");
    require(!obstacle_segment_clear(route.path[route.path.size()-3],goal,blocking,cfg),
            "home obstacle detour retained a removable terminal connector corner");
    require(distance(route.path[route.path.size()-2],goal)>cfg.required_connector_length,
            "home detour still confines its final clear straight leg to one metre");
    require(!required_path_clear(interior_start,{interior_start,goal},goal,blocking,cfg),
            "new obstacle failed to invalidate home straight line");
    route=plan_required_path(interior_start,goal,{{4,-2,.25}},cfg);
    validate(route,interior_start,goal,{{4,-2,.25}},cfg);
    require(route.path.size()==2, "off-route obstacle introduced a home bend");

    // After a safe detour, a shallow final leg can spend more than a metre in
    // the virtual wall inset. Revalidation and resampling must retain it.
    const Path2 long_tail{{1,2},{2,3.9},goal};
    const Vec2 midway{5.45,4.14};
    require(required_path_clear(midway,long_tail,goal,{},cfg),
            "execution lost the long clear home terminal leg after a detour");
    require(required_path_clear({5.45,4.16},{{1,2},{2,3.9},midway,goal},goal,{},cfg),
            "resampling or small drift invalidated the long home terminal leg");
    require(!required_path_clear(midway,long_tail,goal,{{6.6,4.22,.1}},cfg),
            "long home tail ignored a newly observed obstacle");
    require(!required_path_clear({5.45,4.61},long_tail,goal,{},cfg),
            "long home tail waived the actual field boundary");
    cfg.required_direct_path=false;
    require(!required_path_clear(midway,long_tail,goal,{},cfg),
            "home-only long tail permission leaked to ordinary required goals");
}
void home_corners_require_geometric_obstructions()
{
    // Change both scene scale and grid resolution. No flight distance or one
    // particular recorded connector may determine where a home route bends.
    for (double scale : {.5,1.0,1.7}) for (double cell : {.04,.11,.2}) {
        GlobalConfig cfg{-.5*scale,-3.8*scale,9*scale,4.6*scale,
                         cell*scale,.4*scale,.1*scale,.6*scale};
        cfg.required_goal_field_margin=0;
        cfg.required_direct_path=true;
        const Vec2 start{scale,0}, goal{7.75*scale,4.3*scale};
        const Obstacles obstacles{{4.375*scale,2.15*scale,.25*scale},
                                  {5.8*scale,3.2*scale,.15*scale}};
        const auto route=plan_required_path(start,goal,obstacles,cfg);
        validate(route,start,goal,obstacles,cfg);
        require(route.path.size()>2,"visibility simplification crossed a blocking obstacle");
        for (double legacy_length : {.15,1.0,5.0}) {
            auto changed=cfg;changed.required_connector_length=legacy_length;
            const auto same=plan_required_path(start,goal,obstacles,changed);
            validate(same,start,goal,obstacles,changed);
            require(same.path.size()==route.path.size(),"legacy connector radius changed home geometry");
            for (size_t i=0;i<route.path.size();++i)
                require(distance(same.path[i],route.path[i])<1e-9,
                        "home still uses a fixed distance to choose its turn points");
        }
        for (size_t i=0;i+2<route.path.size();++i) {
            for (size_t j=i+2;j<route.path.size();++j) {
                Path2 skipped=route.path;
                skipped.erase(skipped.begin()+i+1,skipped.begin()+j);
                require(!required_path_clear(start,skipped,goal,obstacles,cfg),
                        "home retained guide corners that a safe visible segment can skip");
            }
        }
        const auto empty=plan_required_path(start,goal,{},cfg);
        require(empty.ok && empty.path.size()==2,"empty scaled scene manufactured a waypoint");
    }
}
}  // namespace

int main()
{
    legal_red_and_blocked_seed22();
    remaining_terminal_validation();
    exact_edges_and_detours();
    short_motion_escape();
    required_heading_cone();
    boundary_endpoint_permission();
    forward_field_entry_revalidation();
    interior_required_routes_have_no_artificial_connector_bend();
    home_routes_prefer_the_complete_straight_line();
    home_corners_require_geometric_obstructions();
    std::cout << "global_required_path_test: all checks passed\n";
}
