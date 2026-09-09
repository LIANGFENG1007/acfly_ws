#include "exploration_planner/global_planner.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace exploration;

namespace {

void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}

GlobalConfig config()
{
    return {-5.0, -5.0, 5.0, 5.0, 0.04, 0.30, 0.30, 0.60};
}

void normal_safety()
{
    const auto cfg = config();
    const Obstacles obstacle{{0.0, 0.0, 0.2}};
    require(path_clear({-2.0, 0.81}, {{-2.0, 0.81}, {2.0, 0.81}}, obstacle, cfg),
        "safe path must remain clear");
    require(!path_clear({-2.0, 0.79}, {{-2.0, 0.79}, {2.0, 0.79}}, obstacle, cfg),
        "ordinary path must not enter full clearance");
    require(!path_clear({-2.0, 0.0}, {{-2.0, 0.0}, {2.0, 0.0}}, obstacle, cfg),
        "crossing obstacle stays blocked");
}

void monotonic_escape()
{
    const auto cfg = config();
    const Obstacles obstacle{{0.0, 0.0, 0.2}};
    const Vec2 start{0.78, 0.0};  // Surface distance .58m; required .60m.
    require(path_clear(start, {start, {0.79, 0.0}, {1.5, 0.0}}, obstacle, cfg),
        "monotonic escape from additional margin must be allowed");
    require(path_clear(start, {start, {0.78, 1.0}}, obstacle, cfg),
        "initially tangent direction has increasing distance and is safe");
    require(!path_clear(start, {start, {0.77, 0.2}, {1.5, 0.2}}, obstacle, cfg),
        "initial inward movement must be rejected even if endpoint is farther away");
    require(!path_clear({0.49, 0.0}, {{0.49, 0.0}, {1.5, 0.0}}, obstacle, cfg),
        "physical aircraft overlap must not be relaxed");
    require(!path_clear(start, {start, {1.1, 0.0}, {0.79, 0.0}, {1.5, 0.0}}, obstacle, cfg),
        "leaving then reentering clearance must fail");
    require(!path_clear(start, {start, {0.79, 0.0}}, obstacle, cfg),
        "unfinished escape endpoint must fail");
    require(!path_clear(start, {start, {1.5, 0.0}}, {{0, 0, .2}, {1.1, 0, .1}}, cfg),
        "escaping one obstacle cannot approach another");
    require(path_clear({0.79, 0.0}, {start, {0.79, 0.0}, {1.5, 0.0}}, obstacle, cfg),
        "old path remains valid while progressing through escape");
}

void analytic_intersections()
{
    auto cfg = config();
    cfg.cell = 1.0;
    cfg.robot_radius = 0.0;
    cfg.inflate = 0.0;
    const Path2 path{{0.0, 0.0}, {1.0, 0.0}};
    const Obstacles obstacle{{0.25, 0.0, 0.001}};
    require(!path_clear(path.front(), path, obstacle, cfg),
        "tiny obstacle between old sampling locations must be detected");
    require(std::abs(score_path(path.front(), path, obstacle, cfg).min_clear + 0.001) < 1e-12,
        "clearance scoring must use exact segment distance");
}

void simplify_escape_stub()
{
    const auto cfg = config();
    const Obstacles obstacle{{0.0, 0.0, 0.2}};
    const Vec2 start{0.78, 0.0}, goal{2.0, 0.0};
    const auto result = plan_global_path(start, goal, obstacle, cfg);
    require(result.ok, "A* escape path must exist");
    require(result.path.size() == 2, "safe escape shortcut must remove tiny initial grid stubs");
    require(path_clear(start, result.path, obstacle, cfg), "simplified escape must pass validation");
}

void field_entry()
{
    auto cfg = config();
    cfg.min_x = 0.0;
    require(path_inside_safe_field({{0, 0}, {.2, .1}, {.6, .2}, {2, 1}}, cfg),
        "virtual-wall start must allow monotonic entry");
    require(path_inside_safe_field({{.58, 0}, {.59, 0}, {1, 0}}, cfg),
        "slight boundary-margin violation may recover inward");
    require(!path_inside_safe_field({{.58, 0}, {.57, .1}, {1, 0}}, cfg),
        "initial margin violation must not grow");
    require(!path_inside_safe_field({{0, 0}, {1, 0}, {.5, 0}, {2, 0}}, cfg),
        "after field entry path must not exit again");
    require(!path_inside_safe_field({{0, 0}, {.5, 0}}, cfg),
        "field entry must finish inside safe field");
    require(!path_inside_safe_field({{0, 0}, {.3, 4.6}, {1, 0}}, cfg),
        "another initially valid wall remains enforced");
    require(path_inside_safe_field({{0, -5}, {.3, -4.7}, {.6, -4.4}, {1, -4}}, cfg),
        "corner entry must improve both violated boundaries");
    require(!path_inside_safe_field({{0, -5}, {.3, -5.1}, {1, -4}}, cfg),
        "corner entry cannot worsen either violated boundary");
    require(path_inside_safe_field({{1, 0}, {2, 2}}, cfg), "normal safe field path unchanged");
    require(!path_inside_safe_field({}, cfg), "empty field path is invalid");
}

}  // namespace

int main()
{
    try {
        normal_safety();
        monotonic_escape();
        analytic_intersections();
        simplify_escape_stub();
        field_entry();
        std::cout << "global_path_escape_test: all 5 scenarios passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "global_path_escape_test: " << error.what() << '\n';
        return 1;
    }
}
