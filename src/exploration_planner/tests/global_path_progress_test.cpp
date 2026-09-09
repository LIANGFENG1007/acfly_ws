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

void near(double actual, double expected, const std::string& message)
{
    require(std::abs(actual - expected) < 1e-8,
        message + ": expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}

GlobalConfig config()
{
    return {-1.0, -5.0, 12.0, 5.0, 0.04, 0.30, 0.30, 0.60};
}

void sparse_line_progress()
{
    const Path2 path{{0.0, 0.0}, {10.0, 0.0}};
    const auto cfg = config();
    near(score_path({4.9, 0.0}, path, {}, cfg).length, 5.1, "before segment midpoint");
    near(score_path({5.1, 0.0}, path, {}, cfg).length, 4.9, "after segment midpoint");
    near(score_path({4.9, 1.0}, path, {}, cfg).length, 6.1, "include lateral connection");
    near(score_path({-1.0, 0.0}, path, {}, cfg).length, 11.0, "before path start");
    near(score_path({11.0, 0.0}, path, {}, cfg).length, 1.0, "after path end");
}

void passed_and_new_obstacles()
{
    const Path2 path{{0.0, 0.0}, {10.0, 0.0}};
    const auto cfg = config();
    const Obstacles behind{{2.0, 0.0, 0.2}};
    require(path_clear({4.9, 0.0}, path, behind, cfg), "passed obstacle must not invalidate path");
    near(score_path({4.9, 0.0}, path, behind, cfg).min_clear, 2.7,
        "score excludes already passed obstacle section");
    const Obstacles ahead{{6.0, 0.0, 0.2}};
    require(!path_clear({4.9, 0.0}, path, ahead, cfg), "new obstacle ahead must invalidate path");
    near(score_path({4.9, 0.0}, path, ahead, cfg).min_clear, -0.2,
        "score sees same obstacle ahead");
    require(!path_clear({5.0, 2.0}, path, {{5.0, 1.0, 0.1}}, cfg),
        "lateral connection must not cross obstacle");
    near(score_path({5.0, 2.0}, path, {{5.0, 1.0, 0.1}}, cfg).min_clear, -0.1,
        "score includes blocked lateral connection");
}

void corner_and_loop_progress()
{
    const auto cfg = config();
    const Path2 corner{{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}};
    near(score_path({4.9, 0.0}, corner, {}, cfg).length, 5.1, "before corner");
    near(score_path({5.0, 0.0}, corner, {}, cfg).length, 5.0, "at corner");
    near(score_path({5.0, 0.1}, corner, {}, cfg).length, 4.9, "after corner");
    require(path_clear({5.0, 1.0}, corner, {{3.0, 0.0, 0.2}}, cfg),
        "previous corner leg must not be checked");
    require(!path_clear({5.0, 0.0}, corner, {{5.0, 3.0, 0.2}}, cfg),
        "next corner leg must be checked");

    const Path2 crossing{{-2.0, 0.0}, {2.0, 0.0}, {0.0, -2.0}, {0.0, 2.0}};
    near(score_path({0.0, 0.0}, crossing, {}, cfg).length,
        6.0 + std::sqrt(8.0), "crossing keeps earliest equally near segment");
    require(!path_clear({0.0, 0.0}, crossing, {{1.5, 0.0, 0.1}}, cfg),
        "crossing must not skip obstacle on earlier pending leg");

    const Path2 retrace{{0.0, 0.0}, {10.0, 0.0}, {0.0, 0.0}};
    near(score_path({5.0, 0.0}, retrace, {}, cfg).length, 15.0,
        "retrace keeps earliest equally near segment");
}

void endpoints_and_degenerate_segments()
{
    const auto cfg = config();
    const Path2 path{{0.0, 0.0}, {10.0, 0.0}};
    near(score_path({10.0, 0.0}, path, {}, cfg).length, 0.0, "endpoint remaining length");
    require(path_clear({10.0, 0.0}, path, {{2.0, 0.0, 0.2}}, cfg),
        "endpoint excludes passed path");
    require(!path_clear({10.0, 0.0}, path, {{10.0, 0.0, 0.2}}, cfg),
        "endpoint still checks current position collision");
    near(score_path({10.0, 0.0}, path, {{10.0, 0.0, 0.2}}, cfg).min_clear, -0.2,
        "endpoint score checks current position");

    const Path2 duplicates{{0.0, 0.0}, {0.0, 0.0}, {10.0, 0.0}, {10.0, 0.0}};
    near(score_path({4.9, 0.0}, duplicates, {}, cfg).length, 5.1,
        "zero length segments do not affect progress");
    const Path2 stationary{{3.0, 2.0}, {3.0, 2.0}, {3.0, 2.0}};
    near(score_path({3.0, 2.0}, stationary, {}, cfg).length, 0.0,
        "stationary repeated path has zero length");
    require(path_clear({3.0, 2.0}, stationary, {}, cfg), "clear stationary repeated path");
    require(!path_clear({3.0, 2.0}, stationary, {{3.0, 2.0, 0.2}}, cfg),
        "stationary repeated path collision");
    near(score_path({3.0, 4.0}, stationary, {}, cfg).length, 2.0,
        "stationary repeated path lateral connection");
    require(!path_clear({0.0, 0.0}, {}, {}, cfg), "empty path stays invalid");
    require(!path_clear({0.0, 0.0}, {{0.0, 0.0}}, {}, cfg), "single point stays invalid");
}

void collision_margin_unchanged()
{
    const auto cfg = config();
    const Path2 path{{0.0, 0.0}, {10.0, 0.0}};
    require(!path_clear({4.9, 0.0}, path, {{6.0, 0.79, 0.2}}, cfg),
        "obstacle surface clearance below 0.6 remains blocked");
    require(path_clear({4.9, 0.0}, path, {{6.0, 0.81, 0.2}}, cfg),
        "obstacle surface clearance above 0.6 remains clear");
    near(score_path({4.9, 0.0}, path, {{6.0, 0.81, 0.2}}, cfg).min_clear, 0.61,
        "score remains distance to obstacle surface");
}

}  // namespace

int main()
{
    try {
        sparse_line_progress();
        passed_and_new_obstacles();
        corner_and_loop_progress();
        endpoints_and_degenerate_segments();
        collision_margin_unchanged();
        {
            auto cfg = config();
            cfg.head_cone_half = 22.0 * M_PI / 180.0;
            cfg.head_cone_radius = 0.80;
            const auto result = plan_global_path({2.0, 2.0}, {5.5, -2.5}, {}, cfg, 0.0);
            require(result.ok && result.path.size() >= 2,
                    "head-cone route should be available");
            const Vec2 first{result.path[1].x - result.path[0].x,
                             result.path[1].y - result.path[0].y};
            require(first.x > 0.0 && path_clear(result.path.front(), result.path, {}, cfg),
                    "simplified route must retain a safe forward guide segment");
            require(std::hypot(first.x, first.y) > 0.5,
                    "heading protection must not insert a centimetre-scale grid stop point");
        }
        std::cout << "global_path_progress_test: all scenarios passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "global_path_progress_test: " << error.what() << '\n';
        return 1;
    }
}
