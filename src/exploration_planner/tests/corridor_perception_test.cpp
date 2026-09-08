#include "exploration_planner/corridor_perception.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using exploration::CorridorObservation;
using exploration::CorridorPerception;
using exploration::CorridorPerceptionConfig;
using exploration::Path2;
using exploration::Vec2;

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

void walls(Path2& cloud, const CorridorPerception& perception,
           double until = 8.0, bool include_left = true, bool include_right = true) {
    for (int i = 0; i <= static_cast<int>(until / 0.025); ++i) {
        const double s = i * 0.025;
        if (include_left) cloud.push_back(perception.toWorld(s, 1.0));
        if (include_right) cloud.push_back(perception.toWorld(s, -1.0));
    }
}

void bar(Path2& cloud, const CorridorPerception& perception,
         double s, double from, double to) {
    const int count = static_cast<int>(std::ceil((to - from) / 0.025));
    for (int i = 0; i <= count; ++i) {
        cloud.push_back(perception.toWorld(s, from + (to - from) * i / count));
    }
}

}  // namespace

int main() {
    CorridorPerceptionConfig previous_envelope;
    previous_envelope.robot_width = 0.6;
    CorridorPerception perception(previous_envelope);
    perception.setFrame({0.0, 0.0}, {10.0, 0.0});
    const Vec2 vehicle{1.0, 0.0};
    auto observed = perception.analyze({}, vehicle);
    require(!observed.cloud_observed && !observed.corridor_clear_observed &&
            !observed.gate_passable, "empty cloud must not mean free space");
    require(!CorridorPerception::pathClear({}, vehicle, {2.0, 0.0}, 0.3),
            "empty cloud cannot authorize a sweep");

    Path2 cloud;
    walls(cloud, perception);
    observed = perception.analyze(cloud, vehicle);
    require(observed.walls_observed && observed.corridor_clear_observed &&
            !observed.gate_observed && observed.observed_until > 6.5,
            "continuous side walls establish a bounded observed region");

    Path2 one_wall;
    walls(one_wall, perception, 8.0, true, false);
    bar(one_wall, perception, 3.0, 0.1, 1.0);
    observed = perception.analyze(one_wall, vehicle);
    require(!observed.walls_observed && !observed.gate_passable,
            "missing wall cannot supply the far edge of a gap");

    Path2 sparse = cloud;
    for (const double t : {-0.7, -0.4, 0.1, 0.7}) sparse.push_back({3.0, t});
    observed = perception.analyze(sparse, vehicle);
    require(!observed.gate_passable && !observed.corridor_clear_observed,
            "isolated sparse points cannot certify a door opening");

    Path2 left_gate = cloud;
    bar(left_gate, perception, 3.0, -0.1, 1.0);
    observed = perception.analyze(left_gate, vehicle);
    require(observed.gate_observed && observed.gate_passable &&
            observed.gate_center.y < -0.45 && observed.gap_width > 0.8 &&
            observed.gap_width < 0.91,
            "left-wall bar yields the center of the measured right opening");
    require(CorridorPerception::pathClear(left_gate, {2.5, observed.gate_center.y},
                                        {3.5, observed.gate_center.y}, 0.3),
            "0.9 m doorway admits a 0.6 m body without exploration inflation");
    require(!CorridorPerception::pathClear(left_gate, {2.5, 0.0}, {3.5, 0.0}, 0.3),
            "uncentered crossing still collides with the physical bar");

    Path2 alternating = left_gate;
    bar(alternating, perception, 3.3, -1.0, 0.1);
    observed = perception.analyze(alternating, vehicle);
    require(observed.gate_passable && std::abs(observed.gate_s - 3.0) < 0.05 &&
            observed.gate_center.y < 0.0, "two nearby gates retain the nearest opening");
    observed = perception.analyze(alternating, {3.05, -0.55}, 3.15);
    require(observed.gate_passable && std::abs(observed.gate_s - 3.3) < 0.05 &&
            observed.gate_center.y > 0.0, "passed-gate threshold selects the next opposite opening");

    Path2 closed = cloud;
    bar(closed, perception, 3.0, -1.0, 1.0);
    observed = perception.analyze(closed, vehicle);
    require(observed.gate_observed && !observed.gate_passable,
            "a closed transverse wall has no passable gap");

    Path2 narrow = cloud;
    bar(narrow, perception, 3.0, -0.45, 1.0);
    observed = perception.analyze(narrow, vehicle);
    require(observed.gate_observed && !observed.gate_passable,
            "opening narrower than body is rejected");

    Path2 two_sided = cloud;
    bar(two_sided, perception, 3.0, -1.0, -0.45);
    bar(two_sided, perception, 3.0, 0.45, 1.0);
    observed = perception.analyze(two_sided, vehicle);
    require(observed.gate_passable && std::abs(observed.gate_center.y) < 0.03 &&
            observed.gap_width > 0.8 && observed.gap_width < 0.91,
            "two transverse bars bound a centered opening");

    Path2 interrupted;
    walls(interrupted, perception, 1.5);
    for (int i = 120; i < 200; ++i) {
        interrupted.push_back({i * 0.04, -1.0});
        interrupted.push_back({i * 0.04, 1.0});
    }
    observed = perception.analyze(interrupted, vehicle);
    require(observed.observed_until < 1.6,
            "wall returns beyond an occluded interval do not extend visibility");

    Path2 terminal = cloud;
    bar(terminal, perception, 10.1, -1.0, 1.0);
    observed = perception.analyze(terminal, {7.5, 0.0});
    require(!observed.gate_observed, "terminal wall behind H is not another doorway");

    CorridorPerception rotated;
    rotated.setFrame({2.0, 3.0}, {2.0, -7.0});
    Path2 rotated_cloud;
    walls(rotated_cloud, rotated);
    bar(rotated_cloud, rotated, 3.0, -0.1, 1.0);
    observed = rotated.analyze(rotated_cloud, rotated.toWorld(1.0, 0.0));
    require(observed.gate_passable && std::abs(observed.gate_center.y) < 0.03 &&
            observed.gate_center.x < 1.55,
            "rotated corridor frame preserves door geometry in world coordinates");

    CorridorPerceptionConfig actual_config;
    actual_config.corridor_width = 1.5;
    CorridorPerception actual(actual_config);
    actual.setFrame({7.75, 4.25}, {7.75, -4.25});
    Path2 actual_cloud;
    for (int i = 0; i <= 320; ++i) {
        actual_cloud.push_back(actual.toWorld(i * 0.025, -0.75));
        actual_cloud.push_back(actual.toWorld(i * 0.025, 0.75));
    }
    bar(actual_cloud, actual, 3.0, 0.05, 0.75);
    observed = actual.analyze(actual_cloud, actual.toWorld(1.0, 0.0));
    require(observed.gate_passable && observed.gap_width > 0.73 &&
            observed.gap_width <= 0.8 && std::abs(observed.gate_center.x - 7.40) < 0.03 &&
            std::abs(observed.gate_center.y - 1.25) < 0.03,
            "actual 1.5 m corridor and 0.8 m opening resolve in the supplied SLAM frame");
    const double actual_t = actual.lateral(observed.gate_center);
    require(CorridorPerception::pathClear(actual_cloud, actual.toWorld(2.5, actual_t),
                                        actual.toWorld(3.5, actual_t), 0.3),
            "actual 0.8 m opening also fits the larger 0.6 m complete body");
    Path2 entrance_cloud;
    for (int i = 30; i <= 320; ++i) {
        entrance_cloud.push_back(actual.toWorld(i * 0.025, -0.75));
        entrance_cloud.push_back(actual.toWorld(i * 0.025, 0.75));
    }
    bar(entrance_cloud, actual, 2.45, 0.05, 0.75);
    observed = actual.analyze(entrance_cloud, actual.toWorld(0.0, 0.0));
    require(observed.gate_passable && observed.observed_until > 5.0,
            "side entrance with a 0.75 m initial wall aperture can cold-start door detection");
    Path2 remote_walls;
    for (int i = 120; i <= 320; ++i) {
        remote_walls.push_back(actual.toWorld(i * 0.025, -0.75));
        remote_walls.push_back(actual.toWorld(i * 0.025, 0.75));
    }
    bar(remote_walls, actual, 3.5, 0.05, 0.75);
    observed = actual.analyze(remote_walls, actual.toWorld(1.0, 0.0));
    require(!observed.gate_passable && observed.observed_until <= 1.0,
            "entrance aperture exception does not authorize missing wall support farther inside");

    CorridorPerceptionConfig larger;
    larger.robot_width = 0.9;
    CorridorPerception wide_body(larger);
    wide_body.setFrame({0.0, 0.0}, {10.0, 0.0});
    require(!wide_body.analyze(left_gate, vehicle).gate_passable,
            "configurable true body width controls fit");
    Path2 invalid{{std::numeric_limits<double>::quiet_NaN(), 0.0}};
    require(!perception.analyze(invalid, vehicle).cloud_observed &&
            !CorridorPerception::pathClear(invalid, vehicle, {2.0, 0.0}, 0.3),
            "nonfinite returns do not count as observations");

    std::cout << "corridor_perception_test: all checks passed\n";
    return 0;
}
