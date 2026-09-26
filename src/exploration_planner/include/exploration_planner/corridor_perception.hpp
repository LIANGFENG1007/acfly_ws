#pragma once

#include <limits>
#include <string>

#include "exploration_planner/types.hpp"

namespace exploration {

struct CorridorPerceptionConfig {
    double corridor_width = 2.0;
    double robot_width = 0.50;
    double minimum_gap_extra = 0.0;
    double cell_size = 0.04;
    double lookahead = 6.0;
    double lookbehind = 0.8;
    double wall_search_tolerance = 0.35;
    double wall_exclusion_band = 0.08;
    double wall_min_span = 0.40;
    int wall_min_points = 3;
    double max_wall_gap = 0.55;
    double gate_min_span = 0.18;
    double gate_cluster_depth = 0.12;
    double gate_max_depth = 0.45;
    double surface_sample_gap = 0.35;
    int gate_min_points = 3;
    int gate_min_clear_rays = 2;
    double endpoint_exclusion = 0.30;
    double min_observed_ahead = 0.35;
};

struct CorridorObservation {
    bool frame_valid = false;
    bool cloud_observed = false;
    bool walls_observed = false;
    bool corridor_clear_observed = false;
    bool gate_observed = false;
    bool gate_passable = false;
    double left_wall = 0.0;
    double right_wall = 0.0;
    double observed_until = 0.0;
    double gate_s = 0.0;
    double gate_depth = 0.0;
    double gap_left = 0.0;
    double gap_right = 0.0;
    double gap_width = 0.0;
    int clear_rays = 0;
    Vec2 gate_center;
    std::string reason;
};

// Input points are height-filtered, uninflated obstacle returns in world XY.
class CorridorPerception {
public:
    explicit CorridorPerception(const CorridorPerceptionConfig& config = {});
    void setFrame(const Vec2& entry, const Vec2& goal);
    CorridorObservation analyze(
        const Path2& points, const Vec2& vehicle,
        double minimum_gate_s = -std::numeric_limits<double>::infinity(),
        const Path2* visibility_points = nullptr, const Vec2* sensor_origin = nullptr) const;
    Vec2 toWorld(double s, double t) const;
    double longitudinal(const Vec2& point) const;
    double lateral(const Vec2& point) const;
    double length() const { return length_; }

    // This is a swept-body collision test, not a free-space visibility test.
    // Callers must also require current observation and measured wall support.
    static bool pathClear(const Path2& points, const Vec2& start,
                          const Vec2& end, double body_radius);

private:
    CorridorPerceptionConfig config_;
    Vec2 origin_;
    Vec2 axis_{1.0, 0.0};
    double length_ = 0.0;
    bool frame_valid_ = false;
};

}  // namespace exploration
