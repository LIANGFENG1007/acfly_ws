#include "exploration_planner/corridor_perception.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace exploration {
namespace {

struct LocalPoint {
    double s;
    double t;
};

struct Wall {
    bool valid = false;
    double t = 0.0;
    std::size_t support = 0;   // 纵向独立格数
    std::vector<double> samples;
};

bool finite(const Vec2& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

double median(std::vector<double> values) {
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

using Columns = std::map<int, std::vector<LocalPoint>>;

// 以一列为中心合并 ±band 内的相邻列：纵向独立格数与纵向跨度都够才是墙面。
Wall columnWall(const Columns& columns, int key, const CorridorPerceptionConfig& config) {
    const int neighborhood = std::max(1, static_cast<int>(
        std::ceil(config.wall_exclusion_band / config.cell_size)));
    std::vector<double> lateral_samples;
    std::vector<double> longitudinal_samples;
    std::set<int> longitudinal_cells;
    for (int offset = -neighborhood; offset <= neighborhood; ++offset) {
        const auto found = columns.find(key + offset);
        if (found == columns.end()) continue;
        for (const auto& p : found->second) {
            lateral_samples.push_back(p.t);
            longitudinal_samples.push_back(p.s);
            longitudinal_cells.insert(static_cast<int>(std::floor(p.s / config.cell_size)));
        }
    }
    Wall wall;
    if (static_cast<int>(longitudinal_cells.size()) < config.wall_min_points) return wall;
    const auto bounds = std::minmax_element(longitudinal_samples.begin(), longitudinal_samples.end());
    if (*bounds.second - *bounds.first < config.wall_min_span) return wall;
    wall.valid = true;
    wall.support = longitudinal_cells.size();
    wall.t = median(lateral_samples);
    wall.samples = std::move(longitudinal_samples);
    return wall;
}

Wall findWall(const std::vector<LocalPoint>& points, double expected,
              const CorridorPerceptionConfig& config) {
    Columns columns;
    for (const auto& p : points) {
        if (std::abs(p.t - expected) <= config.wall_search_tolerance) {
            columns[static_cast<int>(std::floor(p.t / config.cell_size))].push_back(p);
        }
    }

    Wall result;
    for (const auto& column : columns) {
        Wall wall = columnWall(columns, column.first, config);
        if (wall.valid && wall.support > result.support) result = std::move(wall);
    }
    std::sort(result.samples.begin(), result.samples.end());
    return result;
}

// 不预设位置：所有满足墙面条件的列，按纵向支撑做非极大抑制(同一面墙只留一条)。
std::vector<Wall> wallCandidates(const std::vector<LocalPoint>& points,
                                 const CorridorPerceptionConfig& config) {
    Columns columns;
    for (const auto& p : points)
        columns[static_cast<int>(std::floor(p.t / config.cell_size))].push_back(p);
    std::vector<Wall> raw;
    for (const auto& column : columns) {
        Wall wall = columnWall(columns, column.first, config);
        if (wall.valid) raw.push_back(std::move(wall));
    }
    std::sort(raw.begin(), raw.end(), [](const Wall& a, const Wall& b) { return a.support > b.support; });
    const double separation = std::max(2.0 * config.wall_exclusion_band, 2.0 * config.cell_size);
    std::vector<Wall> kept;
    for (auto& wall : raw) {
        if (std::any_of(kept.begin(), kept.end(), [&](const Wall& k) { return std::abs(k.t - wall.t) <= separation; }))
            continue;
        std::sort(wall.samples.begin(), wall.samples.end());
        kept.push_back(std::move(wall));
    }
    return kept;
}

double supportedUntil(const Wall& wall, double position_s, double max_gap, double initial_gap) {
    double end = position_s;
    bool started = false;
    for (double sample : wall.samples) {
        if (initial_gap > max_gap && sample < position_s) continue;
        if (sample < position_s - max_gap) continue;
        if (!started) {
            if (sample > position_s + initial_gap) break;
            end = sample;
            started = true;
        } else {
            if (sample - end > max_gap) break;
            end = sample;
        }
    }
    return started ? std::max(position_s, end) : position_s;
}

bool wallAt(const Wall& wall, double s, double tolerance) {
    const auto near = std::lower_bound(wall.samples.begin(), wall.samples.end(), s - tolerance);
    return near != wall.samples.end() && *near <= s + tolerance;
}

}  // namespace

CorridorPerception::CorridorPerception(const CorridorPerceptionConfig& config)
    : config_(config) {}

void CorridorPerception::setFrame(const Vec2& entry, const Vec2& goal) {
    origin_ = entry;
    const double dx = goal.x - entry.x;
    const double dy = goal.y - entry.y;
    length_ = std::hypot(dx, dy);
    frame_valid_ = finite(entry) && finite(goal) && length_ > 0.05;
    if (frame_valid_) axis_ = {dx / length_, dy / length_};
}

Vec2 CorridorPerception::toWorld(double s, double t) const {
    return {origin_.x + s * axis_.x - t * axis_.y,
            origin_.y + s * axis_.y + t * axis_.x};
}

double CorridorPerception::longitudinal(const Vec2& point) const {
    return (point.x - origin_.x) * axis_.x + (point.y - origin_.y) * axis_.y;
}

double CorridorPerception::lateral(const Vec2& point) const {
    return -(point.x - origin_.x) * axis_.y + (point.y - origin_.y) * axis_.x;
}

CorridorObservation CorridorPerception::analyze(
    const Path2& points, const Vec2& vehicle, double minimum_gate_s,
    const Path2* visibility_points, const Vec2* sensor_origin,
    double center_hint, bool free_search) const {
    CorridorObservation result;
    result.frame_valid = frame_valid_;
    if (!frame_valid_ || !finite(vehicle) || !std::isfinite(config_.cell_size) ||
        config_.cell_size <= 0.0 || config_.robot_width <= 0.0 ||
        config_.corridor_width <= config_.robot_width || config_.lookahead <= 0.0) {
        result.reason = "invalid_geometry";
        return result;
    }

    const double position_s = longitudinal(vehicle);
    result.observed_until = position_s;
    const double min_s = std::max(-config_.lookbehind, position_s - config_.lookbehind);
    // H is a destination, not a physical wall or a limit on measured wall support.
    const double max_s = position_s + config_.lookahead;
    const double half_width = config_.corridor_width * 0.5;
    const double reference = std::isfinite(center_hint) ? center_hint : 0.0;
    const double lateral_center = free_search ? lateral(vehicle) : reference;
    const double lateral_limit = free_search
        ? std::max(config_.center_search_range, half_width + config_.wall_search_tolerance)
        : half_width + config_.wall_search_tolerance;
    std::vector<LocalPoint> local;
    std::set<std::pair<int, int>> occupied;
    for (const auto& point : points) {
        if (!finite(point)) continue;
        const double s = longitudinal(point);
        const double t = lateral(point);
        if (s < min_s || s > max_s ||
            std::abs(t - lateral_center) > lateral_limit) continue;
        const auto key = std::make_pair(static_cast<int>(std::floor(s / config_.cell_size)),
                                        static_cast<int>(std::floor(t / config_.cell_size)));
        if (occupied.insert(key).second) local.push_back({s, t});
    }
    result.cloud_observed = !local.empty();
    if (!result.cloud_observed) {
        result.reason = "no_corridor_returns";
        return result;
    }

    Wall right, left;
    if (free_search) {
        // 找一对间距≈corridor_width 的平行长墙；多对时取中点离参考线最近的一对，同距取支撑更强的
        const auto candidates = wallCandidates(local, config_);
        double best_offset = std::numeric_limits<double>::infinity();
        std::size_t best_support = 0;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            for (std::size_t j = 0; j < candidates.size(); ++j) {
                const double width = candidates[j].t - candidates[i].t;
                if (width <= 0.0 || std::abs(width - config_.corridor_width) > config_.width_tolerance) continue;
                const double offset = std::abs(0.5 * (candidates[i].t + candidates[j].t) - reference);
                const std::size_t support = std::min(candidates[i].support, candidates[j].support);
                if (offset < best_offset - config_.cell_size ||
                    (offset <= best_offset + config_.cell_size && support > best_support)) {
                    best_offset = offset;
                    best_support = support;
                    right = candidates[i];
                    left = candidates[j];
                }
            }
        }
    } else {
        right = findWall(local, reference - half_width, config_);
        left = findWall(local, reference + half_width, config_);
    }
    result.right_valid = right.valid;
    result.left_valid = left.valid;
    if (right.valid) result.right_wall = right.t;
    if (left.valid) result.left_wall = left.t;
    if (right.valid && left.valid) {
        result.measured_width = left.t - right.t;
        if (std::abs(result.measured_width - config_.corridor_width) <= config_.width_tolerance) {
            result.center_t = 0.5 * (left.t + right.t);
            result.center_from_pair = true;
        } else {
            // 间距不符(一侧可能是门框/杂点)：取离期望位置更近的那一侧推算中线
            const double right_error = std::abs(right.t - (reference - half_width));
            const double left_error = std::abs(left.t - (reference + half_width));
            result.center_t = right_error <= left_error ? right.t + half_width : left.t - half_width;
        }
        result.center_valid = true;
    } else if (right.valid) {
        result.center_t = right.t + half_width;
        result.center_valid = true;
    } else if (left.valid) {
        result.center_t = left.t - half_width;
        result.center_valid = true;
    }
    result.walls_observed = right.valid && left.valid &&
        left.t - right.t > config_.robot_width;
    if (!result.walls_observed) {
        result.reason = "walls_unconfirmed";
        return result;
    }
    result.right_wall = right.t;
    result.left_wall = left.t;
    // A side entrance leaves one wall absent around the entry point. Allow that
    // initial aperture only at the entry; later wall continuity remains strict.
    const bool at_entry = std::abs(position_s) <= config_.robot_width * 0.5 + config_.cell_size;
    const double initial_gap = at_entry
        ? std::max(config_.max_wall_gap, half_width + config_.cell_size - position_s)
        : config_.max_wall_gap;
    result.observed_until = std::min(supportedUntil(right, position_s, config_.max_wall_gap, initial_gap),
                                     supportedUntil(left, position_s, config_.max_wall_gap, initial_gap));

    std::vector<LocalPoint> interior;
    // Rear returns still participate in collision checks and wall estimation,
    // but a wall wholly behind the aircraft is not the next doorway.
    const double gate_min_s = std::max(minimum_gate_s,
        position_s - config_.robot_width * 0.5 - config_.cell_size);
    for (const auto& point : local) {
        if (point.t > right.t + config_.wall_exclusion_band &&
            point.t < left.t - config_.wall_exclusion_band &&
            point.s >= gate_min_s && point.s < length_ - config_.endpoint_exclusion) {
            interior.push_back(point);
        }
    }
    std::sort(interior.begin(), interior.end(), [](const LocalPoint& a, const LocalPoint& b) {
        return a.s < b.s;
    });

    // Cluster transverse returns by depth. Both side walls must be measured at
    // that depth; an absent wall or an absent cloud never supplies a gap edge.
    for (std::size_t begin = 0; begin < interior.size();) {
        std::size_t end = begin + 1;
        while (end < interior.size() &&
               interior[end].s - interior[end - 1].s <= config_.gate_cluster_depth) ++end;
        std::vector<double> lateral_samples;
        std::vector<double> depth_samples;
        for (std::size_t index = begin; index < end; ++index) {
            lateral_samples.push_back(interior[index].t);
            depth_samples.push_back(interior[index].s);
        }
        const double cluster_start = interior[begin].s;
        const double cluster_end = interior[end - 1].s;
        begin = end;
        if (static_cast<int>(lateral_samples.size()) < config_.gate_min_points) continue;
        std::sort(lateral_samples.begin(), lateral_samples.end());
        const double span = lateral_samples.back() - lateral_samples.front();
        if (span < config_.gate_min_span) continue;

        double continuous_start = lateral_samples.front();
        double continuous_span = 0.0;
        int continuous_count = 1;
        bool transverse_support = false;
        for (std::size_t index = 1; index < lateral_samples.size(); ++index) {
            if (lateral_samples[index] - lateral_samples[index - 1] > config_.surface_sample_gap) {
                continuous_start = lateral_samples[index];
                continuous_count = 1;
            } else {
                ++continuous_count;
            }
            continuous_span = lateral_samples[index] - continuous_start;
            if (continuous_span >= config_.gate_min_span &&
                continuous_count >= config_.gate_min_points) transverse_support = true;
        }
        if (!transverse_support) continue;

        // A door is transverse. Long, thin traces from walls or isolated noise
        // cannot certify a traversable opening, even when they have many points.
        if (cluster_end - cluster_start > config_.gate_max_depth) {
            result.reason = "unclassified_interior_returns";
            return result;
        }
        result.gate_observed = true;
        result.gate_s = median(depth_samples);
        result.gate_depth = std::max(config_.cell_size, cluster_end - cluster_start);
        if (!wallAt(right, result.gate_s, config_.max_wall_gap) ||
            !wallAt(left, result.gate_s, config_.max_wall_gap) ||
            result.gate_s > result.observed_until + config_.max_wall_gap) {
            result.reason = "gate_boundaries_unconfirmed";
            return result;
        }

        // Quantization accounts only for measurement resolution. The aircraft
        // width is checked separately; no exploration obstacle inflation applies.
        const double half_cell = config_.cell_size * 0.5;
        std::vector<std::pair<double, double>> solid;
        solid.push_back({right.t - half_cell, right.t + half_cell});
        for (double sample : lateral_samples) {
            const double lower = sample - half_cell;
            const double upper = sample + half_cell;
            if (lower <= solid.back().second + config_.surface_sample_gap) {
                solid.back().second = std::max(solid.back().second, upper);
            } else {
                solid.push_back({lower, upper});
            }
        }
        if (left.t - half_cell <= solid.back().second + config_.surface_sample_gap) {
            solid.back().second = std::max(solid.back().second, left.t + half_cell);
        } else {
            solid.push_back({left.t - half_cell, left.t + half_cell});
        }

        const double required_width = config_.robot_width +
            std::max(0.0, config_.minimum_gap_extra);
        double best_score = std::numeric_limits<double>::infinity();
        bool wide_gap_seen = false;
        for (std::size_t index = 1; index < solid.size(); ++index) {
            const double gap_right = solid[index - 1].second;
            const double gap_left = solid[index].first;
            const double width = gap_left - gap_right;
            if (width < required_width) {
                if (!result.gate_passable) result.gap_width = std::max(result.gap_width, width);
                continue;
            }
            wide_gap_seen = true;
            const Vec2 ray_origin = sensor_origin ? *sensor_origin : vehicle;
            const double origin_s = longitudinal(ray_origin), origin_t = lateral(ray_origin);
            const double front = result.gate_s - result.gate_depth * 0.5;
            const double back = result.gate_s + result.gate_depth * 0.5;
            std::set<int> clear_ray_bins;
            // A missing return is not an opening. Require distinct current-frame
            // rays that cross the gap and actually reach a surface behind it.
            for (const auto& hit : visibility_points ? *visibility_points : points) {
                if (!finite(hit) || !finite(ray_origin) || front <= origin_s) continue;
                const double hit_s = longitudinal(hit), hit_t = lateral(hit);
                if (hit_s <= back + config_.cell_size) continue;
                const double front_t = origin_t + (hit_t - origin_t) * (front - origin_s) / (hit_s - origin_s);
                const double back_t = origin_t + (hit_t - origin_t) * (back - origin_s) / (hit_s - origin_s);
                if (front_t > gap_right && front_t < gap_left && back_t > gap_right && back_t < gap_left)
                    clear_ray_bins.insert(static_cast<int>(std::floor(front_t / config_.cell_size)));
            }
            if (static_cast<int>(clear_ray_bins.size()) < config_.gate_min_clear_rays) {
                if (!result.gate_passable) {
                    result.gap_width = std::max(result.gap_width, width);
                    result.clear_rays = std::max(result.clear_rays, static_cast<int>(clear_ray_bins.size()));
                }
                continue;
            }
            const double center = 0.5 * (gap_left + gap_right);
            const double score = std::abs(center - lateral(vehicle));
            if (score >= best_score) continue;
            best_score = score;
            result.gate_passable = true;
            result.gap_right = gap_right;
            result.gap_left = gap_left;
            result.gap_width = width;
            result.clear_rays = static_cast<int>(clear_ray_bins.size());
            result.gate_center = toWorld(result.gate_s, center);
        }
        result.reason = result.gate_passable ? "gate_opening_measured" :
            (wide_gap_seen ? "gate_free_space_unconfirmed" : "gate_closed_or_too_narrow");
        return result;
    }

    if (!interior.empty()) {
        result.reason = "unclassified_interior_returns";
        return result;
    }
    result.corridor_clear_observed = result.observed_until >= position_s + config_.min_observed_ahead;
    result.reason = result.corridor_clear_observed ? "walls_support_forward_region" : "forward_region_unconfirmed";
    return result;
}

bool CorridorPerception::pathClear(const Path2& points, const Vec2& start,
                                  const Vec2& end, double body_radius) {
    if (!finite(start) || !finite(end) || !std::isfinite(body_radius) || body_radius <= 0.0) return false;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double length_squared = dx * dx + dy * dy;
    const double radius_squared = body_radius * body_radius;
    bool observed = false;
    for (const auto& point : points) {
        if (!finite(point)) continue;
        observed = true;
        const double projection = length_squared > 1e-12 ?
            std::max(0.0, std::min(1.0, ((point.x - start.x) * dx +
                                       (point.y - start.y) * dy) / length_squared)) : 0.0;
        const double ex = point.x - (start.x + projection * dx);
        const double ey = point.y - (start.y + projection * dy);
        if (ex * ex + ey * ey <= radius_squared) return false;
    }
    return observed;
}

}  // namespace exploration
