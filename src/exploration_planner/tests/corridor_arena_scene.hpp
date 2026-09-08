#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>
#include <vector>

#include "exploration_planner/types.hpp"

namespace corridor_arena {

using exploration::Path2;
using exploration::Vec2;

struct Box {
    double x_min, x_max, y_min, y_max;
};

// edc_arena.sdf, delivery_field yaw=-pi/2, takeoff world=(-4, 0).
// These are physical boxes, including thickness, rather than visible point samples.
inline constexpr std::array<Box, 7> walls{{
    {8.95, 9.05, -5.0, 5.0},
    {-1.05, -0.95, -5.0, 5.0},
    {-1.0, 9.0, -5.05, -4.95},
    {-1.0, 9.0, 4.95, 5.05},
    {7.45, 7.55, -5.0, 3.5},
    {8.30, 9.00, 0.85, 0.95},
    {7.50, 8.20, -2.05, -1.95},
}};

inline double bodyClearance(const Vec2& position) {
    double clearance = std::numeric_limits<double>::infinity();
    for (const auto& wall : walls) {
        const double dx = std::max({wall.x_min - position.x, 0.0, position.x - wall.x_max});
        const double dy = std::max({wall.y_min - position.y, 0.0, position.y - wall.y_max});
        clearance = std::min(clearance, std::hypot(dx, dy));
    }
    return clearance;
}

inline bool bodyClear(const Vec2& position, double radius) {
    return bodyClearance(position) > radius;
}

namespace detail {

struct Point3 { double x, y, z; };

inline const std::vector<Point3>& rays() {
    static const std::vector<Point3> directions = [] {
        std::vector<Point3> result;
        result.reserve(1024 * 32);
        for (int vertical = 0; vertical < 32; ++vertical) {
            const double elevation = -0.7853982 + (0.2617994 + 0.7853982) * vertical / 31.0;
            for (int horizontal = 0; horizontal < 1024; ++horizontal) {
                const double azimuth = -3.14159 + 6.28318 * horizontal / 1023.0;
                result.push_back({std::cos(elevation) * std::cos(azimuth),
                                  std::cos(elevation) * std::sin(azimuth),
                                  std::sin(elevation)});
            }
        }
        return result;
    }();
    return directions;
}

inline bool intersectAxis(double origin, double direction, double minimum, double maximum,
                          double& near, double& far) {
    if (std::abs(direction) < 1e-12) return origin >= minimum && origin <= maximum;
    double first = (minimum - origin) / direction;
    double last = (maximum - origin) / direction;
    if (first > last) std::swap(first, last);
    near = std::max(near, first);
    far = std::min(far, last);
    return near <= far;
}

inline double hit(const Point3& origin, const Point3& direction, const Box& wall) {
    double near = 0.0, far = 6.0;
    if (!intersectAxis(origin.x, direction.x, wall.x_min, wall.x_max, near, far) ||
        !intersectAxis(origin.y, direction.y, wall.y_min, wall.y_max, near, far) ||
        !intersectAxis(origin.z, direction.z, 0.0, 3.0, near, far)) {
        return std::numeric_limits<double>::infinity();
    }
    return near;
}

}  // namespace detail

// First-hit 1024x32 LiDAR, 0.5 m blind range, capped at the planner's 6 m lookahead.
// The voxelized case also applies the configured point_filter_num=3 before
// XYZ centroids, matching Point-LIO's surface input rather than a full scan.
// No range noise is added, so geometric occlusion failures remain deterministic.
inline Path2 arenaCloud(const Vec2& position, double voxel_size = 0.0) {
    const detail::Point3 origin{position.x, position.y, 0.82};
    std::vector<detail::Point3> returns;
    returns.reserve(16384);
    std::size_t ray_index = 0;
    for (const auto& ray : detail::rays()) {
        if (voxel_size > 0.0 && ray_index++ % 3 != 0) continue;
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto& wall : walls) nearest = std::min(nearest, detail::hit(origin, ray, wall));
        if (ray.z < -1e-12) {
            const double ground_distance = -origin.z / ray.z;
            const double ground_x = origin.x + ray.x * ground_distance;
            const double ground_y = origin.y + ray.y * ground_distance;
            if (ground_x >= -1.0 && ground_x <= 9.0 && ground_y >= -5.0 && ground_y <= 5.0) {
                nearest = std::min(nearest, ground_distance);
            }
        }
        if (nearest < 0.5 || nearest > 6.0 || !std::isfinite(nearest)) continue;
        returns.push_back({origin.x + nearest * ray.x, origin.y + nearest * ray.y,
                           origin.z + nearest * ray.z});
    }

    Path2 cloud;
    const auto append = [&cloud](const detail::Point3& p) {
        if (p.z >= 0.40 && p.z <= 1.0) cloud.push_back({p.x, p.y});
    };
    if (voxel_size <= 0.0) {
        for (const auto& p : returns) append(p);
        return cloud;
    }

    struct Voxel { double x = 0.0, y = 0.0, z = 0.0; int count = 0; };
    std::map<std::tuple<int, int, int>, Voxel> voxels;
    for (const auto& p : returns) {
        const auto key = std::make_tuple(static_cast<int>(std::floor(p.x / voxel_size)),
                                        static_cast<int>(std::floor(p.y / voxel_size)),
                                        static_cast<int>(std::floor(p.z / voxel_size)));
        auto& voxel = voxels[key];
        voxel.x += p.x;
        voxel.y += p.y;
        voxel.z += p.z;
        ++voxel.count;
    }
    for (const auto& item : voxels) {
        const auto& v = item.second;
        append({v.x / v.count, v.y / v.count, v.z / v.count});
    }
    return cloud;
}

}  // namespace corridor_arena
