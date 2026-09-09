// ============================================================================
//  bezier.hpp  ── Centripetal Catmull-Rom → 三次贝塞尔 轨迹平滑
//
//  把牛耕折线航点平滑成插值曲线（穿过每个航点，掉头处不自交），
//  按弧长等距采样，每个采样点预存切线方向 θ 与曲率 κ，供轨迹跟踪用。
// ============================================================================

#pragma once

#include <functional>

#include "exploration_planner/types.hpp"

namespace exploration {

// 参考轨迹上的一个采样点
struct TrajPoint {
    Vec2   p;        // 位置 (SLAM 系)
    double theta;    // 切线方向 (rad)
    double kappa;    // 曲率 (1/m)，带符号
    double s;        // 从轨迹起点起的累计弧长 (m)
};

using Trajectory = std::vector<TrajPoint>;

// 用 Centripetal Catmull-Rom(α=0.5) 把折线 waypoints 平滑，
// 按弧长步长 ds 采样为参考轨迹。waypoints 至少 2 个点。
Trajectory smooth_catmull_rom(const Path2& waypoints, double ds);

struct CornerRounding {
    double max_distance = 0.80;
    double min_distance = 0.25;
    double min_angle = 0.2617993877991494;
    double max_angle = 2.0943951023931953;
    double max_curvature = 5.0;
    double sample_distance = 0.05;
    int samples = 12;
};

// Replace each eligible interior vertex with a local quadratic Bezier whose
// tangents join the incoming and outgoing lines. An unsafe corner stays sharp;
// that rejection does not discard already validated rounds elsewhere.
Path2 round_path_corners(const Path2& path, const CornerRounding& options,
                        const std::function<bool(const Path2&)>& valid);

// Attach arc length, tangent and geometric curvature without changing any
// validated points. Sharp vertices remain visible to the tracker's stop logic.
// sharp_angle is the tracker's geometric stop threshold. Curvature is undefined
// at that discontinuity; its speed budget is the explicit stop distance instead.
Trajectory trajectory_from_polyline(const Path2& path, double sample_distance = 0.05,
                                    double sharp_angle = 3.14159265358979323846);

}  // namespace exploration
