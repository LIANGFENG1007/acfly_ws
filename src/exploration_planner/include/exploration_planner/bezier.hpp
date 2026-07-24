// ============================================================================
//  bezier.hpp  ── Centripetal Catmull-Rom → 三次贝塞尔 轨迹平滑
//
//  把牛耕折线航点平滑成插值曲线（穿过每个航点，掉头处不自交），
//  按弧长等距采样，每个采样点预存切线方向 θ 与曲率 κ，供轨迹跟踪用。
// ============================================================================

#pragma once

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

}  // namespace exploration
