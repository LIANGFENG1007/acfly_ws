// ============================================================================
//  types.hpp  ── 模块间共享的小型数据类型
// ============================================================================

#pragma once

#include <vector>

namespace exploration {

// 2D 点 / 向量 (SLAM 系, m)
struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

using Path2 = std::vector<Vec2>;

// 障碍物：拟合成圆柱(俯视=圆)。圆心(cx,cy) + 半径 r，均 SLAM 系 (m)。
//   雷达点云聚类后拟合得到；DWA 避障、视野遮挡、弹窗绿圆都用它。
struct Obstacle {
    double cx = 0.0;
    double cy = 0.0;
    double r  = 0.0;
};

using Obstacles = std::vector<Obstacle>;

}  // namespace exploration
