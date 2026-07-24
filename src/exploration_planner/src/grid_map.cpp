#include "exploration_planner/grid_map.hpp"

#include <cmath>
#include <algorithm>

namespace exploration {

namespace {
// 把角度归一化到 (-pi, pi]
inline double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// 视线遮挡：从 P(px,py) 到 C(cx,cy) 的线段，是否被某障碍圆挡住(障碍在 P 与 C 之间)。
//   解二次方程求线段与圆相交区间 [t1,t2]（t∈[0,1]，P→C）。
//   遮挡条件：相交区间在到达 C 之前出现 → t2>eps 且 t1<1-eps。
//   ⇒ C 在障碍前方(障碍更远)不挡；C 在障碍后方/内部都挡。
inline bool los_blocked(double px, double py, double cx, double cy,
                        const exploration::Obstacles& obs)
{
    const double dx = cx - px, dy = cy - py;
    const double a = dx * dx + dy * dy;
    if (a < 1e-12) return false;
    const double eps = 1e-3;
    for (const auto& o : obs) {
        const double fx = px - o.cx, fy = py - o.cy;
        const double b = 2.0 * (fx * dx + fy * dy);
        const double c = fx * fx + fy * fy - o.r * o.r;
        const double disc = b * b - 4.0 * a * c;
        if (disc < 0.0) continue;                 // 线与圆不相交
        const double sq = std::sqrt(disc);
        const double t1 = (-b - sq) / (2.0 * a);  // 近交点
        const double t2 = (-b + sq) / (2.0 * a);  // 远交点
        if (t2 > eps && t1 < 1.0 - eps) return true;   // 障碍卡在 P→C 之间
    }
    return false;
}
}  // namespace

GridMap::GridMap(const GridConfig& cfg)
    : cfg_(cfg)
{
    const double w = cfg_.max_x - cfg_.min_x;
    const double h = cfg_.max_y - cfg_.min_y;

    small_nx_ = std::max(1, static_cast<int>(std::lround(w / cfg_.small_cell)));
    small_ny_ = std::max(1, static_cast<int>(std::lround(h / cfg_.small_cell)));

    cells_per_big_ = std::max(1, static_cast<int>(std::lround(cfg_.big_cell / cfg_.small_cell)));

    // 大格数 = 小格数 / 每大格小格数（向上取整，保证边缘也算一个大格）
    big_nx_ = (small_nx_ + cells_per_big_ - 1) / cells_per_big_;
    big_ny_ = (small_ny_ + cells_per_big_ - 1) / cells_per_big_;
    big_total_ = big_nx_ * big_ny_;

    small_scanned_.assign(static_cast<size_t>(small_nx_) * small_ny_, 0);
    big_count_.assign(static_cast<size_t>(big_total_), 0);
    big_explored_.assign(static_cast<size_t>(big_total_), 0);
}

void GridMap::world_to_small(double x, double y, int& gi, int& gj) const
{
    gi = static_cast<int>(std::floor((x - cfg_.min_x) / cfg_.small_cell));
    gj = static_cast<int>(std::floor((y - cfg_.min_y) / cfg_.small_cell));
}

bool GridMap::in_small_bounds(int gi, int gj) const
{
    return gi >= 0 && gi < small_nx_ && gj >= 0 && gj < small_ny_;
}

void GridMap::mark_scan(double px, double py, double yaw, const Obstacles& obstacles)
{
    const double half_fov = (cfg_.fov_deg * 0.5) * M_PI / 180.0;
    const double r = cfg_.fov_range;

    // 只对"可能挡在视野内"的障碍做遮挡判定（圆心离飞机 < 视野半径+障碍半径）。
    Obstacles near_obs;
    for (const auto& o : obstacles) {
        if (std::hypot(o.cx - px, o.cy - py) <= r + o.r) near_obs.push_back(o);
    }

    // 只遍历飞机周围 r 的包围盒对应的小格，而非全图
    int gi0, gj0, gi1, gj1;
    world_to_small(px - r, py - r, gi0, gj0);
    world_to_small(px + r, py + r, gi1, gj1);
    gi0 = std::max(gi0, 0);
    gj0 = std::max(gj0, 0);
    gi1 = std::min(gi1, small_nx_ - 1);
    gj1 = std::min(gj1, small_ny_ - 1);

    const double r2 = r * r;

    for (int gi = gi0; gi <= gi1; ++gi) {
        // 小格中心世界坐标
        const double cx = cfg_.min_x + (gi + 0.5) * cfg_.small_cell;
        const double dx = cx - px;
        for (int gj = gj0; gj <= gj1; ++gj) {
            const int idx = s_idx(gi, gj);
            if (small_scanned_[idx]) continue;   // 已扫过，跳过

            const double cy = cfg_.min_y + (gj + 0.5) * cfg_.small_cell;
            const double dy = cy - py;

            const double dist2 = dx * dx + dy * dy;
            if (dist2 > r2) continue;            // 超出半径

            const double ang = wrap_pi(std::atan2(dy, dx) - yaw);
            if (std::fabs(ang) > half_fov) continue;  // 超出开角

            // 模拟摄像头：被障碍挡住的格(障碍背后/内部)看不到，不标记
            if (!near_obs.empty() && los_blocked(px, py, cx, cy, near_obs)) continue;

            // 命中：标记小格 + 更新所属大格
            mark_small_cell(gi, gj);
        }
    }
}

// 把障碍圆覆盖的小格标成已扫(永久)。遍历每个障碍圆的包围盒，圆内小格全标。
void GridMap::fill_obstacle_cells(const Obstacles& obstacles)
{
    for (const auto& o : obstacles) {
        const double r2 = o.r * o.r;
        int gi0, gj0, gi1, gj1;
        world_to_small(o.cx - o.r, o.cy - o.r, gi0, gj0);
        world_to_small(o.cx + o.r, o.cy + o.r, gi1, gj1);
        gi0 = std::max(gi0, 0);
        gj0 = std::max(gj0, 0);
        gi1 = std::min(gi1, small_nx_ - 1);
        gj1 = std::min(gj1, small_ny_ - 1);
        for (int gi = gi0; gi <= gi1; ++gi) {
            const double cx = cfg_.min_x + (gi + 0.5) * cfg_.small_cell;
            const double dx = cx - o.cx;
            for (int gj = gj0; gj <= gj1; ++gj) {
                if (small_scanned_[s_idx(gi, gj)]) continue;
                const double cy = cfg_.min_y + (gj + 0.5) * cfg_.small_cell;
                const double dy = cy - o.cy;
                if (dx * dx + dy * dy > r2) continue;   // 只填圆内
                mark_small_cell(gi, gj);
            }
        }
    }
}

// 标记单个小格为已扫并增量更新大格（mark_scan/fill_obstacle_cells 共用）。
void GridMap::mark_small_cell(int gi, int gj)
{
    const int idx = s_idx(gi, gj);
    if (small_scanned_[idx]) return;
    small_scanned_[idx] = 1;
    const int bi = gi / cells_per_big_;
    const int bj = gj / cells_per_big_;
    const int bidx = b_idx(bi, bj);
    if (big_explored_[bidx]) return;
    ++big_count_[bidx];
    // 大格阈值：该大格内小格总数 * thresh（边缘大格小格数可能不足满格）
    const int gi_lo = bi * cells_per_big_;
    const int gj_lo = bj * cells_per_big_;
    const int gi_hi = std::min(gi_lo + cells_per_big_, small_nx_);
    const int gj_hi = std::min(gj_lo + cells_per_big_, small_ny_);
    const int cells_in_big = (gi_hi - gi_lo) * (gj_hi - gj_lo);
    if (big_count_[bidx] >= static_cast<int>(std::ceil(cells_in_big * cfg_.coverage_thresh))) {
        big_explored_[bidx] = 1;
        ++explored_total_;
    }
}

double GridMap::coverage_ratio() const
{
    if (big_total_ == 0) return 0.0;
    return static_cast<double>(explored_total_) / big_total_;
}

bool GridMap::big_explored(int bi, int bj) const
{
    if (bi < 0 || bi >= big_nx_ || bj < 0 || bj >= big_ny_) return false;
    return big_explored_[b_idx(bi, bj)] != 0;
}

int GridMap::big_count(int bi, int bj) const
{
    if (bi < 0 || bi >= big_nx_ || bj < 0 || bj >= big_ny_) return 0;
    return big_count_[b_idx(bi, bj)];
}

bool GridMap::small_scanned(int gi, int gj) const
{
    if (!in_small_bounds(gi, gj)) return false;
    return small_scanned_[s_idx(gi, gj)] != 0;
}

}  // namespace exploration
