// ============================================================================
//  grid_map.hpp  ── 双层栅格地图
//
//  大格"探索量"(BIG_CELL=0.5m) + 小格"完程度"(SMALL_CELL=0.05m)。
//  飞机每拍用位姿(x,y,yaw)把前方 150°/3m 扇形内的小格标记为"已扫"，
//  某大格被扫小格占比 >= COVERAGE_THRESH 时该大格判定"已探索"。
//
//  坐标系：输入位姿为 SLAM/camera_init 系（原点=起飞点=场地中心）。
// ============================================================================

#pragma once

#include <cstdint>
#include <vector>

#include "exploration_planner/types.hpp"

namespace exploration {

struct GridConfig {
    double min_x, min_y, max_x, max_y;   // 场地范围 (SLAM 系, m)
    double big_cell;                     // 大格边长 (m)
    double small_cell;                   // 小格边长 (m)
    double coverage_thresh;              // 大格完成阈值 (占比)
    double fov_deg;                      // 可视总开角 (度)
    double fov_range;                    // 可视半径 (m)
};

class GridMap
{
public:
    explicit GridMap(const GridConfig& cfg);

    // 用当前位姿把 150°/3m 扇形内的小格标记为已扫，并增量更新触及的大格。
    // px,py,yaw 为 SLAM 系（yaw 弧度）。
    //   obstacles：障碍圆列表。模拟摄像头：障碍背后被遮挡的小格看不到，不标记
    //   （从飞机到该格的视线被某障碍圆挡住就跳过）。传空 = 无遮挡(退化为原行为)。
    void mark_scan(double px, double py, double yaw, const Obstacles& obstacles = {});

    // 把障碍圆覆盖的小格直接标成"已扫"。障碍内部飞机永远看不到(实体+遮挡)，靠它扫不到会
    //   一直拖累覆盖率；既然雷达已确定该处是实体障碍，等价于"已知、无需再扫"，直接填上。
    //   ★只要扫到一次就永久保留★：标进 small_scanned_/big_count_(从不清除)，即使该障碍圆之后
    //   因点云衰减而消失，填充也不消失(障碍圆会变，栅格记忆不变)。每拍调用幂等(已标的跳过)。
    void fill_obstacle_cells(const Obstacles& obstacles);

    // 已探索大格占比 [0,1]
    double coverage_ratio() const;

    // ---- 给可视化用的只读访问 ----
    int    big_nx() const { return big_nx_; }
    int    big_ny() const { return big_ny_; }
    int    small_nx() const { return small_nx_; }
    int    small_ny() const { return small_ny_; }
    int    cells_per_big() const { return cells_per_big_; }

    bool   big_explored(int bi, int bj) const;
    int    big_count(int bi, int bj) const;      // 该大格已扫小格数
    bool   small_scanned(int gi, int gj) const;

    const GridConfig& config() const { return cfg_; }

    // SLAM 坐标 → 小格索引（不做边界裁剪，调用方需自行检查 in_small_bounds）
    void   world_to_small(double x, double y, int& gi, int& gj) const;
    bool   in_small_bounds(int gi, int gj) const;

private:
    GridConfig cfg_;

    int small_nx_, small_ny_;      // 小格网格尺寸
    int big_nx_,   big_ny_;        // 大格网格尺寸
    int cells_per_big_;            // 每大格边上小格数 (big_cell/small_cell)
    int big_total_;                // 大格总数

    std::vector<uint8_t> small_scanned_;   // small_nx_ * small_ny_ ，行优先 idx=gi*ny+gj
    std::vector<int>     big_count_;        // big_nx_ * big_ny_
    std::vector<uint8_t> big_explored_;     // big_nx_ * big_ny_
    int                  explored_total_ = 0;  // 已探索大格数（缓存，避免每次遍历）

    inline int s_idx(int gi, int gj) const { return gi * small_ny_ + gj; }
    inline int b_idx(int bi, int bj) const { return bi * big_ny_ + bj; }

    // 标记一个小格为已扫(若未标过)，并增量更新其所属大格计数/已探索判定。
    //   mark_scan(视野)与 fill_obstacle_cells(障碍)共用，保证两条路径口径一致。
    void mark_small_cell(int gi, int gj);
};

}  // namespace exploration
