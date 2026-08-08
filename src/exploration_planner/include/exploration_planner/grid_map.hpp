// ============================================================================
//  grid_map.hpp  ── 双层栅格地图
//
//  大格"探索量"(BIG_CELL=0.5m) + 小格"完程度"(SMALL_CELL=0.05m)。
//  飞机每拍用位姿(x,y,yaw)把前方 FOV_DEG/FOV_RANGE 扇形(当前 100°/3m)内的小格
//  标记为"已扫"，某大格被扫小格占比 >= COVERAGE_THRESH 时该大格判定"已探索"。
//
//  坐标系：输入位姿为 SLAM/camera_init 系（原点=起飞点=场地中心）。
//
//  ★线程安全★：内部自带互斥锁(与 obstacle_map 同一套路)。写方(50Hz 主循环:
//    mark_scan/fill_obstacle_cells)与读方(20Hz 可视化线程)并发访问不再是竞态。
//    可视化【不要】直接持 GridMap 引用逐格读——那会在锁外读到撕裂状态；
//    请用 snapshot() 一次性拷出只读副本(见 GridSnapshot)。
// ============================================================================

#pragma once

#include <cmath>
#include <cstdint>
#include <mutex>
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

// ---------------------------------------------------------------------------
// 栅格快照：GridMap 在锁内一次性拷出的【只读副本】，供其他线程(可视化)安全渲染。
//   只读接口与 GridMap 同名同义 → 渲染代码对"活地图/快照"一视同仁，无需改写。
//   代价：一次约 (小格数 + 大格数) 字节的拷贝(当前场地 ≈32KB)，20Hz 下可忽略。
// ---------------------------------------------------------------------------
struct GridSnapshot {
    GridConfig           cfg{};
    int                  snx = 0, sny = 0;   // 小格网格尺寸
    int                  bnx = 0, bny = 0;   // 大格网格尺寸
    int                  cpb = 1;            // 每大格边上小格数
    double               cov = 0.0;          // 已探索大格占比 [0,1]
    std::vector<uint8_t> small;              // 已扫小格位图，行优先 idx = gi*sny+gj
    std::vector<uint8_t> big;                // 已探索大格位图，行优先 idx = bi*bny+bj

    const GridConfig& config() const { return cfg; }
    int    small_nx() const { return snx; }
    int    small_ny() const { return sny; }
    int    big_nx() const { return bnx; }
    int    big_ny() const { return bny; }
    int    cells_per_big() const { return cpb; }
    double coverage_ratio() const { return cov; }
    bool   small_scanned(int gi, int gj) const {
        return gi >= 0 && gi < snx && gj >= 0 && gj < sny && small[static_cast<size_t>(gi) * sny + gj] != 0;
    }
    bool   big_explored(int bi, int bj) const {
        return bi >= 0 && bi < bnx && bj >= 0 && bj < bny && big[static_cast<size_t>(bi) * bny + bj] != 0;
    }

    // 点 p 的【邻域半径 R 内】是否还有未探索大格。★探索目标失效判定★用：
    //   飞向某目标的途中，机载视野常已把它那片扫完 —— 此时再飞过去不增加任何进度
    //   ("明明已覆盖还在往前走")。R 内已无未扫格 ⇒ 这个目标已无信息可拿 ⇒ 该改投别处。
    //   与 coverage_ratio/plan_explore 选点同一口径(都看 big_explored)，不引入新的"已探索"定义。
    //   ★为何看邻域而非仅目标格本身★：只看目标格会误判——格心被扫完但周边仍是大片未知时，
    //   飞过去依然有收益；仿真显示"仅看目标格"总用时与基线持平(283.3s vs 281.0s，6 场景中 3 个更慢)，
    //   而看邻域 R=1.0m 六场景全面快于基线(242.3s，-13.8%)。
    bool   has_gain_within(const Vec2& p, double R) const {
        if (R <= 0.0 || cfg.big_cell <= 0.0) return false;
        const double R2 = R * R;
        for (int bi = 0; bi < bnx; ++bi) {
            const double cx = cfg.min_x + (bi + 0.5) * cfg.big_cell;
            if (std::fabs(cx - p.x) > R) continue;              // 整列超距，跳过
            for (int bj = 0; bj < bny; ++bj) {
                if (big[static_cast<size_t>(bi) * bny + bj]) continue;   // 已探索，不算收益
                const double cy = cfg.min_y + (bj + 0.5) * cfg.big_cell;
                const double dx = cx - p.x, dy = cy - p.y;
                if (dx * dx + dy * dy <= R2) return true;
            }
        }
        return false;
    }
};

class GridMap
{
public:
    explicit GridMap(const GridConfig& cfg);

    // 用当前位姿把 FOV_DEG/FOV_RANGE 扇形内的小格标记为已扫，并增量更新触及的大格。
    // px,py,yaw 为 SLAM 系（yaw 弧度）。整个操作在内部锁里一次做完。
    //   obstacles：障碍圆列表。模拟摄像头：障碍背后被遮挡的小格看不到，不标记
    //   （从飞机到该格的视线被某障碍圆挡住就跳过）。传空 = 无遮挡(退化为原行为)。
    void mark_scan(double px, double py, double yaw, const Obstacles& obstacles = {});

    // 把障碍圆覆盖的小格直接标成"已扫"。障碍内部飞机永远看不到(实体+遮挡)，靠它扫不到会
    //   一直拖累覆盖率；既然雷达已确定该处是实体障碍，等价于"已知、无需再扫"，直接填上。
    //   ★只要扫到一次就永久保留★：标进 small_scanned_/big_count_(从不清除)，即使该障碍圆之后
    //   因点云衰减而消失，填充也不消失(障碍圆会变，栅格记忆不变)。每拍调用幂等(已标的跳过)。
    void fill_obstacle_cells(const Obstacles& obstacles);

    // 在锁内一次性拷出只读副本，供【其他线程】渲染。跨线程读栅格只走这条路。
    GridSnapshot snapshot() const;

    // 已探索大格占比 [0,1]
    double coverage_ratio() const;

    // ---- 只读访问（各自加锁；跨线程批量读请改用 snapshot()）----
    //   尺寸类在构造后即只读、不再改变，故无需加锁。
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

    // ---- 以下四项由 mtx_ 保护（写方=50Hz 主循环，读方=可视化线程/规划）----
    std::vector<uint8_t> small_scanned_;   // small_nx_ * small_ny_ ，行优先 idx=gi*ny+gj
    std::vector<int>     big_count_;        // big_nx_ * big_ny_
    std::vector<uint8_t> big_explored_;     // big_nx_ * big_ny_
    int                  explored_total_ = 0;  // 已探索大格数（缓存，避免每次遍历）

    mutable std::mutex   mtx_;              // 保护上面四项；尺寸/cfg_ 构造后只读不锁

    inline int s_idx(int gi, int gj) const { return gi * small_ny_ + gj; }
    inline int b_idx(int bi, int bj) const { return bi * big_ny_ + bj; }

    // 标记一个小格为已扫(若未标过)，并增量更新其所属大格计数/已探索判定。
    //   mark_scan(视野)与 fill_obstacle_cells(障碍)共用，保证两条路径口径一致。
    //   ★调用方必须已持有 mtx_★（内部直接操作被保护的数组，自身不再加锁）。
    void mark_small_cell(int gi, int gj);
};

}  // namespace exploration
