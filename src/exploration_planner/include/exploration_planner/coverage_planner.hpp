// ============================================================================
//  coverage_planner.hpp  ── 牛耕往返(Boustrophedon)覆盖路径规划
//
//  无障碍单矩形场地 → 蛇形往返折线，覆盖全场后接到终点。
//  车道平行 x 轴，车道间距 = lane_spacing（按可视 swath 取，留重叠）。
//  起始车道选离起点近的一侧；蛇形方向使末车道端点尽量靠近 goal。
// ============================================================================

#pragma once

#include "exploration_planner/types.hpp"
#include "exploration_planner/grid_map.hpp"

namespace exploration {

struct CoverageConfig {
    double min_x, min_y, max_x, max_y;   // 场地范围 (SLAM 系, m)
    double lane_spacing;                 // 车道间距 (m)
    double margin;                       // 离墙留边 (m)，车道不贴到边界
};

// 生成牛耕折线航点（旧静态版，动态探索不再调用，保留备查）。
//   start : 起点（通常 = 当前/起飞位置，场地中心附近）
//   goal  : 终点
// 返回：折线航点序列（含起点，末尾为 goal）。
Path2 plan_boustrophedon(const CoverageConfig& cfg, const Vec2& start, const Vec2& goal);

// ---------------------------------------------------------------------------
// 动态前沿覆盖（分层：上层扫描条带定大顺序，下层条带内朝未扫前沿贪心串链）。
//   上层：把可达区按 band_width(≈视野宽) 沿 y 切成条带，蛇形顺序逐带推进；
//         当前条带内未扫格数 ≤ band_clear_cnt 才推进下一带(迟滞，不回退)，避免折返/横跳。
//   下层：在"当前条带(±band_tol 容差)"内选未扫前沿，代价 = 距离 + turn_penalty*转角
//         − cluster_weight*未扫邻居 − along_bonus*沿条带主方向(x)的推进量(鼓励顺着扫)。
//         这一层是未来接【避障】(局部绕行)和【必经点】(插队优先)的落点。
//   目标点夹进安全区(离墙 margin)；墙根大格靠 3m 视野扫到。串到累计长度 ≈ horizon。
// 状态：cur_band 由调用方持有并传入/带出（上层条带进度）。
// 返回含 cur 起点的折线；若全场无未探索大格 → all_explored=true 返回 {cur}。
// ---------------------------------------------------------------------------
struct FrontierConfig {
    double min_x, min_y, max_x, max_y;   // 场地范围 (SLAM 系, m)
    double big_cell;                     // 大格边长 (m)
    double margin;                       // 安全内缩 (m)，飞机可达区 = 场地内缩 margin
    double near_weight;                  // 邻近系数：距离项权重，越大越优先最近未扫区
    double turn_penalty;                 // 选目标转向代价 (m/rad)
    double cluster_weight;               // 未扫邻居加成 (m/个)
    double horizon;                      // 单次链路总长 (m)
    double chain_gap;                    // 链路相邻目标最小间隔 (m)
    // ---- 分层新增 ----
    double band_width;                   // 条带宽度 (m)，≈视野单侧覆盖，取 lane_spacing
    double band_tol;                     // 当前条带上下容差 (m)，选点时把略出带的格也纳入
    double along_bonus;                  // 沿条带主方向推进加成 (m/m)，越大越爱顺着横扫
    int    band_clear_cnt;               // 当前条带剩余未扫格 ≤ 此值才推进下一带(迟滞)
};

// unreachable/block_r: explore give-up blacklist. Skip candidate cells within
//   block_r of any unreachable point so the drone stops retrying dead-end areas.
//   Pass nullptr/empty to disable filtering (same as old behavior).
Path2 plan_explore(const GridMap& grid, const FrontierConfig& cfg,
                   const Vec2& cur, double cur_yaw, int& cur_band, bool& all_explored,
                   const std::vector<Vec2>* unreachable = nullptr, double block_r = 0.0);

}  // namespace exploration
