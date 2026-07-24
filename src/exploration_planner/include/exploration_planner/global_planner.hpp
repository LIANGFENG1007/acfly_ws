// ============================================================================
//  global_planner.hpp  ── 全局点到点绕障路径搜索（手写 A*）
//
//  角色：全场【唯一】的"点到点避障移动原语"。探索/POI/归航三个分支都靠它绕障。
//        在障碍圆+四面墙膨胀的栅格上做分辨率完备的 A* 搜索，只要有路就能找到一条
//        绕过障碍、离墙到达目标的折线——不存在"贴近才决定方向"的局部最优(摆头/停死)，
//        过不去(被围死)也会直接返回无解，让上层提前决断(放弃该区/报警悬停)。
//
//  用途：
//    探索   : plan_explore 给出覆盖目标 → 本规划器改造成绕障折线 → smooth_catmull_rom
//             平滑 → tracker(pure-pursuit) 沿线跟随。
//    POI/归航: 直奔某点 → 搜绕障折线 → 平滑 → 节点沿线取 carrot 用 PD 跟随并精确刹停。
//
//  碰撞：obstacle_map 的障碍圆(snapshot) + 四面墙。cell 中心距任一障碍 < r+robot+inflate，
//        或距任一场地边界 < wall_margin，即视为禁入。零新依赖（纯 std）。
// ============================================================================

#pragma once

#include <cmath>

#include "exploration_planner/types.hpp"

namespace exploration {

struct GlobalConfig {
    double min_x, min_y, max_x, max_y;  // 场地范围 (SLAM 系, m)：搜索栅格边界，也是四面墙位置
    double cell;                        // 搜索栅格分辨率 (m)：越小路径越贴墙但越慢
    double robot_radius;                // 飞机半径 (m)
    double inflate;                     // 障碍额外安全余量 (m)：障碍禁入半径 = 障碍r + robot_radius + 此值
    double wall_margin;                 // 离墙安全距离 (m)：cell 中心距任一场地边界 < 此值即禁入
    // ★机头锥★：让 A* 起点段只朝飞机机头方向延伸，根除"新路从侧后方起步→飞机边转边走横切撞柱"。
    //   仅约束起点 head_cone_radius 半径内的扩展(起点附近的格相对起点的方位须落在机头±head_cone_half
    //   楔形内)，走出此半径恢复全向 A*(不扭曲远处绕障路径)。两值任一 ≤0 即关闭锥(退化老行为)。
    double head_cone_half   = 0.0;      // 锥半角 (rad)，≤0 关闭
    double head_cone_radius = 0.0;      // 锥作用半径 (m)，≤0 关闭
};

struct GlobalResult {
    bool  ok = false;            // 找到一条到目标(或其最近可达格)的绕障路径
    Path2 path;                  // 绕障折线（已简化为稀疏拐点，含起点≈飞机、终点=目标）
    bool  start_blocked = false; // 起点落在膨胀障碍内（飞机紧贴障碍）→ 已就近放行起点
    bool  goal_blocked  = false; // 目标落在膨胀障碍内（目标贴/陷障碍）→ 路由到最近可达格
};

// 在障碍圆+墙膨胀的栅格上 A* 搜 start→goal 的绕障折线。
//   start/goal：SLAM 系。obs：障碍圆列表。
//   start_yaw：飞机当前机头朝向 (rad)。传有限值 + cfg 的锥参数 >0 时启用★机头锥★(起点段只朝机头延伸)；
//              传 NaN(默认)则不加锥，行为与旧版完全一致(向后兼容，POI/归航可据此回退)。
//   返回 ok=false 表示目标被完全围死/不连通——上层据此处理：探索→放弃该区跳带；
//   POI/终点(必达)→报警悬停(无解属异常，不乱撞)。
//   起点/终点即便落在膨胀障碍/墙内也尽力求解（就近放行），避免贴障碍/墙时直接判死。
GlobalResult plan_global_path(const Vec2& start, const Vec2& goal,
                              const Obstacles& obs, const GlobalConfig& cfg,
                              double start_yaw = NAN);

// 校验一条【已采纳的绕障折线】在当前障碍图下是否仍全程无碰撞（不含墙——墙准静态，
//   靠 plan_global_path 重算时处理；这里只防"旧路径被新出现/移动的障碍挡住"）。
//   用法（路径承诺/迟滞）：飞机当前位置 cur 接到 path 上"最近拐点之后的剩余拐点"逐段查；
//   仍 clear → 续用旧路径，绝不因 near-tie 翻边；返回 false(被挡) → 上层重算 A* 换边。
//   判定与 plan_global_path 内部一致（只查障碍圆 r+robot+inflate），自洽不误杀。
bool path_clear(const Vec2& cur, const Path2& path,
                const Obstacles& obs, const GlobalConfig& cfg);

// 路径评分（★换路评估/迟滞★用）：只评"从 cur 接到路径最近点之后的剩余段"(与 path_clear 同口径,
//   已走过的不计，cur 与候选两条路才是同起点、可公平比)。
//     length    = 剩余弧长 (m)，越短→时间效益越高(走得越快)。
//     min_clear = 剩余段全程离任一障碍【边缘】的最小余量 (m)，越大→越安全；无障碍记一个大值。
//   上层据此决定"新算的路是否值得替换当前已承诺的路"：只有新路同时(或任一,可配)
//   【更安全≥X%】且【更快≥Y%】才换，否则续用旧路——near-tie 左右几乎等分→两项都不达标→不换，
//   根除"一会想走左边一会想走右边"的反复横跳。
struct PathScore {
    double length    = 0.0;   // 剩余弧长 (m)
    double min_clear = 0.0;   // 剩余段最小障碍边距 (m)，无障碍=很大值
};
PathScore score_path(const Vec2& cur, const Path2& path,
                     const Obstacles& obs, const GlobalConfig& cfg);

}  // namespace exploration
