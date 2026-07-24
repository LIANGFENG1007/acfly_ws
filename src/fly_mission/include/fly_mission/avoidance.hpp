#pragma once

// ============================================================================
//  avoidance.hpp  ── 全局避障：贝塞尔绕行的【纯数学】(无 ROS、无状态)
//
//  角色：给 DroneController 用的避障规划。输入"当前位置 S、目标 G、一组圆形障碍"，
//        输出"这一拍 PD 应该追的点(gx,gy)"——路径畅通就是 G 本身；被某根杆挡住就
//        是一条绕开杆的【二次贝塞尔曲线】上的前瞻点。全是纯函数，便于单测、也让
//        DroneController 保持轻。障碍的时效/订阅/线程安全在 DroneController 侧处理。
//
//  障碍：CircleObstacle 是【通用圆形障碍】(不特指杆)——杆检测把每根杆的 (x,y,半径)
//        喂进来；以后圆环/其它检测也能喂同一个列表。膨胀在规划里做(inflate=半径+裕度)。
//
//  坐标：与 DroneController 的 current_pose_ 同一 SLAM/camera_init 世界系，米。只算 xy
//        平面(z 由 DroneController 的 PD 独立处理，避障不碰高度)。
//
//  用法：DroneController::update_effective_goal() 每拍调 plan_lookahead(...)，
//        若 result.avoiding 则把 eff_gx_/eff_gy_ 设为 result.gx/gy 喂给 PD。
// ============================================================================

#include <vector>

namespace fly_mission {
namespace avoid {

// 通用圆形障碍(SLAM 系, m)。radius=原始拟合半径(未膨胀)；id 仅用于日志。
struct CircleObstacle {
    double x = 0.0;
    double y = 0.0;
    double radius = 0.0;
    int    id = -1;
};

// 规划参数(由 DroneController 从 params.hpp 填入，避免本模块依赖 params)。
struct AvoidCfg {
    double safety_margin = 0.30;   // 膨胀量：inflate = 障碍半径 + 此(含机身半宽+缓冲)
    double clearance     = 0.20;   // 额外余量：贝塞尔顶点离障碍心 = inflate + 此
    double lookahead     = 0.80;   // 沿曲线弧长前瞻多少米取点喂 PD
    double max_radius    = 1.00;   // 障碍半径 > 此直接忽略(坏拟合/把墙当大圆)
    double trigger_dist  = 1.00;   // 飞机离挡路障碍心 > 此 → 不绕(照直飞)；≤此才启贝塞尔
    double rejoin_ahead  = 0.50;   // 贝塞尔终点=原航线上"过障碍后 此米"处(绕完尽快回线，不斜奔远处目标)
    int    samples       = 20;     // 贝塞尔弧长采样段数
};

// 规划结果。avoiding=false → 走直线(gx,gy 即传入的目标)；true → gx,gy 是绕行前瞻点。
struct PlanResult {
    bool   avoiding = false;
    double gx = 0.0;
    double gy = 0.0;
    int    obstacle_id = -1;   // 正在绕的障碍编号(日志用)
    // ---- 诊断(总是填，供上层排查"为什么没避")----
    int    n_obstacles = 0;    // 传入的障碍数(过滤前)
    int    n_valid     = 0;    // 半径合法、参与判定的障碍数
    double nearest_dist = -1.0;    // 最近那根杆到 S→G 线段的距离(m)；-1=无有效障碍
    double nearest_inflate = 0.0;  // 该杆的膨胀半径(dist<此即挡路)
    int    nearest_id = -1;        // 该杆编号
};

// ★核心★：给起点 S(sx,sy)、目标 G(tx,ty)、障碍列表、配置，算出这一拍 PD 应追的点。
//   直线 S→G 不被任何(膨胀后)障碍挡 → 返回 {avoiding=false, gx=tx, gy=ty}。
//   被挡 → 构造绕开【最近挡路障碍】的二次贝塞尔，返回其上弧长前瞻点 {avoiding=true, ...}。
//   纯函数、不抛异常、对退化输入返回安全值(见 .cpp 各边界)。
PlanResult plan_lookahead(double sx, double sy, double tx, double ty,
                          const std::vector<CircleObstacle>& obstacles,
                          const AvoidCfg& cfg);

}  // namespace avoid
}  // namespace fly_mission
