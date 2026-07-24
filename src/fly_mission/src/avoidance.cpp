// ============================================================================
//  avoidance.cpp  ── 全局避障纯数学实现(详见同名 .hpp)
// ============================================================================

#include "fly_mission/avoidance.hpp"

#include <cmath>

namespace fly_mission {
namespace avoid {

namespace {

struct Vec2 { double x, y; };

inline double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline double norm(const Vec2& a) { return std::hypot(a.x, a.y); }

// 线段 S→G 上离点 C 最近的点(参数 t 夹在 [0,len]，即端点内)。返回最近点 + 距离 + 沿路进度 t。
struct SegClosest { double dist; double t; Vec2 foot; };
SegClosest segment_closest(const Vec2& S, const Vec2& G, const Vec2& C)
{
    const Vec2 d{ G.x - S.x, G.y - S.y };
    const double len = norm(d);
    if (len < 1e-6) {                         // S≈G：退化成点，最近点就是 S
        const double dist = std::hypot(C.x - S.x, C.y - S.y);
        return { dist, 0.0, S };
    }
    const Vec2 u{ d.x / len, d.y / len };
    double t = dot(Vec2{ C.x - S.x, C.y - S.y }, u);
    if (t < 0.0)   t = 0.0;
    if (t > len)   t = len;
    const Vec2 foot{ S.x + t * u.x, S.y + t * u.y };
    const double dist = std::hypot(C.x - foot.x, C.y - foot.y);
    return { dist, t, foot };
}

// 一个障碍的膨胀半径。半径非法/超上限 → 返回 <0 表示"忽略此障碍"。
inline double inflated(const CircleObstacle& o, const AvoidCfg& cfg)
{
    if (!std::isfinite(o.radius) || o.radius <= 0.0 || o.radius > cfg.max_radius) return -1.0;
    return o.radius + cfg.safety_margin;
}

// 点 P 是否在某障碍(膨胀后)内。用于"顶点撞另一根杆"检测。skip_idx 跳过自身。
bool point_in_any(const Vec2& P, const std::vector<CircleObstacle>& obs,
                  const AvoidCfg& cfg, int skip_idx)
{
    for (int i = 0; i < static_cast<int>(obs.size()); ++i) {
        if (i == skip_idx) continue;
        const double inf = inflated(obs[i], cfg);
        if (inf < 0.0) continue;
        if (std::hypot(P.x - obs[i].x, P.y - obs[i].y) < inf) return true;
    }
    return false;
}

// 二次贝塞尔 B(τ)=(1-τ)²S + 2(1-τ)τ P1 + τ²G
inline Vec2 bezier(const Vec2& S, const Vec2& P1, const Vec2& G, double tau)
{
    const double a = (1.0 - tau) * (1.0 - tau);
    const double b = 2.0 * (1.0 - tau) * tau;
    const double c = tau * tau;
    return { a * S.x + b * P1.x + c * G.x, a * S.y + b * P1.y + c * G.y };
}

// 沿贝塞尔曲线按【弧长】走 look 米取点(采样 samples 段线性插值累积)。
//   ★弧长不足 look 时★：不返回终点(终点在原直线上、不带横移，会让飞机直穿杆)，
//   而是返回曲线【顶点 B(0.5)】——那里离原直线横移最大、最能把飞机拽出去绕开杆。
//   (配合每拍重规划：飞机不断追顶点→绕出去；过了杆 blocking 判定自动解除→恢复直线。)
Vec2 bezier_arclen_lookahead(const Vec2& S, const Vec2& P1, const Vec2& G,
                             int samples, double look)
{
    if (samples < 2) samples = 2;
    Vec2 prev = S;
    double acc = 0.0;
    for (int i = 1; i <= samples; ++i) {
        const double tau = static_cast<double>(i) / samples;
        const Vec2 cur = bezier(S, P1, G, tau);
        const double seg = std::hypot(cur.x - prev.x, cur.y - prev.y);
        if (acc + seg >= look) {              // 目标弧长落在 prev→cur 段内，线性插值
            const double rem = look - acc;
            const double f = (seg > 1e-9) ? (rem / seg) : 0.0;
            return { prev.x + f * (cur.x - prev.x), prev.y + f * (cur.y - prev.y) };
        }
        acc += seg;
        prev = cur;
    }
    return bezier(S, P1, G, 0.5);              // 弧长不足 → 取顶点(横移最大)，绝不退化成直线终点
}

}  // namespace

PlanResult plan_lookahead(double sx, double sy, double tx, double ty,
                          const std::vector<CircleObstacle>& obstacles,
                          const AvoidCfg& cfg)
{
    PlanResult res;
    res.gx = tx; res.gy = ty; res.avoiding = false;   // 默认直线(走真实目标)

    // 输入防御
    if (!std::isfinite(sx) || !std::isfinite(sy) ||
        !std::isfinite(tx) || !std::isfinite(ty)) return res;

    const Vec2 S{ sx, sy }, G{ tx, ty };
    const Vec2 d{ G.x - S.x, G.y - S.y };
    const double len = norm(d);
    if (len < 1e-3) return res;               // 已在目标：不规划(PD 自会停)

    const Vec2 u{ d.x / len, d.y / len };
    const Vec2 n{ -u.y, u.x };                // 左法向

    // ---- 选"沿路径最近"的挡路障碍；跳过 目标落在其膨胀圈内 的障碍(任务故意靠近它) ----
    int    block_idx = -1;
    double block_t   = 1e18;
    double block_inf = 0.0;
    res.n_obstacles = static_cast<int>(obstacles.size());
    double nearest_seg = 1e18;   // 诊断：所有有效杆里离线段最近的距离
    for (int i = 0; i < static_cast<int>(obstacles.size()); ++i) {
        const double inf = inflated(obstacles[i], cfg);
        if (inf < 0.0) continue;              // 半径非法/超限，忽略
        const Vec2 C{ obstacles[i].x, obstacles[i].y };
        if (!std::isfinite(C.x) || !std::isfinite(C.y)) continue;
        res.n_valid++;

        const SegClosest sc = segment_closest(S, G, C);
        if (sc.dist < nearest_seg) {          // 诊断：记录最近杆(不管挡不挡)
            nearest_seg = sc.dist;
            res.nearest_dist = sc.dist;
            res.nearest_inflate = inf;
            res.nearest_id = obstacles[i].id;
        }

        // 目标就在这根杆(膨胀)里 → 任务有意靠近它(如绕杆 standoff)，不避它
        if (std::hypot(G.x - C.x, G.y - C.y) < inf) continue;

        if (sc.dist < inf) {                  // 挡路
            if (sc.t < block_t) { block_t = sc.t; block_idx = i; block_inf = inf; }
        }
    }
    if (block_idx < 0) return res;            // 无挡路 → 直线

    const CircleObstacle& ob = obstacles[block_idx];
    const Vec2 C{ ob.x, ob.y };
    res.obstacle_id = ob.id;

    // ---- 边界：飞机已在膨胀圈内 → 径向逃逸(直接往外飞 lookahead) ----
    const Vec2 SC{ S.x - C.x, S.y - C.y };
    const double sc_len = norm(SC);
    if (sc_len < block_inf) {
        Vec2 out;
        if (sc_len > 1e-6) out = Vec2{ SC.x / sc_len, SC.y / sc_len };
        else               out = Vec2{ 1.0, 0.0 };     // 恰在杆心：任取 +x
        res.gx = S.x + out.x * cfg.lookahead;
        res.gy = S.y + out.y * cfg.lookahead;
        res.avoiding = true;
        return res;
    }

    // ---- ★何时开始绕★：飞机离挡路杆心还远(> trigger_dist) → 照直飞，等靠近了再绕 ----
    //   (上面"已在膨胀圈内"的径向逃逸先于此判定，圈内一定绕；此处只挡"安全距离外"的提前绕行)
    if (sc_len > cfg.trigger_dist) return res;   // 远处直线直冲，avoiding=false

    // ---- 回归点 R：原航线 S→G 上、杆的投影点再往前 rejoin_ahead 米。★绕完就近切回航线★ ----
    //   贝塞尔终点用 R(近处、在原直线上)而非 G(远处航点)，避免绕出去后斜奔终点、在线外拖太久。
    //   过了 R 之后(下一拍杆已在身后→不挡路)自动恢复直线冲 G。
    double tC = dot(Vec2{ C.x - S.x, C.y - S.y }, u);   // 杆心在航线上的投影进度
    if (tC < 0.0) tC = 0.0;
    if (tC > len) tC = len;
    double tR = tC + cfg.rejoin_ahead;
    if (tR > len) tR = len;                              // 不越过真实目标 G
    Vec2 R{ S.x + tR * u.x, S.y + tR * u.y };            // 回归点(原航线上)
    // ★回归点别一头扎进下一根杆★：若 R 落在别的杆膨胀圈里，把回归点收回到本杆并排处(tC)，
    //   这样绕完先回到本杆侧、下一拍再规划绕下一根(支持连续绕一排杆)。
    if (point_in_any(R, obstacles, cfg, block_idx)) {
        R = Vec2{ S.x + tC * u.x, S.y + tC * u.y };
    }

    // ---- 定绕行侧：绕到障碍【对侧】(h=(C-S)·n；杆恰在线上 h≈0 → 定左 +n) ----
    const double h = dot(Vec2{ C.x - S.x, C.y - S.y }, n);
    double side = (std::fabs(h) < 1e-3) ? +1.0 : (h >= 0.0 ? -1.0 : +1.0);

    const double Rc = block_inf + cfg.clearance;   // 顶点离杆心距离

    // 途经点 V = C + Rc*m(m=净空侧单位向量)；控制点 P1 = 2V-(S+R)/2 ⇒ B(0.5)=V(终点用回归点 R)
    auto make_P1 = [&](double sgn) -> Vec2 {
        const Vec2 m{ sgn * n.x, sgn * n.y };
        const Vec2 V{ C.x + Rc * m.x, C.y + Rc * m.y };
        const Vec2 M{ 0.5 * (S.x + R.x), 0.5 * (S.y + R.y) };
        return { 2.0 * V.x - M.x, 2.0 * V.y - M.y };
    };
    auto apex_of = [&](double sgn) -> Vec2 {
        const Vec2 m{ sgn * n.x, sgn * n.y };
        return { C.x + Rc * m.x, C.y + Rc * m.y };
    };

    // 顶点撞到另一根杆 → 翻到另一侧重算一次；两侧顶点都撞 → 放弃绕行(走直线)
    if (point_in_any(apex_of(side), obstacles, cfg, block_idx)) {
        if (!point_in_any(apex_of(-side), obstacles, cfg, block_idx)) {
            side = -side;
        } else {
            return res;                       // 两侧都堵：本地贪心放弃，交直线(PD 会在杆前减速)
        }
    }

    const Vec2 P1 = make_P1(side);
    const Vec2 look = bezier_arclen_lookahead(S, P1, R, cfg.samples, cfg.lookahead);  // 终点=回归点 R
    if (!std::isfinite(look.x) || !std::isfinite(look.y)) return res;   // 兜底

    res.gx = look.x; res.gy = look.y; res.avoiding = true;
    return res;
}

}  // namespace avoid
}  // namespace fly_mission
