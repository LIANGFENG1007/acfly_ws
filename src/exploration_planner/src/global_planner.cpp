// ============================================================================
//  global_planner.cpp  ── 全局点到点绕障 A* 实现（详见同名 .hpp）
// ============================================================================

#include "exploration_planner/global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace exploration {

namespace {

// 内部栅格：把场地按 cell 切方格，每格中心距任一障碍 < r+robot+inflate 即禁入。
struct Grid {
    double min_x, min_y, cell;
    int    nx, ny;
    std::vector<char> blocked;   // 1=禁入(膨胀障碍内)，行优先 idx = ix*ny + iy

    inline int  idx(int ix, int iy) const { return ix * ny + iy; }
    inline bool in_bounds(int ix, int iy) const { return ix >= 0 && ix < nx && iy >= 0 && iy < ny; }
    inline double cx(int ix) const { return min_x + (ix + 0.5) * cell; }
    inline double cy(int iy) const { return min_y + (iy + 0.5) * cell; }
    inline void world_to_cell(double x, double y, int& ix, int& iy) const {
        ix = static_cast<int>(std::floor((x - min_x) / cell));
        iy = static_cast<int>(std::floor((y - min_y) / cell));
    }
};

// 点 (x,y) 是否落在某膨胀障碍内（禁入）。rr = robot_radius + inflate（障碍 r 单独加）。
bool point_blocked(double x, double y, const Obstacles& obs, double rr)
{
    for (const auto& o : obs) {
        const double dx = x - o.cx, dy = y - o.cy;
        if (dx * dx + dy * dy < (o.r + rr) * (o.r + rr)) return true;
    }
    return false;
}

// 线段 a→b 全程无碰撞（按 step 采样，每个采样点都不在禁入圆内）。
bool segment_free(const Vec2& a, const Vec2& b, const Obstacles& obs,
                  double rr, double step)
{
    const double L = std::hypot(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(std::ceil(L / std::max(step, 1e-3))));
    for (int k = 0; k <= n; ++k) {
        const double t = static_cast<double>(k) / n;
        const double px = a.x + t * (b.x - a.x);
        const double py = a.y + t * (b.y - a.y);
        if (point_blocked(px, py, obs, rr)) return false;
    }
    return true;
}

// 线段 a→b 全程离任一障碍【边缘】的最小余量（按 step 采样；边距 = 点到圆心距 − 障碍r）。
//   供 score_path 评"路径安全性"用：值越大，这段路离障碍越远越安全。obs 为空时返回 +inf。
double segment_min_clear(const Vec2& a, const Vec2& b, const Obstacles& obs, double step)
{
    const double L = std::hypot(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(std::ceil(L / std::max(step, 1e-3))));
    double mn = std::numeric_limits<double>::infinity();
    for (int k = 0; k <= n; ++k) {
        const double t = static_cast<double>(k) / n;
        const double px = a.x + t * (b.x - a.x);
        const double py = a.y + t * (b.y - a.y);
        for (const auto& o : obs) {
            const double d = std::hypot(px - o.cx, py - o.cy) - o.r;
            if (d < mn) mn = d;
        }
    }
    return mn;
}

// 视线串拉简化：把 A* 的逐格折线压成稀疏拐点（相邻拐点间直线可走），
// 首尾必留。让 Catmull-Rom 平滑出柔顺曲线、PD 沿线 carrot 跟随不抖。
Path2 simplify(const Path2& pts, const Obstacles& obs, double rr, double step)
{
    const int n = static_cast<int>(pts.size());
    if (n <= 2) return pts;
    Path2 out;
    out.push_back(pts[0]);
    int anchor = 0;
    for (int i = 2; i < n; ++i) {
        if (!segment_free(pts[anchor], pts[i], obs, rr, step)) {
            out.push_back(pts[i - 1]);   // anchor→i 撞了 → 保留上一个可直达点
            anchor = i - 1;
        }
    }
    if (out.empty() || std::hypot(out.back().x - pts[n - 1].x,
                                  out.back().y - pts[n - 1].y) > 1e-9)
        out.push_back(pts[n - 1]);
    return out;
}

}  // namespace

GlobalResult plan_global_path(const Vec2& start, const Vec2& goal,
                              const Obstacles& obs, const GlobalConfig& cfg,
                              double start_yaw)
{
    GlobalResult res;

    // ---- 建栅格 + 膨胀障碍 + 四面墙 ----
    //   ★热点★：本函数在【原地转身找解】期间每拍(50Hz)都跑，且持有节点 mtx_。
    //   旧写法是 nx*ny 全格 × 每格遍历所有障碍 = 50,000×N 次距离判定/次搜索。
    //   现改为两步"只算需要算的"，判定式与旧版逐字相同 → 结果逐位一致，只是不再做无用功：
    //     ① 墙：条件可分离(只看 ix 或只看 iy) → 预算两个一维布尔表 O(nx+ny)，
    //           再按列填充(列在内存中连续，memset 级)。
    //     ② 障碍：只盖各自【包围盒】内的格 O(Σ 盒面积)，盒外的格必然不满足距离判定。
    Grid g;
    g.min_x = cfg.min_x; g.min_y = cfg.min_y; g.cell = cfg.cell;
    g.nx = std::max(1, static_cast<int>(std::ceil((cfg.max_x - cfg.min_x) / cfg.cell)));
    g.ny = std::max(1, static_cast<int>(std::ceil((cfg.max_y - cfg.min_y) / cfg.cell)));
    g.blocked.assign(static_cast<size_t>(g.nx) * g.ny, 0);

    // ① 墙禁入：cell 中心距任一场地边界 < wall_margin。x 侧只与 ix 有关、y 侧只与 iy 有关。
    const double wm = cfg.wall_margin;   // 墙坐标已知(=场地边界)，直接画进栅格，不靠点云
    std::vector<char> wall_x(g.nx), wall_y(g.ny);
    for (int ix = 0; ix < g.nx; ++ix) {
        const double cx = g.cx(ix);
        wall_x[ix] = (cx - cfg.min_x < wm) || (cfg.max_x - cx < wm);
    }
    for (int iy = 0; iy < g.ny; ++iy) {
        const double cy = g.cy(iy);
        wall_y[iy] = (cy - cfg.min_y < wm) || (cfg.max_y - cy < wm);
    }
    for (int ix = 0; ix < g.nx; ++ix) {
        char* col = &g.blocked[static_cast<size_t>(ix) * g.ny];   // 第 ix 列在内存中连续
        if (wall_x[ix]) { std::fill(col, col + g.ny, static_cast<char>(1)); continue; }
        for (int iy = 0; iy < g.ny; ++iy) if (wall_y[iy]) col[iy] = 1;
    }

    // ② 障碍禁入：只遍历每个障碍的包围盒。盒取 ±1 格余量，确保不漏任何"中心距 < R"的格；
    //    盒内仍用与旧版【完全相同】的判定式 dx*dx+dy*dy < R*R，故结果逐位一致。
    const double rr = cfg.robot_radius + cfg.inflate;  // 禁入半径 = 障碍r + 此
    for (const auto& o : obs) {
        const double R = o.r + rr;
        // cell 中心 = min + (i+0.5)*cell ⇒ 由 |中心−圆心| ≤ R 反解 i 的范围
        int ix_lo = static_cast<int>(std::floor((o.cx - R - cfg.min_x) / cfg.cell)) - 1;
        int ix_hi = static_cast<int>(std::ceil ((o.cx + R - cfg.min_x) / cfg.cell)) + 1;
        int iy_lo = static_cast<int>(std::floor((o.cy - R - cfg.min_y) / cfg.cell)) - 1;
        int iy_hi = static_cast<int>(std::ceil ((o.cy + R - cfg.min_y) / cfg.cell)) + 1;
        ix_lo = std::max(ix_lo, 0);  ix_hi = std::min(ix_hi, g.nx - 1);
        iy_lo = std::max(iy_lo, 0);  iy_hi = std::min(iy_hi, g.ny - 1);
        const double R2 = R * R;
        for (int ix = ix_lo; ix <= ix_hi; ++ix) {
            const double dx = g.cx(ix) - o.cx;
            const double dx2 = dx * dx;
            if (dx2 >= R2) continue;                      // 整列都在圆外，跳过
            char* col = &g.blocked[static_cast<size_t>(ix) * g.ny];
            for (int iy = iy_lo; iy <= iy_hi; ++iy) {
                if (col[iy]) continue;                    // 已被墙/其他障碍禁入
                const double dy = g.cy(iy) - o.cy;
                if (dx2 + dy * dy < R2) col[iy] = 1;
            }
        }
    }

    // ---- 起点/终点定格（越界则钳进栅格） ----
    int sx, sy, gx, gy;
    g.world_to_cell(start.x, start.y, sx, sy);
    g.world_to_cell(goal.x,  goal.y,  gx, gy);
    sx = std::clamp(sx, 0, g.nx - 1); sy = std::clamp(sy, 0, g.ny - 1);
    gx = std::clamp(gx, 0, g.nx - 1); gy = std::clamp(gy, 0, g.ny - 1);

    const int start_idx = g.idx(sx, sy);
    res.start_blocked = g.blocked[start_idx];

    // 终点落在膨胀障碍内 → 环形外扩找最近可达格当搜索目标（真目标末尾再补，PD 精确逼近）。
    int goal_idx = g.idx(gx, gy);
    if (g.blocked[goal_idx]) {
        res.goal_blocked = true;
        bool found = false;
        const int max_ring = std::max(g.nx, g.ny);
        for (int rad = 1; rad <= max_ring && !found; ++rad) {
            for (int dx = -rad; dx <= rad && !found; ++dx)
                for (int dy = -rad; dy <= rad && !found; ++dy) {
                    if (std::max(std::abs(dx), std::abs(dy)) != rad) continue;  // 只看当前环
                    const int nxc = gx + dx, nyc = gy + dy;
                    if (!g.in_bounds(nxc, nyc)) continue;
                    if (!g.blocked[g.idx(nxc, nyc)]) { gx = nxc; gy = nyc; found = true; }
                }
        }
        if (!found) return res;   // 全场无可达格 → ok=false，节点回退 DWA
        goal_idx = g.idx(gx, gy);
    }

    // start/goal 视为可通行（即便起点贴障碍，也允许从这里向外扩展）。
    //   ★起点突围★：不只放行起点单格，还放行起点周围 relax 圈内的格。否则飞机被挤进
    //   "柱墙之间"窄区时，起点虽放行但 8 邻居全在禁入里 → 一步都迈不出 → 误判无解。
    //   relax = 飞机自身膨胀对应格数(ceil((robot+inflate)/cell))：只打开"飞机本就占据/紧邻"
    //   的一圈让它挪出去，走出这片即恢复正常禁入，不额外拓宽真实通道(障碍圆中心附近仍禁)。
    const int relax = std::max(1, static_cast<int>(std::ceil((cfg.robot_radius + cfg.inflate) / cfg.cell)));
    auto cell_blocked = [&](int ix, int iy) -> bool {
        if (!g.in_bounds(ix, iy)) return true;
        const int id = g.idx(ix, iy);
        if (id == start_idx || id == goal_idx) return false;
        // 起点 relax 邻域：放行，但障碍【圆心实体】附近不放行(防直接穿柱心)。
        if (std::abs(ix - sx) <= relax && std::abs(iy - sy) <= relax) {
            for (const auto& o : obs) {
                const double dx = g.cx(ix) - o.cx, dy = g.cy(iy) - o.cy;
                if (dx * dx + dy * dy < o.r * o.r) return true;   // 落在障碍实体圆内仍禁
            }
            return false;   // 否则放行(让飞机从窄区挪出第一步)
        }
        return g.blocked[id] != 0;
    };

    // ★机头锥★：让 A* 起点段只朝飞机机头 start_yaw 延伸——根除"新路从侧后方起步→飞机边转边走横切撞柱"。
    //   作用对象 = 被扩展到的【目标格相对起点的方位角】(非单跳方向)：起点是唯一种子，其余节点全部
    //   经此门才入队，故"起点 cone_radius 内的节点都落在 ±cone_half 楔形里"这一不变式自动成立；
    //   走出 cone_radius 后此门恒放行 → 恢复全向 8 邻 A*，不扭曲远处绕障路径(分辨率完备)。
    const bool   cone_active = std::isfinite(start_yaw) &&
                               cfg.head_cone_half   > 0.0 &&
                               cfg.head_cone_radius > 0.0;
    // ★近场放行半径★：栅格 8 邻方位是 45° 整数倍，±cone_half(如 22°→44°宽)楔形可能正好卡在两个栅格
    //   方向之间(如 yaw=22.5°，楔形[0.5°,44.5°]既不含 0° 也不含 45°)→起点 1 格邻居全被锥挡→假死锁。
    //   放行内圈 2 格(2*cell)：1 格邻居总能迈出，锥从 2 格外起咬(该处方位间隔≤26.6°<44°，楔形必含一格)。
    const double cone_inner  = 2.0 * g.cell;
    auto cone_block = [&](int ix, int iy) -> bool {
        if (!cone_active) return false;
        const double dxs = g.cx(ix) - start.x;
        const double dys = g.cy(iy) - start.y;
        const double d   = std::hypot(dxs, dys);
        if (d <= cone_inner || d > cfg.head_cone_radius) return false;  // 近场放行 / 远场恢复全向
        double da = std::atan2(dys, dxs) - start_yaw;
        while (da >  M_PI) da -= 2.0 * M_PI;
        while (da <= -M_PI) da += 2.0 * M_PI;
        return std::fabs(da) > cfg.head_cone_half;                      // 楔形外 → 禁入
    };

    // ---- A*（8 邻，欧氏启发，禁止贴角斜穿） ----
    const int N = g.nx * g.ny;
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> gscore(N, INF);
    std::vector<int>    came(N, -1);
    std::vector<char>   closed(N, 0);

    auto heur = [&](int ix, int iy) -> double {
        const double dx = g.cx(ix) - g.cx(gx), dy = g.cy(iy) - g.cy(gy);
        return std::hypot(dx, dy);
    };

    using QItem = std::pair<double, int>;  // (f, idx)
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
    gscore[start_idx] = 0.0;
    open.push({heur(sx, sy), start_idx});

    const double diag = std::sqrt(2.0) * g.cell;
    const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

    bool reached = false;
    while (!open.empty()) {
        const int cur = open.top().second; open.pop();
        if (closed[cur]) continue;
        closed[cur] = 1;
        if (cur == goal_idx) { reached = true; break; }

        const int cix = cur / g.ny, ciy = cur % g.ny;
        for (const auto& d : dirs) {
            const int nix = cix + d[0], niy = ciy + d[1];
            if (cell_blocked(nix, niy)) continue;
            if (cone_block(nix, niy)) continue;     // ★机头锥门★：起点段只放行朝机头楔形内的目标格
            if (d[0] != 0 && d[1] != 0) {           // 斜向：两正交格都通才允许（不贴角斜穿）
                if (cell_blocked(cix + d[0], ciy) || cell_blocked(cix, ciy + d[1])) continue;
            }
            const int nid = g.idx(nix, niy);
            if (closed[nid]) continue;
            const double step = (d[0] != 0 && d[1] != 0) ? diag : g.cell;
            const double ng = gscore[cur] + step;
            if (ng < gscore[nid]) {
                gscore[nid] = ng;
                came[nid] = cur;
                open.push({ng + heur(nix, niy), nid});
            }
        }
    }

    if (!reached) return res;   // 目标被围死/不连通 → ok=false

    // ---- 回溯逐格路径 → 世界点（首尾换成精确 start/goal） ----
    Path2 raw;
    for (int cur = goal_idx; cur != -1; cur = came[cur]) {
        const int cix = cur / g.ny, ciy = cur % g.ny;
        raw.push_back({g.cx(cix), g.cy(ciy)});
    }
    std::reverse(raw.begin(), raw.end());
    raw.front() = start;        // 精确起点(飞机当前位置)
    if (!res.goal_blocked) raw.back() = goal;
    else                   raw.push_back(goal);   // 真目标在障碍内：末尾补一段直奔(PD 尽力逼近)

    // ---- 串拉简化成稀疏拐点 ----
    res.path = simplify(raw, obs, rr, g.cell * 0.5);
    res.ok = true;
    return res;
}

bool path_clear(const Vec2& cur, const Path2& path,
                const Obstacles& obs, const GlobalConfig& cfg)
{
    if (path.size() < 2) return false;
    const double rr = cfg.robot_radius + cfg.inflate;
    const double step = cfg.cell * 0.5;

    // 找路径上离飞机当前位置最近的采样点，只校验"它之后"的剩余段
    //   （飞机已走过的前半段不必查；这样飞机沿途偏移也不会误判旧路径失效）。
    size_t ni = 0; double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.size(); ++i) {
        const double dx = path[i].x - cur.x, dy = path[i].y - cur.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; ni = i; }
    }

    // 飞机当前位置 → 最近点：先确认这一跳本身无碰撞（飞机可能偏离了旧线）
    if (!segment_free(cur, path[ni], obs, rr, step)) return false;
    // 剩余拐点逐段
    for (size_t i = ni; i + 1 < path.size(); ++i)
        if (!segment_free(path[i], path[i + 1], obs, rr, step)) return false;
    return true;
}

PathScore score_path(const Vec2& cur, const Path2& path,
                     const Obstacles& obs, const GlobalConfig& cfg)
{
    PathScore s;
    if (path.size() < 2) return s;        // length=0, min_clear=0
    const double step = cfg.cell * 0.5;

    // 找路径上离 cur 最近的点，只评估"它之后"的剩余段（与 path_clear 同口径：已走过的不算；
    //   候选路 gr.path 起点≈cur→最近点就是首点，旧路则从飞机当前所在处往后量——两条同起点可比）。
    size_t ni = 0; double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.size(); ++i) {
        const double dx = path[i].x - cur.x, dy = path[i].y - cur.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) { best = d2; ni = i; }
    }

    double len = 0.0;
    double mn  = std::numeric_limits<double>::infinity();
    auto accum = [&](const Vec2& a, const Vec2& b) {
        len += std::hypot(b.x - a.x, b.y - a.y);
        if (!obs.empty()) mn = std::min(mn, segment_min_clear(a, b, obs, step));
    };
    accum(cur, path[ni]);                                 // 飞机当前位置 → 最近点
    for (size_t i = ni; i + 1 < path.size(); ++i)         // 之后的剩余拐点逐段
        accum(path[i], path[i + 1]);

    s.length    = len;
    s.min_clear = obs.empty() ? 1e6 : mn;                 // 无障碍：安全性视为很大（不参与"更安全"门）
    return s;
}

}  // namespace exploration
