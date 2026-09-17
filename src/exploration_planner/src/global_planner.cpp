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

double segment_distance2(const Vec2& point, const Vec2& a, const Vec2& b)
{
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    const double t = length2 > 0.0
        ? std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / length2, 0.0, 1.0)
        : 0.0;
    const double ex = a.x + t * dx - point.x;
    const double ey = a.y + t * dy - point.y;
    return ex * ex + ey * ey;
}

// A start inside the extra margin may escape only with nondecreasing distance
// throughout this segment. Actual aircraft overlap is never an escape route.
bool segment_free(const Vec2& a, const Vec2& b, const Obstacles& obs,
                  double rr, double robot_radius)
{
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y)) return false;
    for (const auto& obstacle : obs) {
        const double ax = a.x - obstacle.cx, ay = a.y - obstacle.cy;
        const double start_distance2 = ax * ax + ay * ay;
        const double radius = obstacle.r + rr;
        const double physical_radius = obstacle.r + robot_radius;
        if (start_distance2 < radius * radius) {
            if (start_distance2 <= physical_radius * physical_radius) return false;
            const double radial_derivative = ax * (b.x - a.x) + ay * (b.y - a.y);
            if (radial_derivative < -1e-12) return false;
        } else if (segment_distance2({obstacle.cx, obstacle.cy}, a, b) < radius * radius) {
            return false;
        }
    }
    return true;
}

// Exact distance to obstacle surfaces, using the same continuous segments as
// collision validation. No sampling interval can hide a narrow intersection.
double segment_min_clear(const Vec2& a, const Vec2& b, const Obstacles& obs)
{
    double mn = std::numeric_limits<double>::infinity();
    for (const auto& o : obs) {
        const double d = std::sqrt(segment_distance2({o.cx, o.cy}, a, b)) - o.r;
        mn = std::min(mn, d);
    }
    return mn;
}

bool field_segment_free(const Vec2& a, const Vec2& b, const GlobalConfig& cfg)
{
    const double low[2]{cfg.min_x + cfg.wall_margin, cfg.min_y + cfg.wall_margin};
    const double high[2]{cfg.max_x - cfg.wall_margin, cfg.max_y - cfg.wall_margin};
    const double prior[2]{a.x, a.y}, values[2]{b.x, b.y};
    for (size_t axis = 0; axis < 2; ++axis) {
        if (!std::isfinite(prior[axis]) || !std::isfinite(values[axis]) || low[axis] > high[axis])
            return false;
        if (prior[axis] < low[axis]) {
            if (values[axis] < prior[axis] || values[axis] > high[axis]) return false;
        } else if (prior[axis] > high[axis]) {
            if (values[axis] > prior[axis] || values[axis] < low[axis]) return false;
        } else if (values[axis] < low[axis] || values[axis] > high[axis]) {
            return false;
        }
    }
    return true;
}

bool inside_field_margin(const Vec2& point, const GlobalConfig& cfg, double margin)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        point.x >= cfg.min_x + margin && point.x <= cfg.max_x - margin &&
        point.y >= cfg.min_y + margin && point.y <= cfg.max_y - margin;
}

bool required_config_valid(const GlobalConfig& cfg)
{
    return std::isfinite(cfg.min_x) && std::isfinite(cfg.min_y) &&
        std::isfinite(cfg.max_x) && std::isfinite(cfg.max_y) &&
        std::isfinite(cfg.cell) && cfg.cell > 0.0 &&
        std::isfinite(cfg.robot_radius) && cfg.robot_radius > 0.0 &&
        std::isfinite(cfg.inflate) && cfg.inflate >= 0.0 &&
        std::isfinite(cfg.wall_margin) && cfg.wall_margin >= cfg.robot_radius &&
        std::isfinite(required_field_margin(cfg)) && required_field_margin(cfg) >= 0.0 &&
        required_field_margin(cfg) <= cfg.wall_margin &&
        cfg.max_x - cfg.min_x > 2.0 * cfg.wall_margin &&
        cfg.max_y - cfg.min_y > 2.0 * cfg.wall_margin &&
        std::isfinite(cfg.required_connector_length) && cfg.required_connector_length > 0.0;
}

bool terminal_extension_anchor(const Vec2& point, const Vec2& goal,
                               const GlobalConfig& cfg, Vec2& anchor)
{
    const double low[2]{cfg.min_x + cfg.wall_margin, cfg.min_y + cfg.wall_margin};
    const double high[2]{cfg.max_x - cfg.wall_margin, cfg.max_y - cfg.wall_margin};
    const double origin[2]{goal.x, goal.y}, direction[2]{point.x - goal.x, point.y - goal.y};
    if (std::hypot(direction[0], direction[1]) < 1e-9) {
        anchor = {std::clamp(goal.x, low[0], high[0]), std::clamp(goal.y, low[1], high[1])};
        return true;
    }
    double near = 0.0, far = std::numeric_limits<double>::infinity();
    for (size_t axis = 0; axis < 2; ++axis) {
        if (std::fabs(direction[axis]) < 1e-12) {
            if (origin[axis] < low[axis] || origin[axis] > high[axis]) return false;
            continue;
        }
        double first = (low[axis] - origin[axis]) / direction[axis];
        double last = (high[axis] - origin[axis]) / direction[axis];
        if (first > last) std::swap(first, last);
        near = std::max(near, first);
        far = std::min(far, last);
        if (near > far) return false;
    }
    anchor = {std::clamp(goal.x + near * direction[0], low[0], high[0]),
              std::clamp(goal.y + near * direction[1], low[1], high[1])};
    return true;
}

bool required_field_shape(const Path2& path, const Vec2& goal, const GlobalConfig& cfg,
                          size_t& terminal_start)
{
    if (path.size() < 2 || !inside_field_margin(goal, cfg, required_field_margin(cfg)) ||
        std::hypot(path.back().x - goal.x, path.back().y - goal.y) > 1e-6) return false;
    terminal_start = path.size() - 1;
    if (path_inside_safe_field(path, cfg)) return true;
    if (inside_field_margin(goal, cfg, cfg.wall_margin)) return false;

    Vec2 anchor;
    bool found_inside = false;
    for (size_t i = path.size() - 1; i-- > 0;) {
        if (inside_field_margin(path[i], cfg, cfg.wall_margin)) {
            terminal_start = i;
            anchor = path[i];
            found_inside = true;
            break;
        }
    }
    if (found_inside) {
        const Path2 prefix(path.begin(), path.begin() + terminal_start + 1);
        if (!path_inside_safe_field(prefix, cfg)) return false;
    } else {
        terminal_start = 0;
        if (!terminal_extension_anchor(path.front(), goal, cfg, anchor)) return false;
    }
    const double dx = goal.x - anchor.x, dy = goal.y - anchor.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 > cfg.required_connector_length * cfg.required_connector_length + 1e-12 ||
        length2 < 1e-18) return false;
    double progress = -1e-9;
    for (size_t i = terminal_start; i < path.size(); ++i) {
        if (!inside_field_margin(path[i], cfg, required_field_margin(cfg)) ||
            segment_distance2(path[i], anchor, goal) > 1e-12) return false;
        const double along = ((path[i].x - anchor.x) * dx + (path[i].y - anchor.y) * dy) / length2;
        if (along < progress - 1e-9) return false;
        progress = along;
    }
    return true;
}

// Project onto continuous segments, keeping the earliest segment on ties so a
// crossing or return leg cannot skip still-pending path sections.
template<typename Visitor>
bool visit_remaining_segments(const Vec2& cur, const Path2& path, Visitor visit,
                              size_t* projected_segment = nullptr,
                              size_t forward_rejoin_before = 0)
{
    if (path.size() < 2) return false;

    size_t nearest_segment = 0;
    Vec2 projection = path.front();
    double best_distance2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Vec2& a = path[i];
        const Vec2& b = path[i + 1];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double length2 = dx * dx + dy * dy;
        const double u = length2 > 0.0
            ? std::clamp(((cur.x - a.x) * dx + (cur.y - a.y) * dy) / length2, 0.0, 1.0)
            : 0.0;
        const Vec2 candidate{a.x + u * dx, a.y + u * dy};
        const double ex = cur.x - candidate.x, ey = cur.y - candidate.y;
        const double distance2 = ex * ex + ey * ey;
        if (distance2 < best_distance2 - 1e-12) {
            best_distance2 = distance2;
            nearest_segment = i;
            projection = candidate;
        }
    }

    // Keep zero-length connections: even at the endpoint, the current position
    // must still be checked against a newly observed obstacle.
    if (projected_segment) *projected_segment = nearest_segment;
    size_t rejoin_end = nearest_segment + 1;
    if (nearest_segment < forward_rejoin_before) {
        // While entering the wall inset, the perpendicular projection can lie
        // behind the aircraft on a constrained axis. Even the next sample may
        // lie behind it: use the end of this straight leg, independently of
        // sampling density. Retain every bend and the terminal connector anchor.
        while (rejoin_end < forward_rejoin_before && rejoin_end + 1 < path.size()) {
            const Vec2& a = path[nearest_segment];
            const Vec2& b = path[rejoin_end + 1];
            if (segment_distance2(path[rejoin_end], a, b) > 1e-18) break;
            const double ax = path[rejoin_end].x - a.x;
            const double ay = path[rejoin_end].y - a.y;
            if (ax * (b.x - path[rejoin_end].x) + ay * (b.y - path[rejoin_end].y) < 0.0) break;
            ++rejoin_end;
        }
        if (!visit(cur, path[rejoin_end])) return false;
    } else if (!visit(cur, projection) || !visit(projection, path[nearest_segment + 1])) {
        return false;
    }
    for (size_t i = rejoin_end; i + 1 < path.size(); ++i) {
        if (!visit(path[i], path[i + 1])) return false;
    }
    return true;
}

// 视线串拉简化：把 A* 的逐格折线压成稀疏拐点（相邻拐点间直线可走），
// 首尾必留。让 Catmull-Rom 平滑出柔顺曲线、PD 沿线 carrot 跟随不抖。
Path2 simplify(const Path2& pts, const Obstacles& obs, double rr, double robot_radius)
{
    const int n = static_cast<int>(pts.size());
    if (n <= 2) return pts;
    Path2 out;
    out.push_back(pts[0]);
    int anchor = 0;
    for (int i = 2; i < n; ++i) {
        if (!segment_free(pts[anchor], pts[i], obs, rr, robot_radius)) {
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

static GlobalResult plan_path(const Vec2& start, const Vec2& goal,
                              const Obstacles& obs, GlobalConfig cfg,
                              double start_yaw, bool required_goal)
{
    GlobalResult res;
    if (required_goal) {
        cfg.validate_complete_path = true;
        if (!required_config_valid(cfg)) return res;
        if (!inside_field_margin(goal, cfg, required_field_margin(cfg)) ||
            point_blocked(goal.x, goal.y, obs, cfg.robot_radius + cfg.inflate)) {
            res.goal_blocked = true;
            return res;
        }
    }
    if (cfg.validate_complete_path &&
        (!std::isfinite(start.x) || !std::isfinite(start.y) ||
         !std::isfinite(goal.x) || !std::isfinite(goal.y) ||
         !segment_free(start, start, obs, cfg.robot_radius + cfg.inflate, cfg.robot_radius)))
        return res;

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
    if (cfg.validate_complete_path) {
        // Centre a search cell on the exact current pose. At a virtual wall
        // inset, a fixed half-cell offset can make every first edge approach
        // an obstacle even when a tangential escape exists. Physical bounds
        // remain cfg's bounds and are checked independently on every edge.
        const int ix = std::clamp(static_cast<int>(std::floor((start.x - cfg.min_x) / g.cell)), 0, g.nx - 1);
        const int iy = std::clamp(static_cast<int>(std::floor((start.y - cfg.min_y) / g.cell)), 0, g.ny - 1);
        g.min_x = start.x - (ix + .5) * g.cell;
        g.min_y = start.y - (iy + .5) * g.cell;
    }
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
    int goal_idx = required_goal ? g.nx * g.ny : g.idx(gx, gy);
    const bool exact_goal_blocked = cfg.validate_complete_path &&
        (point_blocked(goal.x, goal.y, obs, rr) || !path_inside_safe_field({goal}, cfg));
    if (!required_goal && (g.blocked[goal_idx] || exact_goal_blocked)) {
        res.goal_blocked = true;
        bool found = false;
        if (cfg.validate_complete_path) {
            double nearest2 = std::numeric_limits<double>::infinity();
            for (int ix = 0; ix < g.nx; ++ix) {
                for (int iy = 0; iy < g.ny; ++iy) {
                    if (g.blocked[g.idx(ix, iy)]) continue;
                    const double dx = g.cx(ix) - goal.x, dy = g.cy(iy) - goal.y;
                    const double distance2 = dx * dx + dy * dy;
                    if (distance2 < nearest2) {
                        nearest2 = distance2;
                        gx = ix; gy = iy; found = true;
                    }
                }
            }
        } else {
            const int max_ring = std::max(g.nx, g.ny);
            for (int rad = 1; rad <= max_ring && !found; ++rad) {
                for (int dx = -rad; dx <= rad && !found; ++dx)
                    for (int dy = -rad; dy <= rad && !found; ++dy) {
                        if (std::max(std::abs(dx), std::abs(dy)) != rad) continue;
                        const int nxc = gx + dx, nyc = gy + dy;
                        if (!g.in_bounds(nxc, nyc)) continue;
                        if (!g.blocked[g.idx(nxc, nyc)]) { gx = nxc; gy = nyc; found = true; }
                    }
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
        if (cfg.validate_complete_path && !g.blocked[id]) return false;
        // Strict search already checks every edge for monotonic recovery.
        // Do not truncate a safe field-entry detour to the old start-cell box.
        if (cfg.validate_complete_path ||
            (std::abs(ix - sx) <= relax && std::abs(iy - sy) <= relax)) {
            for (const auto& o : obs) {
                const double dx = g.cx(ix) - o.cx, dy = g.cy(iy) - o.cy;
                if (cfg.validate_complete_path) {
                    const double ax = start.x - o.cx, ay = start.y - o.cy;
                    const double physical = o.r + cfg.robot_radius;
                    const double inflated = o.r + rr;
                    const double minimum2 = std::max(physical * physical,
                        std::min(ax * ax + ay * ay, inflated * inflated));
                    if (dx * dx + dy * dy < minimum2) return true;
                } else if (dx * dx + dy * dy < o.r * o.r) return true;
            }
            if (cfg.validate_complete_path &&
                !field_segment_free(start, {g.cx(ix), g.cy(iy)}, cfg)) return true;
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
    auto connector_cone_clear = [&](const Vec2& a, const Vec2& b) {
        if (!cone_active) return true;
        const int steps = std::max(1, static_cast<int>(std::ceil(
            std::hypot(b.x - a.x, b.y - a.y) / (0.5 * g.cell))));
        for (int i = 1; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            const double dx = a.x + t * (b.x - a.x) - start.x;
            const double dy = a.y + t * (b.y - a.y) - start.y;
            const double radius = std::hypot(dx, dy);
            if (radius <= cone_inner || radius > cfg.head_cone_radius) continue;
            const double error = std::atan2(std::sin(std::atan2(dy, dx) - start_yaw),
                                           std::cos(std::atan2(dy, dx) - start_yaw));
            if (std::fabs(error) > cfg.head_cone_half) return false;
        }
        return true;
    };

    // ---- A*（8 邻，欧氏启发，禁止贴角斜穿） ----
    const int N = g.nx * g.ny + (required_goal ? 1 : 0);
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> gscore(N, INF);
    std::vector<int>    came(N, -1);
    std::vector<char>   closed(N, 0);

    auto heur = [&](int ix, int iy) -> double {
        if (required_goal) {
            const Vec2 point = g.idx(ix, iy) == start_idx ? start : Vec2{g.cx(ix), g.cy(iy)};
            return std::hypot(point.x - goal.x, point.y - goal.y);
        }
        const double dx = g.cx(ix) - g.cx(gx), dy = g.cy(iy) - g.cy(gy);
        return std::hypot(dx, dy);
    };

    using QItem = std::pair<double, int>;  // (f, idx)
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
    gscore[start_idx] = 0.0;
    open.push({heur(sx, sy), start_idx});

    const double diag = std::sqrt(2.0) * g.cell;
    const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const Vec2 final_goal = res.goal_blocked ? Vec2{g.cx(gx), g.cy(gy)} : goal;
    auto exact_point = [&](int ix, int iy) {
        const int id = g.idx(ix, iy);
        if (id == start_idx) return start;
        if (id == goal_idx) return final_goal;
        return Vec2{g.cx(ix), g.cy(iy)};
    };

    bool reached = false;
    while (!open.empty()) {
        const int cur = open.top().second; open.pop();
        if (closed[cur]) continue;
        closed[cur] = 1;
        if (cur == goal_idx) { reached = true; break; }

        const int cix = cur / g.ny, ciy = cur % g.ny;
        if (required_goal) {
            const Vec2 point = exact_point(cix, ciy);
            const double distance = std::hypot(point.x - goal.x, point.y - goal.y);
            size_t terminal_start = 0;
            // The virtual goal can only be reached by a verified final connector;
            // normal grid expansion keeps the unchanged full field inset.
            if (distance <= cfg.required_connector_length + 1e-9 &&
                (cur == start_idx || inside_field_margin(point, cfg, cfg.wall_margin)) &&
                required_field_shape({point, goal}, goal, cfg, terminal_start) &&
                connector_cone_clear(point, goal) &&
                segment_free(point, goal, obs, rr, cfg.robot_radius)) {
                const double ng = gscore[cur] + distance;
                if (ng < gscore[goal_idx]) {
                    gscore[goal_idx] = ng;
                    came[goal_idx] = cur;
                    open.push({ng, goal_idx});
                }
            }
        }
        for (const auto& d : dirs) {
            const int nix = cix + d[0], niy = ciy + d[1];
            if (cell_blocked(nix, niy)) continue;
            if (cone_block(nix, niy)) continue;     // ★机头锥门★：起点段只放行朝机头楔形内的目标格
            if (d[0] != 0 && d[1] != 0) {           // 斜向：两正交格都通才允许（不贴角斜穿）
                if (cell_blocked(cix + d[0], ciy) || cell_blocked(cix, ciy + d[1])) continue;
            }
            const int nid = g.idx(nix, niy);
            if (closed[nid]) continue;
            if (cfg.validate_complete_path) {
                const Vec2 a = exact_point(cix, ciy), b = exact_point(nix, niy);
                if (!segment_free(a, b, obs, rr, cfg.robot_radius) ||
                    !field_segment_free(a, b, cfg)) continue;
            }
            const Vec2 a = exact_point(cix, ciy), b = exact_point(nix, niy);
            const double step = required_goal ? std::hypot(a.x - b.x, a.y - b.y)
                : ((d[0] != 0 && d[1] != 0) ? diag : g.cell);
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
        if (required_goal && cur == goal_idx) {
            raw.push_back(goal);
            continue;
        }
        const int cix = cur / g.ny, ciy = cur % g.ny;
        raw.push_back({g.cx(cix), g.cy(ciy)});
    }
    std::reverse(raw.begin(), raw.end());
    raw.front() = start;        // 精确起点(飞机当前位置)
    if (cfg.validate_complete_path) {
        if (raw.size() == 1) raw.push_back(final_goal);
        else raw.back() = final_goal;
    } else if (!res.goal_blocked) raw.back() = goal;
    else raw.push_back(goal);   // 默认必达点模式保留真目标。

    // ---- 串拉简化成稀疏拐点 ----
    if (required_goal) {
        // Preserve the connector anchor so string pulling cannot enlarge the
        // locally authorized exception to the virtual boundary margin.
        const Path2 prefix(raw.begin(), raw.end() - 1);
        res.path = simplify(prefix, obs, rr, cfg.robot_radius);
        res.path.push_back(goal);
        if (!required_path_clear(start, res.path, goal, obs, cfg)) {
            res.path.clear();
            return res;
        }
        res.ok = true;
        return res;
    }
    // Keep geometric guide vertices sparse. Heading continuity is handled by
    // the validated start curve, not by retaining a near-origin grid corner.
    res.path = simplify(raw, obs, rr, cfg.robot_radius);
    if (cfg.validate_complete_path &&
        (!path_clear(start, res.path, obs, cfg) || !path_inside_safe_field(res.path, cfg))) {
        res.path.clear();
        return res;
    }
    res.ok = true;
    return res;
}

GlobalResult plan_global_path(const Vec2& start, const Vec2& goal,
                              const Obstacles& obs, const GlobalConfig& cfg,
                              double start_yaw)
{
    return plan_path(start, goal, obs, cfg, start_yaw, false);
}

GlobalResult plan_required_path(const Vec2& start, const Vec2& goal,
                                const Obstacles& obs, const GlobalConfig& cfg,
                                double start_yaw)
{
    return plan_path(start, goal, obs, cfg, start_yaw, true);
}

bool required_path_clear(const Vec2& cur, const Path2& path, const Vec2& goal,
                         const Obstacles& obs, const GlobalConfig& cfg)
{
    if (!required_config_valid(cfg) || !std::isfinite(cur.x) || !std::isfinite(cur.y) ||
        point_blocked(goal.x, goal.y, obs, cfg.robot_radius + cfg.inflate)) return false;
    size_t terminal_start = 0;
    if (!required_field_shape(path, goal, cfg, terminal_start)) return false;
    size_t projected_segment = 0;
    bool first = true;
    return visit_remaining_segments(cur, path, [&](const Vec2& a, const Vec2& b) {
        if (first) {
            first = false;
            const bool terminal_rejoin = projected_segment >= terminal_start &&
                terminal_start + 1 < path.size() &&
                (!inside_field_margin(a, cfg, cfg.wall_margin) ||
                 !inside_field_margin(b, cfg, cfg.wall_margin));
            // Progress can reach the anchor while the aircraft is still just
            // before it. A normal inward rejoin needs no terminal exception.
            const bool normal_rejoin = field_segment_free(a, b, cfg) &&
                (!terminal_rejoin || inside_field_margin(b, cfg, cfg.wall_margin));
            if (!normal_rejoin) {
                if (!terminal_rejoin || !inside_field_margin(a, cfg, required_field_margin(cfg)) ||
                    std::hypot(a.x - goal.x, a.y - goal.y) > cfg.required_connector_length + 1e-9)
                    return false;
            }
        }
        return segment_free(a, b, obs, cfg.robot_radius + cfg.inflate, cfg.robot_radius);
    }, &projected_segment, terminal_start);
}

bool obstacle_segment_clear(const Vec2& start, const Vec2& end,
                             const Obstacles& obs, const GlobalConfig& cfg)
{
    if (!std::isfinite(cfg.robot_radius) || cfg.robot_radius <= 0.0 ||
        !std::isfinite(cfg.inflate) || cfg.inflate < 0.0) return false;
    return segment_free(start, end, obs, cfg.robot_radius + cfg.inflate, cfg.robot_radius);
}

bool path_clear(const Vec2& cur, const Path2& path,
                const Obstacles& obs, const GlobalConfig& cfg)
{
    const double rr = cfg.robot_radius + cfg.inflate;
    if (path.size() < 2 || point_blocked(path.back().x, path.back().y, obs, rr)) return false;
    return visit_remaining_segments(cur, path, [&](const Vec2& a, const Vec2& b) {
        return segment_free(a, b, obs, rr, cfg.robot_radius);
    });
}

bool path_inside_safe_field(const Path2& path, const GlobalConfig& cfg)
{
    if (path.empty()) return false;
    Vec2 previous = path.front();
    for (const Vec2& point : path) {
        if (!field_segment_free(previous, point, cfg)) return false;
        previous = point;
    }
    return previous.x >= cfg.min_x + cfg.wall_margin && previous.x <= cfg.max_x - cfg.wall_margin &&
           previous.y >= cfg.min_y + cfg.wall_margin && previous.y <= cfg.max_y - cfg.wall_margin;
}

PathScore score_path(const Vec2& cur, const Path2& path,
                     const Obstacles& obs, const GlobalConfig&)
{
    PathScore s;
    if (path.size() < 2) return s;        // length=0, min_clear=0
    double len = 0.0;
    double mn  = std::numeric_limits<double>::infinity();
    visit_remaining_segments(cur, path, [&](const Vec2& a, const Vec2& b) {
        len += std::hypot(b.x - a.x, b.y - a.y);
        if (!obs.empty()) mn = std::min(mn, segment_min_clear(a, b, obs));
        return true;
    });

    s.length    = len;
    s.min_clear = obs.empty() ? 1e6 : mn;                 // 无障碍：安全性视为很大（不参与"更安全"门）
    return s;
}

}  // namespace exploration
