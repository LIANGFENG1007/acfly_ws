#include "exploration_planner/coverage_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace exploration {

namespace {
inline double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}
inline double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

Path2 plan_boustrophedon(const CoverageConfig& cfg, const Vec2& start, const Vec2& goal)
{
    Path2 path;

    // 车道在 y 方向的可用范围（留边）
    const double y_lo = cfg.min_y + cfg.margin;
    const double y_hi = cfg.max_y - cfg.margin;
    const double x_lo = cfg.min_x + cfg.margin;
    const double x_hi = cfg.max_x - cfg.margin;

    if (y_hi <= y_lo || x_hi <= x_lo) {
        // 场地太小，直接起点→终点
        path.push_back(start);
        path.push_back(goal);
        return path;
    }

    // 车道 y 坐标：从 y_lo 起每 lane_spacing 一条，覆盖到 y_hi
    std::vector<double> lane_y;
    for (double y = y_lo; y <= y_hi + 1e-6; y += cfg.lane_spacing) {
        lane_y.push_back(std::min(y, y_hi));
    }
    if (lane_y.empty() || std::fabs(lane_y.back() - y_hi) > 1e-6) {
        lane_y.push_back(y_hi);   // 保证最后一条车道压到上边界
    }

    const int n_lane = static_cast<int>(lane_y.size());

    // 起始车道：选离起点 y 更近的一端（从下往上 or 从上往下）
    const bool start_from_bottom =
        std::fabs(start.y - lane_y.front()) <= std::fabs(start.y - lane_y.back());

    // 蛇形每条车道的 x 走向：选第一条车道先朝离起点 x 远的方向，
    // 让整体形成连续蛇形。这里用起点 x 在场地中点的哪侧决定首条朝向。
    const double x_mid = 0.5 * (x_lo + x_hi);
    bool go_right = (start.x <= x_mid);   // 起点偏左 → 先向右扫

    path.push_back(start);

    for (int k = 0; k < n_lane; ++k) {
        // 实际车道顺序：start_from_bottom 决定从 front 还是 back 开始
        const int li = start_from_bottom ? k : (n_lane - 1 - k);
        const double y = lane_y[li];

        const double x_first = go_right ? x_lo : x_hi;
        const double x_second = go_right ? x_hi : x_lo;

        // 进入本车道：先到车道近端（保持 x，换 y），再扫到远端
        path.push_back({x_first, y});
        path.push_back({x_second, y});

        go_right = !go_right;   // 下一条车道反向，形成蛇形
    }

    // 末尾接终点
    path.push_back(goal);

    return path;
}

Path2 plan_explore(const GridSnapshot& grid, const FrontierConfig& cfg,
                   const Vec2& cur, double cur_yaw, int& cur_band, bool& all_explored,
                   const std::vector<Vec2>* unreachable, double block_r)
{
    all_explored = false;

    // Give-up locations are executable waypoints, after field-margin clamping.
    const double br2 = block_r * block_r;
    auto blacklisted = [&](const Vec2& p) {
        if (!unreachable || block_r <= 0.0) return false;
        for (const auto& u : *unreachable) {
            const double dx = p.x - u.x, dy = p.y - u.y;
            if (dx * dx + dy * dy <= br2) return true;
        }
        return false;
    };

    const int NX = grid.big_nx();
    const int NY = grid.big_ny();
    const GridConfig& gc = grid.config();

    // 飞机可达安全区（离墙 margin）
    const double x_lo = cfg.min_x + cfg.margin, x_hi = cfg.max_x - cfg.margin;
    const double y_lo = cfg.min_y + cfg.margin, y_hi = cfg.max_y - cfg.margin;
    auto reachable_waypoint = [&](const Vec2& point) {
        return Vec2{clampd(point.x, x_lo, x_hi), clampd(point.y, y_lo, y_hi)};
    };

    // 收集未探索大格：中心坐标 + 8 邻域里"也未扫"的邻居数（衡量是否成片）+ 所属条带
    struct Cell { Vec2 c; Vec2 waypoint; int nbr; int band; };
    std::vector<Cell> un;
    un.reserve(static_cast<size_t>(NX) * NY);

    const double bw = std::max(cfg.band_width, 1e-3);
    auto band_of = [&](double y) {
        int b = static_cast<int>(std::floor((y - y_lo) / bw));
        return b < 0 ? 0 : b;
    };
    const int n_band = std::max(1, band_of(y_hi) + 1);

    for (int bi = 0; bi < NX; ++bi) {
        for (int bj = 0; bj < NY; ++bj) {
            if (grid.big_explored(bi, bj)) continue;
            Vec2 c{ gc.min_x + (bi + 0.5) * gc.big_cell,
                    gc.min_y + (bj + 0.5) * gc.big_cell };
            int nbr = 0;
            for (int di = -1; di <= 1; ++di) {
                for (int dj = -1; dj <= 1; ++dj) {
                    if (di == 0 && dj == 0) continue;
                    const int ni = bi + di, nj = bj + dj;
                    if (ni < 0 || ni >= NX || nj < 0 || nj >= NY) continue;
                    if (!grid.big_explored(ni, nj)) ++nbr;
                }
            }
            // ★nbr 饱和(2026-08)★：解决"障碍遮挡出的小片未扫区，飞机走很远后才回来补扫"。
            //   成因(孤格饥饿)：遮挡区在飞机路过时被障碍挡住扫不到 → 周围格陆续扫完 →
            //   它的 nbr 从 8 掉到 1~2 → 代价里 -cluster_weight*nbr 这一项从 -14.4 变成 -1.8，
            //   代价【跳升 12.6】(≈要多绕 9m 才划算) → 排名从队首暴跌到队尾 → 再没机会被选中，
            //   直到全场只剩它们才被迫回头 —— 实测有格 t=0 就看见、拖到 47s 才扫完。
            //   对策：cluster 加成【饱和】——nbr 超过 CAP 后不再线性增长，大片区不会把
            //   零星孤格彻底压死；孤格与大片区的代价差从 12.6 收窄到约 9.0，仍优先大片区
            //   (不破坏"别追孤格"的原意)，但孤格不再被无限延后。
            //   实测(7 场景 × 4 起飞点 = 28 组)：CAP=6 总用时 -9.2%、里程 -8.9%、
            //   滞后>10s 的格 1160→908、最大滞后 61.9s→58.8s，零超时。
            //   ★CAP 不可调太小★：CAP=4 有 2 组超时(+44.5%) —— 压得太狠会让飞机放弃
            //   "优先扫大片"的策略、在零散格之间乱跳。CAP≥8 等于关闭(8 邻域上限就是 8)。
            if (cfg.cluster_cap > 0) nbr = std::min(nbr, cfg.cluster_cap);
            un.push_back({c, reachable_waypoint(c), nbr, band_of(c.y)});
        }
    }

    Path2 wp;
    wp.push_back(cur);

    if (un.empty()) { all_explored = true; return wp; }

    // ---- 上层：推进当前条带（迟滞，不回退）----
    // 当前条带内剩余未扫格 ≤ band_clear_cnt 就推进到"下一个还有未扫格的条带"。
    //
    // ★"不回退"的真实作用范围（勿按字面理解）★：cur_band 由调用方持有，但每次
    //   replan 传进来的若是负值(首次 -1 / 上次收尾置的 -2)，这里都会重新取
    //   band_of(飞机当前 y)。所以：
    //     · 推进阶段(cur_band ≥ 0)：确实单调向上，不回退、不横跳 —— 迟滞在这里生效。
    //     · 收尾阶段(上一拍置了 -2)：下一拍即被重置成"飞机当前所在带"，等价于
    //       允许回到下方条带捡残格。★这是有意保留的行为★——残格常散落在已过条带，
    //       不许回头就够不到，覆盖率会卡在 DONE_COVERAGE 以下永远无法转归航。
    if (cur_band < 0) cur_band = band_of(cur.y);          // 首次(-1) 或 上拍收尾(-2)：用当前所在条带
    cur_band = std::clamp(cur_band, 0, n_band - 1);

    auto count_in_band = [&](int b) {
        int n = 0;
        for (const auto& u : un) if (u.band == b) ++n;
        return n;
    };
    // 若当前条带已基本扫完，向上找下一个仍有未扫格的条带
    for (int safety = 0; safety < n_band; ++safety) {
        if (count_in_band(cur_band) > cfg.band_clear_cnt) break;
        // 找 > cur_band 中最近的、有未扫格的条带
        int nxt = -1;
        for (int b = cur_band + 1; b < n_band; ++b) {
            if (count_in_band(b) > 0) { nxt = b; break; }
        }
        if (nxt < 0) {
            // 上方没有了：可能还有零星残格散落在已过条带 → 退而求其次，扫全场剩余。
            //   -2 只在【本次调用内】生效(下面 in_active_band 恒真=无条带约束)；
            //   下一拍会被上面 `cur_band < 0` 重置成飞机当前所在带 —— 见那里的说明，
            //   这正是收尾阶段能回下方条带捡残格的原因，有意为之。
            cur_band = -2;   // 标记"无条带约束，扫全场残格"(仅本次调用)
            break;
        }
        cur_band = nxt;
    }

    // ---- 下层：在当前条带(±band_tol)内选未扫前沿贪心串链 ----
    // cur_band==-2 表示无条带约束（收尾扫残格）。
    // 条带主方向蛇形交替：偶数带朝 +x、奇数带朝 -x，扫完一条带后在同侧衔接下一带，
    // 不用横跨整场折返回起点 → 大幅减少大角度掉头。收尾(-2)固定 +x。
    const double main_dir = (cur_band >= 0 && (cur_band % 2 == 1)) ? -1.0 : +1.0;

    std::vector<char> used(un.size(), 0);
    // 先把"不在当前条带"的格临时排除（cur_band==-2 时不排除）
    auto in_active_band = [&](const Cell& u) {
        if (cur_band == -2) return true;
        const double yb_lo = y_lo + cur_band * bw - cfg.band_tol;
        const double yb_hi = y_lo + (cur_band + 1) * bw + cfg.band_tol;
        return u.c.y >= yb_lo && u.c.y <= yb_hi;
    };

    Vec2 from = cur;
    double head = cur_yaw;
    double acc = 0.0;
    int guard = 0;
    while (acc < cfg.horizon && static_cast<int>(wp.size()) < 24 && guard++ < 80) {
        int best = -1; double best_cost = 1e18;
        for (size_t i = 0; i < un.size(); ++i) {
            if (used[i]) continue;
            if (!in_active_band(un[i])) continue;
            if (blacklisted(un[i].waypoint)) continue;
            const double dx = un[i].c.x - from.x, dy = un[i].c.y - from.y;
            const double d = std::hypot(dx, dy);
            const double ang = std::fabs(wrap_pi(std::atan2(dy, dx) - head));
            // 沿条带主方向(x)的推进量：鼓励顺着横扫；只奖励正向推进
            const double along = std::max(0.0, main_dir * dx);
            const double cost = cfg.near_weight * d + cfg.turn_penalty * ang
                              - cfg.cluster_weight * un[i].nbr
                              - cfg.along_bonus * along;
            if (cost < best_cost) { best_cost = cost; best = static_cast<int>(i); }
        }
        if (best < 0) break;

        const Vec2 w = un[best].waypoint;
        if (std::hypot(w.x - wp.back().x, w.y - wp.back().y) > 1e-3) wp.push_back(w);

        acc += std::hypot(w.x - from.x, w.y - from.y);
        head = std::atan2(w.y - from.y, w.x - from.x);
        from = w;

        // 标记 chain_gap 内的未扫格已用，避免下一步选到太近的点
        for (size_t i = 0; i < un.size(); ++i) {
            if (used[i]) continue;
            if (std::hypot(un[i].c.x - un[best].c.x, un[i].c.y - un[best].c.y) <= cfg.chain_gap)
                used[i] = 1;
        }
    }

    // 至少给一个目标点（极端情况下保证 tracker 有得跟）：取整场最近未扫格
    if (wp.size() < 2) {
        int best = -1; double bd = 1e18;
        for (size_t i = 0; i < un.size(); ++i) {
            if (blacklisted(un[i].waypoint)) continue;
            const double d = std::hypot(un[i].c.x - cur.x, un[i].c.y - cur.y);
            if (d < bd) { bd = d; best = static_cast<int>(i); }
        }
        if (best >= 0)
            wp.push_back(un[best].waypoint);
    }
    return wp;
}

namespace {
struct UnknownCell {
    Vec2 center;
    std::vector<Vec2> samples;
    int x = 0, y = 0, neighbors = 0, component = -1;
};

struct UnknownView {
    std::vector<UnknownCell> cells;
    std::vector<int> component_sizes;
};

UnknownView prepare_unknown_view(const GridSnapshot& grid)
{
    UnknownView view;
    const int nx = grid.big_nx(), ny = grid.big_ny();
    if (nx <= 0 || ny <= 0 || grid.cfg.big_cell <= 0.0 ||
        grid.big.size() < static_cast<size_t>(nx) * ny) return view;
    std::vector<int> lookup(static_cast<size_t>(nx) * ny, -1);
    const bool have_small = grid.snx > 0 && grid.sny > 0 && grid.cpb > 0 &&
        grid.cfg.small_cell > 0.0 &&
        grid.small.size() >= static_cast<size_t>(grid.snx) * grid.sny;
    const double quarter = grid.cfg.big_cell * 0.25;
    for (int x = 0; x < nx; ++x) {
        for (int y = 0; y < ny; ++y) {
            if (grid.big_explored(x, y)) continue;
            UnknownCell cell;
            cell.x = x; cell.y = y;
            cell.center = {grid.cfg.min_x + (x + 0.5) * grid.cfg.big_cell,
                           grid.cfg.min_y + (y + 0.5) * grid.cfg.big_cell};
            if (!have_small) {
                cell.samples.push_back(cell.center);
            } else {
                // Retain up to five actual unscanned small cells, close to the
                // center and four quadrants. Already observed parts of a nearly
                // complete big cell cannot contribute imaginary information.
                const Vec2 desired[] = {
                    cell.center,
                    {cell.center.x - quarter, cell.center.y - quarter},
                    {cell.center.x - quarter, cell.center.y + quarter},
                    {cell.center.x + quarter, cell.center.y - quarter},
                    {cell.center.x + quarter, cell.center.y + quarter}};
                double best_d2[5];
                Vec2 best[5]{};
                std::fill(std::begin(best_d2), std::end(best_d2),
                          std::numeric_limits<double>::infinity());
                for (int sx = x * grid.cpb; sx < std::min((x + 1) * grid.cpb, grid.snx); ++sx) {
                    for (int sy = y * grid.cpb; sy < std::min((y + 1) * grid.cpb, grid.sny); ++sy) {
                        if (grid.small_scanned(sx, sy)) continue;
                        const Vec2 p{grid.cfg.min_x + (sx + 0.5) * grid.cfg.small_cell,
                                     grid.cfg.min_y + (sy + 0.5) * grid.cfg.small_cell};
                        for (int k = 0; k < 5; ++k) {
                            const double dx = p.x - desired[k].x, dy = p.y - desired[k].y;
                            const double d2 = dx * dx + dy * dy;
                            if (d2 < best_d2[k]) { best_d2[k] = d2; best[k] = p; }
                        }
                    }
                }
                for (int k = 0; k < 5; ++k) {
                    if (!std::isfinite(best_d2[k])) continue;
                    bool duplicate = false;
                    for (const auto& p : cell.samples)
                        if (std::hypot(p.x - best[k].x, p.y - best[k].y) < 1e-9) duplicate = true;
                    if (!duplicate) cell.samples.push_back(best[k]);
                }
            }
            lookup[static_cast<size_t>(x) * ny + y] = static_cast<int>(view.cells.size());
            view.cells.push_back(std::move(cell));
        }
    }
    for (auto& cell : view.cells) {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if ((dx == 0 && dy == 0) || cell.x + dx < 0 || cell.x + dx >= nx ||
                    cell.y + dy < 0 || cell.y + dy >= ny) continue;
                if (lookup[static_cast<size_t>(cell.x + dx) * ny + cell.y + dy] >= 0)
                    ++cell.neighbors;
            }
        }
    }
    std::vector<int> queue;
    for (size_t i = 0; i < view.cells.size(); ++i) {
        if (view.cells[i].component >= 0) continue;
        const int component = static_cast<int>(view.component_sizes.size());
        queue.clear();
        queue.push_back(static_cast<int>(i));
        view.cells[i].component = component;
        for (size_t q = 0; q < queue.size(); ++q) {
            const UnknownCell& cell = view.cells[queue[q]];
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if ((dx == 0 && dy == 0) || cell.x + dx < 0 || cell.x + dx >= nx ||
                        cell.y + dy < 0 || cell.y + dy >= ny) continue;
                    const int neighbor = lookup[static_cast<size_t>(cell.x + dx) * ny + cell.y + dy];
                    if (neighbor >= 0 && view.cells[neighbor].component < 0) {
                        view.cells[neighbor].component = component;
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        view.component_sizes.push_back(static_cast<int>(queue.size()));
    }
    return view;
}

bool clear_observation_ray(const Vec2& from, const Vec2& to, const Obstacles& obstacles)
{
    const double dx = to.x - from.x, dy = to.y - from.y;
    const double length2 = dx * dx + dy * dy;
    for (const auto& obstacle : obstacles) {
        if (obstacle.r <= 0.0) continue;
        const double t = length2 > 1e-12
            ? std::clamp(((obstacle.cx - from.x) * dx +
                          (obstacle.cy - from.y) * dy) / length2, 0.0, 1.0) : 0.0;
        const double ox = from.x + t * dx - obstacle.cx;
        const double oy = from.y + t * dy - obstacle.cy;
        if (ox * ox + oy * oy < obstacle.r * obstacle.r) return false;
    }
    return true;
}

bool cell_visible(const UnknownCell& cell, const GridConfig& grid_config,
                  const Vec2& point, double heading, const Obstacles& obstacles)
{
    if (!std::isfinite(heading) || grid_config.fov_range <= 0.0 || grid_config.fov_deg <= 0.0)
        return false;
    const double radius2 = grid_config.fov_range * grid_config.fov_range;
    const double half_fov = std::min(M_PI, grid_config.fov_deg * M_PI / 360.0);
    const double c = std::cos(heading), s = std::sin(heading);
    const double cos_half = std::cos(half_fov);
    const double cx = cell.center.x - point.x, cy = cell.center.y - point.y;
    const double outer_range = grid_config.fov_range + grid_config.big_cell;
    if (std::fabs(cx) > outer_range || std::fabs(cy) > outer_range) return false;
    for (const auto& sample : cell.samples) {
        const double dx = sample.x - point.x, dy = sample.y - point.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 > radius2) continue;
        if (dx * c + dy * s < cos_half * std::sqrt(d2) - 1e-12) continue;
        if (!clear_observation_ray(point, sample, obstacles)) continue;
        return true;
    }
    return false;
}

int count_visible(const UnknownView& view, const GridConfig& grid_config,
                  const Vec2& point, double heading, const Obstacles& obstacles)
{
    int count = 0;
    for (const auto& cell : view.cells)
        if (cell_visible(cell, grid_config, point, heading, obstacles)) ++count;
    return count;
}

double observation_clearance(const Vec2& point, const FrontierConfig& cfg,
                             const Obstacles& obstacles)
{
    double clearance = std::min({point.x - cfg.min_x, cfg.max_x - point.x,
                                 point.y - cfg.min_y, cfg.max_y - point.y});
    for (const auto& obstacle : obstacles)
        clearance = std::min(clearance,
            std::hypot(point.x - obstacle.cx, point.y - obstacle.cy) - obstacle.r);
    return clearance;
}
}  // namespace

int visible_unknown_count(const GridSnapshot& grid, const Vec2& point, double heading,
                          const Obstacles& obstacles, const FrontierConfig& cfg)
{
    (void)cfg;  // The actual coverage sensor model belongs to the grid.
    return count_visible(prepare_unknown_view(grid), grid.config(), point, heading, obstacles);
}

FrontierSelection select_observation_target(
    const GridSnapshot& grid, const FrontierConfig& cfg, const Vec2& cur,
    double cur_yaw, int& candidate_band, const Obstacles& obstacles,
    const std::vector<Vec2>* unreachable, double block_r, double route_heading)
{
    FrontierSelection result;
    const UnknownView view = prepare_unknown_view(grid);
    if (view.cells.empty() || !std::isfinite(cur_yaw)) return result;
    // A band organizes which region to observe; its parity must not command
    // an arbitrary reversal. Preserve progress along a safe route, even when
    // its old viewpoint has just finished revealing its cells.
    const bool continuing_route = std::isfinite(route_heading);
    const double preferred_heading = continuing_route ? route_heading : cur_yaw;
    const double x_lo = cfg.min_x + cfg.margin, x_hi = cfg.max_x - cfg.margin;
    const double y_lo = cfg.min_y + cfg.margin, y_hi = cfg.max_y - cfg.margin;
    if (x_lo > x_hi || y_lo > y_hi) return result;
    const double bw = std::max(cfg.band_width, 1e-3);
    const auto band_of = [&](double y) {
        return std::max(0, static_cast<int>(std::floor((y - y_lo) / bw)));
    };
    const int band_count = std::max(1, band_of(y_hi) + 1);
    int active_band = std::clamp(candidate_band < 0 ? band_of(cur.y) : candidate_band,
                                 0, band_count - 1);
    std::vector<int> band_sizes(static_cast<size_t>(band_count), 0);
    for (const auto& cell : view.cells)
        ++band_sizes[std::min(band_of(cell.center.y), band_count - 1)];
    while (band_sizes[active_band] <= cfg.band_clear_cnt) {
        int next = active_band + 1;
        while (next < band_count && band_sizes[next] == 0) ++next;
        if (next >= band_count) { active_band = -2; break; }
        active_band = next;
    }
    const auto in_active_band = [&](const UnknownCell& cell) {
        return active_band < 0 ||
            (cell.center.y >= y_lo + active_band * bw - cfg.band_tol &&
             cell.center.y <= y_lo + (active_band + 1) * bw + cfg.band_tol);
    };
    const int small_limit = std::max(0, cfg.small_region_cells);
    bool any_large = false, active_large = false;
    for (const auto& cell : view.cells) {
        if (view.component_sizes[cell.component] > small_limit) {
            any_large = true;
            if (in_active_band(cell)) active_large = true;
        }
    }
    const auto blacklisted = [&](const Vec2& point) {
        if (!unreachable || block_r <= 0.0) return false;
        for (const auto& blocked : *unreachable)
            if (std::hypot(point.x - blocked.x, point.y - blocked.y) <= block_r) return true;
        return false;
    };
    struct Evaluation { bool valid = false; double distance = 0, turn = 0, clearance = 0; int gain = 0; };
    std::map<std::pair<long long, long long>, Evaluation> cache;
    double best_cost = std::numeric_limits<double>::infinity();
    int selected_band = active_band;
    // Pass 1 restores candidates outside a locally blocked band. Pass 2 is an
    // explicit observation-turn fallback only when no waypoint has forward
    // gain anywhere; it cannot compete against a normal flowing observation.
    for (int pass = 0; pass < 3 && !result.valid; ++pass) {
        for (const auto& cell : view.cells) {
            const int region_size = view.component_sizes[cell.component];
            const bool local = in_active_band(cell);
            if (pass == 0 && !local && (active_large || region_size <= small_limit)) continue;
            const double dx = cell.center.x - cur.x, dy = cell.center.y - cur.y;
            const double d = std::hypot(dx, dy);
            const double ux = d > 1e-6 ? dx / d : std::cos(cur_yaw);
            const double uy = d > 1e-6 ? dy / d : std::sin(cur_yaw);
            const double standoff = std::max(0.0, cfg.observation_standoff);
            const Vec2 points[] = {
                cell.center,
                {cell.center.x - standoff * ux, cell.center.y - standoff * uy},
                {cell.center.x - 1.5 * standoff * ux, cell.center.y - 1.5 * standoff * uy},
                {cell.center.x - standoff * uy, cell.center.y + standoff * ux},
                {cell.center.x + standoff * uy, cell.center.y - standoff * ux}};
            for (auto point : points) {
                point.x = clampd(point.x, x_lo, x_hi);
                point.y = clampd(point.y, y_lo, y_hi);
                const auto key = std::make_pair(std::llround(point.x * 10000.0),
                                                std::llround(point.y * 10000.0));
                auto entry = cache.emplace(key, Evaluation{});
                Evaluation& evaluation = entry.first->second;
                if (entry.second) {
                    evaluation.distance = std::hypot(point.x - cur.x, point.y - cur.y);
                    evaluation.clearance = observation_clearance(point, cfg, obstacles);
                    evaluation.valid = evaluation.distance >= std::max(0.0, cfg.observation_min_distance) &&
                        evaluation.distance > 1e-4 &&
                        evaluation.clearance + 1e-9 >= std::max(0.0, cfg.minimum_clearance) &&
                        !blacklisted(point);
                    if (evaluation.valid) {
                        const double heading = std::atan2(point.y - cur.y, point.x - cur.x);
                        evaluation.turn = std::fabs(wrap_pi(heading - cur_yaw));
                        evaluation.gain = count_visible(view, grid.config(), point, heading, obstacles);
                    }
                }
                if (!evaluation.valid) continue;
                const double arrival_heading = std::atan2(point.y - cur.y, point.x - cur.x);
                const double observation_heading = pass == 2
                    ? std::atan2(cell.center.y - point.y, cell.center.x - point.x) : arrival_heading;
                if (!cell_visible(cell, grid.config(), point, observation_heading, obstacles)) continue;
                const int gain = pass == 2
                    ? count_visible(view, grid.config(), point, observation_heading, obstacles) : evaluation.gain;
                if (gain == 0) continue;
                const double clearance_range = std::max(0.05, cfg.preferred_clearance - cfg.minimum_clearance);
                const double clearance_deficit = clampd(
                    (cfg.preferred_clearance - evaluation.clearance) / clearance_range, 0.0, 1.0);
                const double small_deficit = small_limit > 0 && region_size <= small_limit
                    ? static_cast<double>(small_limit + 1 - region_size) / (small_limit + 1) : 0.0;
                const int cell_band = std::min(band_of(cell.center.y), band_count - 1);
                const double band_penalty = !local && active_band >= 0
                    ? 0.5 * std::min(2, std::abs(cell_band - active_band)) : 0.0;
                const double along = std::max(0.0,
                    std::cos(preferred_heading) * (point.x - cur.x) +
                    std::sin(preferred_heading) * (point.y - cur.y));
                const double route_turn = continuing_route
                    ? std::fabs(wrap_pi(arrival_heading - route_heading)) : 0.0;
                const int neighbors = cfg.cluster_cap > 0
                    ? std::min(cell.neighbors, cfg.cluster_cap) : cell.neighbors;
                const double cost = cfg.near_weight * evaluation.distance +
                    cfg.turn_penalty * evaluation.turn - cfg.along_bonus * along -
                    cfg.cluster_weight * neighbors - cfg.gain_weight * gain +
                    cfg.clearance_weight * clearance_deficit + cfg.small_region_penalty * small_deficit +
                    band_penalty + std::max(0.0, cfg.continuity_turn_penalty) * route_turn;
                if (cost >= best_cost) continue;
                best_cost = cost;
                result = {true, point, gain, evaluation.clearance, region_size,
                          !any_large || pass == 2, cell.center, pass == 2, observation_heading};
                selected_band = active_band < 0 ? active_band : (local ? active_band : cell_band);
            }
        }
    }
    if (result.valid) candidate_band = selected_band;
    return result;
}

}  // namespace exploration
