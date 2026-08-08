#include "exploration_planner/coverage_planner.hpp"

#include <algorithm>
#include <cmath>
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

    // give-up blacklist: skip candidate cells within block_r of any unreachable point
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

    // 收集未探索大格：中心坐标 + 8 邻域里"也未扫"的邻居数（衡量是否成片）+ 所属条带
    struct Cell { Vec2 c; int nbr; int band; };
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
            un.push_back({c, nbr, band_of(c.y)});
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
            if (blacklisted(un[i].c)) continue;
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

        const Vec2 w{ clampd(un[best].c.x, x_lo, x_hi),
                      clampd(un[best].c.y, y_lo, y_hi) };
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
            if (blacklisted(un[i].c)) continue;
            const double d = std::hypot(un[i].c.x - cur.x, un[i].c.y - cur.y);
            if (d < bd) { bd = d; best = static_cast<int>(i); }
        }
        if (best >= 0)
            wp.push_back({ clampd(un[best].c.x, x_lo, x_hi), clampd(un[best].c.y, y_lo, y_hi) });
    }
    return wp;
}

}  // namespace exploration
