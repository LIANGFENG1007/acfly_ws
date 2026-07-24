#include "exploration_planner/obstacle_map.hpp"

#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>

namespace exploration {

ObstacleMap::ObstacleMap(const ObstacleConfig& cfg)
    : cfg_(cfg)
{
    const double w = cfg_.max_x - cfg_.min_x;
    const double h = cfg_.max_y - cfg_.min_y;
    nx_ = std::max(1, static_cast<int>(std::lround(w / cfg_.cell)));
    ny_ = std::max(1, static_cast<int>(std::lround(h / cfg_.cell)));
    hit_.assign(static_cast<size_t>(nx_) * ny_, 0);
    latched_.assign(static_cast<size_t>(nx_) * ny_, 0);   // latch 永久占据位，初始全 0
    ghost_miss_.assign(static_cast<size_t>(nx_) * ny_, 0);// latch 回收·幽灵计数，初始全 0
}

void ObstacleMap::integrate(const std::vector<Vec2>& pts,
                            double px, double py, double yaw, bool pose_valid)
{
    std::lock_guard<std::mutex> lk(mtx_);

    // 本帧命中标记（只标一次，避免同格多点重复加分）
    std::vector<uint8_t> hit_now(hit_.size(), 0);

    const double ex = cfg_.edge_ignore;
    for (const auto& p : pts) {
        // 去掉贴墙的点（墙不当障碍，靠 WALL_MARGIN 边界逻辑兜底）
        if (p.x < cfg_.min_x + ex || p.x > cfg_.max_x - ex ||
            p.y < cfg_.min_y + ex || p.y > cfg_.max_y - ex)
            continue;

        const int ix = static_cast<int>(std::floor((p.x - cfg_.min_x) / cfg_.cell));
        const int iy = static_cast<int>(std::floor((p.y - cfg_.min_y) / cfg_.cell));
        if (!in_grid(ix, iy)) continue;
        hit_now[idx(ix, iy)] = 1;
    }

    // 滑动累计 + 衰减：本帧命中的格 +hit_inc(封顶 hit_max)，未命中的格 -hit_decay(到 0 消失)。
    //   这样飞机摆头/挪位后，旧障碍格会随时间掉到阈值以下被清掉，绿圆不再无限累积/裂开。
    const int inc = cfg_.hit_inc, dec = cfg_.hit_decay, hmax = cfg_.hit_max;
    for (size_t i = 0; i < hit_.size(); ++i) {
        int v = static_cast<int>(hit_[i]);
        if (hit_now[i]) v = std::min(v + inc, hmax);
        else            v = std::max(v - dec, 0);
        hit_[i] = static_cast<uint16_t>(v);
    }

    // ★latch 智能回收(防幽灵)★：latch 格若【当前在视野内】却【连续无点命中】→ 判为幽灵，撤销 latch。
    //   看不到的 latch 格(视野外)冻结计数，不回收——不误伤真障碍(飞机没在看它时不该清)。
    //   真静态障碍:飞机看它时总会命中→计数归零→永不回收。幽灵:视野扫到却无点→计数涨过阈值→清除。
    if (pose_valid && cfg_.ghost_clear_frames > 0) {
        const double half_fov = (cfg_.fov_deg * 0.5) * M_PI / 180.0;
        const double r2 = cfg_.fov_range * cfg_.fov_range;
        const double cyaw = std::cos(yaw), syaw = std::sin(yaw);
        for (int ix = 0; ix < nx_; ++ix) {
            const double gx = cfg_.min_x + (ix + 0.5) * cfg_.cell;
            for (int iy = 0; iy < ny_; ++iy) {
                const int id = idx(ix, iy);
                if (!latched_[id]) continue;               // 只管 latch 格
                if (hit_now[id]) { ghost_miss_[id] = 0; continue; }  // 本帧命中 → 活的，清零

                // 该格中心是否落在当前 FOV 扇形内(距离 + 开角)
                const double dx = gx - px;
                const double dy = (cfg_.min_y + (iy + 0.5) * cfg_.cell) - py;
                if (dx * dx + dy * dy > r2) continue;       // 视野外(太远)→冻结，不计
                // 机头系前向投影+横向投影判开角：cosθ=fwd/dist，|atan2(lat,fwd)|<=half_fov
                const double fwd =  cyaw * dx + syaw * dy;
                const double lat = -syaw * dx + cyaw * dy;
                if (fwd <= 0.0) continue;                   // 在身后 → 不在视野，冻结
                if (std::fabs(std::atan2(lat, fwd)) > half_fov) continue;  // 超开角 → 冻结

                // 在视野内却没命中 → 幽灵嫌疑 +1；累计超阈值 → 撤销 latch(连 hit 一起清零)
                if (++ghost_miss_[id] > cfg_.ghost_clear_frames) {
                    latched_[id]    = 0;
                    hit_[id]        = 0;
                    ghost_miss_[id] = 0;
                }
            }
        }
    }

    rebuild_locked();
}

void ObstacleMap::rebuild_locked()
{
    Obstacles result;

    // 占据掩码：命中累计 >= 阈值 → 本帧占据，并【永久 latch】该格；latched 格恒为占据(即使 hit 已衰减)。
    //   → 静态障碍一旦识别就永不消失；半径不受影响(下面仍按当前 occ 聚类，随观测变大/变小)。
    std::vector<uint8_t> occ(static_cast<size_t>(nx_) * ny_, 0);
    for (size_t i = 0; i < occ.size(); ++i) {
        if (hit_[i] >= cfg_.hit_thresh) latched_[i] = 1;   // 达阈值 → 永久 latch
        occ[i] = (latched_[i] || hit_[i] >= cfg_.hit_thresh) ? 1 : 0;
    }

    std::vector<uint8_t> visited(occ.size(), 0);
    std::vector<int> stack;             // 复用的 BFS/DFS 栈（存 idx）
    std::vector<std::pair<int,int>> comp;  // 当前连通域的格子 (ix,iy)

    for (int ix = 0; ix < nx_; ++ix) {
        for (int iy = 0; iy < ny_; ++iy) {
            const int id0 = idx(ix, iy);
            if (!occ[id0] || visited[id0]) continue;

            // 迭代式 8 邻连通域扫描（避免递归爆栈）
            comp.clear();
            stack.clear();
            stack.push_back(id0);
            visited[id0] = 1;
            while (!stack.empty()) {
                const int id = stack.back();
                stack.pop_back();
                const int cx = id / ny_;
                const int cy = id % ny_;
                comp.emplace_back(cx, cy);
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        const int nxc = cx + dx, nyc = cy + dy;
                        if (!in_grid(nxc, nyc)) continue;
                        const int nid = idx(nxc, nyc);
                        if (occ[nid] && !visited[nid]) {
                            visited[nid] = 1;
                            stack.push_back(nid);
                        }
                    }
                }
            }

            if (static_cast<int>(comp.size()) < cfg_.min_cells) continue;  // 太小，噪声

            // 质心（格中心世界坐标）
            double sx = 0.0, sy = 0.0;
            for (const auto& c : comp) {
                sx += cfg_.min_x + (c.first  + 0.5) * cfg_.cell;
                sy += cfg_.min_y + (c.second + 0.5) * cfg_.cell;
            }
            const double cx = sx / comp.size();
            const double cy = sy / comp.size();

            // 半径：用簇的【等效圆半径】(按占据格数估面积 → r=sqrt(N·cell²/π))，而非"最远格距离"。
            //   最远格距离对离群格极敏感——远处杆点云横向发散、逐帧飘进不同外围格，会把半径一下拉很大
            //   且逐帧跳动。等效半径只看格【数量】(≈面积)、不看排布，远处杆稳定收敛到小圆，不再被
            //   单个离群格支配。避障安全无虞：A* 禁入 = r + robot_radius + global_margin，余量足够。
            const double area = static_cast<double>(comp.size()) * cfg_.cell * cfg_.cell;
            double r = std::sqrt(area / M_PI) + cfg_.r_inflate;
            r = std::clamp(r, cfg_.min_r, cfg_.max_r);

            result.push_back(Obstacle{cx, cy, r});
        }
    }

    // 顺手记录所有占据格中心点(rviz 可视化用)——就是算法当障碍的那些点。
    occ_points_.clear();
    for (int ix = 0; ix < nx_; ++ix)
        for (int iy = 0; iy < ny_; ++iy)
            if (occ[idx(ix, iy)])
                occ_points_.push_back({ cfg_.min_x + (ix + 0.5) * cfg_.cell,
                                        cfg_.min_y + (iy + 0.5) * cfg_.cell });

    // 时序平滑：不直接输出本帧聚类结果(逐帧重算→闪动/瞬移)，而是关联到已有障碍做 EMA 后再输出。
    track_and_smooth(result);
}

// 把本帧测量圆 meas 关联到已有 track 做加权平均(EMA)，压住位置/半径的逐帧跳变。
//   关联：每个 track 找最近的、未被占用的测量圆(中心距 < track_assoc_dist)；
//   更新：EMA 把瞬移压成缓动(半径同样平滑→最初照不全偏小、看全了慢慢长到真实大小)；
//   新增：没被关联的测量圆 → 新障碍(首帧直接用测量值，立刻可见)；
//   删除：连续 > track_max_misses 帧没再被关联到 → 视为离开视野，删掉(只看最近一段时间)。
void ObstacleMap::track_and_smooth(const Obstacles& meas)
{
    const double alpha = cfg_.track_alpha;
    const double gate2 = cfg_.track_assoc_dist * cfg_.track_assoc_dist;

    for (auto& t : tracks_) t.matched = false;
    std::vector<char> used(meas.size(), 0);

    // 1) 每个已有 track 关联最近的未占用测量圆
    for (auto& t : tracks_) {
        int best = -1; double best_d2 = gate2;
        for (size_t i = 0; i < meas.size(); ++i) {
            if (used[i]) continue;
            const double dx = meas[i].cx - t.cx, dy = meas[i].cy - t.cy;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best = static_cast<int>(i); }
        }
        if (best >= 0) {
            const Obstacle& m = meas[best];
            t.cx += alpha * (m.cx - t.cx);
            t.cy += alpha * (m.cy - t.cy);
            t.r  += alpha * (m.r  - t.r);
            t.matched = true;
            t.misses  = 0;
            used[best] = 1;
        }
    }

    // 2) 未被关联的测量圆 → 新建 track(首帧不平滑，立刻可见)
    for (size_t i = 0; i < meas.size(); ++i)
        if (!used[i]) tracks_.push_back(Track{ meas[i].cx, meas[i].cy, meas[i].r, 0, true });

    // 3) 本帧没匹配到的 track：丢帧计数 +1；超过上限(最近一段时间没再出现)→ 删除
    for (auto& t : tracks_) if (!t.matched) ++t.misses;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                  [&](const Track& t) { return t.misses > cfg_.track_max_misses; }),
                  tracks_.end());

    // 4) 对外输出 = 所有存活 track 的平滑圆
    obstacles_.clear();
    obstacles_.reserve(tracks_.size());
    for (const auto& t : tracks_)
        obstacles_.push_back(Obstacle{ t.cx, t.cy, t.r });
}

Obstacles ObstacleMap::snapshot() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return obstacles_;
}

std::vector<Vec2> ObstacleMap::occupied_points() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return occ_points_;
}

}  // namespace exploration
