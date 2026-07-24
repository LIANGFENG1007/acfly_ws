#include "exploration_planner/bezier.hpp"

#include <cmath>
#include <algorithm>

namespace exploration {

namespace {

inline double dist(const Vec2& a, const Vec2& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

inline double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// 三次贝塞尔在参数 t∈[0,1] 处取值
inline Vec2 bezier_at(const Vec2& b0, const Vec2& b1, const Vec2& b2, const Vec2& b3, double t)
{
    const double u = 1.0 - t;
    const double w0 = u * u * u;
    const double w1 = 3.0 * u * u * t;
    const double w2 = 3.0 * u * t * t;
    const double w3 = t * t * t;
    return { w0 * b0.x + w1 * b1.x + w2 * b2.x + w3 * b3.x,
             w0 * b0.y + w1 * b1.y + w2 * b2.y + w3 * b3.y };
}

}  // namespace

Trajectory smooth_catmull_rom(const Path2& waypoints, double ds)
{
    Trajectory traj;
    if (waypoints.size() < 2) {
        if (waypoints.size() == 1) traj.push_back({waypoints[0], 0.0, 0.0, 0.0});
        return traj;
    }

    // 去掉相邻重复点（centripetal 用到的距离不能为 0）
    Path2 wp;
    wp.push_back(waypoints.front());
    for (size_t i = 1; i < waypoints.size(); ++i) {
        if (dist(waypoints[i], wp.back()) > 1e-6) wp.push_back(waypoints[i]);
    }
    if (wp.size() < 2) {
        traj.push_back({wp[0], 0.0, 0.0, 0.0});
        return traj;
    }

    // ---- 1) 逐段 Catmull-Rom(centripetal α=0.5) → 三次贝塞尔，密采样成折线 ----
    Path2 dense;
    const int n = static_cast<int>(wp.size());
    const double alpha = 0.5;

    for (int i = 0; i < n - 1; ++i) {
        // 段端点 P1=wp[i], P2=wp[i+1]，邻点 P0,P3（端点处镜像延拓）
        const Vec2 P1 = wp[i];
        const Vec2 P2 = wp[i + 1];
        const Vec2 P0 = (i - 1 >= 0) ? wp[i - 1]
                                     : Vec2{2.0 * P1.x - P2.x, 2.0 * P1.y - P2.y};
        const Vec2 P3 = (i + 2 < n) ? wp[i + 2]
                                    : Vec2{2.0 * P2.x - P1.x, 2.0 * P2.y - P1.y};

        // centripetal 参数（弦长的 alpha 次幂），防零
        const double d01 = std::pow(std::max(dist(P0, P1), 1e-6), alpha);
        const double d12 = std::pow(std::max(dist(P1, P2), 1e-6), alpha);
        const double d23 = std::pow(std::max(dist(P2, P3), 1e-6), alpha);

        // Catmull-Rom 切线 → 贝塞尔控制点 B1,B2（标准 centripetal 公式）
        Vec2 m1, m2;
        m1.x = (P2.x - P1.x) + d12 * ((P1.x - P0.x) / d01 - (P2.x - P0.x) / (d01 + d12));
        m1.y = (P2.y - P1.y) + d12 * ((P1.y - P0.y) / d01 - (P2.y - P0.y) / (d01 + d12));
        m2.x = (P2.x - P1.x) + d12 * ((P3.x - P2.x) / d23 - (P3.x - P1.x) / (d12 + d23));
        m2.y = (P2.y - P1.y) + d12 * ((P3.y - P2.y) / d23 - (P3.y - P1.y) / (d12 + d23));

        const Vec2 B0 = P1;
        const Vec2 B1 = { P1.x + m1.x / 3.0, P1.y + m1.y / 3.0 };
        const Vec2 B2 = { P2.x - m2.x / 3.0, P2.y - m2.y / 3.0 };
        const Vec2 B3 = P2;

        // 密采样：步数按段长定，至少 8 步
        const double seg_len = dist(P1, P2);
        const int steps = std::max(8, static_cast<int>(std::ceil(seg_len / std::max(ds, 1e-3))) * 2);
        const int t0 = (i == 0) ? 0 : 1;   // 非首段跳过 t=0 避免与上段末点重复
        for (int k = t0; k <= steps; ++k) {
            const double t = static_cast<double>(k) / steps;
            dense.push_back(bezier_at(B0, B1, B2, B3, t));
        }
    }

    if (dense.size() < 2) {
        traj.push_back({dense.empty() ? wp[0] : dense[0], 0.0, 0.0, 0.0});
        return traj;
    }

    // ---- 2) 按弧长等距重采样 ----
    // 先算 dense 的累计弧长
    std::vector<double> cum(dense.size(), 0.0);
    for (size_t i = 1; i < dense.size(); ++i) {
        cum[i] = cum[i - 1] + dist(dense[i], dense[i - 1]);
    }
    const double total = cum.back();

    Path2 resampled;
    for (double s = 0.0; s <= total + 1e-6; s += ds) {
        // 在 cum 中找 s 所在段，线性插值
        auto it = std::lower_bound(cum.begin(), cum.end(), s);
        size_t idx = static_cast<size_t>(it - cum.begin());
        if (idx == 0) { resampled.push_back(dense.front()); continue; }
        if (idx >= dense.size()) { resampled.push_back(dense.back()); break; }
        const double s0 = cum[idx - 1], s1 = cum[idx];
        const double r = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;
        resampled.push_back({ dense[idx - 1].x + r * (dense[idx].x - dense[idx - 1].x),
                              dense[idx - 1].y + r * (dense[idx].y - dense[idx - 1].y) });
    }
    if (dist(resampled.back(), dense.back()) > 1e-6) resampled.push_back(dense.back());

    // ---- 3) 数值差分算 θ(切线) 和 κ(曲率)，并累计弧长 ----
    const int m = static_cast<int>(resampled.size());
    traj.resize(m);
    double acc = 0.0;
    for (int i = 0; i < m; ++i) {
        traj[i].p = resampled[i];
        if (i > 0) acc += dist(resampled[i], resampled[i - 1]);
        traj[i].s = acc;
    }
    for (int i = 0; i < m; ++i) {
        // 切线：用中心差分
        const Vec2& prev = resampled[std::max(0, i - 1)];
        const Vec2& next = resampled[std::min(m - 1, i + 1)];
        const double tx = next.x - prev.x;
        const double ty = next.y - prev.y;
        traj[i].theta = std::atan2(ty, tx);
    }
    for (int i = 0; i < m; ++i) {
        // 曲率：相邻切线角变化 / 弧长变化
        if (i == 0 || i == m - 1) { traj[i].kappa = 0.0; continue; }
        const double dtheta = wrap_pi(traj[i + 1].theta - traj[i - 1].theta);
        const double dssum = traj[i + 1].s - traj[i - 1].s;
        traj[i].kappa = (dssum > 1e-6) ? (dtheta / dssum) : 0.0;
    }

    return traj;
}

}  // namespace exploration
