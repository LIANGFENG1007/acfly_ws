#include "exploration_planner/trajectory_tracker.hpp"

#include <cmath>
#include <algorithm>

namespace exploration {

namespace {
inline double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}
inline double clamp_abs(double v, double lim)
{
    if (v >  lim) return lim;
    if (v < -lim) return -lim;
    return v;
}
inline double dist(const Vec2& a, const Vec2& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

void TrajectoryTracker::set_trajectory(const Trajectory& traj)
{
    traj_ = traj;
    progress_idx_ = 0;
    prev_valid_ = false;
}

size_t TrajectoryTracker::advance_to_nearest(double px, double py)
{
    // 从当前进度向前搜窗口内最近点，单调前进（防止在掉头处回退到旧车道）
    const size_t N = traj_.size();
    size_t best = progress_idx_;
    double best_d = dist(traj_[best].p, {px, py});

    const size_t window = 200;   // 向前搜的最大点数（弧长 ~200*ds）
    const size_t hi = std::min(N, progress_idx_ + window);
    for (size_t i = progress_idx_; i < hi; ++i) {
        const double d = dist(traj_[i].p, {px, py});
        if (d < best_d) { best_d = d; best = i; }
    }
    progress_idx_ = best;
    last_nearest_dist_ = best_d;
    return best;
}

VelCmd TrajectoryTracker::update(double px, double py, double yaw,
                                 double v_fwd_est, double v_lat_est,
                                 double goal_tol)
{
    (void)v_fwd_est;   // 预留：前进方向的 D 项（当前 v_fwd 由曲率前馈，不做 D）
    VelCmd cmd;
    if (traj_.empty()) return cmd;

    const Vec2 cur{px, py};
    const Vec2& goal = traj_.back().p;

    // 到终点判定
    const double d_goal = dist(cur, goal);
    if (d_goal <= goal_tol) {
        cmd.at_goal = true;
        return cmd;   // 全零，悬停
    }

    // 1) 推进到最近点
    const size_t near = advance_to_nearest(px, py);

    // 2) 找前瞻点：从最近点沿弧长前进 lookahead
    const double s_target = traj_[near].s + g_.lookahead;
    size_t look = near;
    while (look + 1 < traj_.size() && traj_[look].s < s_target) ++look;
    const TrajPoint& ref = traj_[look];
    last_look_ = ref.p;

    // 3) 误差
    // 朝向误差：机头 → 前瞻点切线方向
    const double e_yaw = wrap_pi(ref.theta - yaw);

    // 横向偏差：当前点相对最近点切线的带符号垂距
    const TrajPoint& np = traj_[near];
    const double dx = px - np.p.x;
    const double dy = py - np.p.y;
    // 左正右负：把偏差投影到切线的左法向 (-sinθ, cosθ)
    const double e_ct = -std::sin(np.theta) * dx + std::cos(np.theta) * dy;

    // D 项（数值差分，update 周期固定 20Hz → 用固定 dt）
    const double dt = 0.05;
    double de_yaw = 0.0, de_ct = 0.0;
    if (prev_valid_) {
        de_yaw = wrap_pi(e_yaw - prev_e_yaw_) / dt;
        de_ct  = (e_ct - prev_e_ct_) / dt;
    }
    prev_e_yaw_ = e_yaw;
    prev_e_ct_  = e_ct;
    prev_valid_ = true;

    // 4) 三路输出
    // yaw_rate：主转向
    cmd.yaw_rate = clamp_abs(g_.kp_yaw * e_yaw + g_.kd_yaw * de_yaw, g_.max_yaw_rate);

    // ★先转再走·朝向门控★：机头偏离前瞻方向越大,越压住移动(前进+横向都乘),让飞机先转够再走。
    //   |e_yaw| > 阈值 → 门=0,本拍只转身不移动(防规划出身后/大角度路径时机体平移甩出线外撞柱);
    //   阈值内 → cos(e_yaw) 平滑过渡。正常巡航 e_yaw≈0→门≈1,不影响。
    const double head_gate =
        (std::fabs(e_yaw) > g_.heading_gate_rad) ? 0.0 : std::max(0.0, std::cos(e_yaw));

    // v_lat：横向纠偏（注意符号：飞机在轨迹左侧 e_ct>0，应向右移即机体 -y）
    cmd.v_lat = clamp_abs(-(g_.kp_lat * e_ct + g_.kd_lat * de_ct), g_.max_v_lat);
    cmd.v_lat *= head_gate;   // ★关键★:不压横向则飞机仍沿线法向侧移甩出去
    (void)v_lat_est;

    // v_fwd：曲率前馈降速 + 朝向门控降速 + 临近终点斜坡降速
    double v = g_.v_max / (1.0 + g_.k_curv * std::fabs(ref.kappa));
    v *= head_gate;                                // 机头没对正时减速(超阈值清零→原地转身)
    if (d_goal < g_.endpoint_slow_r) {
        v *= (d_goal / g_.endpoint_slow_r);        // 临近终点线性降到 0
    }
    cmd.v_fwd = std::clamp(v, 0.0, g_.v_max);
    // 下限保护：除非在终点附近或机头偏(门控已压低)，否则不低于 v_min，避免卡死
    if (d_goal >= g_.endpoint_slow_r && head_gate > 0.5 && cmd.v_fwd < g_.v_min) {
        cmd.v_fwd = g_.v_min;
    }

    return cmd;
}

}  // namespace exploration
