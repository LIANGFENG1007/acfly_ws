// ============================================================================
//  trajectory_tracker.hpp  ── 沿贝塞尔参考轨迹的 pure-pursuit + PID 跟踪
//
//  输入：参考轨迹 + 当前位姿(x,y,yaw) + 当前估计速度(用于 D 项)。
//  输出：机体系速度命令 —— 前进 v_fwd / 横向纠偏 v_lat / yaw_rate。
//    v_fwd 上限 V_MAX，按曲率/朝向误差/临近终点动态降速；
//    yaw_rate 为主转向（把机头拉向切线）；v_lat 仅低限纠偏。
// ============================================================================

#pragma once

#include <cstddef>

#include "exploration_planner/types.hpp"
#include "exploration_planner/bezier.hpp"

namespace exploration {

struct TrackerGains {
    double v_max, v_min, k_curv, lookahead, endpoint_slow_r;
    double kp_yaw, kd_yaw, max_yaw_rate;
    double kp_lat, kd_lat, max_v_lat;
    double heading_gate_rad;   // 机头偏离超此角度→前进+横向全清零,只原地转身(防大角度甩出线外撞柱)
};

struct VelCmd {
    double v_fwd    = 0.0;   // 机体系前进 (m/s)
    double v_lat    = 0.0;   // 机体系横向纠偏 (m/s)
    double yaw_rate = 0.0;   // (rad/s)
    bool   at_goal  = false; // 已到轨迹末端(终点)容差内
};

class TrajectoryTracker
{
public:
    explicit TrajectoryTracker(const TrackerGains& g) : g_(g) {}

    void set_trajectory(const Trajectory& traj);
    bool has_trajectory() const { return !traj_.empty(); }

    // 给定当前位姿与估计的机体前进/横向速度(用于 D 项)，算一拍速度命令。
    // goal_tol：到终点容差 (m)。
    VelCmd update(double px, double py, double yaw,
                  double v_fwd_est, double v_lat_est,
                  double goal_tol);

    // 最近一次用到的前瞻参考点（给可视化）
    Vec2 last_lookahead() const { return last_look_; }

    // 最近一次 update 时，飞机到轨迹最近点的距离（m）——给"偏航就重规划"用
    double last_nearest_dist() const { return last_nearest_dist_; }

private:
    TrackerGains g_;
    Trajectory   traj_;
    size_t       progress_idx_ = 0;   // 沿轨迹推进的最近点索引（单调前进）
    double       last_nearest_dist_ = 0.0;

    double prev_e_yaw_ = 0.0;
    double prev_e_ct_  = 0.0;
    bool   prev_valid_ = false;

    Vec2   last_look_;

    // 从 progress_idx_ 起找离当前位置最近的轨迹点（只向前搜，禁止倒退）
    size_t advance_to_nearest(double px, double py);
};

}  // namespace exploration
