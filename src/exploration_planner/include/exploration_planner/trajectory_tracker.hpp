// ============================================================================
//  trajectory_tracker.hpp  ── 沿贝塞尔参考轨迹的 pure-pursuit + PID 跟踪
//
//  输入：参考轨迹 + 当前位姿(x,y,yaw) + 当前估计速度(用于 D 项)。
//  输出：机体系速度命令 —— 前进 v_fwd / 横向纠偏 v_lat / yaw_rate。
//    v_fwd 上限 V_MAX，按曲率/朝向误差/临近终点动态降速；
//    yaw_rate 为主转向；车式模式下 v_lat 强制为 0，只沿机头前向行驶。
// ============================================================================

#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>

#include "exploration_planner/types.hpp"
#include "exploration_planner/bezier.hpp"

namespace exploration {

struct TrackerGains {
    double v_max, v_min, k_curv, lookahead, endpoint_slow_r;
    double kp_yaw, kd_yaw, max_yaw_rate;
    double kp_lat, kd_lat, max_v_lat;
    double heading_gate_rad;   // 车式模式开始减速的角度；全向模式仍使用停车门限
    bool   forward_only = false; // 车式模式：只允许机体前向速度，禁止横向侧移
    // ★控制周期 (s)★：D 项数值差分的分母。★必须等于 update() 的真实调用周期★
    //   (= TIMER_PERIOD_MS/1000)，由节点构造时按 TIMER_PERIOD_MS 推导填入，勿写死。
    //   2026-08 修：此前 tracker 内写死 0.05 而实际周期是 0.02，微分项恒为真值的 0.4 倍
    //   (KD_YAW=0.50 实际只等效 0.20)。现改为显式传入并把 KD 同步缩放，输出逐位不变；
    //   之后再改 TIMER_PERIOD_MS，阻尼会跟着正确变化，不再隐性跳变。
    double dt;
    double max_accel = 0.30;
    double max_yaw_accel = 1.20;
    double yaw_filter_tau = 0.12;
    double prediction_time = 0.20;
    double lateral_prediction_time = 0.50;
    double stop_align_rad = 1.3089969389957472;
    double corner_stop_rad = 0.3490658503988659;
    double max_lateral_accel = 0.35;
    double align_resume_rad = 0.4363323129985824;
    double align_stop_speed = 0.06;
    double align_stop_yaw_rate = 0.12;
    double align_settle_s = 0.06;
};

struct VelCmd {
    double v_fwd    = 0.0;   // 机体系前进 (m/s)
    double v_lat    = 0.0;   // 机体系横向纠偏 (m/s)
    double yaw_rate = 0.0;   // (rad/s)
    bool   at_goal  = false; // 已到轨迹末端(终点)容差内
    bool   needs_replan = false; // 已滑过未通过的尖角，停稳后从实际位置重新规划
};

class TrajectoryTracker
{
public:
    explicit TrajectoryTracker(const TrackerGains& g) : g_(g) {}

    void set_trajectory(const Trajectory& traj);
    bool has_trajectory() const { return !traj_.empty(); }
    const Trajectory& trajectory() const { return traj_; }

    // 给定当前位姿与估计的机体前进/横向速度(用于 D 项)，算一拍速度命令。
    // goal_tol：到终点容差 (m)。
    VelCmd update(double px, double py, double yaw,
                  double v_fwd_est, double v_lat_est,
                  double goal_tol,
                  double measured_yaw_rate = std::numeric_limits<double>::quiet_NaN());

    bool reorienting() const { return aligning_; }
    double progress_distance() const { return progress_s_; }
    double remaining_distance() const { return traj_.empty() ? 0.0 : traj_.back().s - progress_s_; }
    double heading_error() const { return last_heading_error_; }
    double reference_curvature() const { return last_curvature_; }
    double next_corner_distance() const {
        const size_t corner = next_corner();
        return corner < traj_.size() ? traj_[corner].s - progress_s_ : -1.0;
    }
    Path2 remaining_path(double px, double py) const;
    Path2 remaining_reference_path() const
    {
        if (traj_.empty()) return {};
        Path2 result{sample_at(progress_s_).p};
        for (const auto& point : traj_) {
            if (point.s > progress_s_ + 1e-9 &&
                (point.p.x != result.back().x || point.p.y != result.back().y))
                result.push_back(point.p);
        }
        return result;
    }

    // Keep acceleration recovery consistent with a downstream safety speed cap.
    void constrain_forward_command(double maximum)
    {
        previous_forward_command_ = std::clamp(previous_forward_command_, 0.0, std::max(0.0, maximum));
    }

    // 最近一次用到的前瞻参考点（给可视化）
    Vec2 last_lookahead() const { return last_look_; }

    // 最近一次 update 时，飞机到轨迹最近点的距离（m）——给"偏航就重规划"用
    double last_nearest_dist() const { return last_nearest_dist_; }

private:
    TrackerGains g_;
    Trajectory   traj_;
    size_t       progress_idx_ = 0;   // 沿轨迹推进的最近点索引（单调前进）
    double       progress_s_ = 0.0;
    double       passed_corner_s_ = -1.0;
    bool         progress_valid_ = false;
    Vec2         progress_position_{};
    double       last_nearest_dist_ = 0.0;

    double prev_e_yaw_ = 0.0;
    double prev_e_ct_  = 0.0;
    bool   prev_valid_ = false;

    bool motion_valid_ = false;
    double prev_yaw_ = 0.0;
    double filtered_yaw_rate_ = 0.0;
    double previous_yaw_command_ = 0.0;
    double previous_forward_command_ = 0.0;
    double previous_lateral_command_ = 0.0;
    double last_heading_error_ = 0.0, last_curvature_ = 0.0;
    bool aligning_ = false;
    bool alignment_heading_valid_ = false;
    double alignment_heading_ = 0.0;
    double alignment_settled_ = 0.0;
    int turn_direction_ = 0;

    Vec2   last_look_;
    double handoff_heading_ = 0.0;
    double handoff_remaining_ = 0.0;
    bool handoff_valid_ = false;

    // 从 progress_idx_ 起找离当前位置最近的轨迹点（只向前搜，禁止倒退）
    void advance_to_nearest(double px, double py);
    double projected_progress(double px, double py) const;
    TrajPoint sample_at(double s) const;
    size_t next_corner() const;
};

}  // namespace exploration
