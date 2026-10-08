// ============================================================================
//  trajectory_tracker.hpp  ── 沿贝塞尔参考轨迹的 pure-pursuit + PID 跟踪
//
//  输入：参考轨迹 + 当前位姿(x,y,yaw) + 当前估计速度(用于 D 项)。
//  输出：机体系速度命令 —— 前进 v_fwd / 横向纠偏 v_lat / yaw_rate。
//    v_fwd 上限 V_MAX，按曲率/朝向误差/临近终点动态降速；
//    yaw_rate 为主转向；节点启用受避障检查的巡航横漂补偿，转向时保持XY。
// ============================================================================

#pragma once

#include <algorithm>
#include <cstddef>
#include <deque>
#include <limits>

#include "exploration_planner/types.hpp"
#include "exploration_planner/bezier.hpp"
#include "exploration_planner/position_hold.hpp"

namespace exploration {

struct TrackerGains {
    double v_max, v_min, k_curv, lookahead, endpoint_slow_r;
    double kp_yaw, kd_yaw, max_yaw_rate;
    double kp_lat, kd_lat, max_v_lat;
    double heading_gate_rad;   // 车式模式开始减速的角度；全向模式仍使用停车门限
    bool   forward_only = false; // 巡航纵向只允许前进；转向允许XY定点纠偏
    // Opt in only when the caller validates the added lateral velocity.
    bool   compensate_lateral_drift = false;
    double drift_velocity_gain = 0.80;
    // ★控制周期 (s)★：D 项数值差分的分母。★必须等于 update() 的真实调用周期★
    //   (= TIMER_PERIOD_MS/1000)，由节点构造时按 TIMER_PERIOD_MS 推导填入，勿写死。
    //   2026-08 修：此前 tracker 内写死 0.05 而实际周期是 0.02，微分项恒为真值的 0.4 倍
    //   (KD_YAW=0.50 实际只等效 0.20)。现改为显式传入并把 KD 同步缩放，输出逐位不变；
    //   之后再改 TIMER_PERIOD_MS，阻尼会跟着正确变化，不再隐性跳变。
    double dt;
    double max_accel = 0.30;
    // 巡航前进指令的减速斜坡上限(m/s²)。弯道/终点减速由前瞻速度剖面提前开始，
    // 不再让前进指令一拍内从巡航速跌到近零(2026-09-23 仿真：真值加速度 RMS 0.38 的主因)。
    double max_decel = 0.60;
    // 弯道角速度前馈增益(0=关闭，1=完全按 v·κ 前馈)。
    double curvature_feedforward = 1.0;
    // 终点/尖角减速段的最低爬行速度(m/s)，防止 sqrt 停车律与曲率门叠加后趴窝。
    double creep_speed = 0.08;
    // 随速度的前瞻：lookahead_time·上一拍前进指令，限在 [lookahead, lookahead_max]；lookahead_time=0 为固定前瞻(旧行为)。
    //   高速直线前瞻长(稳、航向不抖)，急弯本就低速、前瞻回到 lookahead，不切弯贴柱。
    double lookahead_time = 0.0, lookahead_max = 0.0;
    // 减速半径 endpoint_slow_r 处的速度(m/s)：半径内按 slow_speed·d/R 线性停车，半径外按 max_decel 反推。
    //   旧律斜率 v_max/R 随 V_MAX 变陡：0.8 m/s 时要 1.3 m/s² 才刹得住(斜坡只给 0.6)，到尖角/终点仍 0.38 m/s。0=用 v_max(旧行为)。
    double endpoint_slow_speed = 0.50;
    // 停车转向锚点的滑停估计：减速度(m/s²)与前移上限(m)。
    double stop_anchor_decel = 0.80;
    double stop_anchor_max_m = 0.20;
    // 滑停估计中"指令到实际减速"的等效延迟(s)。原写死 0.10；定点改为从已发指令斜坡刹停后，
    //   PX4 约 0.4 s 延迟 + 0.3 s 时间常数使实际滑行 ~0.26 m@0.3 m/s，锚点低估会被 0.2 m/s 回拉(r49 点头 -2.9°)。
    double stop_anchor_latency = 0.10;
    // 定点保持是否跟随惯性滑行(不反推)。必达点 tracker 关闭，保持原行为。
    bool hold_slide = true;
    double max_yaw_accel = 1.20;
    // 航向预测/阻尼所用角速度中"上一拍指令"的权重(0=全用实测，1=全用指令)。
    // PX4 已闭环跟踪角速度指令，指令→实测有约 0.4 s 滞后；对滞后的实测再做阻尼会与飞控内环
    // 形成振荡环(台架：实机级滞后下直线增幅振荡)。指令本身无噪声、无滞后(Smith 预估思想)。
    double yaw_damping_command_weight = 0.0;
    // 巡航期望角速度与横漂补偿目标的一阶低通时间常数(s)，0=关闭。SLAM 位置噪声经前瞻几何
    // 进入航向误差、每帧(10~20 Hz)阶跃一次；不滤波时指令只受角加速度限幅，呈满幅锯齿。
    double command_filter_tau = 0.0;
    // ★Smith 预估★(s，0=关闭，沿用 prediction_time/lateral_prediction_time 的实测速度外推)。
    //   预测位置 = 当前位置 + 最近 smith_time 内【已发出】速度指令的积分；预测航向同理用角速度指令。
    //   指令无噪声，且正是飞机在控制滞后期内将要执行的运动。实测速度外推(旧)在 1.5 cm SLAM 噪声下
    //   是航向指令最大的高频源(台架：每拍跳变 0.049 rad，为几何误差的 2 倍)。
    // 横向滑移减速：v *= 1/(1 + gain·max(0, |横向速度| - deadband))，横向速度先经 lateral_slow_tau 低通。
    //   旧版 gain=3、无死区、不滤波：仿真 69% 时间是它在限前进速度(均值压 9%)，实机横向速度估计的
    //   噪声会直接调制前进指令(前后点头)。默认值保持旧行为。
    double lateral_slow_gain = 3.0;
    double lateral_slow_deadband = 0.0;
    double lateral_slow_tau = 0.0;
    double smith_time = 0.0;
    double smith_yaw_time = 0.0;
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
    double turn_hold_kp = 0.8;
    double turn_hold_kd = 0.35;
    double turn_hold_speed = 0.20;
};

struct VelCmd {
    double v_fwd    = 0.0;   // 机体系前进 (m/s)
    double v_lat    = 0.0;   // 机体系横向纠偏 (m/s)
    double yaw_rate = 0.0;   // (rad/s)
    bool   at_goal  = false; // 已到轨迹末端(终点)容差内
    bool   needs_replan = false; // 已滑过未通过的尖角，停稳后从实际位置重新规划
    bool   holding_position = false; // 原地转向的XY补偿，由调用方校验实际运动方向
    bool   lateral_correction = false; // 巡航横漂补偿也必须校验实际速度向量
};

class TrajectoryTracker
{
public:
    explicit TrajectoryTracker(const TrackerGains& g) : g_(g) {}

    void set_trajectory(const Trajectory& traj);
    // 运行期切换末端停车律(减速半径/半径处速度)：同一个必达 tracker 归航时要更早减速，投放接近保持原律。
    void set_endpoint_slow(double radius, double speed) { g_.endpoint_slow_r = radius; g_.endpoint_slow_speed = speed; }
    bool has_trajectory() const { return !traj_.empty(); }
    const Trajectory& trajectory() const { return traj_; }

    // 给定当前位姿与估计的机体前进/横向速度(用于 D 项)，算一拍速度命令。
    // goal_tol：到终点容差 (m)。
    VelCmd update(double px, double py, double yaw,
                  double v_fwd_est, double v_lat_est,
                  double goal_tol,
                  double measured_yaw_rate = std::numeric_limits<double>::quiet_NaN());
    // 换用另一个 tracker 继续飞(探索→归航/插点/视觉目标)时继承其指令与滤波状态，
    //   使新轨迹的第一拍从当前已发出的指令按斜坡过渡，而不是从零起步(r01：交接时前进指令一拍 0.34→0)。
    void inherit_motion(const TrajectoryTracker& other);
    // 调用方对本拍指令做了安全裁剪/置零后，把最终发出的指令告知 tracker(Smith 预估用)。
    // 同时同步 tracker 自身的斜坡状态与定点保持速度(审查发现：只改历史时，守卫置零的角速度在约束解除后会一拍跳回旧值)。
    void set_last_command(double v_fwd, double v_lat, double yaw_rate)
    {
        previous_yaw_command_ = yaw_rate;
        previous_forward_command_ = std::max(0.0, v_fwd);
        previous_lateral_command_ = v_lat;
        if (history_.empty()) return;
        auto& h = history_.back();
        h.fwd = v_fwd; h.lat = v_lat; h.rate = yaw_rate;
        if (aligning_ || holonomic_hold_active_) {
            const double c = std::cos(h.yaw), s = std::sin(h.yaw);
            alignment_hold_.set_velocity({c * v_fwd - s * v_lat, s * v_fwd + c * v_lat});
        }
    }
    // A temporary external XY action pauses update(), but physical time still
    // advances. Append its actual command so the Smith predictor cannot replay
    // pre-recovery cruise commands when the route tracker resumes.
    void record_external_command(double yaw, double v_fwd, double v_lat, double yaw_rate)
    {
        previous_forward_command_ = std::max(0.0, v_fwd);
        previous_lateral_command_ = v_lat;
        previous_yaw_command_ = yaw_rate;
        const double keep = std::max(g_.smith_time, g_.smith_yaw_time);
        if (keep <= 0.0 || traj_.empty()) return;
        history_.push_back({v_fwd, v_lat, yaw_rate, yaw});
        const size_t limit = static_cast<size_t>(std::ceil(keep / std::max(1e-3, g_.dt))) + 1;
        while (history_.size() > limit) history_.pop_front();
    }
    // 实测滑行方向不安全时由调用方结束定点滑行(恢复反推)。
    void end_hold_slide() { alignment_hold_.end_slide(); }
    bool hold_sliding() const { return (aligning_ || holonomic_hold_active_) && alignment_hold_.sliding(); }

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
        if (!history_.empty()) history_.back().fwd = std::min(history_.back().fwd, maximum);
    }

    void constrain_lateral_command(double accepted)
    {
        previous_lateral_command_ = accepted;
        if (!history_.empty()) history_.back().lat = accepted;
    }
    double forward_command() const { return previous_forward_command_; }

    // 最近一次用到的前瞻参考点（给可视化）
    Vec2 last_lookahead() const { return last_look_; }
    // Geometric progress for callers that control world-XY velocity themselves.
    // Uses the same bounded, monotone projection; does not run yaw/velocity PID.
    void advance_reference(double px, double py) { if (!traj_.empty()) advance_to_nearest(px, py); }
    TrajPoint reference_at(double s) const { return traj_.empty() ? TrajPoint{} : sample_at(s); }
    // 最近一拍航向律各分量(诊断/离线台架用，不参与控制)
    // limiter：本拍前进限速的主导约束 1=前瞻窗曲率 2=前瞻速度剖面 3=终点/尖角停车律 4=航向门
    //   5=横向加速度(曲率) 6=实测角速度 7=横向速度因子；0=非巡航分支。
    struct Debug {
        double geo_error = 0, lead_error = 0, p_term = 0, d_term = 0, ff_term = 0, desired = 0;
        int limiter = 0; double gate = 1, lat_factor = 1, v_target = 0;
    };
    const Debug& debug() const { return debug_; }

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
    double filtered_desired_rate_ = 0.0, filtered_lateral_target_ = 0.0, filtered_slip_ = 0.0;
    double last_heading_error_ = 0.0, last_curvature_ = 0.0;
    bool aligning_ = false;
    bool alignment_heading_valid_ = false;
    double alignment_heading_ = 0.0;
    double alignment_settled_ = 0.0;
    int turn_direction_ = 0;
    PositionHold alignment_hold_;
    bool holonomic_hold_active_ = false;

    Vec2   last_look_;
    Debug  debug_;
    struct Sent { double fwd, lat, rate, yaw; };
    std::deque<Sent> history_;             // 最近发出的指令(Smith 预估)
    VelCmd compute(double px, double py, double yaw, double v_fwd_est, double v_lat_est,
                   double goal_tol, double measured_yaw_rate);
    Vec2 smith_displacement() const;
    Vec2 issued_world_velocity(double yaw) const;
    double smith_heading() const;
    double handoff_heading_ = 0.0;
    double last_steering_heading_ = 0.0;
    double handoff_remaining_ = 0.0;
    bool handoff_valid_ = false;

    // 从 progress_idx_ 起找离当前位置最近的轨迹点（只向前搜，禁止倒退）
    void advance_to_nearest(double px, double py);
    double projected_progress(double px, double py) const;
    TrajPoint sample_at(double s) const;
    size_t next_corner() const;
    Vec2 stop_anchor(const Vec2& cur, double yaw, double v_fwd_est, double v_lat_est) const;
};

}  // namespace exploration
