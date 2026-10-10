#pragma once

#include <deque>
#include <string>

#include "exploration_planner/corridor_perception.hpp"
#include "exploration_planner/position_hold.hpp"

namespace exploration {

struct CorridorConfig {
    CorridorPerceptionConfig perception;
    bool nearest_cardinal_yaw = true; // 到终点后选最近的 0/±90/180 度，并锁定到本次通道任务结束
    bool position_control = false; // 主节点参数默认开启；独立旧速度控制用例可关闭
    bool wall_entry = false; // 主节点默认开启；入口由横墙墙端生成，旧入口坐标只兼容历史模式
    double wall_entry_offset = .60; // 雷达中心从横墙入口端向外留出的距离(m)
    double position_lead = 1.00;
    double position_acceleration = .60; // 位置参考的加速度上限；不用于飞控速度/PID接口
    double position_lateral_gain = 1.5; // 中线另一侧的额外位置补偿/当前横向误差
    double position_lateral_max = .18; // 额外横向位置补偿上限，另受门洞机体余量限制
    double position_lateral_brake = .90; // 根据已有横向速度提前收回位置补偿(s)，与增强后的侧向补偿配套
    double position_lateral_response = .30; // 横向位置参考单独收敛时间(s)，不再共用前進减速时间
    double initial_yaw = -1.5707963267948966;
    // 转向期间可同时降高；航向和外部高度反馈(set_altitude_ready)都到位才横移进入口。false=只等待航向
    bool descend_before_entry = false;
    double entry_speed = 0.30;
    double cruise_speed = 0.30;
    double gate_speed = 0.30;
    double align_speed = 0.20;
    double yaw_kp = 1.6;
    double max_yaw_rate = 0.60;
    double yaw_accel = 1.2;
    double acceleration = 0.30;
    double position_kp = 0.8;
    double velocity_kd = 0.35;
    double velocity_filter_tau = 0.15;
    double arrival_hysteresis = 1.5;
    double heading_tolerance = 0.08726646259971647;
    double heading_stop = 0.2617993877991494;
    double point_tolerance = 0.08;
    double stop_speed = 0.04;
    double settle_time = 0.25;
    double lookahead = 0.25;
    double approach_distance = 1.20;
    double moving_lookahead = 0.80;
    double center_prediction_time = 0.25;
    bool continuous_approach = false;
    double continuous_center_reserve = 0.15;
    double continuous_time_margin = 0.50;
    double exit_distance = 1.00;
    double min_forward_speed = 0.30;   // 隧道里往前走的速度下限(m/s)：涡流里太慢推不动；只在快到 H、门还没确认时才低于它
    double lateral_speed = 0.35;       // 隧道里横向纠偏速度上限(m/s)，和前进速度分开算、互不挤占
    double lateral_acceleration = 0.60; // 隧道里横向指令斜坡(m/s²)
    double h_approach_distance = 1.20; // 离 H 只剩这么远时直接朝 H 飞：隧道墙可能在 H 前就结束，不再要求墙延伸过 H
    double center_tolerance = 0.04;
    double gate_association = 0.15;
    int confirm_frames = 3;
    double gate_track_timeout = .35; // 候选门允许的短暂漏检间隔，不能跨长期断流确认
    double gate_filter_time = .15; // 候选门中心滤波时间(s)，锁定后不跟单帧抖动走
    double cloud_timeout = 0.50;
    double cloud_window = 0.30;
    double pose_timeout = 0.30;
};

enum class CorridorPhase { Idle, WaitRoute, Rotate, Entry, Search, Approach, Cross, Finish, Done };

struct CorridorCommand {
    bool position_control = false;
    double forward = 0.0;
    double lateral = 0.0;
    double yaw_rate = 0.0;
    bool finished = false;
    Vec2 target;
    Path2 path;
    std::string status;
};

class CorridorController {
public:
    explicit CorridorController(const CorridorConfig& config = {});
    void reset();
    bool configure(const Vec2& entry, const Vec2& h);
    void start(const Vec2& red, double now);
    void observe(const Path2& points, double stamp);
    void observe(const Path2& points, double stamp, const Vec2& sensor_origin);
    CorridorCommand update(const Vec2& position, double yaw, double forward_velocity,
                           double lateral_velocity, double now, bool pose_fresh);
    bool configured() const { return configured_; }
    // 转头已完成(之后走廊全程用走廊高度)；外部每拍告知实际高度是否已到走廊高度。
    bool rotationDone() const { return rotation_done_; }
    bool headingSelected() const { return heading_selected_; }
    double targetYaw() const { return target_yaw_; } // headingSelected() 后有效
    bool positionActive() const { return position_active_; }
    void positionFeedback(bool ready, const Vec2& anchor);
    void positionFeedbackTimeout(const Vec2& position);
    void set_altitude_ready(bool ready) { altitude_ready_ = ready; }
    bool active() const { return phase_ != CorridorPhase::Idle; }
    bool startedMoving() const;
    CorridorPhase phase() const { return phase_; }
    Vec2 entry() const { return entry_; }
    Vec2 goal() const { return h_; }
    Vec2 axis() const { const auto p=perception_.toWorld(1,0);return {p.x-entry_.x,p.y-entry_.y}; }
    const CorridorEntrance& entrancePlan() const { return entrance_plan_; }
    bool entranceReady() const { return !cfg_.wall_entry || entrance_locked_; }
    const Path2& plannedPath() const { return planned_path_; }
    const Path2& points() const { return points_; }
    const CorridorObservation& observation() const { return observation_; }
    const CorridorObservation& gate() const { return locked_gate_; }
    bool gateLocked() const { return gate_locked_; }
    const CorridorObservation& displayGate() const;
    int gateConfirmations() const { return confirmations_; }
    bool positionGateAligned() const { return position_gate_aligned_; }
    bool positionRecovering() const { return position_recovering_; }
    int gatesPassed() const { return gates_passed_; }

private:
    CorridorConfig cfg_;
    CorridorPerception perception_;
    CorridorPhase phase_ = CorridorPhase::Idle;
    bool configured_ = false;
    Vec2 entry_, h_, red_;
    CorridorEntrance entrance_plan_, entrance_candidate_;
    bool entrance_locked_=false, entrance_staged_=false;
    int entrance_confirmations_=0;
    unsigned long entrance_sequence_=0;
    double entrance_time_=-1e9;
    Path2 planned_path_;
    PositionHold rotation_hold_, heading_hold_, waiting_hold_;
    bool waiting_for_evidence_ = false;
    bool heading_recovery_ = false;
    bool rotation_done_ = false, altitude_ready_ = true;
    bool heading_selected_ = false;
    double target_yaw_ = 0.0;
    bool position_active_ = false, position_ready_ = false, position_waiting_ = false;
    bool position_gate_aligned_ = false, position_gate_approach_ = false;
    bool position_recovering_ = false;
    Vec2 position_reference_;
    Vec2 position_reference_velocity_; // 仅用于生成连续切向的位置点，不作为速度指令发送
    struct Cloud { double stamp; Path2 points; };
    std::deque<Cloud> clouds_;
    Path2 points_;
    Path2 visibility_points_;
    Vec2 sensor_origin_;
    bool has_sensor_origin_ = false;
    double cloud_time_ = -1e9;
    double evidence_time_ = -1e9;
    double last_update_ = -1.0;
    double settled_since_ = -1.0;
    Vec2 previous_velocity_, filtered_velocity_;
    bool velocity_valid_ = false;
    double previous_yaw_rate_ = 0.0;
    unsigned long cloud_sequence_ = 0, processed_sequence_ = 0;
    unsigned long candidate_sequence_ = 0;
    CorridorObservation observation_, candidate_, locked_gate_;
    double candidate_time_ = -1e9;
    int confirmations_ = 0, gates_passed_ = 0;
    double passed_s_ = -1e9;
    double last_gate_back_ = -1e9;
    bool gate_locked_ = false;
    bool stable(bool condition, double now);
    void confirmGate(const CorridorObservation& observed, double now);
    void trackGate(const Vec2& position, double now);
    void trackEntrance(const Vec2& position, double now);
    CorridorCommand updateMotion(const Vec2& position, double yaw, double vf, double vl,
                                 double now, bool pose_fresh);
    void transition(CorridorPhase phase);
    CorridorCommand hold(const Vec2& position, const std::string& reason);
    CorridorCommand rotateAt(PositionHold& hold, const Vec2& position, double yaw,
                             double vf, double vl, double dt);
    CorridorCommand enter(const Vec2& position, double yaw, double vf, double vl, double dt);
    CorridorCommand translate(const Vec2& position, double yaw, const Vec2& target,
                              double speed, double vf, double vl, double dt,
                              double longitudinal_limit = -1.0, double min_forward = 0.0);
    CorridorCommand forwardCorridor(const Vec2& position, double yaw, double vf, double vl, double dt);
    CorridorCommand positionCommand(const Vec2& position, const Vec2& target, double speed, double dt);
    double approachForwardLimit(double lateral_error, double lateral_speed,
                                double remaining_distance) const;
};

}  // namespace exploration
