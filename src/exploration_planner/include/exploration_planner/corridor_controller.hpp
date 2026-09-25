#pragma once

#include <deque>
#include <string>

#include "exploration_planner/corridor_perception.hpp"
#include "exploration_planner/position_hold.hpp"

namespace exploration {

struct CorridorConfig {
    CorridorPerceptionConfig perception;
    double initial_yaw = -1.5707963267948966;
    // 转到 initial_yaw 后先原地(XY 定点、保持航向)等外部降到走廊高度，再横移进入口(set_altitude_ready)。false=转好即进入(旧行为)
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
    double center_tolerance = 0.04;
    double gate_association = 0.15;
    int confirm_frames = 3;
    double cloud_timeout = 0.50;
    double cloud_window = 0.30;
    double pose_timeout = 0.30;
    // ★按宽度对中模式(2026-09-25)★：不用入口点。configure 的第一个点只是探索终点，H 只决定隧道在哪一侧和
    //   最后停在哪(纵向)。隧道轴 = initial_yaw 方向；降到走廊高度后锁住纵向(轴沿 SLAM Y 时即锁 Y)，
    //   沿横向平移到雷达测得的隧道中线；前进时实时保持中线，只在离门 gate_align_distance 内对准门中心，
    //   离开门同样距离后回到中线。false = 旧的入口点流程。
    bool centerline_mode = false;
    double gate_align_distance = 0.60;   // 离门前沿/后沿多远开始/结束对准门中心(m)
    double center_kp = 1.50;             // 横向对中 P (1/s)
    double center_kd = 0.30;             // 横向实测速度阻尼
    double lateral_max_speed = 0.30;     // 横向纠偏速度上限(m/s)，涡流大不能太小
    double lateral_acceleration = 0.60;  // 横向指令斜坡(m/s²)
    double lateral_latency = 0.40;       // 横向速度响应延迟估计(s)，用于大横移的提前刹车包络
    double lateral_brake_accel = 0.30;   // 包络假设的实际刹车减速度(m/s²)
    double wall_latency = 1.0;           // 离墙净距包络的响应时间(s)：朝墙横移按 v·T+v²/2a≤净距 限速；0=关
    double wall_brake_accel = 0.30;      // 离墙净距包络的刹车减速度(m/s²)
    double smith_time = 0.0;             // 横向 Smith 预估：用最近此时长已发出的速度指令积分预测位置(s)；0=不预估
    double min_forward_speed = 0.20;     // 隧道内前进最低速度(m/s)：只有门未确认/过不去/阻挡/到 H 才低于此值
    double center_filter_time = 0.30;    // 中线估计低通时间常数(s)
    double entry_center_tolerance = 0.05; // 横移到中线的完成容差(m)
};

enum class CorridorPhase { Idle, WaitRoute, Rotate, Entry, Search, Approach, Cross, Finish, Done };

struct CorridorCommand {
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
    void set_altitude_ready(bool ready) { altitude_ready_ = ready; }
    bool active() const { return phase_ != CorridorPhase::Idle; }
    bool startedMoving() const;
    CorridorPhase phase() const { return phase_; }
    Vec2 entry() const { return entry_; }
    Vec2 goal() const { return h_; }
    const Path2& points() const { return points_; }
    const CorridorObservation& observation() const { return observation_; }
    const CorridorObservation& gate() const { return locked_gate_; }
    bool gateLocked() const { return gate_locked_; }
    int gatesPassed() const { return gates_passed_; }
    // 按宽度对中模式：隧道中线是否已由雷达锁定、中线横向偏移(相对锁定点，m)、中线上与飞机同纵向的世界坐标
    bool centerLocked() const { return center_locked_; }
    double centerOffset() const { return center_t_; }
    Vec2 centerWorld(const Vec2& position) const {
        return perception_.toWorld(perception_.longitudinal(position), center_t_);
    }

private:
    CorridorConfig cfg_;
    CorridorPerception perception_;
    CorridorPhase phase_ = CorridorPhase::Idle;
    bool configured_ = false;
    Vec2 entry_, h_, red_;
    PositionHold rotation_hold_, heading_hold_, waiting_hold_;
    bool waiting_for_evidence_ = false;
    bool heading_recovery_ = false;
    bool rotation_done_ = false, altitude_ready_ = true;
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
    int confirmations_ = 0, gates_passed_ = 0;
    double passed_s_ = -1e9;
    double last_gate_back_ = -1e9;
    bool gate_locked_ = false;
    // 按宽度对中模式状态
    bool red_known_ = false;
    bool center_locked_ = false;
    double center_t_ = 0.0, center_candidate_ = 0.0, center_time_ = -1e9;
    int center_votes_ = 0, center_outliers_ = 0;
    double reference_t_ = 0.0;   // H 的横向，自由搜索时选最近的墙对
    bool door_valid_ = false, door_passable_ = false;
    CorridorObservation door_, next_door_;
    struct CommandSample { double time; Vec2 velocity; };
    std::deque<CommandSample> command_history_;   // 已发出的世界系速度指令(Smith 预估)
    Vec2 smithDisplacement(double now) const;
    bool setCenterlineFrame(const Vec2& reference);
    CorridorObservation analyzeCenterline(const Vec2& position, double minimum_gate_s) const;
    void updateCenter(const CorridorObservation& observed, double now);
    void updateDoor(const Vec2& position);
    CorridorCommand updateCenterline(const Vec2& position, double yaw, double vf, double vl,
                                     double now, double dt);
    CorridorCommand followLine(const Vec2& position, double yaw, double s_target, double t_ref,
                               double forward_cap, double lateral_cap, bool allow_reverse,
                               double forward_floor, double dt);
    double alignForwardLimit(double lateral_error, double lateral_speed, double remaining) const;
    bool stable(bool condition, double now);
    CorridorCommand updateMotion(const Vec2& position, double yaw, double vf, double vl,
                                 double now, bool pose_fresh);
    void transition(CorridorPhase phase);
    CorridorCommand hold(const Vec2& position, const std::string& reason);
    CorridorCommand rotateAt(PositionHold& hold, const Vec2& position, double yaw,
                             double vf, double vl, double dt);
    CorridorCommand enter(const Vec2& position, double yaw, double vf, double vl, double dt);
    CorridorCommand translate(const Vec2& position, double yaw, const Vec2& target,
                              double speed, double vf, double vl, double dt,
                              double longitudinal_limit = -1.0);
    double approachForwardLimit(double lateral_error, double lateral_speed,
                                double remaining_distance) const;
};

}  // namespace exploration
