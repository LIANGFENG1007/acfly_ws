#include "exploration_planner/corridor_controller.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace exploration {
namespace {
double distance(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }
double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
bool finite(const Vec2& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
bool positive(std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v) && v > 0.0; });
}
bool nonnegative(std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v) && v >= 0.0; });
}
}

CorridorController::CorridorController(const CorridorConfig& config)
    : cfg_(config), perception_(config.perception)
{
    const auto& p = cfg_.perception;
    if (!positive({cfg_.entry_speed, cfg_.cruise_speed, cfg_.gate_speed, cfg_.align_speed,
                   cfg_.yaw_kp, cfg_.max_yaw_rate, cfg_.yaw_accel, cfg_.acceleration,
                   cfg_.position_kp, cfg_.velocity_kd, cfg_.velocity_filter_tau, cfg_.arrival_hysteresis,
                   cfg_.heading_tolerance, cfg_.heading_stop, cfg_.point_tolerance,
                   cfg_.stop_speed, cfg_.lookahead, cfg_.approach_distance, cfg_.exit_distance,
                   cfg_.moving_lookahead, cfg_.center_prediction_time, cfg_.position_lead, cfg_.position_acceleration,
                   cfg_.position_lateral_response,cfg_.position_brake_time,cfg_.gate_track_timeout,cfg_.detection_new_weight,cfg_.wall_entry_offset,
                   cfg_.center_tolerance, cfg_.gate_association, cfg_.cloud_timeout,
                   cfg_.pose_timeout, p.corridor_width, p.robot_width, p.cell_size, p.lookahead,
                   p.wall_min_span, p.max_wall_gap, p.gate_min_span, p.gate_cluster_depth,
                   p.gate_max_depth, p.surface_sample_gap, p.min_observed_ahead}) ||
        !nonnegative({cfg_.settle_time, cfg_.cloud_window, cfg_.continuous_center_reserve, cfg_.min_forward_speed,
                      cfg_.position_lateral_gain,cfg_.position_lateral_max,cfg_.position_lateral_brake,
                      cfg_.continuous_time_margin, p.minimum_gap_extra, p.lookbehind,
                      p.wall_search_tolerance, p.wall_exclusion_band, p.endpoint_exclusion}) ||
        cfg_.detection_new_weight > 1.0 || !std::isfinite(cfg_.initial_yaw) || cfg_.confirm_frames < 1 || p.wall_min_points < 2 ||
        p.gate_min_points < 2 || p.gate_min_clear_rays < 1 || p.corridor_width <= p.robot_width ||
        cfg_.heading_stop <= cfg_.heading_tolerance || cfg_.arrival_hysteresis < 1.0 ||
        (cfg_.wall_entry && cfg_.wall_entry_offset<=p.robot_width*.5+.02)) {
        throw std::invalid_argument("Invalid corridor geometry, speed, tolerance, or timing parameter");
    }
}

void CorridorController::reset()
{
    *this = CorridorController(cfg_);
}

bool CorridorController::startedMoving() const
{
    return phase_ != CorridorPhase::Idle && phase_ != CorridorPhase::WaitRoute;
}

bool CorridorController::configure(const Vec2& entry, const Vec2& h)
{
    if (startedMoving() || !finite(h) || (!cfg_.wall_entry && (!finite(entry) || distance(entry, h) < 0.5)))
        return false;
    entry_ = cfg_.wall_entry ? Vec2{h.x,h.y+1.0} : entry; // 自动入口不使用旧第一个点
    h_ = h;
    perception_.setFrame(entry_, h_);
    configured_ = true;
    if (phase_ == CorridorPhase::WaitRoute) transition(CorridorPhase::Rotate);
    return true;
}

void CorridorController::start(const Vec2& red, double now)
{
    red_ = red;
    rotation_hold_.begin(red);
    heading_recovery_ = false;
    rotation_done_ = false;
    heading_selected_ = false;
    target_yaw_ = 0.0;
    position_active_=position_ready_=position_waiting_=position_gate_aligned_=false;
    position_height_admitted_=false;
    position_measured_braking_=false;
    position_gate_approach_=false;
    position_recovering_=false;
    position_reference_=red;
    position_reference_velocity_={};
    previous_yaw_rate_ = 0.0;
    settled_since_ = -1.0;
    candidate_={};confirmations_=0;candidate_time_=-1e9;candidate_sequence_=0;
    gate_locked_=gate_blocked_=candidate_blocked_=false;locked_gate_={};
    passed_s_=last_gate_back_=-1e9;gates_passed_=0;
    entrance_plan_={};entrance_candidate_={};entrance_locked_=entrance_staged_=false;
    entrance_confirmations_=0;entrance_sequence_=0;entrance_time_=-1e9;planned_path_.clear();
    if(cfg_.wall_entry && configured_) {entry_=red;perception_.setFrame(entry_,h_);}
    last_update_ = now;
    transition(configured_ ? CorridorPhase::Rotate : CorridorPhase::WaitRoute);
}

void CorridorController::observe(const Path2& points, double stamp)
{
    if (!std::isfinite(stamp)) return;
    if (stamp < cloud_time_) {
        cloud_voxels_.clear();
        visibility_points_.clear();
        evidence_time_ = -1e9;
    }
    cloud_time_ = stamp;
    if (!points.empty()) {
        visibility_points_ = points;
        has_sensor_origin_ = false;
        evidence_time_ = stamp;
        ++cloud_sequence_;  // Empty packets are not independent door confirmations.
    }
    // 每格保留最新真实坐标和观测时间。无需每帧在主锁内重扫过去5秒的所有原始点；
    // 格中最新点过期时该格的旧点也已过期，和原来逐帧覆盖的窗口语义相同。
    const double cell = std::max(0.01, cfg_.perception.cell_size * 0.5);
    for (const auto& p : points) {
        if (!finite(p)) continue;
        cloud_voxels_[{static_cast<long long>(std::floor(p.x / cell)),
                       static_cast<long long>(std::floor(p.y / cell))}] = {stamp,p};
    }
    points_.clear();
    // Sparse scan packets may alternate between wall hits and no height-slice
    // hits. Keep only the explicitly bounded measured window, never infer free
    // space from an empty packet or extend the age of its last valid rays.
    for(auto it=cloud_voxels_.begin();it!=cloud_voxels_.end();) {
        if(stamp-it->second.stamp>cfg_.cloud_window)it=cloud_voxels_.erase(it);
        else {points_.push_back(it->second.point);++it;}
    }
}

void CorridorController::observe(const Path2& points, double stamp, const Vec2& sensor_origin)
{
    observe(points, stamp);
    if (!points.empty()) {
        sensor_origin_ = sensor_origin;
        has_sensor_origin_ = true;
    }
}

void CorridorController::transition(CorridorPhase phase)
{
    if (phase == phase_) return;
    phase_ = phase;
    settled_since_ = -1.0;
    heading_recovery_ = false;
}

bool CorridorController::stable(bool condition, double now)
{
    if (!condition) { settled_since_ = -1.0; return false; }
    if (settled_since_ < 0.0) settled_since_ = now;
    return now - settled_since_ >= cfg_.settle_time;
}

const CorridorObservation& CorridorController::displayGate() const
{
    // 显示的是本次任务记住的几何，不以当前点云/单帧检测是否有效来闪灭。
    // 数据新鲜度和实测受阻另行约束运动，不能用“还显示着”代表“可以穿过”。
    return gate_locked_ ? locked_gate_ : candidate_;
}

CorridorObservation CorridorController::averageGate(const CorridorObservation& previous,
                                                    const CorridorObservation& observed) const
{
    if(!previous.gate_passable)return observed;
    const double w=cfg_.detection_new_weight;
    auto result=observed;
    const double center=(1-w)*.5*(previous.gap_left+previous.gap_right)+
        w*.5*(observed.gap_left+observed.gap_right);
    result.gate_s=(1-w)*previous.gate_s+w*observed.gate_s;
    // 中心严格加权更新。通行宽度只取当前实测开口中围绕该中心的安全部分；
    // 平滑中心尚未进入可用开口时保留标记，但暂不允许穿门。
    result.gap_width=std::max(0.0,std::min((1-w)*previous.gap_width+w*observed.gap_width,
        2*std::min(center-observed.gap_right,observed.gap_left-center)));
    result.gap_left=center+result.gap_width*.5;result.gap_right=center-result.gap_width*.5;
    result.gate_depth=observed.gate_depth+2*std::abs(result.gate_s-observed.gate_s);
    result.gate_center=perception_.toWorld(result.gate_s,center);
    return result;
}

void CorridorController::confirmGate(const CorridorObservation& observed,double now)
{
    if(!observed.gate_passable || observed.gate_s<=passed_s_ || candidate_sequence_==cloud_sequence_)return;
    const bool same=candidate_.gate_passable &&
        distance(candidate_.gate_center,observed.gate_center)<=cfg_.gate_association;
    const bool confirmed=confirmations_>=cfg_.confirm_frames;
    const bool recent=now>=candidate_time_ && now-candidate_time_<=cfg_.gate_track_timeout;
    // 已确认的门保留身份，不被另一扇门/离群帧抢走；未确认候选超时后允许重新寻找。
    if(candidate_.gate_passable && !same && (confirmed || recent))return;
    if(!confirmed && !recent)confirmations_=0;
    candidate_=same ? averageGate(candidate_,observed) : observed;
    confirmations_=same ? std::min(cfg_.confirm_frames,confirmations_+1) : 1;
    candidate_blocked_=candidate_.gap_width<cfg_.perception.robot_width+cfg_.perception.minimum_gap_extra;
    candidate_time_=now;candidate_sequence_=cloud_sequence_;
}

void CorridorController::trackEntrance(const Vec2& position,double now)
{
    if(!cfg_.wall_entry)return;
    // 完成入口动作后，通道坐标系不再被身后的入口观测拖动；入口记录保留到任务重置。
    if(entrance_locked_ && phase_!=CorridorPhase::Rotate && phase_!=CorridorPhase::Entry)return;
    const bool recent=now>=entrance_time_ && now-entrance_time_<=cfg_.gate_track_timeout;
    if(!entrance_locked_ && !recent)entrance_confirmations_=0;
    if(entrance_sequence_==cloud_sequence_ || now<evidence_time_ ||
       now-evidence_time_>cfg_.cloud_timeout || points_.empty() || !has_sensor_origin_)return;
    entrance_sequence_=cloud_sequence_;
    auto found=perception_.entrance(points_,visibility_points_,sensor_origin_,red_,h_,cfg_.wall_entry_offset,
        entrance_locked_ ? &entrance_plan_ : nullptr);
    if(!found.valid) {
        if(!entrance_plan_.valid)entrance_plan_.reason=found.reason;
        return;
    }
    const bool same=entrance_candidate_.valid &&
        distance(found.wall_end,entrance_candidate_.wall_end)<=cfg_.gate_association &&
        found.axis.x*entrance_candidate_.axis.x+found.axis.y*entrance_candidate_.axis.y>std::cos(.05);
    if(entrance_candidate_.valid && !same && (entrance_locked_ || recent))return;
    if(same) {
        const double w=cfg_.detection_new_weight;
        const auto mix=[&](Vec2 old,Vec2 point){return Vec2{(1-w)*old.x+w*point.x,(1-w)*old.y+w*point.y};};
        found.wall_end=mix(entrance_candidate_.wall_end,found.wall_end);
        found.axis=mix(entrance_candidate_.axis,found.axis);
        const double length=std::hypot(found.axis.x,found.axis.y);
        found.axis={found.axis.x/length,found.axis.y/length};
        Vec2 n{-found.axis.y,found.axis.x};
        if(n.x*found.inward.x+n.y*found.inward.y<0)n={-n.x,-n.y};
        found.inward=n;
        const double outside=(red_.x-found.wall_end.x)*n.x+(red_.y-found.wall_end.y)*n.y;
        const Vec2 aperture{found.wall_end.x-found.axis.x*cfg_.wall_entry_offset,
                            found.wall_end.y-found.axis.y*cfg_.wall_entry_offset};
        found.staging={aperture.x+n.x*outside,aperture.y+n.y*outside};
        found.center={aperture.x+n.x*cfg_.perception.corridor_width*.5,
                      aperture.y+n.y*cfg_.perception.corridor_width*.5};
        const double goal_s=(h_.x-found.center.x)*found.axis.x+(h_.y-found.center.y)*found.axis.y;
        found.line_goal={found.center.x+found.axis.x*goal_s,found.center.y+found.axis.y*goal_s};
        const double radius=cfg_.perception.robot_width*.5+.02;
        if(goal_s<.5 || !CorridorPerception::pathClear(points_,red_,found.staging,radius) ||
           !CorridorPerception::pathClear(points_,found.staging,found.center,radius))return;
    }
    entrance_confirmations_=same ? std::min(cfg_.confirm_frames,entrance_confirmations_+1) : 1;
    entrance_candidate_=found;entrance_plan_=found;entrance_time_=now;
    if(!entrance_locked_ && entrance_confirmations_<cfg_.confirm_frames)return;
    // 更新入口时保留门的世界坐标和确认次数，不能因坐标系原点小幅修正重新闪灭门标记。
    const auto old_frame=perception_;
    entry_=found.center;perception_.setFrame(entry_,found.line_goal);
    const auto reframe=[&](CorridorObservation& gate) {
        if(!gate.gate_passable)return;
        const auto left=old_frame.toWorld(gate.gate_s,gate.gap_left);
        const auto right=old_frame.toWorld(gate.gate_s,gate.gap_right);
        const auto front=old_frame.toWorld(gate.gate_s-gate.gate_depth*.5,0);
        const auto back=old_frame.toWorld(gate.gate_s+gate.gate_depth*.5,0);
        gate.gate_s=perception_.longitudinal(gate.gate_center);
        gate.gap_left=perception_.lateral(left);gate.gap_right=perception_.lateral(right);
        gate.gap_width=gate.gap_left-gate.gap_right;
        gate.gate_depth=std::abs(perception_.longitudinal(back)-perception_.longitudinal(front))+
            std::abs(perception_.longitudinal(left)-perception_.longitudinal(right));
    };
    reframe(candidate_);reframe(locked_gate_);
    entrance_locked_=true;
    planned_path_={position,found.staging,found.center};
}

void CorridorController::trackGate(const Vec2& position,double now)
{
    if(cfg_.wall_entry && !entrance_locked_)return;
    if(confirmations_<cfg_.confirm_frames &&
       (now<candidate_time_ || now-candidate_time_>cfg_.gate_track_timeout))confirmations_=0;
    if(cloud_sequence_==processed_sequence_ || now<evidence_time_ ||
       now-evidence_time_>cfg_.cloud_timeout || points_.empty())return;
    processed_sequence_=cloud_sequence_;
    const auto analyze=[&](double minimum) {
        return perception_.analyze(points_,position,minimum,&visibility_points_,
            has_sensor_origin_ ? &sensor_origin_ : nullptr);
    };
    observation_=analyze(passed_s_);
    if(gate_locked_ && observation_.gate_observed &&
       std::abs(observation_.gate_s-locked_gate_.gate_s)<=cfg_.gate_association) {
        if(observation_.gate_passable &&
           distance(observation_.gate_center,locked_gate_.gate_center)<=cfg_.gate_association) {
            locked_gate_=averageGate(locked_gate_,observation_);
            gate_blocked_=locked_gate_.gap_width<cfg_.perception.robot_width+cfg_.perception.minimum_gap_extra;
        } else if(observation_.reason=="gate_closed_or_too_narrow")gate_blocked_=true;
    }
    const auto observed=gate_locked_ ? analyze(locked_gate_.gate_s+locked_gate_.gate_depth*.5+
        std::max(cfg_.perception.cell_size,cfg_.perception.gate_cluster_depth)) : observation_;
    if(observed.gate_passable)confirmGate(observed,now);
    else if(observed.reason=="gate_closed_or_too_narrow" && candidate_.gate_passable &&
            std::abs(observed.gate_s-candidate_.gate_s)<=cfg_.gate_association) {
        candidate_blocked_=true;
        if(confirmations_<cfg_.confirm_frames)confirmations_=0;
    }
}

CorridorCommand CorridorController::hold(const Vec2& position, const std::string& reason)
{
    if (position_active_) {
        position_recovering_=false;
        if (!position_waiting_ && finite(position)) position_reference_=position;
        position_waiting_=true;
        position_reference_velocity_={}; // 定位/交接/高度异常时立即保持，不继续执行旧曲线
        CorridorCommand cmd;cmd.position_control=true;cmd.target=position_reference_;
        cmd.path={position,position_reference_};cmd.status=reason;
        return cmd;
    }
    previous_velocity_ = {};
    previous_yaw_rate_ = 0.0;
    rotation_hold_.stop();
    heading_hold_.stop();
    CorridorCommand cmd;
    cmd.target = position;
    cmd.status = reason;
    return cmd;
}

CorridorCommand CorridorController::rotateAt(PositionHold& hold, const Vec2& position,
                                              double yaw, double vf, double vl, double dt)
{
    auto correction = hold.update(position, yaw, vf, vl, cfg_.position_kp,
                                  cfg_.velocity_kd, cfg_.align_speed, cfg_.acceleration, dt);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const Vec2 world{c * correction.x - s * correction.y, s * correction.x + c * correction.y};
    const double speed = std::hypot(world.x, world.y);
    const double horizon = std::max(0.5, speed / (2.0 * cfg_.acceleration));
    const Vec2 projected{position.x + world.x * horizon, position.y + world.y * horizon};
    // Rotation predates corridor cloud acquisition. Available returns can veto
    // XY correction without cancelling yaw; empty scans never authorize entry.
    if (!points_.empty() && !CorridorPerception::pathClear(
            points_, position, projected, cfg_.perception.robot_width * 0.5)) {
        correction = {};
        hold.stop();
    }
    CorridorCommand cmd;
    cmd.forward = correction.x;
    cmd.lateral = correction.y;
    cmd.target = hold.anchor();
    cmd.path = {position, cmd.target};
    const double desired = std::clamp(cfg_.yaw_kp * angle(target_yaw_ - yaw),
                                      -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
    const double step = cfg_.yaw_accel * dt;
    cmd.yaw_rate = std::clamp(desired, previous_yaw_rate_ - step, previous_yaw_rate_ + step);
    previous_yaw_rate_ = cmd.yaw_rate;
    previous_velocity_ = {c * cmd.forward - s * cmd.lateral, s * cmd.forward + c * cmd.lateral};
    return cmd;
}

CorridorCommand CorridorController::enter(const Vec2& position, double yaw,
                                         double vf, double vl, double dt)
{
    if(cfg_.wall_entry) {
        if(!entrance_locked_)return hold(position,"Entry: "+entrance_plan_.reason);
        // 必须实际到达墙外的对准位置，不能因参考点已到就提前斜切进入。
        if(!entrance_staged_ && distance(position,entrance_plan_.staging)<=std::min(.08,cfg_.point_tolerance))
            entrance_staged_=true;
        if(!entrance_staged_) {
            planned_path_={position,entrance_plan_.staging,entry_};
            auto cmd=translate(position,yaw,entrance_plan_.staging,cfg_.entry_speed,vf,vl,dt);
            if(cmd.status.empty())cmd.status="Entry: aligning outside wall-end opening";
            return cmd;
        }
        planned_path_={position,entry_};
        auto cmd=translate(position,yaw,entry_,cfg_.entry_speed,vf,vl,dt);
        if(cmd.status.empty())cmd.status="Entry: aligned, entering perpendicular to wall";
        return cmd;
    }
    planned_path_={position,entry_};
    auto cmd = translate(position, yaw, entry_, cfg_.entry_speed, vf, vl, dt);
    if (cmd.status.empty()) cmd.status = "Translating to entry with fixed heading";
    return cmd;
}

CorridorCommand CorridorController::translate(const Vec2& position, double yaw,
                                             const Vec2& target, double speed,
                                             double vf, double vl, double dt,
                                             double longitudinal_limit, double min_forward)
{
    if (position_active_) return positionCommand(position,target,speed,dt);
    const double error = angle(target_yaw_ - yaw);
    const bool forward_phase = phase_ == CorridorPhase::Search ||
        phase_ == CorridorPhase::Approach || phase_ == CorridorPhase::Cross;
    // 隧道里往前走时不原地回正：边走边纠机头(2026-09-26 用户要求前进不停)
    if (!forward_phase && !heading_recovery_ && std::fabs(error) > cfg_.heading_stop) {
        heading_hold_.begin(position);
        heading_recovery_ = true;
    }
    if (heading_recovery_) {
        if (std::fabs(error) > cfg_.heading_tolerance) {
            auto cmd = rotateAt(heading_hold_, position, yaw, vf, vl, dt);
            cmd.status = "Restoring corridor heading with XY hold";
            return cmd;
        }
        heading_recovery_ = false;
    }
    const double d = distance(position, target);
    const double deadband = std::min(cfg_.point_tolerance, cfg_.center_tolerance) * 0.25;
    const double c = std::cos(yaw), s = std::sin(yaw);
    const Vec2 measured{c * vf - s * vl, s * vf + c * vl};
    const double error_scale = d > deadband ? (d - deadband) / d : 0.0;
    Vec2 desired{cfg_.position_kp * (target.x - position.x) * error_scale - cfg_.velocity_kd * measured.x,
                 cfg_.position_kp * (target.y - position.y) * error_scale - cfg_.velocity_kd * measured.y};
    const bool forward_corridor = phase_ == CorridorPhase::Search ||
        phase_ == CorridorPhase::Approach || phase_ == CorridorPhase::Cross;
    const Vec2 axis=this->axis();
    if (forward_corridor && perception_.longitudinal(position) < perception_.length()) {
        const double along = desired.x * axis.x + desired.y * axis.y;
        if (along < 0.0) { desired.x -= along * axis.x; desired.y -= along * axis.y; }
    }
    if (forward_corridor) {
        // ★隧道里纵向/横向分开算★(2026-09-26 用户要求)：纵向在 [min_forward, speed] 之间，只有快到目标(H、门还没确认时的
        //   门前等待点)才放开下限减速停住；横向独立限幅 lateral_speed、独立斜坡，不挤占前进。不做"快撞了就刹停"。
        const Vec2 normal{-axis.y, axis.x};
        const double backward = perception_.longitudinal(position) < perception_.length() ? 0.0 : -speed;   // 冲过 H 允许退回
        double along = std::clamp(desired.x * axis.x + desired.y * axis.y, backward, speed);
        double cross = std::clamp(desired.x * normal.x + desired.y * normal.y, -cfg_.lateral_speed, cfg_.lateral_speed);
        const double floor = std::min(min_forward, speed);
        const double remaining_along = (target.x - position.x) * axis.x + (target.y - position.y) * axis.y;
        if (floor > 0.0 && remaining_along > cfg_.point_tolerance + floor * floor / (2.0 * cfg_.acceleration))
            along = std::max(along, floor);
        if (longitudinal_limit >= 0.0) along = std::min(along, longitudinal_limit);
        const double ratio = std::clamp((std::fabs(error) - cfg_.heading_tolerance) /
            (cfg_.heading_stop - cfg_.heading_tolerance), 0.0, 1.0);
        const double head_gate = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio);
        (void)head_gate;   // 前进阶段机头偏差不让速度打折(世界系速度照发，机头由 yaw 回路边走边纠)
        const double previous_along = previous_velocity_.x * axis.x + previous_velocity_.y * axis.y;
        const double previous_cross = previous_velocity_.x * normal.x + previous_velocity_.y * normal.y;
        along = std::clamp(along, previous_along - cfg_.acceleration * dt, previous_along + cfg_.acceleration * dt);
        cross = std::clamp(cross, previous_cross - cfg_.lateral_acceleration * dt,
                           previous_cross + cfg_.lateral_acceleration * dt);
        const Vec2 velocity{axis.x * along + normal.x * cross, axis.y * along + normal.y * cross};
        CorridorCommand cmd;
        cmd.forward = c * velocity.x + s * velocity.y;
        cmd.lateral = -s * velocity.x + c * velocity.y;
        const double desired_yaw = std::clamp(cfg_.yaw_kp * error, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
        const double yaw_step = cfg_.yaw_accel * dt;
        cmd.yaw_rate = std::clamp(desired_yaw, previous_yaw_rate_ - yaw_step, previous_yaw_rate_ + yaw_step);
        cmd.target = target;
        cmd.path = {position, target};
        previous_velocity_ = velocity;
        previous_yaw_rate_ = cmd.yaw_rate;
        return cmd;
    }
    if (longitudinal_limit >= 0.0) {
        const double along = desired.x * axis.x + desired.y * axis.y;
        const double limited = std::clamp(along, 0.0, longitudinal_limit);
        desired.x += (limited - along) * axis.x;
        desired.y += (limited - along) * axis.y;
    }
    const double desired_speed = std::hypot(desired.x, desired.y);
    if (desired_speed > speed && desired_speed > 1e-9) {
        desired.x *= speed / desired_speed;
        desired.y *= speed / desired_speed;
    }
    // Reduce motion continuously between the alignment and stop thresholds.
    const double ratio = std::clamp((std::fabs(error) - cfg_.heading_tolerance) /
        (cfg_.heading_stop - cfg_.heading_tolerance), 0.0, 1.0);
    const double head_gate = 1.0 - ratio * ratio * (3.0 - 2.0 * ratio);
    desired.x *= head_gate;
    desired.y *= head_gate;
    Vec2 delta{desired.x - previous_velocity_.x, desired.y - previous_velocity_.y};
    const double delta_norm = std::hypot(delta.x, delta.y);
    const double step = cfg_.acceleration * dt;
    if (delta_norm > step) { delta.x *= step / delta_norm; delta.y *= step / delta_norm; }
    Vec2 velocity{previous_velocity_.x + delta.x, previous_velocity_.y + delta.y};
    const double velocity_norm = std::hypot(velocity.x, velocity.y);
    if (velocity_norm > speed && velocity_norm > 1e-9) {
        velocity.x *= speed / velocity_norm;
        velocity.y *= speed / velocity_norm;
    }
    if (head_gate == 0.0) velocity = {};
    const double horizon = std::min(d, std::max(cfg_.lookahead, std::hypot(vf, vl) * 0.5));
    const Vec2 carrot = d > 1e-6 ? Vec2{position.x + (target.x - position.x) * horizon / d,
                                      position.y + (target.y - position.y) * horizon / d} : position;
    // 只在横移进入口时保留"快撞了就停"(要从墙的开口钻进去)；到 H 定点不因出口结构卡住
    const bool collision_guard = phase_ == CorridorPhase::Entry;
    if (collision_guard && !CorridorPerception::pathClear(points_, position, carrot, cfg_.perception.robot_width * 0.5))
        return hold(position, "Translation blocked: waiting for a clear centerline");
    const double commanded_speed = std::hypot(velocity.x, velocity.y);
    if (commanded_speed > 1e-6) {
        const double travel = std::max(commanded_speed * 0.5,
                                      commanded_speed * commanded_speed / (2.0 * cfg_.acceleration));
        const Vec2 projected{position.x + velocity.x * travel / commanded_speed,
                             position.y + velocity.y * travel / commanded_speed};
        if (collision_guard && !CorridorPerception::pathClear(points_, position, projected, cfg_.perception.robot_width * 0.5))
            return hold(position, "Braking: commanded motion is blocked");
    }
    CorridorCommand cmd;
    cmd.forward = c * velocity.x + s * velocity.y;
    cmd.lateral = -s * velocity.x + c * velocity.y;
    const double desired_yaw = std::clamp(cfg_.yaw_kp * error, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
    const double yaw_step = cfg_.yaw_accel * dt;
    cmd.yaw_rate = std::clamp(desired_yaw, previous_yaw_rate_ - yaw_step, previous_yaw_rate_ + yaw_step);
    cmd.target = target;
    cmd.path = {position, target};
    previous_velocity_ = velocity;
    previous_yaw_rate_ = cmd.yaw_rate;
    return cmd;
}

// 隧道里往前走：纵向恒定 cruise_speed(不低于 min_forward_speed)，只在快到 H 时减速停到 H；
// 旧速度横向目标：平时是原行进线(t=0)，门前 approach_distance 内对中；位置接口可提前斜向对中。
// 旧速度接口保持连续前进；位置接口还要求新鲜点云/扫掠路径可通行，并在门前先对中。
// 门的确认/锁定/通过计数两种接口共用。
CorridorCommand CorridorController::forwardCorridor(const Vec2& position, double yaw, double vf, double vl, double dt)
{
    const double s = perception_.longitudinal(position);
    const double radius = cfg_.perception.robot_width * 0.5;
    const double resolution = cfg_.perception.cell_size;
    const bool near_h = perception_.length() - s <= cfg_.h_approach_distance;
    // 1. 当前门的通过判定：出门 exit_distance 算通过；机身已离开门、且下一扇门已进入对齐区(或快到 H)时提前交接
    if (gate_locked_) {
        const double back = locked_gate_.gate_s + locked_gate_.gate_depth * 0.5;
        const double body_clear_s = back + radius + resolution;
        const bool next_in_zone = candidate_.gate_passable && candidate_.gate_s > back + resolution &&
            s >= candidate_.gate_s - candidate_.gate_depth * 0.5 - cfg_.approach_distance - 0.05;
        if (s >= back + cfg_.exit_distance || (s >= body_clear_s && (near_h || next_in_zone))) {
            last_gate_back_ = back;
            passed_s_ = back + resolution;
            gate_locked_ = false;gate_blocked_=false;
            position_gate_aligned_=false;
            position_gate_approach_=false;
            ++gates_passed_;
            // 下一扇门在锁定当前门期间一直在确认，保留它的确认次数；没有候选才重新看一眼
            if (!(candidate_.gate_passable && candidate_.gate_s > passed_s_)) {
                candidate_={};confirmations_=0;candidate_time_=-1e9;candidate_blocked_=false;
                observation_ = perception_.analyze(points_, position, passed_s_, &visibility_points_,
                    has_sensor_origin_ ? &sensor_origin_ : nullptr);
                if (observation_.gate_passable && observation_.gate_s > passed_s_) {
                    confirmGate(observation_,last_update_);
                }
            }
        }
    }
    // 2. 锁定已确认的最近一扇门(交接后当拍就能锁上下一扇)
    if (!gate_locked_ && confirmations_ >= cfg_.confirm_frames &&
        candidate_.gate_passable && candidate_.gate_s > passed_s_) {
        locked_gate_ = candidate_;gate_blocked_=candidate_blocked_;
        gate_locked_ = true;
        candidate_={};confirmations_=0;candidate_time_=-1e9;candidate_blocked_=false;
        position_gate_aligned_=false;
        position_gate_approach_=false;
    }
    if((gate_locked_ && gate_blocked_) || (!gate_locked_ && candidate_blocked_)) {
        // 保留标记/暂停穿门不能覆盖已有的贴墙脱离机制；机身已重叠时仍允许
        // 经逐点单调远离检查的横向回正，不把当前位置锁在墙面上。
        if(position_active_ && !CorridorPerception::pathClear(points_,position,position,radius)) {
            const auto& gate=gate_locked_ ? locked_gate_ : candidate_;
            return positionCommand(position,perception_.toWorld(s,.5*(gate.gap_left+gate.gap_right)),
                                   cfg_.align_speed,dt);
        }
        return hold(position,"Doorway blocked by current measurement; retaining tracked marker");
    }
    // 3. 横向目标：平时原行进线(t=0)；锁定门后离门前沿 approach_distance 内对门中心
    double t_ref = 0.0;
    CorridorPhase phase = CorridorPhase::Search;
    std::string status = "Forward: holding travel line";
    if (gate_locked_) {
        const double front = locked_gate_.gate_s - locked_gate_.gate_depth * 0.5;
        if (s >= front - cfg_.approach_distance - 0.05) {
            t_ref = 0.5 * (locked_gate_.gap_left + locked_gate_.gap_right);
            phase = s >= front - radius ? CorridorPhase::Cross : CorridorPhase::Approach;
            status = phase == CorridorPhase::Cross ? "Crossing door" : "Aligning to door center";
        } else {
            phase = CorridorPhase::Approach;
            status = "Forward: door locked, before alignment zone";
        }
    }
    phase_ = phase;   // 只用于显示/日志，不重置计时和机头状态
    // 4. 纵向目标总在前方；快到 H 且前面没有要对准的门时指向 H，让它减速停到 H
    // moving_lookahead 是内部追踪终点，不是实际发送点。位置模式要为参考收敛留出距离，
    // 否则旧的 0.8 m 内部终点会把调大的 position_lead 截短，飞机仍拉不开位置差。
    // 最终输出仍由 positionCommand 限制在 position_lead 内，包含减速停止距离。
    const double reference_speed=std::max(cfg_.cruise_speed,cfg_.gate_speed);
    const double moving_ahead=position_active_ ? std::max(cfg_.moving_lookahead,
        cfg_.position_lead+reference_speed*std::max(.5,reference_speed/cfg_.position_acceleration))
        : cfg_.moving_lookahead;
    Vec2 target = perception_.toWorld(s + moving_ahead, t_ref);
    Vec2 route_target=target; // 中线规划和用来增强拉回的控制点分开显示
    if (position_active_ && gate_locked_) {
        const double front=locked_gate_.gate_s-locked_gate_.gate_depth*.5;
        const double back=locked_gate_.gate_s+locked_gate_.gate_depth*.5;
        const double center=.5*(locked_gate_.gap_left+locked_gate_.gap_right);
        const double before=front-radius-cfg_.continuous_center_reserve;
        const double staging=std::max(s,before);
        const Vec2 staging_point=perception_.toWorld(staging,center);
        const bool in_alignment_zone=s>=front-cfg_.approach_distance-.05;
        // 门一经确认且整条斜线可容纳机身，就直接朝门前中心推进。
        // 不能等到固定距离才横移，也不能在斜向接近后反复切回通道中线。
        if(!position_gate_approach_ &&
            CorridorPerception::pathClear(points_,position,staging_point,radius) &&
            CorridorPerception::pathClear(points_,position_reference_,staging_point,radius))
            position_gate_approach_=true;
        if (position_gate_approach_ || in_alignment_zone) {
            position_gate_approach_=true;
            const double tolerance=std::min(cfg_.center_tolerance,
                std::max(.01,(locked_gate_.gap_width-cfg_.perception.robot_width)*.5-.02));
            const Vec2 axis=this->axis();
            const double cross_speed=-axis.y*filtered_velocity_.x+axis.x*filtered_velocity_.y;
            const double t=perception_.lateral(position);
            const double risk=std::max(std::abs(t-center),
                std::abs(t+cfg_.center_prediction_time*cross_speed-center));
            // 对中持续有效才可穿门。进入/退出采用不同阈值，避免几毫米抖动反复切换。
            if(risk>tolerance) position_gate_aligned_=false;
            if (in_alignment_zone && risk<=tolerance*.65)
                position_gate_aligned_=true;
            // 位置接口先对中再贯穿，参考点不能在尚未对中时自行跑到门后。
            // 偏离后重新对中时，暂驻点不能在实机后方，也不能跟随已经越过门的旧参考向前跑。
            // 旧切向仍平滑减速；若已侵入障碍包络，则进入下面的单调脱离恢复分支。
            // 横向目标实时越过中线适当延伸；机身接近中线/已有回正速度时自动收回。
            // 不能把“加大纠偏”变成把位置目标放进另一侧门框。
            const double door_room=std::max(0.0,(locked_gate_.gap_width-cfg_.perception.robot_width)*.5-.02);
            // 门前仍在宽通道中时，制动点可以留在来向一侧，不能提前被窄门
            // 的几厘米余量夹住。进入机身穿门区以后才按真实门宽收紧。
            const double room=s<before ? std::max(door_room,cfg_.position_lateral_max) : door_room;
            const double extra=std::clamp(cfg_.position_lateral_gain*(t-center)+
                (cfg_.position_lateral_brake+cfg_.position_lateral_gain*cfg_.position_brake_time)*cross_speed,
                -std::min(cfg_.position_lateral_max,room),std::min(cfg_.position_lateral_max,room));
            // 放行标志保留诊断用途；前进目标按当前/预测误差连续收放，避免
            // 几毫米跨阈值就在门前点、门后点之间往返切换。
            const double admission=std::clamp((tolerance-risk)/std::max(.005,tolerance*.65),0.0,1.0);
            const double through=std::max(back+cfg_.exit_distance,s+moving_ahead);
            target=perception_.toWorld(staging+admission*(through-staging),center-extra);
            route_target=perception_.toWorld(perception_.longitudinal(target),center);
            status=position_gate_aligned_ ? "Position: crossing door" :
                (in_alignment_zone ? "Position: aligning before door" : "Position: diagonally approaching door");
        }
    }
    if (near_h && !gate_locked_) {
        target = h_;
        route_target=h_;
        status = "Final approach to H";
    } else if(position_active_ && !position_gate_approach_) {
        const double cross_speed=(perception_.lateral({position.x+filtered_velocity_.x,
            position.y+filtered_velocity_.y})-perception_.lateral(position));
        const double extra=std::clamp(cfg_.position_lateral_gain*(perception_.lateral(position)-t_ref)+
            (cfg_.position_lateral_brake+cfg_.position_lateral_gain*cfg_.position_brake_time)*cross_speed,
            -cfg_.position_lateral_max,cfg_.position_lateral_max);
        target=perception_.toWorld(perception_.longitudinal(target),t_ref-extra);
    }
    const double speed=position_active_ && phase==CorridorPhase::Cross ? cfg_.gate_speed : cfg_.cruise_speed;
    planned_path_={position};
    if(CorridorPerception::pathClear(points_,position,route_target,radius)) {
        planned_path_.push_back(route_target);
        if(gate_locked_) {
            const double exit_s=locked_gate_.gate_s+locked_gate_.gate_depth*.5+cfg_.exit_distance;
            const Vec2 exit=perception_.toWorld(exit_s,.5*(locked_gate_.gap_left+locked_gate_.gap_right));
            if(perception_.longitudinal(route_target)<exit_s &&
               CorridorPerception::pathClear(points_,route_target,exit,radius))planned_path_.push_back(exit);
        }
    }
    auto cmd = translate(position, yaw, target, speed, vf, vl, dt, -1.0, cfg_.min_forward_speed);
    if (cmd.status.empty()) cmd.status = status;
    return cmd;
}

void CorridorController::positionFeedback(bool ready, const Vec2& anchor)
{
    if (!position_active_) return;
    if (!ready && finite(anchor)) {
        position_reference_=anchor;position_waiting_=true;position_reference_velocity_={};
    }
    position_ready_=ready;
}

void CorridorController::positionFeedbackTimeout(const Vec2& position)
{
    if(position_active_ && position_ready_ && finite(position)) {
        position_reference_=position;position_waiting_=true;position_reference_velocity_={};
    }
    position_ready_=false;
}

CorridorCommand CorridorController::positionCommand(const Vec2& position,const Vec2& target,double speed,double dt)
{
    const double radius=cfg_.perception.robot_width*.5;
    const bool overlapping=!CorridorPerception::pathClear(points_,position,position,radius);
    Vec2 destination=target;
    const auto reachable=[&](const Vec2& p) {
        return CorridorPerception::pathClear(points_,position,p,radius) ||
            CorridorPerception::pathRecovering(points_,position,p,radius);
    };
    // 已有向安全侧的参考就是纠偏拉力，不能进入恢复时一律撤回到飞机脚下。
    if(overlapping && !points_.empty()) {
        if(!position_recovering_)position_reference_velocity_={};
        position_recovering_=true;
    }
    if(position_recovering_) {
        const bool forward=phase_==CorridorPhase::Search || phase_==CorridorPhase::Approach || phase_==CorridorPhase::Cross;
        if(forward) destination=perception_.toWorld(perception_.longitudinal(position),perception_.lateral(target));
        const double side_error=std::abs(perception_.lateral(position)-perception_.lateral(target));
        if(!overlapping && (!forward || side_error<.02)) position_recovering_=false;
    }
    const bool corridor_forward=phase_==CorridorPhase::Search || phase_==CorridorPhase::Approach || phase_==CorridorPhase::Cross;
    if(!corridor_forward && !position_recovering_) {
        // 入口/最终定点必须按实机动量提前制动，不能只让虚拟参考点减速。
        // 仍是位置接口：提前把目标收向来向，飞控位置PID执行制动。
        destination.x-=cfg_.position_brake_time*filtered_velocity_.x;
        destination.y-=cfg_.position_brake_time*filtered_velocity_.y;
    }
    if(!position_recovering_) {
        // 预测实际机体的前进包络。只对该包络真正会碰到的点施加向外制动，
        // 不把两边门框的正常近距离当成必须停车的理由。
        const double measured_speed=std::hypot(filtered_velocity_.x,filtered_velocity_.y);
        // 参考本身还在运动时，先把它刹住也要时间；这段延迟必须算进实机
        // 包络，不能等实际机体距墙只剩最后几厘米才开始反转虚拟参考。
        const double reference_stop=std::hypot(position_reference_velocity_.x,position_reference_velocity_.y)/cfg_.position_acceleration;
        const double response_time=cfg_.position_brake_time+reference_stop;
        const double horizon=response_time+measured_speed/(2*cfg_.position_acceleration);
        const Vec2 travel{filtered_velocity_.x*horizon,filtered_velocity_.y*horizon};
        const double travel2=travel.x*travel.x+travel.y*travel.y;
        if(travel2>1e-8)for(const auto& p:points_) {
            const double dx=position.x-p.x,dy=position.y-p.y,d=std::hypot(dx,dy);
            if(d<1e-6)continue;
            const double u=std::clamp(-(dx*travel.x+dy*travel.y)/travel2,0.0,1.0);
            if(std::hypot(dx+u*travel.x,dy+u*travel.y)>radius+.02)continue;
            const Vec2 outward{dx/d,dy/d};
            const double closing=-(filtered_velocity_.x*outward.x+filtered_velocity_.y*outward.y);
            if(closing<=0)continue;
            const double stop=closing*response_time+closing*closing/(2*cfg_.position_acceleration);
            const double away=std::clamp(stop-std::max(0.0,d-radius-.02),0.0,cfg_.position_lead*.5);
            const double requested=(destination.x-position.x)*outward.x+(destination.y-position.y)*outward.y;
            if(away>0 && requested<away) {
                position_measured_braking_=true;
                destination.x+=outward.x*(away-requested);
                destination.y+=outward.y*(away-requested);
            }
        }
    }
    // 当前参考已因实测墙面变化而不再安全时，仅把它收至最近的可用位置。
    // 恢复起步若参考在飞机脚下，则直接建立经单调远离检查的安全侧位置差。
    // 二者均保留真正安全的既有参考，不靠反复hold(当前位置)重启斜坡。
    const auto project=[&](const Vec2& requested) {
        const double d=distance(position,requested);
        const double f=d>cfg_.position_lead ? cfg_.position_lead/d : 1.0;
        Vec2 end{position.x+(requested.x-position.x)*f,position.y+(requested.y-position.y)*f};
        if(reachable(end))return end;
        double lo=0,hi=1;
        for(int i=0;i<18;++i) {
            const double mid=(lo+hi)*.5;
            Vec2 q{position.x+(end.x-position.x)*mid,position.y+(end.y-position.y)*mid};
            if(reachable(q))lo=mid;else hi=mid;
        }
        return Vec2{position.x+(end.x-position.x)*lo,position.y+(end.y-position.y)*lo};
    };
    if(!reachable(position_reference_)) {
        const Vec2 repaired=position_recovering_ ? project(destination) : project(position_reference_);
        if(reachable(repaired)) {position_reference_=repaired;position_reference_velocity_={};}
    }
    const Vec2 direction=this->axis(),normal{-direction.y,direction.x};
    const double reference_cross=(destination.x-position_reference_.x)*normal.x+(destination.y-position_reference_.y)*normal.y;
    // 达到领先上限后仍须能换侧纠偏。先把参考的纵向位置轻收进圆内，再沿圆内弦线平滑换侧，
    // 不把“前进余量为零”误当成“所有方向都不能调整”。最终 next/stop 仍经同一严格检查。
    const bool reposition=corridor_forward && !position_recovering_ && std::abs(reference_cross)>.01 &&
        distance(position_reference_,position)>cfg_.position_lead-.08;
    if(reposition) {
        const double side=(destination.x-position.x)*normal.x+(destination.y-position.y)*normal.y;
        const double safe_lead=std::max(.01,cfg_.position_lead-.02);
        const double side_limit=std::clamp(side,-safe_lead*.9,safe_lead*.9);
        const double along=(destination.x-position.x)*direction.x+(destination.y-position.y)*direction.y;
        const double ahead=std::min(along,std::sqrt(std::max(0.0,safe_lead*safe_lead-side_limit*side_limit)));
        destination={position.x+direction.x*ahead+normal.x*side_limit,position.y+direction.y*ahead+normal.y*side_limit};
    }
    // 仅平滑位置参考的切向速度。主控仍只发XYZ/yaw，实际飞机的速度与纠偏由飞控控制。
    // 沿用上一拍切向，在对中/穿门/下一段切换时限制速度向量的变化，不能直接折向新目标。
    const double dx=destination.x-position_reference_.x,dy=destination.y-position_reference_.y;
    const double d=std::hypot(dx,dy),acceleration=cfg_.position_acceleration;
    // 动态纠偏点必须能在配置的响应时间内收回；不能让虚拟点和飞机都经历
    // 一遍同样慢的侧向加减速，形成两层迟滞。这里只提高参考点侧向响应，
    // 真实飞控的速度/加速度接口仍关闭，纵向参考仍用原推进速度。
    const double lateral_reference_speed=std::max(speed,2*cfg_.position_lateral_max/cfg_.position_lateral_response);
    const double lateral_reference_accel=corridor_forward
        ? std::max(acceleration,lateral_reference_speed/cfg_.position_lateral_response) : acceleration;
    // 提前收敛到静止目标，防止带着旧切向冲过入口/对中点再反向拉回。
    const double approach_time=std::max(.5,speed/acceleration);
    const double previous_speed=std::hypot(position_reference_velocity_.x,position_reference_velocity_.y);
    const double remaining_lead=std::max(0.0,cfg_.position_lead-distance(position_reference_,position));
    // 连同本拍位移预留刹停距离，连续收紧推进速度，避免领先上限处反复硬冻结。
    const double lead_speed=std::max(0.0,std::sqrt(std::max(0.0,
        acceleration*acceleration*dt*dt-acceleration*dt*previous_speed+2*acceleration*remaining_lead))
        -acceleration*dt);
    const double scale=d>1e-9 ? std::min({speed/d,lead_speed/d,1.0/approach_time}) : 0.0;
    Vec2 desired{dx*scale,dy*scale};
    if(corridor_forward) {
        const Vec2 axis=this->axis(),normal{-axis.y,axis.x};
        // 侧向拉回不必等待纵向目标的慢收敛，但输出仍是连续XYZ位置点。
        const double along=std::clamp((dx*axis.x+dy*axis.y)/approach_time,-speed,speed);
        const double lateral_limit=reposition ? lateral_reference_speed : std::min(lateral_reference_speed,lead_speed);
        const double cross=std::clamp((dx*normal.x+dy*normal.y)/cfg_.position_lateral_response,-lateral_limit,lateral_limit);
        const double forward=std::clamp(along,-std::min(speed,lead_speed),std::min(speed,lead_speed));
        desired={axis.x*forward+normal.x*cross,axis.y*forward+normal.y*cross};
    }
    const auto slew=[&](const Vec2& requested) {
        const double x=requested.x-position_reference_velocity_.x;
        const double y=requested.y-position_reference_velocity_.y;
        if(corridor_forward) {
            const double along=std::clamp(x*direction.x+y*direction.y,-acceleration*dt,acceleration*dt);
            const double cross=std::clamp(x*normal.x+y*normal.y,-lateral_reference_accel*dt,lateral_reference_accel*dt);
            return Vec2{position_reference_velocity_.x+direction.x*along+normal.x*cross,
                        position_reference_velocity_.y+direction.y*along+normal.y*cross};
        }
        const double change=std::hypot(x,y);
        const double fraction=change>1e-12 ? std::min(1.0,acceleration*dt/change) : 1.0;
        return Vec2{position_reference_velocity_.x+x*fraction,position_reference_velocity_.y+y*fraction};
    };
    const auto integrate=[&](const Vec2& velocity) {
        return Vec2{position_reference_.x+.5*(position_reference_velocity_.x+velocity.x)*dt,
                    position_reference_.y+.5*(position_reference_velocity_.y+velocity.y)*dt};
    };
    const auto clear=[&](const Vec2& from,const Vec2& to) {
        return CorridorPerception::pathClear(points_,from,to,radius) ||
            CorridorPerception::pathRecovering(points_,from,to,radius);
    };
    std::string blocked_reason;
    const auto stopping=[&](const Vec2& point,const Vec2& velocity) {
        if(!corridor_forward) {
            const double time=std::hypot(velocity.x,velocity.y)/(2*acceleration)+.5*dt;
            return Vec2{point.x+velocity.x*time,point.y+velocity.y*time};
        }
        const double along=velocity.x*direction.x+velocity.y*direction.y;
        const double cross=velocity.x*normal.x+velocity.y*normal.y;
        const double a=along*(std::abs(along)/(2*acceleration)+.5*dt);
        const double b=cross*(std::abs(cross)/(2*lateral_reference_accel)+.5*dt);
        return Vec2{point.x+direction.x*a+normal.x*b,point.y+direction.y*a+normal.y*b};
    };
    const auto safe=[&](const Vec2& next,const Vec2& velocity,bool braking=false) {
        const Vec2 stop=stopping(next,velocity);
        // 检查下一点及沿该切向减速到零所需的整段；不能只看下一小步可走就切入门框。
        if(distance(next,position)>cfg_.position_lead+1e-9 ||
           distance(stop,position)>cfg_.position_lead+1e-9) {
            // 新一帧实机漂移/定位噪声可能使原本安全的领先距离越界。
            // 允许沿旧参考的停止包络收敛，不把目标瞬间拉回实机；不能进一步扩大旧包络。
            const Vec2 old_stop=stopping(position_reference_,position_reference_velocity_);
            const double old_limit=std::max(distance(position_reference_,position),distance(old_stop,position));
            if(!braking || distance(next,position)>old_limit+1e-9 || distance(stop,position)>old_limit+1e-9) {
                blocked_reason="reference lead limit";return false;
            }
        }
        if(!clear(position,next) || !clear(position_reference_,next) || !clear(next,stop)) {
            blocked_reason="measured body clearance";return false;
        }
        if(corridor_forward) {
            // 两个轴用不同参考加速度，刹停轨迹会弯曲；终点弦线清空并不够。
            // 逐段检查完整刹停曲线，按加速度上界补偿弦线和曲线之间的偏差。
            const double along=velocity.x*direction.x+velocity.y*direction.y;
            const double cross=velocity.x*normal.x+velocity.y*normal.y;
            const double duration=std::max(std::abs(along)/acceleration,std::abs(cross)/lateral_reference_accel);
            const int steps=std::max(1,static_cast<int>(std::ceil(duration/.05)));
            const double h=duration/steps;
            const double envelope=.125*h*h*std::hypot(acceleration,lateral_reference_accel);
            Vec2 prior{next.x+.5*dt*velocity.x,next.y+.5*dt*velocity.y};
            if(!clear(next,prior)){blocked_reason="reference braking curve";return false;}
            for(int i=1;i<=steps;++i) {
                const double t=i*h;
                const auto displacement=[&](double v,double a) {
                    const double u=std::min(t,std::abs(v)/a);
                    return v*u-std::copysign(.5*a*u*u,v);
                };
                const double a=displacement(along,acceleration),b=displacement(cross,lateral_reference_accel);
                const Vec2 p{next.x+.5*dt*velocity.x+direction.x*a+normal.x*b,
                             next.y+.5*dt*velocity.y+direction.y*a+normal.y*b};
                if(!CorridorPerception::pathClear(points_,prior,p,radius+envelope) &&
                   !CorridorPerception::pathRecovering(points_,prior,p,radius+envelope)) {
                    blocked_reason="reference braking curve";return false;
                }
                const Vec2 old_stop=stopping(position_reference_,position_reference_velocity_);
                const double limit=braking ? std::max({cfg_.position_lead,distance(position_reference_,position),
                                                       distance(old_stop,position)}) : cfg_.position_lead;
                if(distance(p,position)>limit+1e-9){blocked_reason="reference lead limit";return false;}
                prior=p;
            }
        }
        return true;
    };
    Vec2 velocity=slew(desired),next=integrate(velocity);
    bool braking=false;
    if(!safe(next,velocity)) {
        // 先沿当前切向平滑刹住；跟不上或靠近墙时，目标不能每拍走一下又突然停一下。
        velocity=slew({});next=integrate(velocity);braking=true;
        if(!safe(next,velocity,true)) {
            // 新曲线/其停止段受阻不等于现有目标失效。保留可达参考，再逐拍
            // 朝可达的目标修正；仅在参考自身失效时投影，避免几十厘米回跳。
            const Vec2 retained=project(position_reference_);
            if(reachable(retained)) {
                position_reference_=retained;position_reference_velocity_={};position_waiting_=false;
                CorridorCommand cmd;cmd.position_control=true;cmd.target=retained;cmd.path={position,retained};
                cmd.status="Position: retaining safe reference during clearance braking";
                return cmd;
            }
            const Vec2 escape=project(destination);
            if(reachable(escape)) {
                position_reference_=escape;position_reference_velocity_={};position_waiting_=false;
                CorridorCommand cmd;cmd.position_control=true;cmd.target=escape;cmd.path={position,escape};
                cmd.status="Position: recovering along verified escape";return cmd;
            }
            auto cmd=hold(position,"Position: no measured safe escape");
            position_recovering_=overlapping;return cmd;
        }
    }
    position_reference_=next;position_reference_velocity_=velocity;position_waiting_=false;
    CorridorCommand cmd;cmd.position_control=true;cmd.target=next;cmd.path={position,next};
    if(position_recovering_) cmd.status="Position: recovering toward travel line";
    else if(braking) cmd.status="Position: smoothly slowing reference for clearance / lead";
    else if(position_measured_braking_) cmd.status="Position: braking measured aircraft motion near wall";
    return cmd;
}

double CorridorController::approachForwardLimit(double lateral_error, double lateral_speed,
                                                double remaining_distance) const
{
    if (remaining_distance <= 0.0) return 0.0;
    const double component_speed = cfg_.align_speed / std::sqrt(2.0);
    const double response_gain = cfg_.position_kp / (1.0 + cfg_.velocity_kd);
    const double stop_time = std::fabs(lateral_speed) / cfg_.acceleration;
    const double drift = lateral_speed * lateral_speed / (2.0 * cfg_.acceleration);
    const double remaining_error = std::fabs(lateral_error) + drift;
    const double threshold = cfg_.center_tolerance * 0.75;
    const double saturation_error = component_speed / response_gain;
    // Include both the speed-limited transfer and the slow PD tail. Treat
    // existing lateral momentum as extra work even when it currently points inward.
    double alignment_time = stop_time + cfg_.continuous_time_margin;
    if (remaining_error > saturation_error)
        alignment_time += (remaining_error - saturation_error) / component_speed;
    const double settling_error = std::min(remaining_error, saturation_error);
    if (settling_error > threshold)
        alignment_time += std::log(settling_error / threshold) / response_gain;
    const double time_limit = remaining_distance / std::max(0.02, alignment_time);
    const double braking_limit = std::sqrt(2.0 * cfg_.acceleration * remaining_distance);
    return std::min({component_speed, time_limit, braking_limit});
}

CorridorCommand CorridorController::update(const Vec2& position, double yaw, double vf,
                                           double vl, double now, bool pose_fresh)
{
    planned_path_.clear();
    const double dt = last_update_ < 0.0 ? 0.02 : std::clamp(now - last_update_, 0.001, 0.10);
    const double prior_yaw_command = previous_yaw_rate_;
    auto cmd = updateMotion(position, yaw, vf, vl, now, pose_fresh);
    const double evidence_timeout = cfg_.cloud_window > 0.0
        ? std::min(cfg_.cloud_window, cfg_.cloud_timeout) : cfg_.cloud_timeout;
    // With fresh measurements, waiting for wall/door evidence still holds the
    // first waiting position. It must not silently hand XY to the flight controller.
    // Missing pose/cloud data and actual occupied corrections remain stop cases.
    const bool can_hold = !position_active_ && pose_fresh && finite(position) && std::isfinite(yaw) &&
        std::isfinite(vf) && std::isfinite(vl) && std::isfinite(now) &&
        now >= evidence_time_ && now - evidence_time_ <= evidence_timeout && !points_.empty() &&
        phase_ != CorridorPhase::Idle && phase_ != CorridorPhase::WaitRoute &&
        !cmd.finished && cmd.path.empty();
    if (can_hold) {
        if (!waiting_for_evidence_) waiting_hold_.begin(position);
        waiting_for_evidence_ = true;
        previous_yaw_rate_ = prior_yaw_command;
        auto holding = rotateAt(waiting_hold_, position, yaw, vf, vl, dt);
        holding.status = cmd.status;
        return holding;
    }
    waiting_for_evidence_ = false;
    return cmd;
}

CorridorCommand CorridorController::updateMotion(const Vec2& position, double yaw, double vf,
                                                 double vl, double now, bool pose_fresh)
{
    position_measured_braking_=false;
    const double dt = last_update_ < 0.0 ? 0.02 : std::clamp(now - last_update_, 0.001, 0.10);
    if (now < last_update_) { cloud_voxels_.clear(); points_.clear(); cloud_time_ = -1e9; }
    last_update_ = now;
    if (!pose_fresh || !finite(position) || !std::isfinite(yaw) || !std::isfinite(vf) || !std::isfinite(vl) || !std::isfinite(now))
        return hold(position, "Waiting for fresh odometry");
    const double c = std::cos(yaw), sn = std::sin(yaw);
    const Vec2 measured{c * vf - sn * vl, sn * vf + c * vl};
    if (!velocity_valid_) { filtered_velocity_ = measured; velocity_valid_ = true; }
    const double alpha = dt / (cfg_.velocity_filter_tau + dt);
    filtered_velocity_.x += alpha * (measured.x - filtered_velocity_.x);
    filtered_velocity_.y += alpha * (measured.y - filtered_velocity_.y);
    vf = c * filtered_velocity_.x + sn * filtered_velocity_.y;
    vl = -sn * filtered_velocity_.x + c * filtered_velocity_.y;
    if (phase_ == CorridorPhase::Idle) return hold(position, "Exploration");
    if (phase_ == CorridorPhase::WaitRoute) return hold(position, "Waiting for entry / H coordinates");
    if (phase_ == CorridorPhase::Done) {
        auto cmd = position_active_ ? positionCommand(position,h_,cfg_.align_speed,dt) : hold(h_, "H reached");
        cmd.status="H reached";
        cmd.finished = true;
        return cmd;
    }
    // 转头、降高等待及位置交接时也提前跟踪门，不等飞到入口后重新累计确认。
    trackEntrance(position,now);
    trackGate(position,now);
    if(position_active_) {
        if(!position_ready_)return hold(position,"Position: waiting for main handover");
        if(!position_height_admitted_) {
            if(cfg_.descend_before_entry && !altitude_ready_)
                return hold(position,"Position: restoring initial corridor height");
            position_height_admitted_=true;
        }
        // 已进入通道后始终保持XY任务。主控每帧仍发目标Z，飞控同时纠高，
        // 高度跨越首次放行阈值不能清空正在执行的前进/横向恢复参考。
    }
    if (!heading_selected_) {
        // 仅在本次通道开始后的首个有效位姿上选一次；转向/降高/横移中的漂移不能重选。
        // 用世界系正方向，与通道前进轴分开。相同转角(±45/±135度)按远离0度取整确定方向。
        const double quarter_turn = std::acos(-1.0) * 0.5;
        target_yaw_ = cfg_.nearest_cardinal_yaw
            ? angle(std::round(angle(yaw) / quarter_turn) * quarter_turn)
            : angle(cfg_.initial_yaw);
        heading_selected_ = true;
    }
    if (phase_ == CorridorPhase::Rotate) {
        auto cmd = rotateAt(rotation_hold_, position, yaw, vf, vl, dt);
        cmd.status = "Rotate to entry heading with XY hold";
        const double error = angle(target_yaw_ - yaw);
        const bool heading_ready = std::fabs(error) <= cfg_.heading_tolerance;
        if (!rotation_done_ && stable(heading_ready, now)) rotation_done_ = true;
        // A heading reached earlier can drift while Z is still descending.
        // Entry needs the current heading as well as the current altitude.
        if (rotation_done_ && heading_ready) {
            // XY到点后转向与外部降高并行；保持红点XY，航向和高度都到位才横移进入口。
            if (!cfg_.descend_before_entry || altitude_ready_) {
                transition(CorridorPhase::Entry);
                cmd.status = "Heading aligned: entering corridor";
                if(cfg_.position_control) {
                    position_active_=true;position_waiting_=true;position_ready_=false;position_reference_=position;
                    position_reference_velocity_={};
                    cmd={};cmd.position_control=true;cmd.target=position;cmd.path={position,position};
                    cmd.status="Position: requesting handover after height and heading alignment";
                }
            } else {
                cmd.status = "Heading aligned: descending to corridor height with XY hold";
            }
        }
        return cmd;
    }
    const double evidence_timeout = cfg_.cloud_window > 0.0
        ? std::min(cfg_.cloud_window, cfg_.cloud_timeout) : cfg_.cloud_timeout;
    const bool forward_phase = phase_ == CorridorPhase::Search ||
        phase_ == CorridorPhase::Approach || phase_ == CorridorPhase::Cross;
    if ((!forward_phase || position_active_) && (now - evidence_time_ > evidence_timeout || now < evidence_time_ || points_.empty())) {
        confirmations_ = 0;
        settled_since_ = -1.0;
        return hold(position, "Waiting for fresh corridor cloud");
    }
    const double s = perception_.longitudinal(position);
    const double t = perception_.lateral(position);
    const double radius = cfg_.perception.robot_width * 0.5;
    const double resolution = cfg_.perception.cell_size;
    const Vec2 unit = perception_.toWorld(1.0, 0.0);
    const Vec2 axis{unit.x - entry_.x, unit.y - entry_.y};
    const double lateral_speed = -axis.y * filtered_velocity_.x + axis.x * filtered_velocity_.y;
    auto analyze = [&](double minimum) {
        return perception_.analyze(points_, position, minimum, &visibility_points_,
                                    has_sensor_origin_ ? &sensor_origin_ : nullptr);
    };
    auto confirm = [&](const CorridorObservation& observed) {
        confirmGate(observed,now);
    };

    // Intermediate waypoints are passed within this tick; only H has a dwell.
    for (int handoff = 0; handoff < 5; ++handoff) {
        if (phase_ == CorridorPhase::Entry) {
            if(cfg_.wall_entry && (!entrance_locked_ || !entrance_staged_ ||
                distance(position,entry_)>std::min(.08,cfg_.point_tolerance)))return enter(position,yaw,vf,vl,dt);
            if (!cfg_.wall_entry && distance(position, entry_) > cfg_.point_tolerance) return enter(position, yaw, vf, vl, dt);
            transition(CorridorPhase::Search);
        }
        if (!gate_locked_ && distance(position, h_) <= cfg_.point_tolerance)
            transition(CorridorPhase::Finish);
        if (phase_ == CorridorPhase::Finish) {
            planned_path_={position,h_};
            if (distance(position, h_) > cfg_.point_tolerance * cfg_.arrival_hysteresis)
                transition(CorridorPhase::Search);
            else {
                auto cmd = translate(position, yaw, h_, cfg_.align_speed, vf, vl, dt);
                if (stable(!cmd.path.empty() && distance(position, h_) <= cfg_.point_tolerance &&
                    std::fabs(angle(target_yaw_ - yaw)) <= cfg_.heading_tolerance, now))
                    transition(CorridorPhase::Done);
                if (cmd.status.empty()) cmd.status = "Holding position at H";
                return cmd;
            }
        }
        // ★隧道里往前走统一由 forwardCorridor 处理(2026-09-26 用户：前进必须一直 0.3，横向纠偏另算)★
        //   下面原来的 Approach/Cross/Search 分支(看不清就停下等)不再走到
        if (phase_ == CorridorPhase::Search || phase_ == CorridorPhase::Approach || phase_ == CorridorPhase::Cross)
            return forwardCorridor(position, yaw, vf, vl, dt);
        if (!gate_locked_ && confirmations_ >= cfg_.confirm_frames &&
            candidate_.gate_passable && candidate_.gate_s > passed_s_) {
            locked_gate_ = candidate_;
            gate_locked_ = true;
            transition(CorridorPhase::Approach);
        }

        if (gate_locked_) {
            const double center_t = 0.5 * (locked_gate_.gap_left + locked_gate_.gap_right);
            const double front = locked_gate_.gate_s - locked_gate_.gate_depth * 0.5;
            const double back = locked_gate_.gate_s + locked_gate_.gate_depth * 0.5;
            const double first_clear_s = last_gate_back_ + radius + resolution;
            const double last_clear_s = front - radius - resolution;
            if (first_clear_s > last_clear_s)
                return hold(position, "Insufficient measured space between doors for the aircraft body");
            const double approach = std::clamp(locked_gate_.gate_s - cfg_.approach_distance,
                                               first_clear_s, last_clear_s);
            const double body_clear_s = back + radius + resolution;
            const auto next = analyze(back + std::max(resolution, cfg_.perception.gate_cluster_depth));
            if (phase_ == CorridorPhase::Approach) {
                const bool centered = std::fabs(t - center_t) <= cfg_.center_tolerance &&
                    std::fabs(t + cfg_.center_prediction_time * lateral_speed - center_t) <= cfg_.center_tolerance;
                // 对准到容差，或已经到门框前(边走边横移，不停下)，都进入穿门
                if ((centered && std::fabs(angle(target_yaw_ - yaw)) <= cfg_.heading_tolerance) || s >= last_clear_s) {
                    transition(CorridorPhase::Cross);
                    continue;
                }
                if (cfg_.continuous_approach) {
                    // 先往前走：离门 approach_distance 以外保持原行进线(t=0)，进到这个距离内才往门中心对。
                    //   纵向全程 ≥ min_forward，不按"对准要多久"压速度(2026-09-26 实机门前被压到 0.05 m/s 推不动卡 64 s)
                    const bool align_zone = s >= locked_gate_.gate_s - cfg_.approach_distance - 0.05;
                    const double target_s = std::min(perception_.length(),
                        std::max(back + cfg_.exit_distance, s + cfg_.moving_lookahead));
                    auto cmd = translate(position, yaw, perception_.toWorld(target_s, align_zone ? center_t : 0.0),
                                         cfg_.cruise_speed, vf, vl, dt, -1.0, cfg_.min_forward_speed);   // 对齐区内外同一套前进速度
                    if (cmd.status.empty())
                        cmd.status = align_zone ? "Approaching door center continuously" : "Moving forward before door alignment zone";
                    return cmd;
                }
                // Do not reverse toward a staging point already behind the vehicle.
                const double target_s = std::min(last_clear_s, std::max(s, approach));
                auto cmd = translate(position, yaw, perception_.toWorld(target_s, center_t), cfg_.align_speed, vf, vl, dt);
                if (cmd.status.empty()) cmd.status = "Approaching door center";
                return cmd;
            }
            if (phase_ == CorridorPhase::Cross) {
                double exit_limit = perception_.length();
                if (next.gate_observed) {
                    exit_limit = std::min(exit_limit, next.gate_s - next.gate_depth * 0.5 - radius - resolution);
                    if (exit_limit < body_clear_s)
                        return hold(position, "Insufficient measured space to clear this door before the next");
                } else {
                    exit_limit = std::min(exit_limit, std::max(back + cfg_.exit_distance,
                                                              observation_.observed_until - radius));
                }
                const double requested_exit = back + cfg_.exit_distance;
                // Keep the complete straight exit when space allows. Only a
                // constrained exit needs an early handoff before its stop limit.
                const double exit_handoff = requested_exit < exit_limit - 1e-6
                    ? requested_exit : exit_limit - cfg_.point_tolerance;
                const double pass_s = std::max(body_clear_s, exit_handoff);
                if (s >= pass_s) {   // 机身过了门就算通过(擦着门框过也不卡在穿门里)
                    last_gate_back_ = back;
                    passed_s_ = back + resolution;
                    gate_locked_ = false;
                    ++gates_passed_;
                    if (candidate_.gate_s <= passed_s_) confirmations_ = 0;
                    observation_ = analyze(passed_s_);
                    confirm(observation_);
                    transition(CorridorPhase::Search);
                    continue;
                }
                const double target_s = std::min(exit_limit, std::max(back + cfg_.exit_distance, s + cfg_.moving_lookahead));
                auto cmd = translate(position, yaw, perception_.toWorld(target_s, center_t), cfg_.gate_speed, vf, vl, dt,
                                     -1.0, cfg_.min_forward_speed);
                if (cmd.status.empty()) cmd.status = "Crossing door";
                return cmd;
            }
        }

        // 离 H 只剩 h_approach_distance：直接朝 H 飞，不再要求墙延伸过 H、不因看不清前方/认不出的点停下
        //   (2026-09-26 实机：墙在 H 前 0.17 m 就结束，旧逻辑停在 H 前 0.4 m 走走停停 26 s)。前面还有可通过的门时仍按门走
        if (perception_.length() - s <= cfg_.h_approach_distance &&
            !(observation_.gate_observed && observation_.gate_passable)) {
            auto cmd = translate(position, yaw, h_, cfg_.cruise_speed, vf, vl, dt, -1.0, cfg_.min_forward_speed);
            if (cmd.status.empty()) cmd.status = "Final approach to H";
            return cmd;
        }
        if (!observation_.walls_observed) return hold(position, "Waiting for both corridor walls");
        if (observation_.gate_observed) {
            if (observation_.gate_passable) {
                // Confirmation can run while moving safely toward the pre-door point.
                const double safe_s = observation_.gate_s - observation_.gate_depth * 0.5 - radius - resolution;
                const double zone_s = observation_.gate_s - cfg_.approach_distance;
                const double target_s = std::max(s, safe_s);   // 没确认前最远到门框前，一路 ≥ min_forward
                // 先往前走：离门 approach_distance 以外保持原行进线(t=0)，进到这个距离内才往门中心对
                const bool align_zone = s >= std::min(zone_s, safe_s) - 0.05;   // 5 cm：P 控制逼近目标点会差一点点
                const double target_t = align_zone ? 0.5 * (observation_.gap_left + observation_.gap_right) : 0.0;
                auto cmd = translate(position, yaw, perception_.toWorld(target_s, target_t),
                                     cfg_.cruise_speed, vf, vl, dt, -1.0, cfg_.min_forward_speed);   // 对齐区内外同一套前进速度
                if (cmd.status.empty()) cmd.status = "Confirming door while approaching";
                return cmd;
            }
            return hold(position, observation_.reason == "gate_closed_or_too_narrow"
                ? "Door closed or measured gap narrower than aircraft"
                : "Waiting for door boundary evidence: " + observation_.reason);
        }
        if (!observation_.corridor_clear_observed)
            return hold(position, "Waiting for observed corridor ahead: " + observation_.reason);
        const double supported_end = observation_.observed_until - radius;
        const double safe_end = supported_end + resolution >= perception_.length() ? perception_.length() : supported_end;
        if (safe_end < perception_.length() && safe_end <= s + cfg_.point_tolerance)
            return hold(position, "Waiting for more corridor visibility");
        const Vec2 target = safe_end >= perception_.length() ? h_ : perception_.toWorld(safe_end, 0.0);
        auto cmd = translate(position, yaw, target, cfg_.cruise_speed, vf, vl, dt, -1.0, cfg_.min_forward_speed);
        if (cmd.status.empty()) cmd.status = "Following observed corridor to H";
        return cmd;
    }
    return hold(position, "Waiting for next route segment");
}

}  // namespace exploration
