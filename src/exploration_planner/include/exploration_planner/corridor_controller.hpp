#pragma once

#include <deque>
#include <string>

#include "exploration_planner/corridor_perception.hpp"
#include "exploration_planner/position_hold.hpp"

namespace exploration {

struct CorridorConfig {
    CorridorPerceptionConfig perception;
    double initial_yaw = -1.5707963267948966;
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

private:
    CorridorConfig cfg_;
    CorridorPerception perception_;
    CorridorPhase phase_ = CorridorPhase::Idle;
    bool configured_ = false;
    Vec2 entry_, h_, red_;
    PositionHold rotation_hold_, heading_hold_, waiting_hold_;
    bool waiting_for_evidence_ = false;
    bool heading_recovery_ = false;
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
