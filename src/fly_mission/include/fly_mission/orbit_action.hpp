#pragma once

#include "fly_mission/mavros_link.hpp"

#include <optional>

namespace fly_mission {

struct OrbitStep {
  Velocity velocity;
  bool complete = false;
};

class OrbitAction {
public:
  explicit OrbitAction(const Config &config) : config_(config) {}
  void start(const Pose &cruise, double time);
  void interrupt(const char *reason);
  OrbitStep step(const MavrosLink &vehicle, double time);
  const char *hold_reason() const { return hold_reason_; }
  double swept_angle_deg() const;

private:
  bool settled(bool in_tolerance, double time);
  OrbitStep hold(const MavrosLink &vehicle);
  Velocity slew_horizontal(Velocity command, double time);

  const Config &config_;
  Pose cruise_, entry_;
  Velocity previous_command_;
  std::optional<Pose> hold_point_;
  std::optional<double> settled_since_;
  const char *hold_reason_ = nullptr;
  double started_at_ = 0, previous_command_time_ = 0, previous_angle_ = 0, swept_rad_ = 0;
  bool tracking_ = false;
};

} // namespace fly_mission
