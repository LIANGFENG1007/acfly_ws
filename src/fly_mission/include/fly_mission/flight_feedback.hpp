#pragma once

#include "fly_mission/config.hpp"
#include <cstdint>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>

namespace fly_mission {
struct Pose {
  double x{}, y{}, z{}, yaw{};
};
struct Velocity {
  double x{}, y{}, z{}, yaw_rate{};
};

class FlightFeedback {
public:
  explicit FlightFeedback(const Config &config) : config_(config) {}
  void update(const nav_msgs::msg::Odometry &message, double receipt_s);
  bool fresh(double now_s) const;
  const Pose &pose() const {
    return pose_;
  }
  const Velocity &velocity() const {
    return velocity_;
  }

private:
  const Config &config_;
  std::optional<std::int64_t> stamp_;
  std::optional<double> receipt_;
  Pose pose_;
  Velocity velocity_;
};
} // namespace fly_mission
