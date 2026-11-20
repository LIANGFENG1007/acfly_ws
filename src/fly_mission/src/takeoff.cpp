#include "fly_mission/mission_node.hpp"

#include <cmath>

namespace fly_mission {
void MissionNode::takeoff(double time) {
  const auto &feedback = vehicle_.feedback();
  if (!feedback.fresh(time)) {
    vehicle_.publish_velocity({});
    in_tolerance_.reset();
    return;
  }
  vehicle_.publish_velocity(
      vehicle_.hold(takeoff_point_, config_.max_speed_xy_mps, config_.max_climb_mps));
  const auto &p = feedback.pose();
  if (std::hypot(takeoff_point_.x - p.x, takeoff_point_.y - p.y) > config_.tolerance_xy_m ||
      std::abs(takeoff_point_.z - p.z) > config_.tolerance_z_m) {
    in_tolerance_.reset();
    return;
  }
  if (!in_tolerance_)
    in_tolerance_ = time;
  if (time - *in_tolerance_ < config_.takeoff_settle_s + config_.takeoff_hover_s)
    return;
  cruise_ = hold_ = p;
  if (explorer_) {
    explorer_->start(home_, cruise_);
    transition(Stage::Survey);
  } else {
    orbit_.start(cruise_, time);
    transition(Stage::Orbit);
  }
}
} // namespace fly_mission
