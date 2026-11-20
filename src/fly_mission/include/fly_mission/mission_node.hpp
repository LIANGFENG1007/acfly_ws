#pragma once

#include "fly_mission/exploration_link.hpp"
#include "fly_mission/orbit_action.hpp"
#include <optional>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace fly_mission {
enum class Stage { Preflight, Takeoff, Orbit, OrbitHold, Survey, Landing, Finished, Aborted };
const char *stage_name(Stage stage);

class MissionNode : public rclcpp::Node {
public:
  explicit MissionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  Stage stage() const {
    return stage_;
  }
  bool ended() const {
    return stage_ == Stage::Finished || stage_ == Stage::Aborted;
  }

private:
  void tick();
  void preflight(double time);
  void report_preflight_wait(double time, const std::string &reason);
  void takeoff(double time);
  void orbit(double time);
  void survey(double time);
  void land(double time);
  void transition(Stage next);
  bool pilot_authorizes_flight(double time) const;

  const Config config_;
  MavrosLink vehicle_;
  std::optional<ExplorationLink> explorer_;
  OrbitAction orbit_;
  Stage stage_ = Stage::Preflight;
  Pose home_, takeoff_point_, cruise_, hold_, landing_point_;
  std::optional<double> in_tolerance_, lost_authority_;
  std::string last_preflight_reason_;
  double last_preflight_log_s_ = 0.0;
  bool saw_disarmed_on_ground_ = false;
  bool start_signal_ = false, holding_ = true, landing_confirmed_ = false;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace fly_mission
