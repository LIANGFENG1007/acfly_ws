// In-process ROS peers; every command, state, goal and FCU endpoint is remapped
// under a private namespace. No flight controller, simulator or hardware involved.
#include "fly_mission/mission_node.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
namespace {
using fly_mission::MissionNode;
using fly_mission::Stage;
using Command = acfly_interfaces::msg::ExplorationCommand;
using Target = mavros_msgs::msg::PositionTarget;
using SetMode = mavros_msgs::srv::SetMode;

void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}

class Fixture {
public:
  Fixture(std::string scope, bool require_start)
      : prefix_("/acfly_test/" + scope), peer_(std::make_shared<rclcpp::Node>("peer_" + scope)) {
    std::vector<std::string> args{"--ros-args"};
    const std::vector<std::string> endpoints{"/mavros/extended_state",
                                             "/mavros/state",
                                             "/aft_mapped_to_init",
                                             "/mavros/setpoint_raw/local",
                                             "/mavros/set_mode",
                                             "/mission/state",
                                             "/mission/start",
                                             "/exploration/goal",
                                             "/exploration/context",
                                             "/exploration/control",
                                             "/exploration/finished_goal"};
    for (const auto &e : endpoints)
      args.insert(args.end(), {"-r", e + ":=" + prefix_ + e});
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).arguments(args);
    // Speed up elapsed-time gates in this fixture; leave the MAVROS output controls at defaults.
    options.parameter_overrides(
        {rclcpp::Parameter("require_start_signal", require_start),
         rclcpp::Parameter("takeoff_settle_s", 0.06), rclcpp::Parameter("takeoff_hover_s", 0.06),
         rclcpp::Parameter("takeover_debounce_s", 0.12),
         rclcpp::Parameter("control_timeout_s", 0.15), rclcpp::Parameter("landing_retry_s", 0.15)});
    node_ = std::make_shared<MissionNode>(options);
    auto latched = rclcpp::QoS(1).transient_local();
    state_pub_ = peer_->create_publisher<mavros_msgs::msg::State>(prefix_ + "/mavros/state", 10);
    ground_pub_ = peer_->create_publisher<mavros_msgs::msg::ExtendedState>(
        prefix_ + "/mavros/extended_state", 10);
    pose_pub_ = peer_->create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/aft_mapped_to_init",
                                                                 rclcpp::SensorDataQoS());
    start_pub_ = peer_->create_publisher<std_msgs::msg::Int32>(prefix_ + "/mission/start", 1);
    control_pub_ = peer_->create_publisher<Command>(prefix_ + "/exploration/control", 1);
    done_pub_ = peer_->create_publisher<std_msgs::msg::Header>(
        prefix_ + "/exploration/finished_goal", latched);
    goal_sub_ = peer_->create_subscription<geometry_msgs::msg::PointStamped>(
        prefix_ + "/exploration/goal", latched,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr goal) { goal_ = *goal; });
    context_sub_ = peer_->create_subscription<acfly_interfaces::msg::ExplorationContext>(
        prefix_ + "/exploration/context", latched,
        [this](acfly_interfaces::msg::ExplorationContext::ConstSharedPtr msg) { context_ = *msg; });
    output_sub_ = peer_->create_subscription<Target>(prefix_ + "/mavros/setpoint_raw/local", 10,
                                                     [this](Target::ConstSharedPtr output) {
                                                       last_ = *output;
                                                       ++outputs_;
                                                     });
    mode_ = peer_->create_service<SetMode>(
        prefix_ + "/mavros/set_mode",
        [this](SetMode::Request::SharedPtr req, SetMode::Response::SharedPtr response) {
          check(req->custom_mode == "AUTO.LAND", "Unexpected FCU mode change request");
          ++land_requests_;
          response->mode_sent = true;
        });
    executor_.add_node(peer_);
    executor_.add_node(node_);
    pose_.pose.pose.position.x = 1.2;
    pose_.pose.pose.position.y = -0.7;
    pose_.pose.pose.position.z = 0.3;
    pose_.pose.pose.orientation.z = std::sin(std::acos(-1.0) / 4);
    pose_.pose.pose.orientation.w = std::cos(std::acos(-1.0) / 4);
    pump(220ms);
  }

  void pump(std::chrono::milliseconds duration, bool send_pose = true) {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      if (send_pose) {
        pose_.header.stamp = peer_->now();
        pose_pub_->publish(pose_);
      }
      ground_pub_->publish(ground_);
      state_pub_->publish(state_);
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
  }

  void airborne_restart() {
    using Ext = mavros_msgs::msg::ExtendedState;
    state_.connected = state_.armed = true;
    state_.mode = "OFFBOARD";
    ground_.landed_state = Ext::LANDED_STATE_IN_AIR;
    pump(130ms);
    check(node_->stage() == Stage::Preflight && outputs_ == 0, "Airborne restart sent a setpoint");
    ground_.landed_state = Ext::LANDED_STATE_ON_GROUND;
    pump(100ms);
    check(node_->stage() == Stage::Preflight && outputs_ == 0,
          "Armed first sample initialized ground reference");
  }

  void missing_planner() {
    done_pub_.reset(); // Missing only the completion endpoint must already block takeoff.
    pump(160ms);
    authorize();
    pump(100ms);
    check(node_->stage() == Stage::Preflight && outputs_ == 0,
          "Incomplete planner graph allowed takeoff");
    done_pub_ = peer_->create_publisher<std_msgs::msg::Header>(
        prefix_ + "/exploration/finished_goal", rclcpp::QoS(1).transient_local());
    pump(120ms);
    check(node_->stage() == Stage::Preflight && outputs_ == 0,
          "Late planner armed an already-waiting flight");
    state_.armed = false;
    pump(100ms);
    state_.armed = true;
    pump(100ms);
    check(node_->stage() == Stage::Takeoff,
          "Healthy planner with new ground authorization failed to start");
  }

  void authorize() {
    ground_.landed_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
    state_.connected = true;
    state_.mode = "OFFBOARD";
    pump(50ms);
    check(node_->stage() == Stage::Preflight, "Unarmed vehicle left preflight");
    state_.armed = true;
    state_.mode = "POSCTL";
    pump(50ms);
    check(node_->stage() == Stage::Preflight, "Manual mode left preflight");
    state_.mode = "OFFBOARD";
  }

  void full_flight() {
    authorize();
    pump(60ms);
    check(node_->stage() == Stage::Preflight && last_.velocity.z == 0, "Start signal gate failed");
    std_msgs::msg::Int32 start;
    start.data = 1;
    start_pub_->publish(start);
    pump(100ms);
    check(node_->stage() == Stage::Takeoff && last_.velocity.z > 0,
          "Missing takeoff after manual authorization");
    check(last_.coordinate_frame == Target::FRAME_LOCAL_NED &&
              last_.type_mask ==
                  (Target::IGNORE_PX | Target::IGNORE_PY | Target::IGNORE_PZ | Target::IGNORE_AFX |
                   Target::IGNORE_AFY | Target::IGNORE_AFZ | Target::IGNORE_YAW),
          "Velocity MAVROS frame/mask changed");
    ground_.landed_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
    pose_.pose.pose.position.z += node_->get_parameter("takeoff_height_m").as_double();
    pump(270ms);
    check(node_->stage() == Stage::Survey && goal_.header.frame_id == "camera_init" &&
              context_.mission.stamp == goal_.header.stamp &&
              std::abs(context_.home_z - 0.3) < 1e-6,
          "Failed to hand off matching goal and context");
    auto wrong_goal = goal_.header;
    wrong_goal.stamp.sec -= 1;
    done_pub_->publish(wrong_goal);
    auto cmd = Command{};
    cmd.mission = wrong_goal;
    cmd.header.stamp = peer_->now();
    cmd.header.frame_id = "base_link";
    cmd.velocity.linear.x = 0.25;
    cmd.altitude = context_.cruise_z;
    cmd.vertical_speed_limit = 0.10;
    cmd.control_mode = Command::VELOCITY;
    control_pub_->publish(cmd);
    pump(40ms);
    check(node_->stage() == Stage::Survey && std::abs(last_.velocity.y) < 1e-6 &&
              land_requests_ == 0,
          "Previous mission could move or land");
    cmd.mission = goal_.header;
    cmd.header.stamp = peer_->now();
    cmd.altitude -= 0.5;
    control_pub_->publish(cmd);
    pump(40ms);
    check(std::abs(last_.velocity.y) < 1e-6, "Non-survey altitude accepted");
    cmd.altitude = context_.cruise_z;
    cmd.header.stamp = peer_->now();
    control_pub_->publish(cmd);
    pump(50ms);
    check(std::abs(last_.velocity.x) < 1e-5 && std::abs(last_.velocity.y - 0.25) < 1e-5,
          "Body-forward command did not rotate to local ENU");
    pump(170ms);
    check(std::abs(last_.velocity.y) < 1e-6, "Expired command kept moving");
    control_pub_->publish(cmd);
    pump(40ms);
    check(std::abs(last_.velocity.y) < 1e-6, "Duplicate/stale source stamp renewed command");
    done_pub_->publish(goal_.header);
    pump(330ms);
    check(node_->stage() == Stage::Landing && land_requests_ >= 2,
          "Landing failed to retry pending FCU handoff");
    state_.mode = "AUTO.LAND";
    pump(90ms);
    const auto count = outputs_, requests = land_requests_;
    pump(100ms);
    check(outputs_ == count && land_requests_ == requests, "Continued commands after AUTO.LAND");
    ground_.landed_state = mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND;
    state_.armed = false;
    pose_.pose.pose.position.z = 0.3;
    pump(80ms);
    check(node_->stage() == Stage::Finished, "Landing did not finish after disarm near home");
    state_.armed = true;
    state_.mode = "OFFBOARD";
    pump(80ms);
    check(node_->ended() && outputs_ == count, "Terminal flight restarted");
  }

  void pilot_takeover() {
    authorize();
    pump(90ms);
    check(node_->stage() == Stage::Takeoff, "Did not enter takeoff");
    state_.mode = "POSCTL";
    pump(60ms);
    const auto count = outputs_;
    pump(170ms);
    check(node_->stage() == Stage::Aborted && outputs_ == count && land_requests_ == 0,
          "Pilot takeover not respected");
  }

private:
  std::string prefix_;
  rclcpp::Node::SharedPtr peer_;
  std::shared_ptr<MissionNode> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  mavros_msgs::msg::ExtendedState ground_;
  rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr ground_pub_;
  mavros_msgs::msg::State state_;
  nav_msgs::msg::Odometry pose_;
  Target last_;
  geometry_msgs::msg::PointStamped goal_;
  acfly_interfaces::msg::ExplorationContext context_;
  int outputs_{}, land_requests_{};
  rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr start_pub_;
  rclcpp::Publisher<Command>::SharedPtr control_pub_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr done_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<acfly_interfaces::msg::ExplorationContext>::SharedPtr context_sub_;
  rclcpp::Subscription<Target>::SharedPtr output_sub_;
  rclcpp::Service<SetMode>::SharedPtr mode_;
};

} // namespace

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);
    Fixture("missing_planner", false).missing_planner();
    Fixture("restart", false).airborne_restart();
    Fixture("survey", true).full_flight();
    Fixture("takeover", false).pilot_takeover();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "Mission lifecycle: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
