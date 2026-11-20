#include "fly_mission/mavros_link.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace fly_mission {
double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

MavrosLink::MavrosLink(rclcpp::Node &node, const Config &config)
    : node_(node), config_(config), feedback_(config) {
  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", 10, [this](mavros_msgs::msg::State::ConstSharedPtr msg) {
        state_ = *msg;
        last_state_ = steady_seconds();
      });
  using ExtendedState = mavros_msgs::msg::ExtendedState;
  extended_state_sub_ = node_.create_subscription<ExtendedState>(
      "/mavros/extended_state", rclcpp::SensorDataQoS(), [this](ExtendedState::ConstSharedPtr msg) {
        landed_state_ = msg->landed_state;
        last_ground_state_ = steady_seconds();
      });
  odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        feedback_.update(*msg, steady_seconds());
      });
  setpoint_pub_ =
      node_.create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local", 10);
  mode_client_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
}

bool MavrosLink::state_fresh(double time) const {
  return state_wait_reason(time) == nullptr;
}

bool MavrosLink::on_ground(double time) const {
  return ground_wait_reason(time) == nullptr;
}

const char *MavrosLink::state_wait_reason(double time) const {
  if (!last_state_)
    return "未收到 /mavros/state";
  if (time - *last_state_ > config_.fcu_state_timeout_s)
    return "/mavros/state 更新超时";
  if (!state_.connected)
    return "MAVROS 已发布状态，但飞控未连接 (connected=false)";
  return nullptr;
}

const char *MavrosLink::ground_wait_reason(double time) const {
  if (!last_ground_state_)
    return "未收到 /mavros/extended_state";
  if (time - *last_ground_state_ > config_.fcu_state_timeout_s)
    return "/mavros/extended_state 更新超时";
  using Ext = mavros_msgs::msg::ExtendedState;
  switch (landed_state_) {
  case Ext::LANDED_STATE_ON_GROUND:
    return nullptr;
  case Ext::LANDED_STATE_IN_AIR:
    return "飞控报告在空中 (IN_AIR)";
  case Ext::LANDED_STATE_TAKEOFF:
    return "飞控报告正在起飞 (TAKEOFF)";
  case Ext::LANDED_STATE_LANDING:
    return "飞控报告正在降落 (LANDING)";
  default:
    return "飞控尚未确认在地面 (landed_state 未知)";
  }
}

void MavrosLink::publish_velocity(Velocity cmd) {
  using Target = mavros_msgs::msg::PositionTarget;
  if (!(std::isfinite(cmd.x) && std::isfinite(cmd.y) && std::isfinite(cmd.z) &&
        std::isfinite(cmd.yaw_rate))) {
    cmd = {};
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000, "非有限速度，输出零速度");
  }
  Target target;
  target.header.stamp = node_.now();
  target.header.frame_id = "map";
  // 与原 MAVROS 契约一致：ROS 侧是本地 ENU；插件负责转成 FCU 的 NED。
  target.coordinate_frame = Target::FRAME_LOCAL_NED;
  target.type_mask = Target::IGNORE_PX | Target::IGNORE_PY | Target::IGNORE_PZ |
                     Target::IGNORE_AFX | Target::IGNORE_AFY | Target::IGNORE_AFZ |
                     Target::IGNORE_YAW;
  target.velocity.x = cmd.x;
  target.velocity.y = cmd.y;
  target.velocity.z = cmd.z;
  target.yaw_rate = cmd.yaw_rate;
  setpoint_pub_->publish(target);
}

void MavrosLink::request_landing(double time) {
  if (!state_fresh(time) || !armed() || !is_offboard() ||
      (last_request_ && time - *last_request_ < config_.landing_retry_s))
    return;
  last_request_ = time;
  if (!mode_client_->service_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "等待 MAVROS set_mode 服务；保持降落交接点");
    return;
  }
  if (pending_request_)
    mode_client_->remove_pending_request(*pending_request_);
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "AUTO.LAND";
  pending_request_ = mode_client_->async_send_request(request).request_id;
  RCLCPP_INFO(node_.get_logger(), "请求 AUTO.LAND，等待飞控模式确认");
}
} // namespace fly_mission
