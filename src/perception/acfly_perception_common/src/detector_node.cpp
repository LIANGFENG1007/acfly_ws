#include "acfly_perception_common/detector_node.hpp"

#include <cmath>
#include <stdexcept>

namespace acfly::perception {
namespace {
std::string normalized_frame(const std::string& id) {
  return !id.empty() && id.front() == '/' ? id.substr(1) : id;
}
}  // namespace

DetectorNode::DetectorNode(const std::string& package)
    : Node(package + "_node"),
      settings_(*this, package),
      region_{settings_.number("box_min_x"), settings_.number("box_max_x"),
              settings_.number("box_min_y"), settings_.number("box_max_y"),
              settings_.number("box_min_z"), settings_.number("box_max_z")},
      frame_id_(settings_.text("frame_id")),
      min_radius_(settings_.positive("min_radius")),
      marker_topic_(settings_.text("marker_topic")),
      display_queue_depth_(settings_.count("display_queue_depth")),
      min_points_(settings_.count("min_points")),
      log_interval_ms_(settings_.count("log_interval_ms")),
      window_(region_, settings_.count("accum_frames")) {
  center_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      settings_.text("center_topic"), settings_.count("measurement_queue_depth"));
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      settings_.text("cropped_cloud_topic"), display_queue_depth_);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      settings_.text("cloud_topic"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2& msg) { receive_cloud(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      settings_.text("odom_topic"), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry& msg) {
        if (normalized_frame(msg.header.frame_id) != normalized_frame(frame_id_)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_interval_ms_,
                               "忽略坐标系 %s 的里程计，需要 %s", msg.header.frame_id.c_str(),
                               frame_id_.c_str());
          return;
        }
        const auto& p = msg.pose.pose.position;
        const auto& q = msg.pose.pose.orientation;
        const double yaw =
            std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(yaw)) pose_ = {p.x, p.y, yaw};
      });
  RCLCPP_INFO(get_logger(), "坐标系 %s；区域 x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
              frame_id_.c_str(), region_.min_x, region_.max_x, region_.min_y, region_.max_y,
              region_.min_z, region_.max_z);
}

void DetectorNode::receive_cloud(const sensor_msgs::msg::PointCloud2& msg) {
  if (normalized_frame(msg.header.frame_id) != normalized_frame(frame_id_)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_interval_ms_,
                         "忽略坐标系 %s 的点云，需要 %s", msg.header.frame_id.c_str(),
                         frame_id_.c_str());
    return;
  }
  Points points;
  try {
    points = window_.update(msg);
  } catch (const std::invalid_argument& error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_interval_ms_, "忽略无效点云: %s",
                         error.what());
    return;
  }
  auto header = msg.header;
  header.frame_id = frame_id_;
  cloud_pub_->publish(make_cloud(points, header));
  if (points.size() < min_points_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), log_interval_ms_,
                         "裁剪后累计 %zu 点，低于 min_points=%zu", points.size(), min_points_);
    return;
  }
  detect(points, header);
}

void DetectorNode::publish_measurement(std::vector<double> fields) {
  std_msgs::msg::Float64MultiArray result;
  result.data = std::move(fields);
  center_pub_->publish(result);
}

double yaw_error_deg(double bearing, double sensor_yaw) {
  double error = std::remainder(bearing - sensor_yaw, 2.0 * M_PI);
  if (error <= -M_PI) error += 2.0 * M_PI;
  return error * 180.0 / M_PI;
}
}  // namespace acfly::perception
