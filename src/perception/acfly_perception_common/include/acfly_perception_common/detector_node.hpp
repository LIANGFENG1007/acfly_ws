#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "acfly_perception_common/cloud_window.hpp"
#include "acfly_perception_common/settings.hpp"

namespace acfly::perception {

struct Pose2d {
  double x{}, y{}, yaw{};
};

class DetectorNode : public rclcpp::Node {
 protected:
  explicit DetectorNode(const std::string& package);
  virtual void detect(const Points& points, const std_msgs::msg::Header& header) = 0;
  void publish_measurement(std::vector<double> fields);

  Settings settings_;
  const Region region_;
  const std::string frame_id_;
  Pose2d pose_;
  const double min_radius_;
  const std::string marker_topic_;
  const int display_queue_depth_;

 private:
  void receive_cloud(const sensor_msgs::msg::PointCloud2& cloud);
  const size_t min_points_;
  const int log_interval_ms_;
  CloudWindow window_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr center_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

// Preserve the published angle range (-180, 180].
double yaw_error_deg(double bearing, double sensor_yaw);

}  // namespace acfly::perception
