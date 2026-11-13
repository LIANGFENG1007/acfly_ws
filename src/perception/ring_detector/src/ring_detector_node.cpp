#include <acfly_config/initialize.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <memory>

#include "acfly_perception_common/detector_node.hpp"
#include "markers.hpp"
#include "ring_fit.hpp"

namespace ring_detector {
using acfly::perception::DetectorNode;
using acfly::perception::Points;
using acfly::perception::yaw_error_deg;

class Detector : public DetectorNode {
 public:
  Detector() : DetectorNode("ring_detector"), display_(settings_) {
    markers_ =
        create_publisher<visualization_msgs::msg::Marker>(marker_topic_, display_queue_depth_);
    settings_.check_overrides();
  }

 private:
  void detect(const Points& points, const std_msgs::msg::Header& header) override {
    Eigen::Vector3d center, e1, e2;
    double radius;
    if (!fit_circle(points, center, radius, e1, e2) || radius <= min_radius_) return;
    Eigen::Vector3d normal = e1.cross(e2);
    // Pick the same side of the plane as the vector from the sensor to the ring.
    if (normal.head<2>().dot(center.head<2>() - Eigen::Vector2d(pose_.x, pose_.y)) < 0)
      normal = -normal;
    const double yaw = std::atan2(normal.y(), normal.x());
    publish_measurement(
        {center.x(), center.y(), center.z(), yaw_error_deg(yaw, pose_.yaw), yaw * 180.0 / M_PI});
    for (const auto& marker : make_markers(center, radius, e1, e2, normal, header, display_)) {
      markers_->publish(marker);
    }
  }

  Display display_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_;
};
}  // namespace ring_detector

int main(int argc, char** argv) {
  try {
    acfly_config::initialize(argc, argv);
    rclcpp::spin(std::make_shared<ring_detector::Detector>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "ring_detector: %s\n", error.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
