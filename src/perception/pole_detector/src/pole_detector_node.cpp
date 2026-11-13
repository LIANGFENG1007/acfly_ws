#include <acfly_config/initialize.hpp>
#include <cmath>
#include <cstdio>
#include <memory>

#include "acfly_perception_common/detector_node.hpp"
#include "markers.hpp"

namespace pole_detector {
using acfly::perception::DetectorNode;
using acfly::perception::fit_pole;
using acfly::perception::Points;
using acfly::perception::yaw_error_deg;

class Detector : public DetectorNode {
 public:
  Detector()
      : DetectorNode("pole_detector"),
        max_radius_(settings_.positive("max_radius")),
        display_(settings_) {
    if (max_radius_ <= min_radius_)
      throw std::invalid_argument("max_radius must exceed min_radius");
    markers_ =
        create_publisher<visualization_msgs::msg::Marker>(marker_topic_, display_queue_depth_);
    settings_.check_overrides();
  }

 private:
  void detect(const Points& points, const std_msgs::msg::Header& header) override {
    const auto pole = fit_pole(points);
    if (!pole || pole->radius <= min_radius_ || pole->radius > max_radius_) return;
    const double bearing = std::atan2(pole->y - pose_.y, pole->x - pose_.x);
    publish_measurement(
        {pole->x, pole->y, pole->z, yaw_error_deg(bearing, pose_.yaw), pole->radius});
    for (const auto& marker : make_markers(*pole, region_, header, display_))
      markers_->publish(marker);
  }
  double max_radius_;
  Display display_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_;
};
}  // namespace pole_detector

int main(int argc, char** argv) {
  try {
    acfly_config::initialize(argc, argv);
    rclcpp::spin(std::make_shared<pole_detector::Detector>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "pole_detector: %s\n", error.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
