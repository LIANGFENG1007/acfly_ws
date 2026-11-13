#include <acfly_config/initialize.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include "acfly_perception_common/detector_node.hpp"
#include "clustering.hpp"
#include "markers.hpp"
#include "pole_ids.hpp"

namespace multi_pole_detector {
using acfly::perception::DetectorNode;
using acfly::perception::fit_pole;
using acfly::perception::Points;
using acfly::perception::yaw_error_deg;

class Detector : public DetectorNode {
 public:
  Detector()
      : DetectorNode("multi_pole_detector"),
        max_radius_(settings_.positive("max_radius")),
        tolerance_(settings_.positive("cluster_tol")),
        min_cluster_(settings_.count("min_cluster_points")),
        ids_(settings_.positive("assoc_dist"), settings_.number("track_ema")),
        display_(settings_) {
    if (max_radius_ <= min_radius_)
      throw std::invalid_argument("max_radius must exceed min_radius");
    markers_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, display_queue_depth_);
    settings_.check_overrides();
  }

 private:
  void detect(const Points& points, const std_msgs::msg::Header& header) override {
    std::vector<Pole> poles;
    for (const auto& indices : cluster_xy(points, tolerance_)) {
      if (indices.size() < min_cluster_) continue;
      const auto pole = fit_pole(points, indices);
      if (pole && pole->radius > min_radius_ && pole->radius <= max_radius_) poles.push_back(*pole);
    }
    if (poles.empty()) return;
    ids_.assign(poles);
    std::sort(poles.begin(), poles.end(), [](const Pole& a, const Pole& b) { return a.id < b.id; });

    std::vector<double> fields;
    fields.reserve(poles.size() * 6);
    for (const auto& pole : poles) {
      const double bearing = std::atan2(pole.y - pose_.y, pole.x - pose_.x);
      fields.insert(fields.end(), {static_cast<double>(pole.id), pole.x, pole.y, pole.z,
                                   yaw_error_deg(bearing, pose_.yaw), pole.radius});
    }
    publish_measurement(std::move(fields));
    markers_->publish(make_markers(poles, header, display_));
  }
  double max_radius_, tolerance_;
  size_t min_cluster_;
  PoleIds ids_;
  Display display_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
};
}  // namespace multi_pole_detector

int main(int argc, char** argv) {
  try {
    acfly_config::initialize(argc, argv);
    rclcpp::spin(std::make_shared<multi_pole_detector::Detector>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "multi_pole_detector: %s\n", error.what());
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
