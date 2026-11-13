#pragma once
#include <visualization_msgs/msg/marker.hpp>

#include "acfly_perception_common/pole_fit.hpp"
#include "acfly_perception_common/settings.hpp"

namespace ring_detector {
using acfly::perception::Settings;

struct Display {
  explicit Display(Settings& settings);
  std_msgs::msg::ColorRGBA center_color;
  std_msgs::msg::ColorRGBA circle_color;
  std_msgs::msg::ColorRGBA normal_color;
  double center_diameter;
  double line_width;
  int circle_segments;
  double shaft_diameter;
  double head_diameter;
  double normal_length;
};
std::vector<visualization_msgs::msg::Marker> make_markers(const Eigen::Vector3d& center,
                                                          double radius, const Eigen::Vector3d& e1,
                                                          const Eigen::Vector3d& e2,
                                                          const Eigen::Vector3d& n,
                                                          const std_msgs::msg::Header& header,
                                                          const Display& display);
}  // namespace ring_detector
