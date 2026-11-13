#pragma once
#include <visualization_msgs/msg/marker.hpp>

#include "acfly_perception_common/pole_fit.hpp"
#include "acfly_perception_common/settings.hpp"

namespace pole_detector {
using acfly::perception::Pole;
using acfly::perception::Region;
using acfly::perception::Settings;

struct Display {
  explicit Display(Settings& settings);
  std_msgs::msg::ColorRGBA center_color;
  std_msgs::msg::ColorRGBA cylinder_color;
  double center_diameter;
  double min_cylinder_height;
  double min_cylinder_diameter;
};
std::vector<visualization_msgs::msg::Marker> make_markers(const Pole& pole, const Region& region,
                                                          const std_msgs::msg::Header& header,
                                                          const Display& display);
}  // namespace pole_detector
