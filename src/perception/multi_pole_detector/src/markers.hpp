#pragma once
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "acfly_perception_common/pole_fit.hpp"
#include "acfly_perception_common/settings.hpp"

namespace multi_pole_detector {
using acfly::perception::Pole;
using acfly::perception::Settings;

struct Display {
  explicit Display(Settings& settings);
  std_msgs::msg::ColorRGBA center_color;
  std_msgs::msg::ColorRGBA label_color;
  double center_diameter;
  double label_offset;
  double label_height;
};
visualization_msgs::msg::MarkerArray make_markers(const std::vector<Pole>& poles,
                                                  const std_msgs::msg::Header& header,
                                                  const Display& display);
}  // namespace multi_pole_detector
