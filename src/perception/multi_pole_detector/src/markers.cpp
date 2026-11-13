#include "markers.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace multi_pole_detector {
Display::Display(Settings& settings)
    : center_color(settings.color("center_color")),
      label_color(settings.color("label_color")),
      center_diameter(settings.positive("center_diameter")),
      label_offset(settings.positive("label_offset")),
      label_height(settings.positive("label_height")) {}

visualization_msgs::msg::MarkerArray make_markers(const std::vector<Pole>& poles,
                                                  const std_msgs::msg::Header& header,
                                                  const Display& display) {
  visualization_msgs::msg::MarkerArray arr;
  {
    visualization_msgs::msg::Marker clr;
    clr.header = header;
    clr.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clr);
  }
  for (const auto& p : poles) {
    visualization_msgs::msg::Marker sph;
    sph.header = header;
    sph.color = display.center_color;
    sph.ns = "pole_center";
    sph.id = p.id;
    sph.type = visualization_msgs::msg::Marker::SPHERE;
    sph.action = visualization_msgs::msg::Marker::ADD;
    sph.pose.position.x = p.x;
    sph.pose.position.y = p.y;
    sph.pose.position.z = p.z;
    sph.pose.orientation.w = 1.0;
    sph.scale.x = sph.scale.y = sph.scale.z = display.center_diameter;
    arr.markers.push_back(sph);

    visualization_msgs::msg::Marker txt;
    txt.header = header;
    txt.color = display.label_color;
    txt.ns = "pole_id";
    txt.id = p.id;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x = p.x;
    txt.pose.position.y = p.y;
    txt.pose.position.z = p.z + display.label_offset;
    txt.pose.orientation.w = 1.0;
    txt.scale.z = display.label_height;
    txt.text = "#" + std::to_string(p.id);
    arr.markers.push_back(txt);
  }
  return arr;
}
}  // namespace multi_pole_detector
