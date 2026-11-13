#include "markers.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace pole_detector {
Display::Display(Settings& settings)
    : center_color(settings.color("center_color")),
      cylinder_color(settings.color("cylinder_color")),
      center_diameter(settings.positive("center_diameter")),
      min_cylinder_height(settings.positive("min_cylinder_height")),
      min_cylinder_diameter(settings.positive("min_cylinder_diameter")) {}

std::vector<visualization_msgs::msg::Marker> make_markers(const Pole& pole, const Region& region,
                                                          const std_msgs::msg::Header& header,
                                                          const Display& display) {
  visualization_msgs::msg::Marker sphere;
  sphere.header = header;
  sphere.color = display.center_color;
  sphere.ns = "pole_center";
  sphere.id = 0;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.pose.position.x = pole.x;
  sphere.pose.position.y = pole.y;
  sphere.pose.position.z = pole.z;
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = sphere.scale.y = sphere.scale.z = display.center_diameter;
  visualization_msgs::msg::Marker cyl;
  cyl.header = header;
  cyl.color = display.cylinder_color;
  cyl.ns = "pole_fit";
  cyl.id = 1;
  cyl.type = visualization_msgs::msg::Marker::CYLINDER;
  cyl.action = visualization_msgs::msg::Marker::ADD;
  const double h = std::max(display.min_cylinder_height, region.max_z - region.min_z);
  cyl.pose.position.x = pole.x;
  cyl.pose.position.y = pole.y;
  cyl.pose.position.z = 0.5 * (region.min_z + region.max_z);
  cyl.pose.orientation.w = 1.0;
  cyl.scale.x = cyl.scale.y = std::max(display.min_cylinder_diameter, 2.0 * pole.radius);
  cyl.scale.z = h;
  return {sphere, cyl};
}
}  // namespace pole_detector
