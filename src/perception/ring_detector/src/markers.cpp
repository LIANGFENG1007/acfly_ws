#include "markers.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace ring_detector {
Display::Display(Settings& settings)
    : center_color(settings.color("center_color")),
      circle_color(settings.color("circle_color")),
      normal_color(settings.color("normal_color")),
      center_diameter(settings.positive("center_diameter")),
      line_width(settings.positive("line_width")),
      circle_segments(settings.count("circle_segments")),
      shaft_diameter(settings.positive("shaft_diameter")),
      head_diameter(settings.positive("head_diameter")),
      normal_length(settings.positive("normal_length")) {
  if (circle_segments < 3) throw std::invalid_argument("circle_segments must be at least 3");
}

std::vector<visualization_msgs::msg::Marker> make_markers(const Eigen::Vector3d& center,
                                                          double radius, const Eigen::Vector3d& e1,
                                                          const Eigen::Vector3d& e2,
                                                          const Eigen::Vector3d& n,
                                                          const std_msgs::msg::Header& header,
                                                          const Display& display) {
  visualization_msgs::msg::Marker sphere;
  sphere.header = header;
  sphere.color = display.center_color;
  sphere.ns = "ring_center";
  sphere.id = 0;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.pose.position.x = center.x();
  sphere.pose.position.y = center.y();
  sphere.pose.position.z = center.z();
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = sphere.scale.y = sphere.scale.z = display.center_diameter;
  visualization_msgs::msg::Marker circle;
  circle.header = header;
  circle.color = display.circle_color;
  circle.ns = "ring_fit";
  circle.id = 1;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.scale.x = display.line_width;
  circle.pose.orientation.w = 1.0;
  const int seg = display.circle_segments;
  for (int k = 0; k <= seg; ++k) {
    const double ang = 2.0 * M_PI * k / seg;
    const Eigen::Vector3d pt = center + radius * (std::cos(ang) * e1 + std::sin(ang) * e2);
    geometry_msgs::msg::Point p;
    p.x = pt.x();
    p.y = pt.y();
    p.z = pt.z();
    circle.points.push_back(p);
  }
  visualization_msgs::msg::Marker arrow;
  arrow.header = header;
  arrow.color = display.normal_color;
  arrow.ns = "ring_normal";
  arrow.id = 2;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::ADD;
  arrow.scale.x = display.shaft_diameter;
  arrow.scale.y = display.head_diameter;
  arrow.scale.z = 0.0;
  arrow.pose.orientation.w = 1.0;
  {
    const Eigen::Vector3d tip = center + display.normal_length * n.normalized();
    geometry_msgs::msg::Point p0, p1;
    p0.x = center.x();
    p0.y = center.y();
    p0.z = center.z();
    p1.x = tip.x();
    p1.y = tip.y();
    p1.z = tip.z();
    arrow.points.push_back(p0);
    arrow.points.push_back(p1);
  }
  return {sphere, circle, arrow};
}
}  // namespace ring_detector
