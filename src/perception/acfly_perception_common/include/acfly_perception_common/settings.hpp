#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>

#include "acfly_config/defaults.hpp"

namespace acfly::perception {

// Declare startup-only parameters loaded by acfly_config and validate their effective values.
class Settings {
 public:
  Settings(rclcpp::Node& node, const std::string& package);
  void check_overrides() const;
  double number(const std::string& name);
  double positive(const std::string& name);
  int count(const std::string& name);
  std_msgs::msg::ColorRGBA color(const std::string& name);
  std::string text(const std::string& name);

 private:
  rclcpp::Parameter read(const std::string& name);
  rclcpp::Node& node_;
  acfly_config::Defaults defaults_;
};

}  // namespace acfly::perception
