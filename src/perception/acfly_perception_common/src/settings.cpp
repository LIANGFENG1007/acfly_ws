#include "acfly_perception_common/settings.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace acfly::perception {

Settings::Settings(rclcpp::Node& node, const std::string& package)
    : node_(node),
      defaults_(package, {"input.yaml", "region.yaml", "detection.yaml", "display.yaml"}) {}

rclcpp::Parameter Settings::read(const std::string& name) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  // Accept integer and floating forms of numeric parameters, including older ROS CLI commands.
  descriptor.dynamic_typing = true;
  return rclcpp::Parameter(name, defaults_.declare_value(node_, name, descriptor));
}

void Settings::check_overrides() const { defaults_.check(node_); }

double Settings::number(const std::string& name) {
  const auto param = read(name);
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER &&
      param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::invalid_argument(name + " must be numeric");
  }
  const double value = param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                           ? static_cast<double>(param.as_int())
                           : param.as_double();
  if (!std::isfinite(value)) throw std::invalid_argument(name + " must be finite");
  return value;
}

double Settings::positive(const std::string& name) {
  const double value = number(name);
  if (value <= 0) throw std::invalid_argument(name + " must be positive");
  return value;
}

int Settings::count(const std::string& name) {
  const double value = positive(name);
  if (std::floor(value) != value || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(name + " must be a positive whole number");
  }
  return static_cast<int>(value);
}

std_msgs::msg::ColorRGBA Settings::color(const std::string& name) {
  const auto param = read(name);
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    throw std::invalid_argument(name + " must be an array of floating-point RGBA channels");
  }
  const auto& values = param.as_double_array();
  if (values.size() != 4) throw std::invalid_argument(name + " needs four RGBA channels");
  for (const double channel : values) {
    if (!(channel >= 0 && channel <= 1))
      throw std::invalid_argument(name + " must use [0,1] channels");
  }
  std_msgs::msg::ColorRGBA result;
  result.r = values[0];
  result.g = values[1];
  result.b = values[2];
  result.a = values[3];
  return result;
}

std::string Settings::text(const std::string& name) {
  const auto param = read(name);
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    throw std::invalid_argument(name + " must be a string");
  }
  const auto& value = param.as_string();
  if (value.empty()) throw std::invalid_argument(name + " must be nonempty");
  return value;
}

}  // namespace acfly::perception
