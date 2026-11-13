#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <initializer_list>
#include <map>
#include <stdexcept>
#include <string>

namespace acfly_config {

// Stock defaults are not command-line overrides: ROS applies the user's actual
// overrides exactly once, when the node declares a parameter of the expected type.
class Defaults {
public:
  Defaults(const std::string &package, std::initializer_list<std::string> categories) {
    const auto directory = ament_index_cpp::get_package_share_directory(package) + "/config/";
    for (const auto &category : categories) {
      const auto path = directory + category;
      try {
        const auto groups = rclcpp::parameter_map_from_yaml_file(path);
        const auto all = groups.find("/**");
        if (groups.size() != 1 || all == groups.end() || all->second.empty()) {
          throw std::invalid_argument("expected a nonempty /**: ros__parameters: section");
        }
        for (const auto &parameter : all->second) {
          if (!entries_.emplace(parameter.get_name(), Entry{parameter.get_parameter_value(), path})
                   .second) {
            throw std::invalid_argument("duplicate default for " + parameter.get_name());
          }
        }
      } catch (const std::exception &error) {
        throw std::runtime_error(path + ": " + error.what());
      }
    }
  }

  template <typename T>
  T declare(rclcpp::Node &node, const std::string &name,
            const rcl_interfaces::msg::ParameterDescriptor &descriptor =
                rcl_interfaces::msg::ParameterDescriptor()) const {
    const Entry &entry = find(name);
    try {
      // Verify the default type even if an operator supplied an override.
      const auto &initial = entry.value.get<T>();
      return node.declare_parameter<T>(name, initial, descriptor);
    } catch (const std::exception &error) {
      throw std::invalid_argument(entry.path + ": " + name + ": " + error.what());
    }
  }

  rclcpp::ParameterValue
  declare_value(rclcpp::Node &node, const std::string &name,
                const rcl_interfaces::msg::ParameterDescriptor &descriptor) const {
    const Entry &entry = find(name);
    return node.declare_parameter(name, entry.value, descriptor);
  }

  void check(rclcpp::Node &node) const {
    for (const auto &item : entries_) {
      if (!node.has_parameter(item.first)) {
        throw std::invalid_argument(item.second.path + ": unused default: " + item.first);
      }
    }
    for (const auto &item : node.get_node_parameters_interface()->get_parameter_overrides()) {
      if (!node.has_parameter(item.first)) {
        throw std::invalid_argument("Unknown parameter: " + item.first);
      }
    }
  }

private:
  struct Entry {
    rclcpp::ParameterValue value;
    std::string path;
  };
  const Entry &find(const std::string &name) const {
    auto entry = entries_.find(name);
    if (entry == entries_.end()) {
      throw std::invalid_argument("Missing default for parameter: " + name);
    }
    return entry->second;
  }
  std::map<std::string, Entry> entries_;
};

} // namespace acfly_config
