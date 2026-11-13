#pragma once

#include <rcl/arguments.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace acfly_config {

// Humble can silently keep the first type when two explicit CLI sources set the
// same parameter with different types. Parse each source with ROS's own parser
// before ROS merges them. Ordinary same-type overrides remain last-write-wins.
inline void initialize(int argc, char **argv) {
  bool in_ros_args = false;
  std::map<std::pair<std::string, std::string>, rclcpp::ParameterType> types;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--ros-args") {
      in_ros_args = true;
    } else if (arg == "--") {
      in_ros_args = false;
    } else if (in_ros_args && i + 1 < argc &&
               (arg == "-p" || arg == "--param" || arg == "--params-file")) {
      rclcpp::NodeOptions source;
      source.arguments({"--ros-args", arg, argv[++i]});
      rcl_params_t *raw = nullptr;
      const auto status =
          rcl_arguments_get_param_overrides(&source.get_rcl_node_options()->arguments, &raw);
      std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> parsed(
          raw, &rcl_yaml_node_struct_fini);
      if (status != RCL_RET_OK)
        throw std::runtime_error("Cannot read ROS override: " + arg);
      if (!parsed)
        continue;
      for (const auto &group : rclcpp::parameter_map_from(parsed.get())) {
        for (const auto &parameter : group.second) {
          auto [entry, inserted] = types.emplace(std::make_pair(group.first, parameter.get_name()),
                                                 parameter.get_type());
          if (!inserted && entry->second != parameter.get_type()) {
            throw std::invalid_argument("Conflicting override types for " + group.first + ": " +
                                        parameter.get_name());
          }
        }
      }
    }
  }
  rclcpp::init(argc, argv);
}

} // namespace acfly_config
