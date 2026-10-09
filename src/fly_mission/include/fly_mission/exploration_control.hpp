#pragma once
#include <cmath>
#include <acfly_interfaces/msg/exploration_command.hpp>
#include "fly_mission/exploration_completion.hpp"

namespace fly_mission {
inline double message_seconds(const builtin_interfaces::msg::Time& t) {
    return static_cast<double>(t.sec) + t.nanosec * 1e-9;
}
inline bool valid_exploration_control(const acfly_interfaces::msg::ExplorationCommand& msg,
    const std_msgs::msg::Header& mission, double now_seconds, double timeout,
    double max_vertical_speed, const acfly_interfaces::msg::ExplorationCommand* previous = nullptr)
{
    const double stamp = message_seconds(msg.header.stamp);
    const double age = now_seconds - stamp;
    using Command = acfly_interfaces::msg::ExplorationCommand;
    const bool position = msg.control_mode == Command::POSITION;
    const bool valid_mode = position
        ? msg.header.frame_id == "camera_init" && std::isfinite(msg.position.x) &&
          std::isfinite(msg.position.y) && std::isfinite(msg.position.z) && std::isfinite(msg.yaw) &&
          msg.release_index == 0 && std::abs(msg.position.z-msg.altitude)<1e-6
        : msg.control_mode == Command::VELOCITY && msg.header.frame_id == "base_link" &&
          std::isfinite(msg.velocity.linear.x) && std::isfinite(msg.velocity.linear.y) &&
          std::isfinite(msg.velocity.angular.z);
    return valid_mode && completion_matches_goal(msg.mission, mission) &&
        (!previous || previous->control_mode != Command::POSITION || position) &&
        std::isfinite(now_seconds) && age >= -.05 && age <= timeout &&
        stamp >= message_seconds(mission.stamp) &&
        (!previous || stamp > message_seconds(previous->header.stamp)) &&
        std::isfinite(msg.altitude) && std::isfinite(msg.vertical_speed_limit) &&
        msg.vertical_speed_limit > 0 && msg.vertical_speed_limit <= max_vertical_speed &&
        msg.release_index <= 3 &&
        (!msg.release_index || msg.target_id != 0);
}
} // namespace fly_mission
