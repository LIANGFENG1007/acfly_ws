#pragma once

#include <std_msgs/msg/header.hpp>

namespace fly_mission {

// Completion echoes the original goal identity, rather than relying on seeing
// a preceding Bool(false) which a keep-last subscriber can miss while busy.
inline bool completion_matches_goal(const std_msgs::msg::Header& completion,
                                    const std_msgs::msg::Header& goal)
{
    return !goal.frame_id.empty() && goal.stamp.sec >= 0 &&
        (goal.stamp.sec != 0 || goal.stamp.nanosec != 0) &&
        completion.frame_id == goal.frame_id && completion.stamp == goal.stamp;
}

}  // namespace fly_mission
