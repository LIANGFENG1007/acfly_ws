#include <rclcpp/rclcpp.hpp>
#include <rcl/time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "fly_mission/avoidance.hpp"
#include "fly_mission/shm_mailbox.hpp"
#include "fly_mission/exploration_completion.hpp"
#include <atomic>
#include <chrono>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>
#include <cerrno>
#include <cstdarg>
#include <iostream>
#include <stdexcept>

#define private public
#include "fly_mission/drone_controller.hpp"
#undef private

using Target = mavros_msgs::msg::PositionTarget;
std::vector<Target> outputs;

// Capture the actual final MAVROS payload without sending any flight messages.
extern "C" rcl_ret_t __wrap_rcl_publish(const rcl_publisher_t* publisher,
                                        const void* message, rmw_publisher_allocation_t*)
{
    if (std::string(rcl_publisher_get_topic_name(publisher)) != "/velocity_output_test/setpoint")
        throw std::runtime_error("Unexpected publisher in isolated controller test");
    outputs.push_back(*static_cast<const Target*>(message));
    return RCL_RET_OK;
}

extern "C" int __real_open(const char*, int, ...);
extern "C" int __wrap_open(const char* path, int flags, ...)
{
    if (std::strcmp(path, fly_mission::shm::POSE_SHM_PATH) == 0) {
        errno = EACCES;
        return -1;
    }
    if (flags & O_CREAT) {
        va_list args; va_start(args, flags);
        const mode_t mode = va_arg(args, mode_t); va_end(args);
        return __real_open(path, flags, mode);
    }
    return __real_open(path, flags);
}

void check(bool value, const char* reason)
{
    if (!value) throw std::runtime_error(reason);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        // A busy/late subscriber receives only the retained completion, with no
        // preceding false edge. Old mission completions must still be ignored.
        std_msgs::msg::Header goal;
        goal.frame_id = "camera_init";
        goal.stamp.sec = 42;
        goal.stamp.nanosec = 123;
        auto retained = goal;
        check(fly_mission::completion_matches_goal(retained, goal),
              "current completion depends on a preceding reset message");
        ++goal.stamp.nanosec;
        check(!fly_mission::completion_matches_goal(retained, goal),
              "retained completion ended a new mission at the same location");
        retained = goal;
        check(fly_mission::completion_matches_goal(retained, goal), "new mission completion was ignored");
        retained.frame_id = "other_frame";
        const std_msgs::msg::Header empty;
        check(!fly_mission::completion_matches_goal(retained, goal) &&
              !fly_mission::completion_matches_goal(empty, empty), "invalid completion identity accepted");
        auto node = std::make_shared<rclcpp::Node>("velocity_output_test",
            rclcpp::NodeOptions().arguments({"--ros-args", "-r",
                "/mavros/setpoint_raw/local:=/velocity_output_test/setpoint"}));
        auto* clock = node->get_clock()->get_clock_handle();
        check(rcl_enable_ros_time_override(clock) == RCL_RET_OK, "enable deterministic test clock");
        auto time = [&](double seconds) {
            check(rcl_set_ros_time_override(clock, static_cast<int64_t>(seconds * 1e9)) == RCL_RET_OK,
                  "advance test clock");
        };
        time(100);
        fly_mission::DroneController drone(node.get());
        auto pose = std::make_shared<nav_msgs::msg::Odometry>();
        pose->pose.pose.position.z = .8;
        pose->pose.pose.orientation.z = std::sin(M_PI / 4);
        pose->pose.pose.orientation.w = std::cos(M_PI / 4);
        const auto odom = [&] {
            pose->header.stamp = node->now();
            std::shared_ptr<void> message = pose;
            drone.odom_sub_->handle_message(message, rclcpp::MessageInfo{});
        };
        odom();
        drone.enter_exploration();
        drone.set_velocity_body(-.08, .06, .3);
        drone.tick();
        check(!outputs.empty() && std::abs(outputs.back().velocity.x + .06) < 1e-6 &&
              std::abs(outputs.back().velocity.y + .08) < 1e-6 &&
              std::abs(outputs.back().yaw_rate - .3) < 1e-6,
              "body XY/yaw corrections were zeroed or lost at the MAVROS output");
        check((outputs.back().type_mask & Target::IGNORE_YAW) &&
              !(outputs.back().type_mask & (Target::IGNORE_VX | Target::IGNORE_VY | Target::IGNORE_YAW_RATE)),
              "final mask disabled velocity or yaw-rate fields");

        time(100.02); pose->pose.pose.position.z = std::numeric_limits<double>::quiet_NaN(); odom();
        time(100.04); pose->pose.pose.position.z = .8; odom();
        time(100.06); odom();
        drone.set_velocity_body(-.08, .06, .3); drone.tick();
        check(std::isfinite(drone.v_est_z_) && std::abs(outputs.back().velocity.x + .06) < 1e-6 &&
              std::abs(outputs.back().yaw_rate - .3) < 1e-6,
              "one invalid odometry frame permanently poisoned height feedback and zeroed all output");

        time(100.40); drone.tick();
        check(outputs.back().velocity.x == 0 && outputs.back().velocity.y == 0 && outputs.back().yaw_rate == 0,
              "expired external command bypassed the watchdog");
        drone.set_velocity_body(.002, -.003, -.004); drone.tick();
        check(std::abs(outputs.back().velocity.x - .003) < 1e-8 &&
              std::abs(outputs.back().velocity.y - .002) < 1e-8,
              "fresh small commands failed to resume after timeout or were rounded to zero");

        std::cout << "Goal-scoped completion, final MAVROS XY/yaw payload, invalid odometry recovery, watchdog, tiny velocities passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; result = 1;
    }
    rclcpp::shutdown();
    return result;
}
