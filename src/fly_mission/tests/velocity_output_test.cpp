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
#include "fly_mission/exploration_control.hpp"
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
        acfly_interfaces::msg::ExplorationCommand control;
        control.mission=goal;control.header.frame_id="base_link";control.header.stamp.sec=43;
        control.altitude=.5;control.vertical_speed_limit=.4;control.release_index=1;control.target_id=1;
        const auto valid=[&](const auto& m){return fly_mission::valid_exploration_control(m,goal,43.1,.3,.6);};
        check(valid(control),"current scoped command rejected");
        auto bad=control;--bad.mission.stamp.sec;check(!valid(bad),"old mission release accepted");
        bad=control;bad.header.stamp.sec=42;check(!valid(bad),"stale control accepted");
        bad=control;bad.header.stamp.sec=44;check(!valid(bad),"future control accepted");
        bad=control;bad.velocity.linear.x=NAN;check(!valid(bad),"NaN XY accepted");
        bad=control;bad.altitude=NAN;check(!valid(bad),"NaN height accepted");
        bad=control;bad.vertical_speed_limit=.7;check(!valid(bad),"excessive vertical cap accepted");
        bad=control;bad.target_id=0;check(!valid(bad),"anonymous release accepted");
        bad=control;bad.release_index=4;check(!valid(bad),"fourth package accepted");
        check(!fly_mission::valid_exploration_control(control,goal,43.1,.3,.6,&control),"duplicate command refreshed watchdog");
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
        const auto odom = [&](bool advancing = true) {
            if (advancing) pose->header.stamp = node->now();
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
        drone.set_exploration_altitude(.5,.4);
        drone.set_velocity_body(.002, -.003, -.004); drone.tick();
        check(outputs.back().velocity.x==0 && outputs.back().velocity.y==0 &&
              outputs.back().velocity.z==0 && outputs.back().yaw_rate==0,
              "fresh commands bypassed stale odometry and kept moving/descending");
        time(100.42);odom();
        drone.set_exploration_altitude(.8,.1);
        drone.set_velocity_body(.002, -.003, -.004); drone.tick();
        check(std::abs(outputs.back().velocity.x - .003) < 1e-8 &&
              std::abs(outputs.back().velocity.y - .002) < 1e-8,
              "fresh small commands failed to resume after timeout or were rounded to zero");

        time(101); pose->pose.pose.position=geometry_msgs::msg::Point();pose->pose.pose.position.z=1.4;odom();
        drone.v_est_x_=drone.v_est_y_=drone.v_est_z_=0;
        drone.set_exploration_altitude(.5,.4);drone.set_velocity_body(.1,-.05,0);drone.tick();
        check(std::abs(outputs.back().velocity.z+.4)<1e-6 && outputs.back().velocity.x>.04 && outputs.back().velocity.y>.09,
              "descent lost coordinated XYZ output or retained 0.1m/s level cap");
        drone.set_exploration_altitude(2.5,.6);drone.set_velocity_body(.1,-.05,0);drone.tick();
        check(std::abs(outputs.back().velocity.z-.6)<1e-6,"climb lost old 0.6m/s cap");
        time(101.4);odom();drone.v_est_z_=0;drone.tick();
        check(drone.explore_z_==1.4 && outputs.back().velocity.z==0,"watchdog continued old vertical movement");
        time(101.42);pose->pose.pose.position.x=.1;pose->pose.pose.position.y=.05;odom();drone.tick();
        check(outputs.back().velocity.x<0 && outputs.back().velocity.y<0,"watchdog hold gave zero XY despite drift");
        check((outputs.back().type_mask & (Target::IGNORE_PX|Target::IGNORE_PY|Target::IGNORE_PZ))==
              (Target::IGNORE_PX|Target::IGNORE_PY|Target::IGNORE_PZ),"delivery switched FCU position mode");

        time(101.80);pose->pose.pose.position.x=9;odom(false);
        check(!drone.has_fresh_pose() && drone.current_x()==.1,"duplicate odometry refreshed age or replaced current pose");
        drone.set_velocity_body(.1,.1,.1);drone.tick();
        check(outputs.back().velocity.x==0 && outputs.back().velocity.y==0 && outputs.back().velocity.z==0,
              "frozen source stamp bypassed motion guard");
        time(101.82);pose->pose.pose.position.x=.1;odom();
        drone.pose_received_ -= std::chrono::seconds(1); // ROS clock frozen, real receipt already stale.
        check(!drone.has_fresh_pose(),"paused ROS clock defeated local pose timeout");
        drone.set_velocity_body(.1,.1,.1);drone.tick();
        check(outputs.back().velocity.x==0 && outputs.back().velocity.z==0,"stale steady-clock pose was used");
        time(101.84);pose->header.stamp.sec=1;pose->header.stamp.nanosec=0;odom(false);
        check(!drone.has_fresh_pose(),"sensor timestamp restart was accepted before progression");
        pose->header.stamp.nanosec=20000000;odom(false);
        check(drone.has_fresh_pose(),"progressing sensor failed to recover after restart");
        drone.set_exploration_altitude(1.4,.1);drone.set_velocity_body(.02,.03,0);drone.tick();
        check(std::abs(outputs.back().velocity.x+.03)<1e-6 && std::abs(outputs.back().velocity.y-.02)<1e-6,
              "fresh pose after restart failed to resume XY feedback");

        // Source lifetime is checked by the final publisher even when ROS time
        // pauses and the mission dispatcher has not run its next expiry check.
        drone.set_exploration_altitude(.5,.4);drone.set_velocity_body(.1,.1,.1,.01);
        drone.ext_received_ -= std::chrono::milliseconds(20);drone.tick();
        check(outputs.back().velocity.x==0 && outputs.back().velocity.y==0 &&
              outputs.back().velocity.z==0 && outputs.back().yaw_rate==0,
              "cached source deadline was extended by a paused ROS clock");

        std::cout << "Goal-scoped completion, final MAVROS XY/yaw payload, invalid odometry recovery, watchdog, tiny velocities passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; result = 1;
    }
    rclcpp::shutdown();
    return result;
}
