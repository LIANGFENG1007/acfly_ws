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

#include <bits/stdc++.h>
#include <nlohmann/json.hpp>
#include "fly_mission/drone_controller.hpp"
#include "fly_mission/params.hpp"
#include "fly_mission/mission_state.hpp"
#include "fly_mission/find_figure.hpp"
#include "fly_mission/waypoint_runner.hpp"
#include "fly_mission/waypoint_receiver.hpp"
#include "fly_mission/line_follower.hpp"
#include "fly_mission/ring_driller.hpp"
#include "fly_mission/car_tracker.hpp"
#include "fly_mission/shm_mailbox.hpp"   // 视觉 shm 信箱读端，见 params::USE_SHM_CV
#include "fly_mission/udp_cmd_link.hpp"  // 跨机命令(启动指令)UDP 读端，见 params::CMD_USE_UDP
#include "fly_mission/udp_telemetry.hpp" // 遥测上报(飞机→监控端)，见 params::TLM_ENABLE
#include "fly_mission/arduino_serial.hpp"
#include "fly_mission/exploration_drop_sender.hpp"
#include <acfly_interfaces/msg/exploration_context.hpp>
#include <acfly_interfaces/msg/exploration_command.hpp>
#include <acfly_interfaces/msg/drop_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>   // 任务启动指令 /mission/start
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#define private public
#include "fly_mission/fly_mission_node.hpp"
#undef private
#define main fly_mission_original_main
#include "../src/fly_mission_node.cpp"
#undef main

using namespace fly_mission;
using Target = mavros_msgs::msg::PositionTarget;
Target last_output;
unsigned output_count = 0;
extern "C" rcl_ret_t __wrap_rcl_publish(
    const rcl_publisher_t* publisher, const void* msg, rmw_publisher_allocation_t*) {
    if (std::string(rcl_publisher_get_topic_name(publisher)) == "/mavros/setpoint_raw/local") {
        last_output = *static_cast<const Target*>(msg);
        ++output_count;
    }
    return RCL_RET_OK; // All test publications are intercepted; none reach a vehicle.
}
extern "C" rcl_ret_t __wrap_rcl_send_request(const rcl_client_t*,const void*,int64_t*) {
    throw std::runtime_error("unexpected hardware service request");
}
extern "C" int __wrap_socket(int,int,int) { errno=EACCES;return -1; } // No task UDP endpoints.
extern "C" int __real_open(const char*,int,...);
extern "C" int __wrap_open(const char* path,int flags,...) {
    const std::string p(path);
    if(p.rfind("/dev/shm/",0)==0 || p.rfind("/dev/tty",0)==0) {errno=EACCES;return -1;}
    if(flags&O_CREAT) {va_list a;va_start(a,flags);const auto mode=va_arg(a,mode_t);va_end(a);return __real_open(path,flags,mode);}
    return __real_open(path,flags);
}
void require(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        auto node = std::make_shared<FlyMissionNode>();
        node->timer_->cancel();
        auto& drone = node->drone_;
        auto* clock = node->get_clock()->get_clock_handle();
        require(rcl_enable_ros_time_override(clock) == RCL_RET_OK, "enable test clock");
        auto odom = std::make_shared<nav_msgs::msg::Odometry>();
        odom->pose.pose.position.x = 1.1;
        odom->pose.pose.position.y = -0.7;
        odom->pose.pose.position.z = 0.3;
        odom->pose.pose.orientation.w = 1.0;
        bool send_pose = false;
        double t = 100.0;
        const auto tick = [&](double elapsed = 0.02) {
            t += elapsed;
            require(rcl_set_ros_time_override(clock, static_cast<int64_t>(t * 1e9)) == RCL_RET_OK,
                    "advance test clock");
            if (send_pose) {
                odom->header.stamp = node->now();
                std::shared_ptr<void> msg = odom;
                drone.odom_sub_->handle_message(msg, rclcpp::MessageInfo{});
            }
            node->on_timer();
        };
        const auto waiting = [&] {
            require(node->state_ == MissionState::BOOT_CHECK, "left BOOT_CHECK before ready");
            require(drone.action_mode() == ActionMode::IDLE, "started flight while waiting");
            require(!drone.home_captured_, "captured home before manual start completed");
            require(drone.arm_retry_count_ == 0 && !drone.arm_time_valid_, "attempted programmatic arming");
            require(drone.offb_retry_count_ == 0, "attempted programmatic OFFBOARD switch");
            if (params::OFFBOARD_PREHEAT) {
                require(output_count > 0, "preheat stream missing");
                const auto position_ignored = Target::IGNORE_PX | Target::IGNORE_PY | Target::IGNORE_PZ;
                require((last_output.type_mask & position_ignored) == position_ignored,
                        "preheat commanded a position target");
                require(last_output.velocity.x == 0 && last_output.velocity.y == 0 &&
                        last_output.velocity.z == 0 && last_output.yaw_rate == 0,
                        "preheat commanded movement");
            } else {
                require(output_count == 0, "disabled preheat published setpoints");
            }
        };

        // Connection, pose and both beeps remain prerequisites.
        tick(); waiting();
        require(!node->beep_sent_, "beep before connection");
        drone.current_state_.connected = true;
        tick(); waiting();
        require(!node->beep_sent_, "beep before odometry");
        send_pose = true;
        tick(); waiting();
        require(node->beep_sent_ && !node->beep2_sent_, "first beep/gap bypassed");
        if (params::START_CMD_REQUIRED) {
            tick(params::BOOT_BEEP_GAP_S + 1); waiting();
            require(!node->beep2_sent_, "required start command bypassed");
            node->start_recv_ = true;
        }
        tick(params::BOOT_BEEP_GAP_S + 0.1); waiting();
        require(node->beep2_sent_, "second beep missing");

        // OFFBOARD alone cannot arm or start; waiting never exhausts automatic retries.
        drone.current_state_.mode = "OFFBOARD";
        for (int i = 0; i < 10; ++i) { tick(1.0); waiting(); }
        require(!node->check_offboard_done_, "OFFBOARD accepted before manual arming");

        // Manual arming alone cannot start. Disarming while waiting revokes readiness.
        drone.current_state_.mode = "POSCTL";
        drone.current_state_.armed = true;
        tick(); waiting();
        require(node->check_armed_done_, "manual arming not detected");
        tick(2.0); waiting();
        drone.current_state_.armed = false;
        drone.current_state_.mode = "OFFBOARD";
        tick(); waiting();
        require(!node->check_armed_done_, "stale armed readiness survived disarming");

        // Even both vehicle flags cannot bypass a missing connection or pose.
        drone.current_state_.armed = true;
        drone.current_state_.connected = false;
        tick(); waiting();
        drone.current_state_.connected = true;
        send_pose = false;
        drone.has_pose_.store(false);
        tick(); waiting();

        // Use the latest launch pose, captured only after both pilot-controlled flags hold.
        send_pose = true;
        odom->pose.pose.position.x = 1.4;
        odom->pose.pose.position.y = -0.9;
        odom->pose.pose.position.z = 0.4;
        tick();
        require(node->state_ == MissionState::TAKEOFF && drone.home_captured_, "manual start did not enter TAKEOFF");
        require(std::abs(drone.home_x_ - 1.4) < 1e-6 &&
                std::abs(drone.home_y_ + 0.9) < 1e-6 &&
                std::abs(drone.home_z_ - 0.4) < 1e-6, "takeoff used an old home pose");
        tick();
        require(drone.action_mode() == ActionMode::TAKEOFF, "takeoff action not started");
        require(drone.target_x_ == drone.home_x_ && drone.target_y_ == drone.home_y_ &&
                drone.target_z_ > drone.home_z_, "takeoff target is not above launch pose");
        const auto count_before_climb = output_count;
        tick();
        require(output_count > count_before_climb, "takeoff setpoint missing");
        require(drone.arm_retry_count_ == 0 && drone.offb_retry_count_ == 0,
                "manual start issued an automatic arm/mode request");
        std::cout << "Manual takeoff: self-check/beeps, OFFBOARD without arm, arm without OFFBOARD, "
                     "disarm/rearm, long wait, latest home and takeoff passed; no hardware requests.\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
