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
using Target=mavros_msgs::msg::PositionTarget;
Target last_output;
acfly_interfaces::msg::ExplorationContext last_context;
acfly_interfaces::msg::DropStatus last_status;
unsigned status_count=0;
acfly_interfaces::msg::ExplorationControlStatus control_status;
unsigned output_count=0;
extern "C" rcl_ret_t __wrap_rcl_publish(const rcl_publisher_t* publisher,const void* msg,rmw_publisher_allocation_t*) {
    const std::string topic=rcl_publisher_get_topic_name(publisher);
    if(topic=="/mavros/setpoint_raw/local") {last_output=*static_cast<const Target*>(msg);++output_count;}
    if(topic=="/exploration/control_status") control_status=*static_cast<const acfly_interfaces::msg::ExplorationControlStatus*>(msg);
    if(topic=="/exploration/context") last_context=*static_cast<const acfly_interfaces::msg::ExplorationContext*>(msg);
    if(topic=="/exploration/drop_status") {last_status=*static_cast<const acfly_interfaces::msg::DropStatus*>(msg);++status_count;}
    return RCL_RET_OK; // Never send test flight messages over DDS.
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
int main(int argc,char** argv) {
    rclcpp::init(argc,argv);int exit_code=0;
    try {
        auto node=std::make_shared<FlyMissionNode>();node->timer_->cancel();
        auto* clock=node->get_clock()->get_clock_handle();
        require(rcl_enable_ros_time_override(clock)==RCL_RET_OK,"test clock initialization");
        const auto time=[&](double t){require(rcl_set_ros_time_override(clock,static_cast<int64_t>(t*1e9))==RCL_RET_OK,"test clock advance");};
        auto odom=std::make_shared<nav_msgs::msg::Odometry>();odom->pose.pose.position.z=1.4;odom->pose.pose.orientation.w=1;
        const auto pose=[&]{odom->header.stamp=node->now();std::shared_ptr<void> msg=odom;node->drone_.odom_sub_->handle_message(msg,rclcpp::MessageInfo{});};
        time(100);pose();node->drone_.home_z_=.2;
        node->drone_.current_state_.armed=true;node->drone_.current_state_.mode="OFFBOARD";
        node->state_=MissionState::EXPLORATION;node->exploration(4,2);
        require(last_context.home_z==.2 && last_context.cruise_z==1.4 && last_context.height_tolerance==params::TOL_Z,
                "takeoff context did not preserve baseline and tolerance");
        auto command=std::make_shared<acfly_interfaces::msg::ExplorationCommand>();
        command->mission=node->exploration_goal_header_;command->header.frame_id="base_link";
        command->header.stamp.sec=100;command->header.stamp.nanosec=10000000;
        command->velocity.linear.x=.1;command->velocity.linear.y=-.05;command->altitude=.7;command->vertical_speed_limit=.4;
        const auto receive=[&]{std::shared_ptr<void> msg=command;node->exploration_control_sub_->handle_message(msg,rclcpp::MessageInfo{});};
        time(100.49);pose();receive();node->exploration(4,2);node->drone_.tick();
        require(node->unified_control_ && std::abs(last_output.velocity.x-.1)<1e-6 && last_output.velocity.z<-.39,
                "valid envelope lost atomic XY/descent forwarding");
        // Exercise the real dispatch order: tick() publishes the cached command
        // BEFORE exploration() consumes/validates the next envelope.
        time(100.52);pose();node->on_timer();
        require(!node->drone_.ext_valid_ && std::abs(last_output.velocity.z)<1e-6 && std::abs(last_output.velocity.x)<1e-6,
                "delayed command lifetime was extended from arrival time; old descent still forwarded");
        time(100.53);odom->pose.pose.position.x=.1;pose();node->on_timer();
        require(last_output.velocity.x<0,"repeated expiry moved hold anchor instead of correcting drift");
        odom->pose.pose.position.x=0;
        // A release request may arrive just inside its freshness window, then
        // expire before the mission timer consumes it. No serial action then.
        command->header.stamp.nanosec=40000000;command->release_index=1;command->target_id=10;
        receive();time(100.56);pose();node->on_timer();
        require(!node->drop_sender_.result(1,10) && !status_count,
                "release request accepted after its source timestamp expired");
        // A genuinely new request with a closed port reports failure, exactly
        // once in the sender; duplicated callbacks don't create another batch.
        time(101);pose();command->header.stamp=node->now();receive();node->on_timer();
        require(last_status.release_index==1 && last_status.target_id==10 && !last_status.success && status_count==1,
                "serial failure was not returned to the matching delivery");
        receive();node->on_timer();
        require(!node->drop_sender_.pending() && node->drop_sender_.result(1,10),"duplicate serial outcome lost transaction identity");
        time(101.02);pose();command->header.stamp=node->now();command->release_index=0;
        command->altitude=1.4;command->vertical_speed_limit=.6;command->velocity.linear.x=-.08;
        odom->pose.pose.position.z=.7;time(101.04);pose();receive();node->drone_.v_est_z_=0;node->on_timer();
        time(101.06);pose();node->on_timer();
        require(last_output.velocity.z>0 && last_output.velocity.x<0,"climb envelope lost simultaneous XY correction");
        auto legacy=std::make_shared<geometry_msgs::msg::TwistStamped>();legacy->header.stamp=node->now();legacy->twist.linear.x=.5;
        std::shared_ptr<void> legacy_msg=legacy;node->cmd_sub_->handle_message(legacy_msg,rclcpp::MessageInfo{});
        require(node->ext_v_fwd_==-.08,"legacy command overwrote unified climb correction");
        // A new callback must not lend its lifetime to the older command that
        // the controller still holds until the dispatcher runs this cycle.
        time(101.57);odom->pose.pose.position.z=.9;pose();node->drone_.v_est_z_=0;
        const double expected_hold_x=std::clamp(-params::KD_XY*node->drone_.v_est_x_,-.2,.2);
        command->header.stamp=node->now();command->altitude=1.0;command->velocity.linear.x=.06;receive();
        node->on_timer();
        require(std::abs(last_output.velocity.z)<1e-6 && std::abs(last_output.velocity.x-expected_hold_x)<1e-6,
                "new callback renewed the older cached climb before controller handoff");
        time(101.59);pose();node->on_timer();
        require(last_output.velocity.z>0 && std::abs(last_output.velocity.x-.06)<1e-6,
                "fresh command failed to resume after cached command expiry");
        // Real interface handover: final MAVROS mask, uninterrupted publisher,
        // fixed XY anchor, immediate height recovery, and one-way mode latch.
        time(102);odom->pose.pose.position.x=4;odom->pose.pose.position.y=2;odom->pose.pose.position.z=.02;pose();
        command->header.stamp=node->now();command->header.frame_id="camera_init";
        command->control_mode=acfly_interfaces::msg::ExplorationCommand::POSITION;
        command->position.x=5;command->position.y=2;command->position.z=.05;command->altitude=.05;
        command->yaw=M_PI/2;command->release_index=0;
        const auto valid_position=[&](const auto& c) {
            return valid_exploration_control(c,node->exploration_goal_header_,node->now().seconds(),
                params::EXPLORE_CMD_TIMEOUT_S,params::MAX_SPEED_Z);
        };
        require(valid_position(*command),"valid position envelope rejected");
        auto invalid=*command;invalid.header.frame_id="base_link";
        require(!valid_position(invalid),"position envelope accepted body coordinates");
        invalid=*command;invalid.position.z=-.03;
        require(!valid_position(invalid),"position envelope accepted conflicting target heights");
        invalid=*command;invalid.yaw=std::numeric_limits<double>::quiet_NaN();
        require(!valid_position(invalid),"position envelope accepted nonfinite yaw");
        invalid=*command;invalid.control_mode=2;
        require(!valid_position(invalid),"position envelope accepted unknown interface");
        invalid=*command;invalid.release_index=1;invalid.target_id=1;
        require(!valid_position(invalid),"corridor position envelope allowed package release");
        receive();
        const auto before=output_count;node->on_timer();
        require(output_count==before+1 && !control_status.ready,"handover skipped a publish or claimed readiness early");
        time(102.02);pose();node->on_timer();
        const auto position_mask=Target::IGNORE_VX|Target::IGNORE_VY|Target::IGNORE_VZ|
            Target::IGNORE_AFX|Target::IGNORE_AFY|Target::IGNORE_AFZ|Target::IGNORE_YAW_RATE;
        require(last_output.type_mask==position_mask && last_output.position.x==4 && last_output.position.y==2 &&
                last_output.position.z==.05 && last_output.velocity.x==0 && last_output.velocity.z==0 &&
                std::abs(last_output.yaw-M_PI/2)<1e-6 && control_status.position_active && !control_status.ready,
                "first position frame did not hold actual XY / immediately restore target Z with pure position mask");
        time(102.04);odom->pose.pose.position.x=4.1;odom->pose.pose.position.z=-.03;pose();node->on_timer();
        require(last_output.position.x==4 && last_output.position.z==.05,"handover anchor followed drift or captured dropped height");
        // Simulate elapsed local hold time without sleeps or live hardware.
        node->drone_.position_started_-=1;
        time(102.06);pose();node->on_timer();
        require(control_status.ready && last_output.position.x==5 && last_output.position.z==.05,
                "position interface did not release the fixed-XY handover");
        auto old=*command;old.control_mode=acfly_interfaces::msg::ExplorationCommand::VELOCITY;
        old.header.frame_id="base_link";old.header.stamp=node->now();old.velocity.linear.x=1;
        node->receive_exploration_control(old);node->drone_.set_velocity_body(1,1,1);
        require(node->drone_.external_position_active() &&
                node->exploration_control_.control_mode==acfly_interfaces::msg::ExplorationCommand::POSITION,
                "late velocity command stole position control");
        time(102.7);pose();node->on_timer();
        require(last_output.type_mask==position_mask && last_output.position.x==4.1 &&
                last_output.position.z==.05 && !control_status.ready,"expired position chased far target or dropped height");
        time(102.72);odom->pose.pose.position.x=4.2;pose();node->on_timer();
        require(last_output.position.x==4.1 && last_output.position.z==.05,"watchdog hold anchor followed drift");
        time(102.74);pose();command->header.stamp=node->now();receive();node->on_timer();
        time(102.76);pose();node->on_timer();
        require(!control_status.ready && last_output.position.x==4.1,"resume skipped position handover hold");
        time(103.2);node->on_timer();
        require(last_output.type_mask==position_mask && last_output.position.x==4.1 && last_output.position.z==.05,
                "pose loss switched back to velocity or latched wrong height");
        time(103.22);pose();node->drone_.land(true);node->drone_.tick();
        require(last_output.type_mask==position_mask,"H-to-land handover returned to custom velocity PD");
        node->drone_.stop();node->drone_.enter_exploration();node->drone_.set_velocity_body(.1,0,0);node->drone_.tick();
        require((last_output.type_mask&Target::IGNORE_PX)!=0,"new mission could not restore exploration velocity mode");
        std::cout<<"Position handover: pure MAVROS mask, immediate height target, fixed anchor, ACK timing, expiry, pose loss, late commands and reset passed\n";
        std::cout<<"Main handoff: context, real timer order, cached/source expiry, recovery, no expired release, serial failure, climb XY and legacy exclusion passed\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';exit_code=1;}
    rclcpp::shutdown();return exit_code;
}
