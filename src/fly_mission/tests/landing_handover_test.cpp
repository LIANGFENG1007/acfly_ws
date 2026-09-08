#include "fly_mission/drone_controller.hpp"
#include "fly_mission/params.hpp"

#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>

// The controller normally clears the shared pose mailbox on construction.
// Block only that production mailbox in this test executable.
extern "C" int __real_open(const char* path, int flags, ...);
extern "C" int __wrap_open(const char* path, int flags, ...)
{
    if (std::strcmp(path, fly_mission::shm::POSE_SHM_PATH) == 0) {
        errno = EACCES;
        return -1;
    }
    if (flags & O_CREAT) {
        va_list args;
        va_start(args, flags);
        const mode_t mode = va_arg(args, mode_t);
        va_end(args);
        return __real_open(path, flags, mode);
    }
    return __real_open(path, flags);
}

namespace {

using Target = mavros_msgs::msg::PositionTarget;
using Steady = std::chrono::steady_clock;

void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

void near(double value, double expected, const std::string& message)
{
    require(std::isfinite(value) && std::abs(value - expected) < 1e-5, message);
}

void run_test()
{
    auto options = rclcpp::NodeOptions().arguments({
        "--ros-args",
        "-r", "/mavros/state:=/landing_test/state",
        "-r", "/aft_mapped_to_init:=/landing_test/odom",
        "-r", "/mavros/setpoint_raw/local:=/landing_test/setpoint",
        "-r", "/mavros/set_mode:=/landing_test/set_mode",
        "-r", "/mavros/cmd/arming:=/landing_test/arming",
        "-r", "/pole_detector/center:=/landing_test/pole",
        "-r", "/multi_pole_detector/center:=/landing_test/poles",
    });
    auto node = std::make_shared<rclcpp::Node>("landing_handover_test", options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spin_for = [&](double seconds, const std::function<void()>& step = {}) {
        const auto deadline = Steady::now() + std::chrono::duration<double>(seconds);
        do {
            if (step) step();
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } while (Steady::now() < deadline);
    };
    spin_for(0.25);
    require(node->get_node_names().size() == 1, "Selected ROS test domain is occupied");

    std::vector<Target> outputs;
    int mode_requests = 0;
    auto output_sub = node->create_subscription<Target>(
        "/landing_test/setpoint", 100,
        [&](Target::ConstSharedPtr msg) { outputs.push_back(*msg); });
    auto mode_service = node->create_service<mavros_msgs::srv::SetMode>(
        "/landing_test/set_mode",
        [&](mavros_msgs::srv::SetMode::Request::SharedPtr req,
            mavros_msgs::srv::SetMode::Response::SharedPtr res) {
            require(req->custom_mode == "AUTO.LAND", "Unexpected flight mode request");
            ++mode_requests;
            res->mode_sent = true;
        });
    auto pose_pub = node->create_publisher<nav_msgs::msg::Odometry>("/landing_test/odom", 10);
    auto state_pub = node->create_publisher<mavros_msgs::msg::State>("/landing_test/state", 10);

    fly_mission::DroneController drone(node.get());
    nav_msgs::msg::Odometry pose;
    pose.pose.pose.position.x = 8.25;
    pose.pose.pose.position.y = -4.25;
    pose.pose.pose.position.z = 0.8;
    pose.pose.pose.orientation.w = 1.0;
    mavros_msgs::msg::State state;
    state.connected = state.armed = true;
    state.mode = "OFFBOARD";
    auto sensors = [&] {
        pose.header.stamp = node->now();
        pose_pub->publish(pose);
        state_pub->publish(state);
    };
    spin_for(0.3, sensors);
    require(drone.has_pose() && drone.is_offboard(), "Test sensors did not reach controller");
    drone.capture_home();
    const double heading = -M_PI / 2.0;
    pose.pose.pose.orientation.z = std::sin(heading / 2.0);
    pose.pose.pose.orientation.w = std::cos(heading / 2.0);
    spin_for(0.1, sensors);

    // Leave stale mission targets behind, then finish the external velocity task.
    drone.target_pose_slam(0.0, 0.0, 3.0, 0.0);
    drone.enter_exploration();
    drone.set_velocity_body(0.0, 0.0, 0.0);
    drone.tick();
    spin_for(0.03);
    require(!outputs.empty(), "External velocity output missing");
    require(outputs.back().type_mask & Target::IGNORE_YAW, "Velocity task changed yaw contract");
    outputs.clear();
    drone.land(true);
    spin_for(0.10, [&] { sensors(); drone.land(true); drone.tick(); });
    require(mode_requests == 0, "AUTO.LAND requested before heading handover");
    require(!outputs.empty(), "Heading handover stopped the setpoint stream");
    for (const auto& msg : outputs) {
        require(!(msg.type_mask & Target::IGNORE_YAW), "Handover omitted absolute yaw");
        require(msg.type_mask & Target::IGNORE_YAW_RATE, "Handover retained yaw-rate control");
        near(msg.yaw, heading, "Handover turned toward an old mission heading");
        near(msg.velocity.x, 0.0, "Handover pursued stale X target");
        near(msg.velocity.y, 0.0, "Handover pursued stale Y target");
        near(msg.velocity.z, 0.0, "Handover pursued stale altitude");
    }

    // Repeated land calls must not reset the handover timer.
    spin_for(fly_mission::params::EXPLORE_LAND_HEADING_HANDOVER_SEC + 0.15,
             [&] { sensors(); drone.land(true); drone.tick(); });
    require(mode_requests == 1, "Repeated land calls prevented or repeated mode request");
    const auto count_before_wait = outputs.size();
    spin_for(0.04, [&] { sensors(); drone.tick(); });
    require(outputs.size() > count_before_wait, "Stream stopped before AUTO.LAND state confirmed");

    // A position/yaw perturbation must not relatch the landing heading or point.
    pose.pose.pose.position.x += 0.05;
    pose.pose.pose.orientation.z = std::sin((heading + 0.1) / 2.0);
    pose.pose.pose.orientation.w = std::cos((heading + 0.1) / 2.0);
    spin_for(0.05, sensors);
    drone.land(true);
    drone.tick();
    spin_for(0.03);
    near(outputs.back().yaw, heading, "Repeated land call relatched a drifting heading");
    require(outputs.back().velocity.x < 0.0, "Landing hold did not return toward captured position");

    state.mode = "AUTO.LAND";
    spin_for(0.04, sensors);
    const auto count_at_auto = outputs.size();
    spin_for(0.04, [&] { drone.tick(); });
    require(outputs.size() == count_at_auto, "OFFBOARD stream continued after AUTO.LAND took over");
    require(mode_requests == 1, "AUTO.LAND request repeated after takeover");

    // A second exploration using position control locks measured yaw as well.
    drone.stop();
    state.mode = "OFFBOARD";
    spin_for(0.05, sensors);
    drone.enter_exploration_position();
    drone.set_exploration_position_slam(8.25, -4.25, 0.8, 0.0);
    drone.land(true);
    drone.tick();
    spin_for(0.02);
    near(outputs.back().yaw, heading + 0.1, "Position exploration kept its old commanded yaw");

    // Unrelated missions retain their immediate AUTO.LAND behavior.
    drone.stop();
    drone.target_yaw_slam(0.0);
    const auto count_before_plain_land = outputs.size();
    drone.land();
    drone.tick();
    spin_for(0.03);
    require(mode_requests == 2, "Default landing unexpectedly added a handover delay");
    require(outputs.size() == count_before_plain_land, "Default landing gained new setpoints");
    std::cout << "PASS: fixed-yaw handover, retained position, idempotent calls, mode takeover, "
                 "velocity/position exploration and unchanged default landing\n";
}

}  // namespace

int main(int argc, char** argv)
{
    const auto domain = std::to_string(200 + ::getpid() % 25);
    ::setenv("ROS_DOMAIN_ID", domain.c_str(), 1);
    ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
    ::setenv("ROS_LOG_DIR", "/tmp/acfly-landing-handover-test-logs", 1);
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        run_test();
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
