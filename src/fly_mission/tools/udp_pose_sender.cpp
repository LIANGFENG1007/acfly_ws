// ============================================================================
//  udp_pose_sender.cpp  ── 【在雷达机上跑】订 ROS 位姿话题 → UDP 单播发给飞机
//
//  为什么要它：见 udp_pose_link.hpp 顶部说明(DDS 跨机组播发现会打爆 WiFi、
//  拖垮飞机上的 mavros 串口实时性)。本程序把位姿单独用 UDP 点对点送过去，
//  两台机器的 ROS 从此【完全隔离】(建议设不同 ROS_DOMAIN_ID)。
//
//  ★这个文件不参与 fly_mission 的编译★(放在 tools/，不在 CMakeLists 里)。
//  它要拷到【雷达机】上单独编译，因为雷达机不一定有本工程。
//
//  ── 在雷达机上编译(不需要 colcon，一条 g++ 即可) ──
//    把这两个文件拷到雷达机同一目录：
//        udp_pose_sender.cpp
//        udp_pose_link.hpp        (来自 fly_mission/include/fly_mission/)
//    然后：
//        源 /opt/ros/humble/setup.bash 后执行
//        g++ -std=c++17 -O2 udp_pose_sender.cpp -o udp_pose_sender \
//            -I. -I/opt/ros/humble/include \
//            $(pkg-config --cflags --libs 2>/dev/null || true) \
//            -L/opt/ros/humble/lib \
//            -lrclcpp -lrcl -lrcutils -lrmw -lnav_msgs__rosidl_typesupport_cpp \
//            -lgeometry_msgs__rosidl_typesupport_cpp \
//            -lstd_msgs__rosidl_typesupport_cpp -lrosidl_runtime_c
//    ★上面那串库名在不同发行版可能对不上★。更省事的办法是在雷达机上建个最小
//    ROS 包(package.xml 依赖 rclcpp + nav_msgs)，把本文件当唯一源码编——
//    见文件末尾的"最小包做法"。
//
//  ── 运行(雷达机) ──
//    ./udp_pose_sender --ros-args \
//        -p dest_ip:=<飞机的IP> -p port:=9870 \
//        -p in_topic:=/aft_mapped_to_init2 -p rate_hz:=30.0
//
//  ── 飞机端 ──
//    不用起任何节点，主控里直接用 UdpPoseReceiver 读(见 udp_pose_link.hpp 用法)。
//    先用 tools/udp_pose_monitor.cpp 验证收得到再接进主控。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "udp_pose_link.hpp"        // 与本文件同目录(拷过去时一起拷)

#include <cmath>
#include <memory>
#include <string>

using fly_mission::udp_pose::UdpPoseSender;

class UdpPoseSenderNode : public rclcpp::Node
{
public:
    UdpPoseSenderNode() : Node("udp_pose_sender")
    {
        dest_ip_  = declare_parameter<std::string>("dest_ip",  "192.168.1.100");
        port_     = static_cast<int>(declare_parameter<int>("port", 9870));
        in_topic_ = declare_parameter<std::string>("in_topic", "/aft_mapped_to_init2");
        // 发送限频：位姿通常 20~50Hz 就够，别把上游 200Hz 原样打出去浪费带宽
        rate_hz_  = declare_parameter<double>("rate_hz", 30.0);

        tx_ = std::make_unique<UdpPoseSender>(dest_ip_.c_str(), port_);
        if (!tx_->ok()) {
            RCLCPP_ERROR(get_logger(),
                "UDP socket 打开失败(dest_ip=%s 是合法 IPv4 吗?)，本节点不会发数据",
                dest_ip_.c_str());
        }

        // SensorDataQoS：位姿是高频传感器流，用 BEST_EFFORT，别让本机 DDS 背可靠重传
        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            in_topic_, rclcpp::SensorDataQoS(),
            std::bind(&UdpPoseSenderNode::on_odom, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "udp_pose_sender 已启动: %s → UDP %s:%d (限频 %.0fHz)",
            in_topic_.c_str(), dest_ip_.c_str(), port_, rate_hz_);
    }

private:
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 限频：距上次发送不足周期就跳过(用 steady_clock，不受系统时间跳变影响)
        const double now = fly_mission::udp_pose::UdpPoseReceiver::now_mono();
        if (rate_hz_ > 0.0 && (now - last_send_) < (1.0 / rate_hz_)) return;
        last_send_ = now;

        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;

        // 四元数 → yaw(绕Z)。与主控 fly_mission 取 yaw 的公式一致，两端口径统一。
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yaw  = std::atan2(siny, cosy);

        // 防把 NaN 发出去(下游拿它算控制会更糟)
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.z) || !std::isfinite(yaw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "位姿含非有限数，本帧不发");
            return;
        }

        if (!tx_->send(p.x, p.y, p.z, yaw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "UDP 发送失败(网线/WiFi 断了? 目标不可达?)");
            return;
        }

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "已发 seq=%u  pose=(%.2f, %.2f, %.2f) yaw=%.1f°",
            tx_->seq(), p.x, p.y, p.z, yaw * 180.0 / M_PI);
    }

    std::string in_topic_, dest_ip_;
    int    port_ = 9870;
    double rate_hz_ = 30.0;
    double last_send_ = 0.0;
    std::unique_ptr<UdpPoseSender> tx_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpPoseSenderNode>());
    rclcpp::shutdown();
    return 0;
}

// ============================================================================
//  ── 最小包做法(推荐，比手写 g++ 链接串省事) ──
//  在雷达机上：
//    mkdir -p ~/udp_ws/src/udp_pose/src ~/udp_ws/src/udp_pose/include
//    拷 udp_pose_sender.cpp → ~/udp_ws/src/udp_pose/src/
//    拷 udp_pose_link.hpp   → ~/udp_ws/src/udp_pose/src/   (与 cpp 同目录即可)
//
//  package.xml:
//    <?xml version="1.0"?>
//    <package format="3">
//      <name>udp_pose</name>
//      <version>0.0.0</version>
//      <description>UDP pose sender</description>
//      <maintainer email="a@b.c">you</maintainer>
//      <license>TODO</license>
//      <buildtool_depend>ament_cmake</buildtool_depend>
//      <depend>rclcpp</depend>
//      <depend>nav_msgs</depend>
//      <export><build_type>ament_cmake</build_type></export>
//    </package>
//
//  CMakeLists.txt:
//    cmake_minimum_required(VERSION 3.8)
//    project(udp_pose)
//    find_package(ament_cmake REQUIRED)
//    find_package(rclcpp REQUIRED)
//    find_package(nav_msgs REQUIRED)
//    add_executable(udp_pose_sender src/udp_pose_sender.cpp)
//    target_include_directories(udp_pose_sender PRIVATE src)
//    ament_target_dependencies(udp_pose_sender rclcpp nav_msgs)
//    install(TARGETS udp_pose_sender DESTINATION lib/${PROJECT_NAME})
//    ament_package()
//
//  然后：
//    cd ~/udp_ws && colcon build && source install/setup.bash
//    ros2 run udp_pose udp_pose_sender --ros-args \
//      -p dest_ip:=<飞机IP> -p in_topic:=/aft_mapped_to_init2
// ============================================================================
