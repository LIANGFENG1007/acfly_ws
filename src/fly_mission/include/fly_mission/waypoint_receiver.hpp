#pragma once

// ============================================================================
//  waypoint_receiver.hpp  ── 接收外部发来的一整串航点
//
//  角色：主控起飞后在 WAIT_WAYPOINTS 状态悬停等待时，由本类后台监听
//        /mission/waypoints，收到一整串坐标即锁定，供主控取走执行。
//        本类【不订阅位姿、不发指令】——只负责"收没收到 / 收到了哪些点"。
//
//  话题：/mission/waypoints (std_msgs/Float64MultiArray)
//        data 按 [x1,y1, x2,y2, x3,y3, ...] 排列，一条消息=全部航点，只接收一次。
//        发送示例(一条命令发全部)：
//          ros2 topic pub --once /mission/waypoints std_msgs/msg/Float64MultiArray "{data: [3.0, 7.0, 4.0, 6.0, 0.0, 0.0]}"
//          → 依次飞 (3,7)→(4,6)→(0,0)。坐标为 SLAM 系(原点=起飞点)，高度不在此(固定飞行高度)。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <vector>

#include "fly_mission/waypoints.hpp"

namespace fly_mission {

class WaypointReceiver
{
public:
    explicit WaypointReceiver(rclcpp::Node* node);

    // 是否已收到过一整串航点(收到即锁定，之后的消息忽略)。
    bool has_waypoints() const;

    // 取走收到的航点列表(线程安全拷贝)。配合 has_waypoints() 用。
    std::vector<Waypoint> take() const;

private:
    void on_waypoints(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    rclcpp::Node* node_;
    rclcpp::CallbackGroup::SharedPtr cbg_;   // 独立 Reentrant 组：后台收，不阻塞 20Hz 主循环
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

    mutable std::mutex    mtx_;
    bool                  received_ = false;   // 已锁定(收到过一次)——之后的消息丢弃
    std::vector<Waypoint> waypoints_;
};

}  // namespace fly_mission
