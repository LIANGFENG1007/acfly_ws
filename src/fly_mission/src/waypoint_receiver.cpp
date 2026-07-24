// ============================================================================
//  waypoint_receiver.cpp  ── 外部航点接收 实现（详见同名 .hpp）
// ============================================================================

#include "fly_mission/waypoint_receiver.hpp"

namespace fly_mission {

WaypointReceiver::WaypointReceiver(rclcpp::Node* node)
    : node_(node)
{
    // 独立 Reentrant 回调组：配合 main() 的 MultiThreadedExecutor，后台收不阻塞主循环。
    cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = cbg_;
    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/mission/waypoints", rclcpp::QoS(10),
        std::bind(&WaypointReceiver::on_waypoints, this, std::placeholders::_1), opt);

    RCLCPP_INFO(node_->get_logger(),
        "[航点接收] 已启动，等待 /mission/waypoints ([x1,y1,x2,y2,...])");
}

void WaypointReceiver::on_waypoints(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (received_) return;   // 只接收一次，锁定后忽略后续消息

    const auto& d = msg->data;
    if (d.size() < 2 || (d.size() % 2) != 0) {
        RCLCPP_WARN(node_->get_logger(),
            "[航点接收] 数据长度 %zu 非法(需成对 [x,y]，且≥1 对)，忽略本条", d.size());
        return;
    }

    std::vector<Waypoint> wps;
    wps.reserve(d.size() / 2);
    for (size_t i = 0; i + 1 < d.size(); i += 2)
        wps.push_back(Waypoint{ d[i], d[i + 1] });

    waypoints_ = std::move(wps);
    received_  = true;
    RCLCPP_INFO(node_->get_logger(),
        "[航点接收] 收到 %zu 个航点 → 锁定，开始执行", waypoints_.size());
}

bool WaypointReceiver::has_waypoints() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return received_;
}

std::vector<Waypoint> WaypointReceiver::take() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return waypoints_;
}

}  // namespace fly_mission
