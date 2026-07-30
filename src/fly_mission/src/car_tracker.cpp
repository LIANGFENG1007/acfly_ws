// ============================================================================
//  car_tracker.cpp  ── 小车雷达追踪 实现（详见同名 .hpp）
// ============================================================================

#include "fly_mission/car_tracker.hpp"
#include "fly_mission/params.hpp"

#include <cmath>

namespace fly_mission {

CarTracker::CarTracker(rclcpp::Node* node)
    : node_(node)
{
    // 独立 Reentrant 组：后台收小车位姿，不阻塞 50Hz 状态机 timer(与 ring_driller 同套路)。
    cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = cbg_;
    // QoS 与订阅飞机自己的 odom 保持一致(SensorDataQoS)：Point-LIO 发的是 best-effort，
    //   用默认 reliable 订阅会因 QoS 不兼容【一条都收不到】。
    sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init2", rclcpp::SensorDataQoS(),
        std::bind(&CarTracker::on_car_odom, this, std::placeholders::_1), opt);

    RCLCPP_INFO(node_->get_logger(),
        "[追踪] CarTracker 已启动，等 /aft_mapped_to_init2；"
        "小车雷达原点在飞机系(%.2f, %.2f)，yaw 偏置 %.1f°，数据超时 %.1fs，%s",
        params::CAR_ORIGIN_X, params::CAR_ORIGIN_Y, params::CAR_YAW_OFFSET_DEG,
        params::CAR_DATA_TIMEOUT_S,
        params::CAR_TRACK_Z ? "跟随小车 z" : "锁定进入时高度");
}

void CarTracker::begin(double z_lock)
{
    std::lock_guard<std::mutex> lk(mtx_);
    z_lock_       = z_lock;
    z_lock_valid_ = true;
    RCLCPP_INFO(node_->get_logger(),
        "[追踪] 开始追踪小车，锁定高度 %.2fm(SLAM z)%s",
        z_lock, have_ ? "" : "；★还没收到过小车位姿，将悬停等★");
}

// 后台线程：把小车 B 系坐标换算到飞机 A 系存起来。
//   p_A = p_B + CAR_ORIGIN；yaw_A = yaw_B + CAR_YAW_OFFSET_DEG。
void CarTracker::on_car_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const double bx = msg->pose.pose.position.x;
    const double by = msg->pose.pose.position.y;
    const double bz = msg->pose.pose.position.z;

    // yaw：与 drone_controller 的 current_yaw() 同款公式(只算 yaw，不建旋转矩阵)
    const auto& q = msg->pose.pose.orientation;
    const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    double yaw_b = 0.0;
    if (n >= 1e-9) {
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw_b = std::atan2(siny, cosy);
    }

    // 脏数据防御：NaN/Inf 直接丢弃，绝不让它进目标点(会被 publish_setpoint 兜底清零，
    //   但那样飞机会突然停一拍；不如这条当没收到，继续用上一条+超时逻辑)。
    if (!std::isfinite(bx) || !std::isfinite(by) || !std::isfinite(bz)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[追踪] /aft_mapped_to_init2 位置含非有限数，本条忽略");
        return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    car_x_       = bx + params::CAR_ORIGIN_X;    // ★B 系 → A 系：只差平移★
    car_y_       = by + params::CAR_ORIGIN_Y;
    car_z_       = bz;
    car_yaw_deg_ = yaw_b * 180.0 / M_PI + params::CAR_YAW_OFFSET_DEG;
    stamp_       = node_->now();
    stamp_valid_ = true;
    if (!have_) {
        have_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "[追踪] 首次收到小车位姿：小车系(%.2f,%.2f) → 飞机系(%.2f,%.2f) yaw=%.1f°",
            bx, by, car_x_, car_y_, car_yaw_deg_);
    }
}

bool CarTracker::latest(double& out_x, double& out_y, double& out_z, double& out_yaw_deg) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_ || !stamp_valid_) return false;          // 从未收到过

    const double age = (node_->now() - stamp_).seconds();
    if (age > params::CAR_DATA_TIMEOUT_S) {             // 话题真的停了(不是"追不上")
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[追踪] 小车位姿已 %.2fs 无更新(>%.2fs)，悬停等数据恢复",
            age, params::CAR_DATA_TIMEOUT_S);
        return false;
    }

    out_x       = car_x_;
    out_y       = car_y_;
    out_z       = params::CAR_TRACK_Z ? car_z_ : (z_lock_valid_ ? z_lock_ : car_z_);
    out_yaw_deg = car_yaw_deg_;
    return true;
}

bool CarTracker::has_data() const
{
    std::lock_guard<std::mutex> lk(mtx_);
    return have_;
}

}  // namespace fly_mission
