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
    if (params::CAR_USE_UDP) {
        // ★UDP 模式★：不订阅任何 ROS 话题(避免跨机 DDS 发现打爆 WiFi、拖垮 mavros)。
        //   构造即 bind 端口；bind 失败不抛异常，只告警——此时 latest() 恒 false，
        //   飞机会悬停等，不会拿错数据飞。
        udp_rx_ = std::make_unique<udp_pose::UdpPoseReceiver>(params::CAR_UDP_PORT);
        if (!udp_rx_->ok()) {
            RCLCPP_ERROR(node_->get_logger(),
                "[追踪] ★UDP 端口 %d bind 失败★(被占用?)。将收不到小车位姿→飞机只会悬停。"
                "换端口需同时改 params::CAR_UDP_PORT 与小车机 udp_pose_sender 的 -p port:=",
                params::CAR_UDP_PORT);
        }
        RCLCPP_INFO(node_->get_logger(),
            "[追踪] CarTracker 已启动【UDP 模式】监听端口 %d(不订阅 ROS 话题)；"
            "小车雷达原点在飞机系(%.2f, %.2f)，yaw 偏置 %.1f°，数据超时 %.1fs，%s",
            params::CAR_UDP_PORT,
            params::CAR_ORIGIN_X, params::CAR_ORIGIN_Y, params::CAR_YAW_OFFSET_DEG,
            params::CAR_DATA_TIMEOUT_S,
            params::CAR_TRACK_Z ? "跟随小车 z" : "锁定进入时高度");
        RCLCPP_INFO(node_->get_logger(),
            "[追踪] 小车机需运行: ros2 run udp_pose udp_pose_sender --ros-args "
            "-p dest_ip:=<本机IP> -p port:=%d -p in_topic:=/aft_mapped_to_init2",
            params::CAR_UDP_PORT);
        return;
    }

    // ---- ROS 话题模式(旧行为) ----
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
        "[追踪] CarTracker 已启动【ROS 话题模式】等 /aft_mapped_to_init2；"
        "小车雷达原点在飞机系(%.2f, %.2f)，yaw 偏置 %.1f°，数据超时 %.1fs，%s",
        params::CAR_ORIGIN_X, params::CAR_ORIGIN_Y, params::CAR_YAW_OFFSET_DEG,
        params::CAR_DATA_TIMEOUT_S,
        params::CAR_TRACK_Z ? "跟随小车 z" : "锁定进入时高度");
}

// ★UDP 模式每拍必调★：UDP 没有回调线程，必须主动收包。ROS 模式下是空操作。
void CarTracker::poll()
{
    if (!udp_rx_) return;                      // ROS 模式：什么都不做
    if (!udp_rx_->poll()) return;              // 本拍没有新包
    const auto& p = udp_rx_->latest();
    // UDP 包里 yaw 已是弧度(发送端用与 on_car_odom 相同的公式从四元数算好的)
    ingest(p.x, p.y, p.z, p.yaw, "UDP");
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

    ingest(bx, by, bz, yaw_b, "ROS");
}

// 把一帧小车位姿换算并存起来。★UDP 与 ROS 两条来源共用这一份★——保证换算口径、
//   脏数据防御、超时时间戳的处理完全一致，换来源不会有行为差异。
void CarTracker::ingest(double bx, double by, double bz, double yaw_b_rad, const char* src)
{
    // 脏数据防御：NaN/Inf 直接丢弃，绝不让它进目标点(会被 publish_setpoint 兜底清零，
    //   但那样飞机会突然停一拍；不如这条当没收到，继续用上一条+超时逻辑)。
    if (!std::isfinite(bx) || !std::isfinite(by) || !std::isfinite(bz) ||
        !std::isfinite(yaw_b_rad)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[追踪] 小车位姿(%s)含非有限数，本条忽略", src);
        return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    raw_x_       = bx;                           // 原始 B 系(遥测上报用，未加标定)
    raw_y_       = by;
    car_x_       = bx + params::CAR_ORIGIN_X;    // ★B 系 → A 系：只差平移★
    car_y_       = by + params::CAR_ORIGIN_Y;
    car_z_       = bz;
    car_yaw_deg_ = yaw_b_rad * 180.0 / M_PI + params::CAR_YAW_OFFSET_DEG;
    // ★超时判定用本机 ROS 时钟★：UDP 包里的 stamp 是【小车机】的 monotonic，与本机
    //   不同源(两台机器各自从开机计时)，绝不能拿来比。这里记"本机收到的时刻"，
    //   与 ROS 模式完全同一口径。
    stamp_       = node_->now();
    stamp_valid_ = true;
    if (!have_) {
        have_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "[追踪] 首次收到小车位姿(%s)：小车系(%.2f,%.2f) → 飞机系(%.2f,%.2f) yaw=%.1f°",
            src, bx, by, car_x_, car_y_, car_yaw_deg_);
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

// 原始 B 系坐标(遥测用)。超时判定与 latest() 完全一致，但★不打告警★——
//   遥测是旁路功能，不该因为它而刷日志(latest() 里那条告警由飞行逻辑负责打)。
bool CarTracker::latest_raw(double& out_bx, double& out_by) const
{
    std::lock_guard<std::mutex> lk(mtx_);
    if (!have_ || !stamp_valid_) return false;      // 从没收到过 → 出参不动(调用方给的初值)

    // ★数据过期也照样输出最后已知位置★，只是返回 false 表示"不新鲜"。
    //   为什么：这是给遥测用的——过期时输出 0 会让监控端看到小车"跳回原点"，
    //   而输出最后位置能看出"小车最后在哪、从哪断的"，排查价值大得多。
    //   飞行控制不能用不新鲜的数据，但飞行控制走的是 latest()(那个会拦住)，不是本函数。
    out_bx = raw_x_;
    out_by = raw_y_;
    return (node_->now() - stamp_).seconds() <= params::CAR_DATA_TIMEOUT_S;
}

}  // namespace fly_mission
