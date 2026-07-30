#pragma once

// ============================================================================
//  car_tracker.hpp  ── 追踪地面小车雷达：把小车 SLAM 坐标换算到飞机 SLAM 系
//
//  角色：主控 fly_mission 的一个【外挂模块】(与 FindFigure / RingDriller 同套路)。
//        订阅小车雷达的 /aft_mapped_to_init2，把它报的坐标换算成飞机 SLAM 系下的
//        目标点交给主控。本类【不订阅飞机位姿、不发 setpoint】——飞行动作由主控用
//        现成的 target_pose_slam(MOVE_POSE，PD 同拍控制平移+转向)执行，本类只负责
//        "小车现在在飞机坐标系的哪里 / 数据还新不新"。
//
//  输入：/aft_mapped_to_init2 (nav_msgs/Odometry)，格式与飞机的 /aft_mapped_to_init
//        完全一样，但坐标在【小车自己那套 SLAM 的系】(B 系)里。
//
//  ★坐标换算(本模块的全部数学)★：
//        两台雷达摆放朝向一致 ⇒ A/B 两系坐标轴平行、只差原点平移 ⇒
//            p_A = p_B + (CAR_ORIGIN_X, CAR_ORIGIN_Y)
//            yaw_A = yaw_B + CAR_YAW_OFFSET_DEG
//        CAR_ORIGIN_* = 小车雷达初始化位置在飞机 A 系下的坐标(params.hpp 里标定)。
//        ★这是开环的★：标定量错多少、飞机就稳定偏多少，没有任何闭环能纠正。
//
//  ★没有新数据时★：latest() 返回 false，主控保持上一个目标点悬停等(不拿旧坐标硬飞)。
//        注意区分两件事——"小车位置一直是知道的"(话题在发就有数据)，本超时只针对
//        【话题真的停了】(小车 SLAM 挂掉/丢包)。小车跑得比飞机快不算超时，PD 会一直追。
//
//  参数：默认值集中在 params.hpp 的 ★小车雷达追踪★ 段(CAR_*)。改默认改那里。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mutex>

namespace fly_mission {

class CarTracker
{
public:
    explicit CarTracker(rclcpp::Node* node);

    // 进入追踪状态时调一次：锁定当前高度为追踪高度(CAR_TRACK_Z=false 时全程用它)。
    //   z_lock = 调用瞬间飞机的 SLAM z(由主控传入，本类不订阅飞机位姿)。
    void begin(double z_lock);

    // 取本拍应飞向的目标位姿(已换算到【飞机 SLAM 系】)。
    //   返回 true  = 数据新鲜，出参有效，主控 target_pose_slam(out_*) 追过去；
    //   返回 false = 从未收到过 / 超过 CAR_DATA_TIMEOUT_S 没有新数据 →
    //                主控应保持上一个目标悬停等(本类不给目标，避免拿旧坐标硬飞)。
    bool latest(double& out_x, double& out_y, double& out_z, double& out_yaw_deg) const;

    // 是否收到过至少一条小车位姿(进入追踪前可用它判"小车雷达在不在线")。
    bool has_data() const;

private:
    void on_car_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Node* node_;
    rclcpp::CallbackGroup::SharedPtr cbg_;   // 独立 Reentrant 组：后台收，不阻塞 50Hz 主循环
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

    mutable std::mutex mtx_;
    bool         have_ = false;      // 收到过数据
    double       car_x_ = 0.0;       // 最新小车位置(★已换算到飞机 A 系★)
    double       car_y_ = 0.0;
    double       car_z_ = 0.0;       // 小车雷达 z(B 系原样，仅 CAR_TRACK_Z=true 时用)
    double       car_yaw_deg_ = 0.0; // 最新小车偏航(已加 CAR_YAW_OFFSET_DEG，度)
    rclcpp::Time stamp_;             // 最近一条的接收时刻(超时判定)
    bool         stamp_valid_ = false;

    double       z_lock_ = 0.0;      // 进入追踪时锁定的高度(CAR_TRACK_Z=false 时的目标 z)
    bool         z_lock_valid_ = false;
};

}  // namespace fly_mission
