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
//  输入：小车位姿，坐标在【小车自己那套 SLAM 的系】(B 系)里。两种来源，由
//        params::CAR_USE_UDP 选择——★对外接口(begin/latest/has_data)完全一样★，
//        坐标换算与超时逻辑两条来源共用，所以主控状态机不用改一行：
//          · true (★当前★) = ★UDP 单播★，收 tools/udp_pose_sender 发来的 48B 包
//              (小车机上跑，见 tools/README_udp_pose.md)。不订阅任何 ROS 话题。
//              ★为什么★：两台机器同网时 DDS 组播发现会打爆 WiFi、拖垮 mavros 串口
//              实时性("小车机一开 mavros 就不传数据")。详见 params::CAR_USE_UDP 注释。
//          · false = ROS 话题 /aft_mapped_to_init2 (nav_msgs/Odometry)，旧行为。
//
//        ★UDP 与 ROS 的一个差别★：UDP 包里只有 x/y/z/yaw(48B 定长)，yaw 是发送端
//        已经从四元数算好的(公式与本文件 on_car_odom 里一致)，所以两条路口径相同。
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

#include "fly_mission/udp_pose_link.hpp"

#include <memory>
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

    // 取小车的【原始 B 系坐标】(★未加 CAR_ORIGIN 平移标定★)，给遥测上报用。
    //   返回值 = 数据是否【新鲜】(同 latest() 的超时判定)；
    //   ★注意：数据过期时仍会写出【最后已知位置】并返回 false★——遥测要的是
    //   "小车最后在哪"，输出 0 会让监控端看到小车跳回原点。只有从未收到过数据时
    //   才不动出参。★飞行控制不要用本函数★(用 latest()，它会拦住不新鲜的数据)。
    //   ★为什么要原始值★：标定量填错时换算后的坐标是错的，但原始值仍然正确，
    //   对排查"是标定错了还是小车雷达本身有问题"更有用。
    bool latest_raw(double& out_bx, double& out_by) const;

    // ★UDP 模式必须每拍调一次★(ROS 模式下是空操作，调了也没坏处)：
    //   UDP 没有回调线程，必须主动收包。放在主控 50Hz 定时器开头调即可。
    //   不调的话 UDP 模式永远收不到数据 → latest() 恒 false → 飞机一直悬停等。
    void poll();

private:
    void on_car_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    // 把一帧小车位姿(B 系 x/y/z + yaw 弧度)换算存入，两条来源共用这一份逻辑
    void ingest(double bx, double by, double bz, double yaw_b_rad, const char* src);

    rclcpp::Node* node_;
    rclcpp::CallbackGroup::SharedPtr cbg_;   // 独立 Reentrant 组：后台收，不阻塞 50Hz 主循环
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;   // 仅 ROS 模式创建

    // UDP 模式的接收器(仅 UDP 模式创建)。★非线程安全★，只在 poll() 里用，
    //   而 poll() 由主控单线程调 → 安全。
    std::unique_ptr<udp_pose::UdpPoseReceiver> udp_rx_;

    mutable std::mutex mtx_;
    bool         have_ = false;      // 收到过数据
    double       car_x_ = 0.0;       // 最新小车位置(★已换算到飞机 A 系★)
    double       car_y_ = 0.0;
    double       raw_x_ = 0.0;       // 同一帧的★原始 B 系★坐标(未加标定平移，遥测用)
    double       raw_y_ = 0.0;
    double       car_z_ = 0.0;       // 小车雷达 z(B 系原样，仅 CAR_TRACK_Z=true 时用)
    double       car_yaw_deg_ = 0.0; // 最新小车偏航(已加 CAR_YAW_OFFSET_DEG，度)
    rclcpp::Time stamp_;             // 最近一条的接收时刻(超时判定)
    bool         stamp_valid_ = false;

    double       z_lock_ = 0.0;      // 进入追踪时锁定的高度(CAR_TRACK_Z=false 时的目标 z)
    bool         z_lock_valid_ = false;
};

}  // namespace fly_mission
