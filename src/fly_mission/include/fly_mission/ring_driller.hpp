#pragma once

// ============================================================================
//  ring_driller.hpp  ── 钻圈：悬停采集环位姿 → 边飞边转升高到环前对准
//
//  角色：主控 fly_mission 的一个【外挂模块】(与 FindFigure / LineFollower 同套路)。
//        进入 DRILL_RING 状态后，本类订阅 ring_detector 发的 /ring_detector/center，
//        原地悬停攒够 N 帧，取平均算出【环心 (x,y,z) + 环面绝对朝向 yaw】，再算出
//        "环前 standoff 米、正对环面"的停靠位姿交给主控。本类【不订阅位姿、不发
//        setpoint】——飞行动作由主控用现成的 target_pose_slam(MOVE_POSE, PD 同拍
//        控制平移+升降+转向)执行，本类只负责"攒够没 / 该飞到哪 / 飞完没"。
//
//  输入：/ring_detector/center (std_msgs/Float64MultiArray)
//        data = [x, y, z, yaw_error_deg, ring_yaw_deg]
//          x,y,z        环心 (SLAM/camera_init 系, m)
//          yaw_error_deg 飞机还需转多少度(本类不用；对准由绝对 yaw 目标保证)
//          ring_yaw_deg  环面绝对朝向(SLAM 系, 度)——已由检测端消歧成"飞机穿过去"那一侧
//        检测端每条消息本身已是多帧点云累积+圆拟合的结果；本类再对 N 条消息取平均，
//        进一步抑制抖动。ring_yaw 用 (cos,sin) 平均后 atan2，避免 ±180° 环绕出错。
//
//  动作序列(交给 target_pose_slam；两点高度均 = 环心z + DRILL_Z_OFFSET_M、yaw=ring_yaw 正对环面)：
//        ① 环前 D 米对准：front = 环心 - D*(cos,sin)(ring_yaw)   ← 飞机所在这侧
//        ② 悬停 → ③ 穿过圈飞到环后 D 米：back = 环心 + D*(cos,sin)(ring_yaw)  ← 穿过去那侧
//        ④ 悬停 → (由状态机)降落。①③ 用 MOVE_POSE 边飞边转升高，机头全程正对环面。
//  ★高度抬升★：SLAM 位姿点(雷达)在机身中心上方，直接对准环心机身会偏低蹭下沿；两点目标 z
//        统一抬高 DRILL_Z_OFFSET_M(≈雷达到机身中心垂距)，让【机身中心】穿过环心。
//
//  参数：默认值集中在 fly_mission/params.hpp 的 ★钻圈★ 段(DRILL_*)。改默认改那里。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <vector>

namespace fly_mission {

class RingDriller
{
public:
    // 一拍执行结果：主控据此决定这拍对飞机下什么指令。
    enum class Step {
        COLLECTING,    // 悬停采集帧中 → 主控原地悬停(wait_time)等攒够
        APPROACHING,   // 飞向当前段目标(环前对准→穿到环后) → 主控 target_pose_slam(out_*)
        HOVERING,      // 已到位 → 主控 wait_time(DRILL_HOVER_SEC) 悬停(环前/环后各一次)
        DONE,          // 全程(环前对准+穿圈到环后)完成 → 主控转下一状态(降落)
        FAILED         // 采集超时仍不足最小帧数 → 主控转下一状态(降落)
    };

    explicit RingDriller(rclcpp::Node* node);

    // 进入 DRILL_RING 时调一次：清采集缓冲、复位阶段(下一拍起从头采集)。
    void begin();

    // DRILL_RING 每拍调用。
    //   reached = 主控 is_reached()：APPROACHING 期判 MOVE_POSE 到位；HOVERING 期判 wait_time 到点。
    //             (COLLECTING 期本类不看 reached，靠帧数/超时自行推进。)
    //   出参 out_x/out_y/out_z (SLAM系,m) + out_yaw_deg(度) = 当前段目标位姿(环前/环后)，仅 APPROACHING 有效。
    Step tick(bool reached,
              double& out_x, double& out_y, double& out_z, double& out_yaw_deg);

private:
    // 采集回调(后台 Reentrant 线程)：仅采集态记录一条样本(取 x,y,z + ring_yaw 的 cos/sin)。
    void on_center(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 用当前累计样本算环心 → 算环前/环后两个停靠位姿，写入 front_*/back_*。返回 false = 样本不足。
    bool compute_target();

    // ---- ROS ----
    rclcpp::Node* node_;
    rclcpp::CallbackGroup::SharedPtr cbg_;   // 独立 Reentrant 组：后台攒帧，不阻塞 20Hz 主循环
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;

    // ---- 采集缓冲(后台回调写 / 主线程读，mtx_ 保护) ----
    mutable std::mutex  mtx_;
    bool                collecting_ = false;   // 仅采集态记录样本(begin() 置真，攒够/超时置假)
    std::vector<double> sx_, sy_, sz_;         // 环心 x/y/z 样本
    std::vector<double> scos_, ssin_;          // ring_yaw 的 cos/sin 样本(圆周量分量平均)

    // ---- 执行态(主线程) ----
    enum class Phase { COLLECT, APPROACH_FRONT, HOVER_FRONT, THROUGH, HOVER_BACK, DONE, FAIL };
    Phase        phase_ = Phase::COLLECT;
    // 两个停靠位姿(高度均=环心z+DRILL_Z_OFFSET_M、yaw=环面朝向)：环前 standoff 米、环后 standoff 米
    double       front_x_ = 0.0, front_y_ = 0.0, front_z_ = 0.0, front_yaw_deg_ = 0.0;
    double       back_x_  = 0.0, back_y_  = 0.0, back_z_  = 0.0, back_yaw_deg_  = 0.0;
    rclcpp::Time collect_start_;
    bool         collect_time_valid_ = false;
};

}  // namespace fly_mission
