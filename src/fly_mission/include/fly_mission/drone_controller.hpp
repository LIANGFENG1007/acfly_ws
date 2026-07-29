#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "fly_mission/avoidance.hpp"
#include "fly_mission/shm_mailbox.hpp"

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace fly_mission {

// 当前正在执行的动作类型，is_reached() 根据它选用哪种到位判定逻辑
enum class ActionMode {
    IDLE,        // 刚启动
    HOLD,        // 软位保持（含 wait_time 和未派活的悬停）
    TAKEOFF,     // 起飞到目标高度
    MOVE_XY,     // 平移到目标 (x, y)
    MOVE_Z,      // 改变高度
    MOVE_POSE,   // 一次到位：同时控制 x/y/z/yaw(钻圈"边转边走升高"用)
    TURN_YAW,    // 转头到目标 yaw
    CIRCLE,      // 绕前方圆心做圆周运动（机头始终朝圆心）
    EXTERNAL_VEL,// 外部算法直接给机体系速度（探索）
    LAND         // 请求 AUTO.LAND等触底
};

class DroneController
{
public:
    explicit DroneController(rclcpp::Node* node);

    // ====================================================================
    //   运动 API
    // ====================================================================
    void takeoff(double altitude_relative_home);
    // ★退出起飞段的位置环★：起飞后悬停稳定、要切去走航点/找图等之前调一次。
    //   之后所有动作恢复速度环 PD(与改动前逐位一致)。幂等，多调无副作用。
    //   params::TAKEOFF_POSITION_MODE=false 时本函数是空操作(全程本就是速度环)。
    void exit_takeoff_position_mode();
    void land();
    void target_xy_slam(double x_slam, double y_slam);
    void target_xy_body(double dx_body, double dy_body);
    void target_z_slam(double z_slam);
    void target_z_body(double dz_body);
    void target_yaw_slam(double yaw_slam_deg);      // 单位：度
    void target_yaw_body(double dyaw_body_deg);     // 单位：度
    // ★一次到位★：同时设 x/y/z(SLAM系,m) + yaw(度)，PD 同拍控制平移+升降+转向(边转边走升高)。
    //   is_reached() 判 xyz 与 yaw 全部进容差并稳定。钻圈"飞到圆环正前方并对准"用它。
    void target_pose_slam(double x_slam, double y_slam, double z_slam, double yaw_slam_deg);
    // ★绕杆环绕★(无参，参数全在 params.hpp ★绕杆环绕★段)。内部自带子状态机：
    //   悬停订阅 /pole_detector/center 攒帧算杆心+半径 → 飞到距杆表面 standoff 米(机头朝杆、只飞xy)
    //   → 以杆心为圆心绕杆环绕 sweep 度(机头始终朝杆，z 不动)。
    //   像其它动作一样：状态机每拍无脑调它，用 is_reached() 判整个绕杆流程是否完成。
    void description_circle_right();
    void wait_time(double seconds);

    // ── 外部速度（探索）──
    // 进入探索：锁定当前高度为保持高度，切到 EXTERNAL_VEL
    //（收到首个速度命令前由看门狗保高悬停）
    void enter_exploration();
    // 算法每拍调用：缓存机体系速度命令（前进 / 横向纠偏 / yaw_rate）+ 时间戳
    void set_velocity_body(double v_fwd, double v_lat, double yaw_rate);

    // ====================================================================
    //   状态查询
    // ====================================================================
    bool is_reached() const;
    // ★自定水平容差版到位判定★：与 is_reached() 完全同一套逻辑(同稳定计时/同各模式分支)，
    //   只把【水平容差 TOL_XY】换成传入的 tol_xy。给"某个动作要求比正常飞行更准"的场合用——
    //   当前用户是视觉找图(FINDFIGURE 用 params::FF_TOL_XY=0.10 收紧，正常走航点仍用 TOL_XY=0.15)。
    //   z/yaw 容差不变(仍 TOL_Z/TOL_YAW)。稳定计时状态与 is_reached() 共用同一个 settle_*，
    //   ★同一拍内不要既调 is_reached() 又调 is_reached_tol()★(两者会互相打断对方的稳定计时)。
    bool is_reached_tol(double tol_xy) const;
    // 绕杆(description_circle_right)整个流程是否结束：DONE(绕完) 或 FAIL(没看到杆放弃) 都算结束。
    //   ★绕杆专用完成判定★：因该原语内部会切 HOLD/MOVE_POSE/CIRCLE 多个子模式，
    //   不能用 is_reached()(那只反映当前子模式)。状态机 case 用这个判整体是否收尾。
    bool pole_circle_done() const {
        return pole_phase_ == PoleCirclePhase::DONE || pole_phase_ == PoleCirclePhase::FAIL;
    }
    bool pole_circle_failed() const { return pole_phase_ == PoleCirclePhase::FAIL; }

    // ★航点被杆占的放宽到达★：航点(wx,wy)落在某杆膨胀圈内(靠不近)且飞机已接近到"杆边缘+
    //   AVOID_ARRIVE_SLACK_M"内 → 返回 true(判定这个点已尽力到达，可推进下一点)。避免飞机
    //   为一个被杆占的角点卡死/被避障与PD来回拉扯切进邻格。仅 AVOID_ENABLE 时生效；否则恒 false。
    bool waypoint_blocked_arrived(double wx, double wy) const;

    bool is_connected() const  { return current_state_.connected; }
    bool is_offboard() const   { return current_state_.mode == "OFFBOARD"; }
    bool is_armed() const      { return current_state_.armed; }
    bool is_armed_offboard() const {
        return current_state_.armed && current_state_.mode == "OFFBOARD";
    }
    const std::string& mode_string() const { return current_state_.mode; }
    bool has_pose() const      { return has_pose_.load(); }
    ActionMode action_mode() const { return action_mode_; }

    std::string progress_string() const;

    // 把当前雷达位姿记为 home（飞行开始时调一次）
    void capture_home();

    // 请求飞控解锁。非阻塞，每秒发一次。
    // 返回 false 表示放弃；返回 true 表示尝试 / 成功
    bool request_arm();

    // 主动停止：把 action_mode 设回 IDLE，后续 tick 只发零速度，不再跑 PD
    void stop();

    //   根据当前目标计算并发布 setpoint（每 20Hz 调用一次）
    void tick();

private:
    // ---- ROS 接口 ----
    rclcpp::Node* node_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

    // ---- 飞控状态 ----
    mavros_msgs::msg::State current_state_;

    // ---- 雷达位姿 + 位置差分估速度 ----
    geometry_msgs::msg::PoseStamped current_pose_;
    std::atomic<bool>               has_pose_{false};

    // 位姿信箱B写端(主控→视觉,/dev/shm/uav_pose_out)：odom 回调雷达来一条写一条(20Hz)。
    //   视觉进程直读它代替订阅 odom(去 rclpy 用)。布局/协议见 shm_mailbox.hpp。
    shm::PoseMailboxWriter pose_shm_;

    // 用位置差分估算"当前速度"：
    //   inst_v = (p[t] - p[t-1]) / dt
    //   v_est  = α * inst_v + (1-α) * v_est     ← 低通防 SLAM 跳变 D 项爆
    // PD 里用 v_est_* 当 D 项
    double prev_pos_x_   = 0.0;
    double prev_pos_y_   = 0.0;
    double prev_pos_z_   = 0.0;
    rclcpp::Time prev_pose_time_;
    bool         prev_pose_valid_ = false;

    double v_est_x_ = 0.0;
    double v_est_y_ = 0.0;
    double v_est_z_ = 0.0;

    // ---- home 坐标系原点（由 capture_home() 写入）----
    bool   home_captured_ = false;
    double home_x_ = 0.0, home_y_ = 0.0, home_z_ = 0.0;
    double home_yaw_ = 0.0;

    // ---- 当前目标（所有 API 都写到这四个字段）----
    double target_x_   = 0.0;
    double target_y_   = 0.0;
    double target_z_   = 0.0;
    double target_yaw_ = 0.0;
    ActionMode action_mode_ = ActionMode::IDLE;

    // ---- wait_time 计时 ----
    rclcpp::Time wait_until_;
    bool         wait_active_ = false;

    // ---- body 系增量命令缓存（保证 target_*_body 重复调用幂等）----
    double body_dx_cmd_   = 0.0;
    double body_dy_cmd_   = 0.0;
    double body_dz_cmd_   = 0.0;
    double body_dyaw_cmd_ = 0.0;

    // ---- land 请求一次性标记 ----
    bool land_requested_ = false;

    // ---- ★起飞段位置环★(params::TAKEOFF_POSITION_MODE) ----
    //   true = 当前处于"起飞打点上去"阶段，tick() 发位置 setpoint 而非速度。
    //   由 takeoff() 置真、exit_takeoff_position_mode() 置假(状态机在起飞后悬停稳定时调)。
    //   ★用显式标志而不是"看 action_mode_ 是不是 TAKEOFF"★：起飞后 WAIT_AFTER_TAKEOFF
    //   会切成 HOLD，若按 action_mode_ 判就会在"到高度那一瞬间"切回速度环——而那时
    //   可能还有残余爬升速度，PD 接管会抽一下。用标志则由状态机决定何时切，切换点在悬停稳定后。
    bool takeoff_pos_mode_ = false;

    // ---- 外部速度（EXTERNAL_VEL：探索算法直接给机体系速度）----
    double       ext_v_fwd_    = 0.0;   // 机体前进 (m/s)
    double       ext_v_lat_    = 0.0;   // 机体横向纠偏 (m/s)
    double       ext_yaw_rate_ = 0.0;   // (rad/s)
    rclcpp::Time ext_cmd_time_;         // 最近一次收到速度的时间（看门狗用）
    bool         ext_valid_    = false; // 是否已收到过速度命令
    double       explore_z_    = 0.0;   // 进入探索时锁定的保持高度 (slam z)

    // ---- 圆周运动状态（description_circle_right）----
    // 闭环画圆：每拍用飞机实际位置算当前角，目标点放在前方一点的圆上，
    // 半径强制锁回 r，进度按实际转过的角度累计。
    bool   circle_active_   = false;   // 是否已初始化本次圆周
    double circle_cx_       = 0.0;     // 圆心 x (slam)
    double circle_cy_       = 0.0;     // 圆心 y (slam)
    double circle_radius_   = 0.0;     // 目标半径 r (m)，飞机被拉回这个半径
    double circle_speed_    = 0.0;     // 环绕线速度 v (m/s)，用于换算前瞻角
    double circle_sweep_    = 0.0;     // 需要扫过的总角度 (rad)
    double circle_theta_prev_ = 0.0;   // 上一拍飞机的实际角（圆心指向飞机）
    double circle_progress_ = 0.0;     // 已实际扫过的角度 (rad)，用于完成判定 + 进度打印

    // ---- 绕杆环绕（description_circle_right 无参版）的子状态机 ----
    //   一个"有状态原语"：内部按 采集→接近→环绕 三阶段推进，自订 /pole_detector、自攒帧、
    //   自改 action_mode_。对外仍表现为"调一次、tick 推进、is_reached() 判完成"。
    enum class PoleCirclePhase { IDLE, COLLECT, APPROACH, CIRCLE, DONE, FAIL };
    PoleCirclePhase pole_phase_ = PoleCirclePhase::IDLE;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pole_sub_;
    // 采集缓冲(后台回调写/tick 读，pole_mtx_ 保护)：仅 COLLECT 阶段记录杆水平轴心样本
    std::mutex          pole_mtx_;
    bool                pole_collecting_ = false;
    std::vector<double> pole_sx_, pole_sy_, pole_sr_;   // 杆心 x/y + 拟合半径 样本
    rclcpp::Time        pole_collect_start_;
    bool                pole_collect_time_valid_ = false;
    double              pole_cx_ = 0.0, pole_cy_ = 0.0; // 平均后的杆水平轴心
    double              pole_approach_r_ = 0.0;         // 接近/环绕半径 = 杆半径 + standoff
    double              pole_z_lock_ = 0.0;             // 进入时锁定的高度(全程只飞 xy)

    // ---- 全局避障（Obstacle Avoidance，见 avoidance.hpp/params.hpp ★全局避障★）----
    //   订阅 /multi_pole_detector/center 把杆当圆形障碍；位置类动作在 PD 出口做绕行。
    //   ★不改 target_x_/y_★：只把 PD 追的"有效目标" eff_gx_/eff_gy_ 换成贝塞尔前瞻点。
    rclcpp::CallbackGroup::SharedPtr obstacle_cbg_;     // 独立 Reentrant 组(同 ring_driller 套路)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr multi_pole_sub_;
    mutable std::mutex  obstacles_mtx_;                 // 回调写 / update_effective_goal / waypoint_blocked_arrived 读
    std::vector<avoid::CircleObstacle> obstacles_;      // 最新一帧障碍(整帧覆盖)
    rclcpp::Time        obstacles_stamp_;               // 最新帧时刻(TTL 用)
    bool                obstacles_valid_ = false;
    double              eff_gx_ = 0.0, eff_gy_ = 0.0;   // ★有效目标★：tick 写、compute 读(默认=target_x_/y_)
    bool                avoiding_ = false;              // 本拍是否在绕行(仅日志)

    // ---- 解锁重试 ----
    int          arm_retry_count_ = 0;
    bool         arm_giveup_      = false;
    bool         arm_time_valid_  = false;
    rclcpp::Time arm_last_try_;

    // ---- 到位"持续稳定"计时 ----
    mutable bool          settle_valid_ = false;
    mutable rclcpp::Time  settle_start_;

    // ---- 工具函数 ----
    void  publish_setpoint(double vx, double vy, double vz, double yaw_rate);
    // ★位置 setpoint 出口★(起飞段用)：直接发目标位置 x/y/z(SLAM系,m) + 目标 yaw(rad)，
    //   屏蔽速度/加速度/yaw_rate → 飞控内部位置环飞过去。本程序不算速度、不做 PD 矫正。
    void  publish_position_setpoint(double x, double y, double z, double yaw);
    double current_yaw() const;
    void  body_to_slam_xy(double dx_body, double dy_body,
                          double& out_x_slam, double& out_y_slam) const;

    // PD 控制：把目标位置 + 当前位置 + 当前速度 → 速度命令（含限幅）
    void compute_velocity_command(double& vx, double& vy, double& vz, double& yaw_rate) const;

    // 圆周运动：根据已用时间推进目标角，更新 target_x_/y_/yaw_
    void update_circle_target();

    // 绕杆环绕子状态机：每拍推进 采集→接近→环绕。返回 true=整个绕杆流程已完成(供 is_reached 用)。
    bool pole_circle_tick();
    // /pole_detector/center 回调：仅 COLLECT 阶段攒 杆心x,y+半径 样本。
    void on_pole_center(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // /multi_pole_detector/center 回调：整帧覆盖障碍列表(避障用)。
    void on_multi_pole_center(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    // 每拍在 PD 之前算"有效目标" eff_gx_/eff_gy_：默认=target_x_/y_，避障触发时=贝塞尔前瞻点。
    void update_effective_goal();

    // 每 0.2s 在 tick() 末尾自动调用一次：根据当前动作打印进度
    void log_progress();

    // 进度日志节流计时
    rclcpp::Time last_log_;
    bool         log_valid_ = false;
};

}  // namespace fly_mission
