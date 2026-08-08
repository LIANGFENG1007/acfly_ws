#pragma once
// ============================================================================
//  FlyMissionNode —— 主控节点(唯一 ROS 节点)：50Hz 状态机 + 所有外挂模块
//  ---------------------------------------------------------------------------
//  ★代码组织★(2026-08 拆分)：类声明在本文件，实现按功能域分散到多个 .cpp ——
//    fly_mission_node.cpp    构造(话题/UDP/遥测初始化) + on_timer 派发 + main
//    mission_step_basic.cpp  检测/起飞/降落/走线/找图/钻圈/绕杆/探索
//    mission_step_track.cpp  追踪小车 + 视觉锁定投掷 + 纯雷达投掷链
//    mission_step_plat.cpp   落移动平台 + 平台待机/起飞 + 二次起飞流程
//    mission_io.cpp          Arduino / shm 视觉信箱 / UDP 命令 / 遥测上报
//    mission_state.cpp       状态枚举的映射表(纯函数)
//
//  ★状态机写法约定(重要，加新状态照这个来)★：
//    on_timer 的 switch 里【每个 case 只有几行】——调一个 step_xxx()、判到位、
//    切状态。所有实际逻辑放进对应的 step_xxx() 成员函数里。例如：
//        case MissionState::EXPLORATION:
//            exploration(4.0, 1.6);              // 探索终点 (SLAM 系)，改这里即可
//            if (explore_done_) {
//                RCLCPP_INFO(get_logger(), "[探索] 算法报告完成，准备降落");
//                state_ = MissionState::LAND;
//            }
//            break;
//    这样 switch 是一张"流程目录"，能一眼看完全流程；改某个动作的细节则去
//    对应的 step_xxx()，两件事互不干扰。
// ============================================================================

#include "fly_mission/drone_controller.hpp"
#include "fly_mission/params.hpp"
#include "fly_mission/mission_state.hpp"
#include "fly_mission/find_figure.hpp"
#include "fly_mission/waypoint_runner.hpp"
#include "fly_mission/waypoint_receiver.hpp"
#include "fly_mission/line_follower.hpp"
#include "fly_mission/ring_driller.hpp"
#include "fly_mission/car_tracker.hpp"
#include "fly_mission/shm_mailbox.hpp"   // 视觉 shm 信箱读端，见 params::USE_SHM_CV
#include "fly_mission/udp_cmd_link.hpp"  // 跨机命令(启动指令)UDP 读端，见 params::CMD_USE_UDP
#include "fly_mission/udp_telemetry.hpp" // 遥测上报(飞机→监控端)，见 params::TLM_ENABLE
#include "fly_mission/arduino_serial.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>   // 任务启动指令 /mission/start
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

namespace fly_mission {

class FlyMissionNode : public rclcpp::Node
{
public:
    FlyMissionNode();

private:
    // ════════════════════════════════════════════════════════════════════
    //  运动函数转发（薄封装，让状态机 case 读起来像伪代码）
    // ════════════════════════════════════════════════════════════════════
    void takeoff(double z)                       { drone_.takeoff(z); }
    void land()                                  { drone_.land(); }
    void target_xy_body(double dx, double dy)    { drone_.target_xy_body(dx, dy); }
    void target_xy_slam(double x, double y)      { drone_.target_xy_slam(x, y); }
    void target_z_body(double dz)                { drone_.target_z_body(dz); }
    void target_z_slam(double z)                 { drone_.target_z_slam(z); }
    void target_yaw_body(double dyaw)            { drone_.target_yaw_body(dyaw); }
    void target_yaw_slam(double yaw)             { drone_.target_yaw_slam(yaw); }
    // ★一次到位★：同时给 x/y/z(SLAM系,m)+yaw(度)，边飞边转升高(钻圈对准用)
    void target_pose_slam(double x, double y, double z, double yaw_deg) {
        drone_.target_pose_slam(x, y, z, yaw_deg);
    }
    // 绕杆环绕(无参，参数全在 params.hpp ★绕杆环绕★段)：内部自带 采集→接近→环绕 子状态机。
    //   状态机 case 每拍调它，用 pole_circle_done() 判整体完成(不是 is_reached())。
    void description_circle_right()              { drone_.description_circle_right(); }
    bool pole_circle_done() const                { return drone_.pole_circle_done(); }
    void wait_time(double sec)                   { drone_.wait_time(sec); }
    bool is_reached() const                      { return drone_.is_reached(); }
    // ★找图专用到位判定★：同一套逻辑，只把水平容差换成 params::FF_TOL_XY(0.10，比正常飞行
    //   TOL_XY=0.15 收紧)——找图要停得准，正常走航点要流畅，两者分开互不影响。
    //   只在 FINDFIGURE 那个 case 里用它，别的状态一律用上面的 is_reached()。
    bool is_reached_find() const                 { return drone_.is_reached_tol(params::FF_TOL_XY); }
    // 航点被杆占、靠不近时判定"已尽力到达"(放宽到达半径)，供走线状态推进下一点，避免卡死/切邻格。
    bool waypoint_blocked_arrived(double wx, double wy) const {
        return drone_.waypoint_blocked_arrived(wx, wy);
    }

    // ════════════════════════════════════════════════════════════════════
    //  外部 IO（实现在 mission_io.cpp）
    // ════════════════════════════════════════════════════════════════════
    // 串口发 ASCII 指令给 Arduino(115200)：arduino_send("LED ON") → "LED ON\n"×5(次数在
    //   params ★Arduino★段)。★阻塞几十ms，只在状态切换处一次性调，别每拍(50Hz)调★。
    //   没插 Arduino → 打告警继续飞，不中断任务。
    void arduino_send(const std::string& text, int times = params::ARDUINO_SEND_TIMES);
    // ★非阻塞版★：只把字节排进队列(纳秒级返回)，由 on_timer 每拍 arduino_.pump() 发出。
    //   ★飞行中(尤其 OFFBOARD)的指令必须用这个★——阻塞版 send() 要 usleep 5×20ms
    //   + tcdrain，会让主循环停转、setpoint 流中断，飞控可能退出 OFFBOARD。
    void arduino_send_async(const std::string& text, int times = params::ARDUINO_SEND_TIMES);
    // 遥测上报：按 TLM_RATE_HZ 限频发一包。正常时终端静默，失败才节流打红色 ERROR。
    void telemetry_tick();
    // ★shm 视觉信箱消费★(USE_SHM_CV=true 时的正式数据源)：每拍读一次(纳秒级)，
    //   有新帧就喂给 find_/lf_，并更新 LOCK_DROP 用的 dx/dy 与 SLAM 绝对目标点。
    void consume_shm_cv();
    // ★UDP 命令收包★(CMD_USE_UDP=true 时)：置 start_recv_ / trigger_recv_。
    //   命令 2 带状态门：不在 WAIT_TRIGGER 时收到一律丢弃，防提前锁存导致不等指令就起飞。
    void poll_udp_cmd();

    // ════════════════════════════════════════════════════════════════════
    //  横切判定（每拍在 switch 之前跑，实现在 mission_step_basic.cpp）
    // ════════════════════════════════════════════════════════════════════
    // 失锁/飞手接管去抖：非 armed+OFFBOARD 持续 LOST_DEBOUNCE_S → stop() + FINISHED。
    //   返回 true = 已中止任务(调用方应立即 return，不要再跑状态机)。
    bool check_pilot_takeover();
    // 视觉找图打断：正在按航点飞且确认到未拉黑图形 → 记下当前状态、切 FINDFIGURE。
    void check_find_figure_interrupt();

    // ════════════════════════════════════════════════════════════════════
    //  各状态的实现（switch 里每个 case 只调这些，实现分散在 mission_step_*.cpp）
    // ════════════════════════════════════════════════════════════════════
    // ── 基础流程：mission_step_basic.cpp ──
    bool step_boot_check();          // 返回 true = 检测通过可起飞(失败会自己转 FINISHED)
    void step_wait_after_takeoff();  // 起飞后悬停到位 → 退位置环 + 按选定分支初始化并切走
    void exploration(double gx, double gy);   // 自主探索：首拍锁高+发终点，之后转发算法速度
    void step_run_waypoints();       // 写死航点表逐点飞
    void step_run_ext_waypoints();   // 外部航点：未装载则悬停等，装载后逐点飞
    void step_follow_line();         // 视觉寻线
    void step_find_figure();         // 找图：飞向图形中心 → 到点拉黑 → 回被打断的状态
    void step_drill_ring();          // 钻圈
    void step_circle_around();       // 绕杆
    bool step_land();                // 降落；返回 true = 已触底上锁(调用方决定去哪)

    // ── 追踪与投掷：mission_step_track.cpp ──
    void step_track_car();           // 一段追踪小车 → 追上后按 RADAR_DROP_MODE 分岔
    void step_lock_drop();           // 视觉锁定投掷
    void step_radar_descend();       // 纯雷达：边追边降到投掷高度
    void step_radar_drop();          // 纯雷达：判稳投掷
    void step_radar_climb();         // 纯雷达：投后爬升
    void step_return_home_drop();    // 投掷后返航

    // ── 落平台与二次起飞：mission_step_plat.cpp ──
    void step_track_car2();          // 二段追踪 → 追上转边追边降
    void step_track_land();          // 边追边降 → 接触检测 → 主动上锁
    void step_plat_wait();           // 平台待机 → 自己重新起飞
    void step_plat_takeoff();        // 平台起飞：切 OFFBOARD + 解锁 + 爬升
    void step_land_then_wait();      // 第一次降落 → 地面待机
    void step_wait_trigger();        // 地面待机等触发命令
    void step_rearm();               // 二次起飞前置：起 setpoint 流 → OFFBOARD → 解锁
    void step_takeoff_again();       // 二次起飞爬升 → 按 mission2_done_ 分岔
    // ★纯雷达投掷的水平目标★ = 小车雷达位置 + 车身系偏移 RD_OFS_X/Y(随车头旋转)。
    //   偏移定义在小车机体系(车头 X 正 / 左 Y 正)，用小车 yaw 旋转到 SLAM 系，
    //   所以小车转弯时投掷点始终保持在车身后方，不会跑到侧面。
    //   ★只给纯雷达投掷用★：第二段降落(TRACK_CAR2/TRACK_LAND)不加任何偏移。
    void radar_drop_target(double cx, double cy, double cyaw_deg,
                           double& out_x, double& out_y) const;

    // 50Hz 主循环：只做"收包 → 横切判定 → switch 派发"，不含任何飞行逻辑。
    void on_timer();

    // ════════════════════════════════════════════════════════════════════
    //  成员
    // ════════════════════════════════════════════════════════════════════
    DroneController   drone_;
    FindFigure        find_;
    WaypointReceiver  recv_;
    LineFollower      lf_;
    RingDriller       rd_;
    CarTracker        car_;
    WaypointRunner    runner_;
    ArduinoSerial     arduino_{params::ARDUINO_DEV, params::ARDUINO_BOOT_WAIT_S};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             finished_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            start_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             trigger_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           cv_sub_;
    rclcpp::CallbackGroup::SharedPtr                                 cv_cbg_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr    goal_pub_;

    MissionState state_ = MissionState::BOOT_CHECK;
    MissionState state_before_find_ = MissionState::RUN_EXT_WAYPOINTS;  // 进 FINDFIGURE 前的状态(处理完回它)
    bool         wp_loaded_ = false;   // 当前走线状态是否已装载航点(首拍装一次)

    // ---- BOOT_CHECK 的一次性日志标志 ----
    bool check_connected_done_ = false;
    bool check_pose_done_      = false;
    bool check_offboard_done_  = false;

    // ---- 追上判定(TRACK_CAR → LOCK_DROP) ----
    double       catch_timer_ = 0.0;        // 累计"在 CATCH_DIST 内"的时长(s)。
                                            //   ★跑出距离只暂停不清零★(见状态机注释)
    rclcpp::Time catch_last_tick_;          // 上一拍在距离内的时刻(算时间差)
    bool         catch_tick_valid_ = false; // 上一拍是否在距离内(false=本拍重新起算差值)

    // ---- 视觉锁定投掷(LOCK_DROP) ----
    double       lock_dx_ = 0.0, lock_dy_ = 0.0;  // shm 里的机体系偏移(前+/左+)，仅用于投掷距离判定
    double       lock_tgt_x_ = 0.0, lock_tgt_y_ = 0.0;  // 收帧时冻结的 SLAM 绝对目标点
    bool         lock_tgt_valid_ = false;
    rclcpp::Time lock_cv_time_;             // 最近一次"看到目标"的时刻(判数据新鲜度)
    bool         lock_cv_valid_ = false;
    rclcpp::Time lock_cv_alive_time_;       // 最近一次"视觉进程写了帧"的时刻(判进程活着)
    bool         lock_cv_alive_valid_ = false;
    double       lock_z_ = 0.0;             // 锁定期间保持的高度(进入时冻结)
    bool         drop_sent_ = false;        // DIANCI 是否已发(★只发一次★)
    rclcpp::Time drop_hold_until_;          // 投掷后追加锁定的截止时刻
    bool         drop_hold_valid_ = false;
    double       return_z_ = 0.0;           // 返航高度(=投掷完那一刻的当前高度)
    double       return_yaw_ = 0.0;         // 返航朝向(度，进入返航时冻结，途中不主动转头)
    rclcpp::Time lock_start_;               // 进入 LOCK_DROP 的时刻(必投超时用)
    bool         lock_start_valid_ = false;

    // ---- 纯雷达投掷链(RADAR_*) ----
    double       rd_z_ = 0.0;                // 目标高度(SLAM 绝对 z)：降到 home+RD_DROP_H_REL
    rclcpp::Time rd_tick_;                   // 下降积分的上一拍时刻
    bool         rd_tick_valid_ = false;
    rclcpp::Time rd_start_;                  // 进入下降段的时刻(超时用)
    bool         rd_start_valid_ = false;
    rclcpp::Time rd_stable_start_;            // "水平稳住"开始的时刻
    bool         rd_stable_valid_ = false;
    double       rd_climb_x_ = 0.0, rd_climb_y_ = 0.0;   // 爬升段锁住的水平位置
    double       rd_climb_yaw_ = 0.0;                    // 爬升段冻结的朝向
    bool         rd_climb_valid_ = false;

    // ---- 第二段任务：降落到移动平台 ----
    bool         mission2_done_ = false;    // ★第二段是否已跑过★：LAND 靠它判断
                                            //   "还要等命令2" 还是 "真结束"；
                                            //   TAKEOFF_AGAIN 靠它判断进追踪还是回原点
    double       plat_z_ = 0.0;             // 边追边降的目标高度(SLAM 绝对 z)
    rclcpp::Time plat_tick_;                // 上一拍时刻(按时间差积分下降量)
    bool         plat_tick_valid_ = false;
    // ---- 接触检测(判断"已落到平台上") ----
    //   实际下降速率取 drone_.vz_est()(低通)，不在这里存高度历史 —— 单拍差分会被
    //   SLAM 噪声淹没(见 params::PLAT_TOUCH_VZ 说明)。
    rclcpp::Time plat_touch_start_;         // "降不动"开始的时刻
    bool         plat_touch_valid_ = false; // 是否正在累计"降不动"时长
    // ★是否已确认"真的降起来了"★：进本状态时飞机还在悬停(vz≈0)，若不加这道门，
    //   会立刻满足"降不动"→ 在 1.5m 高空上锁 = 摔机。必须先看到实际降速起来。
    bool         plat_descending_ = false;
    rclcpp::Time plat_start_;               // 进入 TRACK_LAND 的时刻(下降段超时用)
    bool         plat_start_valid_ = false;
    rclcpp::Time plat_wait_start_;          // 落到平台的时刻(等待 PLAT_WAIT_SEC 用)
    bool         plat_wait_start_valid_ = false;

    bool beep_sent_            = false;   // BEEP①(连接+雷达就绪)只发一次的标志
    bool beep2_sent_           = false;   // BEEP②(收到启动指令)只发一次的标志
    bool start_recv_           = false;   // 已收到 /mission/start 的 data==1（收到一次即锁定）

    // ---- 失锁去抖 ----
    rclcpp::Time lost_since_;
    bool         lost_since_valid_ = false;

    // ---- 探索：算法速度转发 ----
    double ext_v_fwd_    = 0.0;
    double ext_v_lat_    = 0.0;
    double ext_yaw_rate_ = 0.0;
    bool   ext_cmd_valid_   = false;   // 是否收到过算法速度(一次性锁存，永不清零)
    rclcpp::Time ext_cmd_time_;        // 最近一条算法速度的收到时刻(判新鲜度)
    bool   explore_done_    = false;   // 算法报告探索完成
    bool   explore_entered_ = false;   // EXPLORATION 是否已初始化（锁高+发终点）

    // ---- 二次起飞 ----
    bool         trigger_recv_ = false;   // 已收到触发命令(收到一次即锁定，重复发忽略)
    bool         rearm_inited_ = false;   // REARM 首拍一次性初始化(重记 home + 复位重试)标志
    rclcpp::Time rearm_start_;            // 进 REARM/PLAT_TAKEOFF 的时刻(setpoint 流预热计时)

    // ---- 视觉 shm 信箱 ----
    //   读端(视觉进程建信箱；没建时 read 恒 false 自动回退)
    shm::ShmMailboxReader shm_cv_;
    uint64_t              shm_last_seq_ = 0;   // 上次读到的帧序号(变大=新帧)

    // ---- 跨机 UDP ----
    std::unique_ptr<udp_cmd::UdpCmdReceiver>     cmd_rx_;
    std::unique_ptr<udp_tlm::UdpTelemetrySender> tlm_tx_;
    double   tlm_last_send_ = 0.0;      // 上次发送时刻(steady 秒)，用于限频
    double   tlm_last_warn_ = 0.0;      // 上次告警时刻，用于节流
    uint32_t tlm_fail_run_  = 0;        // 连续失败计数(恢复即清零)
};

}  // namespace fly_mission
