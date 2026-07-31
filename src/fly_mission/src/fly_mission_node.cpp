#include "fly_mission/drone_controller.hpp"
#include "fly_mission/params.hpp"
#include "fly_mission/find_figure.hpp"
#include "fly_mission/waypoint_runner.hpp"
#include "fly_mission/waypoint_receiver.hpp"
#include "fly_mission/line_follower.hpp"
#include "fly_mission/ring_driller.hpp"
#include "fly_mission/car_tracker.hpp"
#include "fly_mission/shm_mailbox.hpp"   // 视觉shm信箱读端(第3步·并行验证期,只对比不切换)
#include "fly_mission/udp_cmd_link.hpp"  // 跨机命令(启动指令)UDP 读端，见 params::CMD_USE_UDP
#include "fly_mission/udp_telemetry.hpp" // 遥测上报(飞机→监控端)，见 params::TLM_ENABLE
#include "fly_mission/arduino_serial.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>   // 任务启动指令 /mission/start
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>       // std::hypot / std::fabs / std::cos / std::sin / std::isfinite
#include <deque>       // 接触检测的高度历史窗口(滑窗算下降速率)
#include <utility>     // std::pair

using namespace std::chrono_literals;
using namespace fly_mission;

// ────────────────────────────────────────────────────────────────────────
//  任务全部状态
// ────────────────────────────────────────────────────────────────────────
enum class MissionState {
    BOOT_CHECK,                 // 检测：连接 + 雷达 → BEEP① → 等启动指令 → BEEP② → OFFBOARD + 解锁
    TAKEOFF,                    // 起飞到 1m
    WAIT_AFTER_TAKEOFF,
    EXPLORATION,
    FINDFIGURE,

    // ★两种走线方式，二选一：在 WAIT_AFTER_TAKEOFF 出口改 state_ 即可切换。两者各自封闭、互不打断。★
    RUN_WAYPOINTS,        // 【写死航点】起飞后直接飞 waypoints.hpp 的 WAYPOINTS
    RUN_EXT_WAYPOINTS,    // 【外部航点】起飞后先悬停等 /mission/waypoints，收到再飞

    FOLLOW_LINE,          // 【视觉寻线】起飞后【直接】进入(不经 WAIT_AFTER_TAKEOFF)：沿黑线飞，机头朝前进方向，丢线超时→降落

    DRILL_RING,           // 【钻圈】悬停采集环位姿 → 边飞边转升高到环前1m对准 → 悬停5s → 降落(独立可调用状态)

    CIRCLE_AROUND,        // 【绕杆】悬停采集杆位姿 → 飞到杆前1m对准 → 绕杆一圈(只飞xy) → 降落(独立可调用状态)

    // ★★★ 二次起飞流程（当前主用）★★★
    //   起飞 → 悬停3s → 飞到 SLAM(1,0) → 降落 → 【地面待机等触发】→ 再起飞 → 回 SLAM 原点 → 降落
    //   与上面各任务的关键差别：中间【真的落地上锁】，之后由程序自己切 OFFBOARD + 解锁再飞。
    HOVER_3S,             // 起飞后悬停 3s
    TRACK_CAR,            // 【追踪小车雷达】飞到小车正上方、机头与小车同向；
                          //   ★追上判定★：水平距离≤CATCH_DIST 累计 CATCH_HOLD_SEC → 转 LOCK_DROP
    LOCK_DROP,            // 【视觉锁定投掷】用 shm 的 dx/dy 精确锁定，★全程保持追踪高度★；
                          //   hypot(dx,dy)≤DROP_DIST → 发 DIANCI → 多锁 DROP_HOLD_SEC → 返航
    RETURN_HOME_DROP,     // 【投掷后返航】以当前高度飞回 (0,0) → 降落

    // ───── 第二段任务：降落到移动平台 → 再起飞 → 回起点 ─────
    TRACK_CAR2,           // 【二段追踪】直接追(不悬停)，累计 CATCH_HOLD_SEC_2 → 转下降
    TRACK_LAND,           // 【边追边降】水平跟小车 + 匀速降；到平台上方 → ★主动上锁★
    PLAT_WAIT,            // 【平台待机】落在平台上等 PLAT_WAIT_SEC 秒
    PLAT_TAKEOFF,         // 【平台起飞】自己切 OFFBOARD + 解锁 + 爬到 1.5m → 回起点
    GO_FORWARD,           // 飞到 SLAM 系 (1,0)（绝对坐标）
    LAND_THEN_WAIT,       // 第一次降落：触底上锁 → 转地面待机（★不进 FINISHED★）
    WAIT_TRIGGER,         // 【地面待机】等 /mission/takeoff_again（收到一次即锁定，重复发忽略）
    REARM,                // 二次起飞前置：起 setpoint 流 → 切 OFFBOARD → 解锁
    TAKEOFF_AGAIN,        // 二次起飞：爬升到 1m
    GO_HOME,              // 飞回 SLAM 原点 (0,0) → 降落收尾

    LAND,
    FINISHED
};

class FlyMissionNode : public rclcpp::Node
{
public:
    FlyMissionNode()
        : Node("fly_mission_node"),
          drone_(this),
          find_(this),
          recv_(this),
          lf_(this),
          rd_(this),
          car_(this)
    {
        // 算法 → 主控：机体系速度命令（前进/横向/yaw_rate）
        cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            "/exploration/cmd_vel", 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                ext_v_fwd_     = msg->twist.linear.x;
                ext_v_lat_     = msg->twist.linear.y;
                ext_yaw_rate_  = msg->twist.angular.z;
                ext_cmd_valid_ = true;
            });

        // 算法 → 主控：探索完成标志（latched）
        const auto latched = rclcpp::QoS(1).transient_local();
        finished_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/exploration/finished", latched,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) explore_done_ = true;
            });

        // 主控 → 算法：探索终点（latched，进入探索时发一次）
        goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "/exploration/goal", latched);

        // ★任务启动指令频道★：BOOT_CHECK 里响完第一声蜂鸣后等启动指令，收到才继续
        //   （响第二声 → 等 OFFBOARD → 解锁起飞）。
        //   ★收到一次就锁定★——对方会 1s 1 次持续发防丢包，之后的一律忽略，不会重复触发。
        //   两种来源由 params::CMD_USE_UDP 选择，状态机只看 start_recv_，逻辑不变：
        if (params::CMD_USE_UDP) {
            // ★UDP 模式★：不订阅 ROS 话题(避免跨机 DDS 组播发现打爆 WiFi、拖垮 mavros)。
            //   构造即 bind；失败只告警——此时收不到启动指令，会一直停在 BOOT_CHECK 等，
            //   不会误起飞。收包在 on_timer 里 cmd_rx_->poll()。
            cmd_rx_ = std::make_unique<udp_cmd::UdpCmdReceiver>(params::CMD_UDP_PORT);
            if (!cmd_rx_->ok()) {
                RCLCPP_ERROR(get_logger(),
                    "[启动] ★命令 UDP 端口 %d bind 失败★(被占用?)。将收不到启动指令→"
                    "一直停在等待。换端口需同时改 params::CMD_UDP_PORT 与发送端 -p port:=",
                    params::CMD_UDP_PORT);
            } else {
                RCLCPP_INFO(get_logger(),
                    "[启动] 启动指令走【UDP】监听端口 %d(不订阅 /mission/start)",
                    params::CMD_UDP_PORT);
            }
        } else {
            // ---- ROS 话题模式(旧行为)：只认 data==1，其他值忽略(防误发) ----
            //   放默认回调组(与 timer 互斥) → 无需加锁。
            start_sub_ = create_subscription<std_msgs::msg::Int32>(
                "/mission/start", rclcpp::QoS(10),
                [this](const std_msgs::msg::Int32::SharedPtr msg) {
                    if (msg->data != 1) return;    // 只认 1
                    if (start_recv_) return;       // 已锁定：重复发直接丢(不刷日志)
                    start_recv_ = true;
                    RCLCPP_INFO(get_logger(), "[启动] 收到启动指令 data=1（已锁定，后续重复发将忽略）");
                });
        }

        // ★二次起飞触发频道★：第一次降落后在 WAIT_TRIGGER 地面待机，等这个话题。
        //   ★收到一次(data=true)就锁定★——用户可能连发很多条防丢包，后续一律忽略，
        //   不会二次触发。放默认回调组(与 timer 互斥) → 与 explore_done_ 同款，无需加锁。
        //   发送命令见文件末尾注释 / 状态机调用速查.txt。
        trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/mission/takeoff_again", rclcpp::QoS(10),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (!msg->data)   return;    // data=false 不当触发（防误发）
                if (trigger_recv_) return;   // 已锁定：重复发直接丢
                trigger_recv_ = true;
                RCLCPP_INFO(get_logger(),
                    "[二次起飞] 收到触发命令（已锁定，后续重复发将忽略）");
            });

        // ★视觉话题统一入口★：主控订阅 /cv/target_info【一次】、只 parse【一次】JSON，
        //   再把已解析对象分发给 FindFigure(找图) 和 LineFollower(寻线)。
        //   —— 消除两个模块各自订阅+各自 parse 同一条消息的重复解析(那是起飞/走线卡顿的根因)。
        //   放独立 Reentrant 组，后台收，不阻塞 20Hz 主循环 timer。
        //   ★USE_SHM_CV=true 时不订阅★——视觉结果改从 shm 信箱读(on_timer 里)，此为回退老路。
        if (!params::USE_SHM_CV) {
            cv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            rclcpp::SubscriptionOptions cv_opt;
            cv_opt.callback_group = cv_cbg_;
            cv_sub_ = create_subscription<std_msgs::msg::String>(
                "/cv/target_info", rclcpp::QoS(10),
                [this](const std_msgs::msg::String::SharedPtr msg) {
                    nlohmann::json j;
                    try {
                        j = nlohmann::json::parse(msg->data);   // 只在这里 parse 一次
                    } catch (const std::exception& ex) {
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "[视觉] /cv/target_info JSON 解析失败: %s", ex.what());
                        return;
                    }
                    find_.ingest(j);   // 找图取 targets（未启用时内部自丢弃）
                    lf_.ingest(j);     // 寻线取 line_x/line_y（没有则视为无效）
                }, cv_opt);
        } else {
            RCLCPP_INFO(get_logger(), "[视觉] 数据源=shm信箱(%s)，不订阅 /cv/target_info", shm::CV_SHM_PATH);
        }

        // ★控制通道横幅★(一眼确认起飞走哪套)
        if (params::TAKEOFF_POSITION_MODE) {
            RCLCPP_INFO(get_logger(),
                "[控制] 起飞段=位置环(打点上去，速度由飞控位置环参数决定，改 MAX_SPEED_Z/KP_Z 无效)；"
                "起飞后悬停稳定即切回速度环 PD，走航点/找图等照常。");
        } else {
            RCLCPP_INFO(get_logger(), "[控制] 全程速度环 PD(起飞也用 PD)。");
        }
        // ★遥测上报★(飞机 → 监控端，只发不收)：正常时终端静默，失败才打红色告警。
        if (params::TLM_ENABLE) {
            tlm_tx_ = std::make_unique<udp_tlm::UdpTelemetrySender>(
                params::TLM_DEST_IP, params::TLM_DEST_PORT);
            if (!tlm_tx_->ok()) {
                RCLCPP_ERROR(get_logger(),
                    "[遥测] ★socket 打开失败★(TLM_DEST_IP=\"%s\" 是合法 IPv4 吗?) → 不上报",
                    params::TLM_DEST_IP);
                tlm_tx_.reset();
            } else {
                RCLCPP_INFO(get_logger(),
                    "[遥测] 上报已启用 → %s:%d @%.0fHz"
                    "(飞机坐标+小车原始坐标+状态[1起飞/2追踪/3投掷/4降落])；"
                    "正常时不打印，失败才红色告警",
                    params::TLM_DEST_IP, params::TLM_DEST_PORT, params::TLM_RATE_HZ);
            }
        }

        // ★预热流开关★：关掉时切 OFFBOARD 前不发任何 setpoint，直接起飞那一拍才开始发。
        //   若飞控要求切 OFFBOARD 前先有 setpoint 流，会切不进/刚进就退 → 改回 true。
        if (params::OFFBOARD_PREHEAT) {
            RCLCPP_INFO(get_logger(), "[控制] OFFBOARD 预热流=开(切模式前发零速度占位)。");
        } else {
            RCLCPP_WARN(get_logger(),
                "[控制] OFFBOARD 预热流=★关★：切 OFFBOARD 前不发任何 setpoint，"
                "起飞那一拍才开始发。若切不进 OFFBOARD 或一进就退出，"
                "把 params::OFFBOARD_PREHEAT 改回 true 重编。");
        }

        timer_ = create_wall_timer(
            std::chrono::milliseconds(params::TIMER_PERIOD_MS),
            std::bind(&FlyMissionNode::on_timer, this));
    }

private:
    // ────────────────────────────────────────────────────────────────────
    //  运动函数转发
    // ────────────────────────────────────────────────────────────────────
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
    //   状态机 case 每拍调它，用 drone_.pole_circle_done() 判整体完成(不是 is_reached())。
    void description_circle_right()              { drone_.description_circle_right(); }
    bool pole_circle_done() const               { return drone_.pole_circle_done(); }
    void wait_time(double sec)                   { drone_.wait_time(sec); }
    bool is_reached() const                      { return drone_.is_reached(); }
    // ★找图专用到位判定★：同一套逻辑，只把水平容差换成 params::FF_TOL_XY(0.10，比正常飞行
    //   TOL_XY=0.15 收紧)——找图要停得准，正常走航点要流畅，两者分开互不影响。
    //   只在 FINDFIGURE 那个 case 里用它，别的状态一律用上面的 is_reached()。
    bool is_reached_find() const                 { return drone_.is_reached_tol(params::FF_TOL_XY); }
    // 串口发 ASCII 指令给 Arduino(115200)：arduino_send("LED ON") → "LED ON\n"×5(次数在
    //   params ★Arduino★段)。★阻塞几十ms，只在状态切换处一次性调，别每拍(50Hz)调★。
    //   没插 Arduino → 打告警继续飞，不中断任务。
    void arduino_send(const std::string& text, int times = params::ARDUINO_SEND_TIMES)
    {
        if (arduino_.send(text, times, params::ARDUINO_SEND_GAP_MS)) {
            RCLCPP_INFO(get_logger(), "[Arduino] 已发 \"%s\" ×%d", text.c_str(), times);
        } else {
            RCLCPP_WARN(get_logger(), "[Arduino] 串口发送失败(设备 %s 没插/名字不对?)，任务继续",
                        params::ARDUINO_DEV);
        }
    }
    // ★非阻塞版★：只把字节排进队列(纳秒级返回)，由 on_timer 每拍 arduino_.pump() 发出。
    //   ★飞行中(尤其 OFFBOARD)的指令必须用这个★——阻塞版 send() 要 usleep 5×20ms
    //   + tcdrain，会让主循环停转、setpoint 流中断，飞控可能退出 OFFBOARD。
    //   代价：不知道有没有真发成功(队列满或串口不通只打告警)。投掷这种"发了就走"
    //   的场合可以接受；BOOT_CHECK 的 BEEP 不在飞行中，仍用阻塞版(顺便把串口打开)。
    void arduino_send_async(const std::string& text, int times = params::ARDUINO_SEND_TIMES)
    {
        if (arduino_.queue(text, times)) {
            RCLCPP_INFO(get_logger(), "[Arduino] 已排队 \"%s\" ×%d(非阻塞，本拍不等发送完成)",
                        text.c_str(), times);
        } else {
            RCLCPP_WARN(get_logger(),
                "[Arduino] 发送队列满，\"%s\" 被丢弃(串口 %s 不通? 待发 %zu 字节)",
                text.c_str(), params::ARDUINO_DEV, arduino_.pending());
        }
    }
    // 状态名(诊断日志用)。加新状态时补一行；漏了会显示 "?" 而不是崩溃。
    static const char* state_name(MissionState s)
    {
        switch (s) {
        case MissionState::BOOT_CHECK:         return "BOOT_CHECK";
        case MissionState::TAKEOFF:            return "TAKEOFF";
        case MissionState::WAIT_AFTER_TAKEOFF: return "WAIT_AFTER_TAKEOFF";
        case MissionState::EXPLORATION:        return "EXPLORATION";
        case MissionState::FINDFIGURE:         return "FINDFIGURE";
        case MissionState::RUN_WAYPOINTS:      return "RUN_WAYPOINTS";
        case MissionState::RUN_EXT_WAYPOINTS:  return "RUN_EXT_WAYPOINTS";
        case MissionState::FOLLOW_LINE:        return "FOLLOW_LINE";
        case MissionState::DRILL_RING:         return "DRILL_RING";
        case MissionState::CIRCLE_AROUND:      return "CIRCLE_AROUND";
        case MissionState::HOVER_3S:           return "HOVER_3S";
        case MissionState::TRACK_CAR:          return "TRACK_CAR";
        case MissionState::LOCK_DROP:          return "LOCK_DROP";
        case MissionState::RETURN_HOME_DROP:   return "RETURN_HOME_DROP";
        case MissionState::TRACK_CAR2:         return "TRACK_CAR2";
        case MissionState::TRACK_LAND:         return "TRACK_LAND";
        case MissionState::PLAT_WAIT:          return "PLAT_WAIT";
        case MissionState::PLAT_TAKEOFF:       return "PLAT_TAKEOFF";
        case MissionState::GO_FORWARD:         return "GO_FORWARD";
        case MissionState::LAND_THEN_WAIT:     return "LAND_THEN_WAIT";
        case MissionState::WAIT_TRIGGER:       return "WAIT_TRIGGER";
        case MissionState::REARM:              return "REARM";
        case MissionState::TAKEOFF_AGAIN:      return "TAKEOFF_AGAIN";
        case MissionState::GO_HOME:            return "GO_HOME";
        case MissionState::LAND:               return "LAND";
        case MissionState::FINISHED:           return "FINISHED";
        }
        return "?";
    }

    // ★状态机状态 → 遥测状态码★(需求：1=起飞 2=追踪 3=投掷 4=降落 0=其它)
    //   把内部十几个状态归并成监控端关心的四类。加新状态时记得在这里归类，
    //   漏了会落到 default(0=其它)——不会出错，只是监控端看不出在干什么。
    static int32_t telemetry_status(MissionState s)
    {
        switch (s) {
        // ── 1 起飞：从解锁爬升到起飞后悬停结束 ──
        case MissionState::TAKEOFF:
        case MissionState::WAIT_AFTER_TAKEOFF:
        case MissionState::HOVER_3S:
        case MissionState::TAKEOFF_AGAIN:      // 二段/平台起飞的爬升段
        case MissionState::REARM:              // 二段起飞前的重新解锁
        case MissionState::PLAT_TAKEOFF:       // 平台上重新解锁起飞
            return udp_tlm::ST_TAKEOFF;

        // ── 2 追踪：跟着小车飞(两段的追踪都算) ──
        case MissionState::TRACK_CAR:
        case MissionState::TRACK_CAR2:
            return udp_tlm::ST_TRACK;

        // ── 3 投掷：视觉锁定 + 投掷 ──
        case MissionState::LOCK_DROP:
            return udp_tlm::ST_DROP;

        // ── 4 降落：投掷后返航、以及所有降落动作 ──
        //   返航归到"降落"是因为它是投掷完成后的收尾段，监控端只关心"任务在收场"。
        case MissionState::RETURN_HOME_DROP:
        case MissionState::GO_HOME:
        case MissionState::LAND:
        case MissionState::LAND_THEN_WAIT:
        // 第二段的"边追边降 + 平台待机"也归到降落——监控端看到 4 就知道在往下落
        case MissionState::TRACK_LAND:
        case MissionState::PLAT_WAIT:
            return udp_tlm::ST_LAND;

        // ── 0 其它：检测/待机/以及当前流程用不到的那些走线状态 ──
        default:
            return udp_tlm::ST_NONE;
        }
    }

    // 遥测上报：按 TLM_RATE_HZ 限频发一包。★不在终端打印★(正常时静默)，
    //   只在连续发送失败时按 TLM_WARN_PERIOD_S 节流打红色 ERROR。
    void telemetry_tick()
    {
        if (!tlm_tx_) return;

        const double now_s = udp_tlm::UdpTelemetrySender::now_mono();
        if (params::TLM_RATE_HZ > 0.0 &&
            (now_s - tlm_last_send_) < (1.0 / params::TLM_RATE_HZ)) return;
        tlm_last_send_ = now_s;

        // 小车坐标要发【原始 B 系值】(未加 CAR_ORIGIN 平移)——标定错了原始值仍然对，
        //   排查时更有用。监控端要画在同一张图上就自己加 CAR_ORIGIN_X/Y。
        double raw_x = 0.0, raw_y = 0.0;
        const bool car_ok = car_.latest_raw(raw_x, raw_y);

        // 视觉数据是否新鲜(与 LOCK_DROP 里同一口径)
        const bool cv_ok = lock_cv_valid_ && lock_tgt_valid_ &&
                           (now() - lock_cv_time_).seconds() <= params::LOCK_CV_TIMEOUT_S;

        // ★飞机位姿有效位★：current_x/y/z 在无位姿时返回 0(见 drone_controller 注释)，
        //   不带这个位的话监控端会把 (0,0,0) 当成"飞机真的在原点"画出来 —— 而实际是
        //   Point-LIO 没起/挂了、坐标毫无意义。这个区分对排查很关键。
        const bool pose_ok = drone_.has_pose();

        int32_t flags = 0;
        if (car_ok)  flags |= udp_tlm::F_CAR_OK;
        if (cv_ok)   flags |= udp_tlm::F_CV_OK;
        if (pose_ok) flags |= udp_tlm::F_POSE_OK;

        const bool ok = tlm_tx_->send(
            drone_.current_x(), drone_.current_y(), drone_.current_z(),
            raw_x, raw_y, telemetry_status(state_), flags);

        if (ok) {
            tlm_fail_run_ = 0;                   // 成功即清零连续失败计数
            return;
        }
        // ★失败才打印，且红色 + 节流★(需求：正常时终端不打印)
        ++tlm_fail_run_;
        if ((now_s - tlm_last_warn_) >= params::TLM_WARN_PERIOD_S) {
            tlm_last_warn_ = now_s;
            RCLCPP_ERROR(get_logger(),
                "[遥测] ★发送失败★ 连续 %u 次(累计成功 %u / 失败 %u) → %s:%d"
                "（网线/WiFi 断了? 目标不可达? 路由不通?）本机飞行不受影响",
                tlm_fail_run_, tlm_tx_->sent(), tlm_tx_->failed(),
                params::TLM_DEST_IP, params::TLM_DEST_PORT);
        }
    }

    // 航点被杆占、靠不近时判定"已尽力到达"(放宽到达半径)，供走线状态推进下一点，避免卡死/切邻格。
    bool waypoint_blocked_arrived(double wx, double wy) const { return drone_.waypoint_blocked_arrived(wx, wy); }

    // ★该状态是否要做"失锁/飞手接管"判定★。
    //   下面这些状态【本来就处于 未解锁 或 非 OFFBOARD】，必须放行，否则 0.5s 去抖一到就被
    //   误判成"飞手接管"→ stop() + FINISHED，二次起飞永远走不到：
    //     BOOT_CHECK      —— 还没解锁、等飞手切 OFFBOARD
    //     LAND / LAND_THEN_WAIT —— 降落触底会上锁(LAND 到位判定本身就要求 !armed)
    //     WAIT_TRIGGER    —— 已落地上锁，在地面等触发命令(可能等很久)
    //     REARM           —— 正在自己切 OFFBOARD + 解锁，成功前必然不满足
    //     FINISHED        —— 任务已结束
    static bool lost_check_active_state(MissionState s)
    {
        return s != MissionState::BOOT_CHECK &&
               s != MissionState::LAND &&
               s != MissionState::LAND_THEN_WAIT &&
               s != MissionState::WAIT_TRIGGER &&
               s != MissionState::REARM &&
               // ★PLAT_WAIT★：已落在平台上并【主动上锁】，桨不转、必然 !armed，
               //   不放行会被 0.5s 去抖误判成"飞手接管" → stop()+FINISHED，
               //   平台上永远起不来。
               s != MissionState::PLAT_WAIT &&
               // ★PLAT_TAKEOFF★：正在自己切 OFFBOARD + 解锁，成功前必然不满足条件
               //   (与 REARM 同理)。
               s != MissionState::PLAT_TAKEOFF &&
               // ★TRACK_LAND★：到上锁高度后会【主动上锁】，从发出 disarm 到状态切走
               //   之间有几拍处于 !armed —— 不放行会被误判"飞手接管"而中止任务。
               //   代价：这个状态下真的失锁(飞手夺权)不会被检出，但它本来就是要落地的
               //   状态，飞手接管反而是期望行为。
               s != MissionState::TRACK_LAND &&
               s != MissionState::FINISHED;
    }

    // 该状态是否允许被"视觉找图"打断：只有【正在按航点走线】时才打断——
    //   RUN_EXT_WAYPOINTS(外部航点，当前主用) / RUN_WAYPOINTS(写死表，备用)。
    //   起飞前后、悬停等航点(WAIT_WAYPOINTS)、降落、找图态本身都不打断。
    static bool find_figure_active_state(MissionState s)
    {
        return s == MissionState::RUN_EXT_WAYPOINTS ||
               s == MissionState::RUN_WAYPOINTS;
    }

    // ────────────────────────────────────────────────────────────────────
    //  探索：首次进入锁高 + 发一次终点；每拍把算法最新速度交给控制器
    //  gx,gy = 探索终点 (SLAM 系，原点=起飞点)，由 on_timer 里硬编码传入
    // ────────────────────────────────────────────────────────────────────
    void exploration(double gx, double gy)
    {
        if (!explore_entered_) {
            drone_.enter_exploration();
            geometry_msgs::msg::PointStamped goal;
            goal.header.stamp    = now();
            goal.header.frame_id = "camera_init";
            goal.point.x = gx;
            goal.point.y = gy;
            goal.point.z = 0.0;
            goal_pub_->publish(goal);
            explore_entered_ = true;
            RCLCPP_INFO(get_logger(), "[探索] 开始，终点 (%.2f, %.2f)", gx, gy);
        }
        // 有新速度就转发；无新速度时不发，由控制器看门狗保高悬停
        if (ext_cmd_valid_) {
            drone_.set_velocity_body(ext_v_fwd_, ext_v_lat_, ext_yaw_rate_);
        }
    }

    // ────────────────────────────────────────────────────────────────────
    //  50Hz 主循环：状态机
    // ────────────────────────────────────────────────────────────────────
    void on_timer()
    {
        drone_.tick();

        // ★Arduino 非阻塞发送泵★：把 arduino_send_async() 排队的字节往串口写一点。
        //   非阻塞(O_NONBLOCK + 不 tcdrain/不 usleep)，没东西发时直接返回，零开销。
        arduino_.pump();

        // ★小车位姿收包(UDP 模式必须每拍调)★：UDP 没有回调线程，不主动收就永远没数据
        //   → car_.latest() 恒 false → 追踪时飞机只会悬停。
        //   params::CAR_USE_UDP=false(ROS 话题模式)时本调用是空操作，留着无副作用。
        //   收一次是几次 recv 系统调用(非阻塞)，微秒级，放在每拍开头零负担。
        car_.poll();

        // ★命令收包(UDP 模式)★：同样必须每拍主动收。收到 CMD_START 就置 start_recv_，
        //   状态机 BOOT_CHECK 只看这个标志 → 与 ROS 话题版行为完全一致。
        //   "收到一次即锁定"由 UdpCmdReceiver 内部保证，重复发的包不会重复触发。
        if (cmd_rx_) {
            if (cmd_rx_->poll()) {          // 本拍有【新】命令(重复发的不算)
                if (cmd_rx_->got(udp_cmd::CMD_START) && !start_recv_) {
                    start_recv_ = true;
                    RCLCPP_INFO(get_logger(),
                        "[启动] 收到启动指令(UDP seq=%u，累计收包 %u)（已锁定，后续重复发将忽略）",
                        cmd_rx_->last_seq(), cmd_rx_->rx_count());
                }
                // ★命令 2(第二段任务触发)★
                //   ★只在 WAIT_TRIGGER(地面待机)状态才接受★——这一条很关键：
                //   本段代码每拍都跑，不看状态。若对方的发送脚本在第一段还在飞的时候
                //   就已经在发命令 2(提前开了脚本 / 上一轮的进程没关)，trigger_recv_
                //   会被提前锁存；等第一段降落进 WAIT_TRIGGER 时标志已是 true →
                //   ★不等指令直接起飞★，人还没准备好桨就转了。
                //   所以这里加状态门：不在待机状态收到的命令 2 一律忽略(只提示)。
                //   UdpCmdReceiver 内部仍然锁存，所以要用 reset() 清掉，
                //   否则进入 WAIT_TRIGGER 后 got() 立刻为真，等于门没加。
                if (cmd_rx_->got(udp_cmd::CMD_TAKEOFF_AGAIN) && !trigger_recv_) {
                    if (state_ == MissionState::WAIT_TRIGGER) {
                        trigger_recv_ = true;
                        RCLCPP_INFO(get_logger(),
                            "[二段] 收到命令 2(UDP)（已锁定，后续重复发将忽略）");
                    } else {
                        cmd_rx_->reset(udp_cmd::CMD_TAKEOFF_AGAIN);   // 丢弃，别提前锁存
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                            "[二段] ★收到命令 2 但当前不在地面待机(状态=%s)，已忽略★"
                            "——请等飞机第一段降落上锁后再发",
                            state_name(state_));
                    }
                }
            }
        }

        // ★shm 视觉信箱消费(USE_SHM_CV=true 时的正式数据源)★：每拍读一次(纳秒级)，
        //   有新帧(seq 变大)就把 shm 帧转成与 /cv/target_info 相同的 JSON 结构，
        //   喂给 find_/lf_ ——两个模块的 ingest 接口一行没动，只是换了"喂数据的人"。
        //   50Hz 读 vs 视觉30/120Hz 写：无新帧的拍直接跳过，不重复喂同一帧。
        if (params::USE_SHM_CV) {
            shm::CvFrame f;
            if (shm_cv_.read(f) && f.seq != shm_last_seq_) {
                shm_last_seq_ = f.seq;
                static const char* kColor[] = { "red", "green", "yellow", "blue" };
                static const char* kShape[] = { "triangle", "square", "round", "unknow" };
                nlohmann::json j;
                j["line_x"] = f.line_x;               // 寻线字段(finding 程序恒0；blackline 接入后有值)
                j["line_y"] = f.line_y;
                auto arr = nlohmann::json::array();
                for (int i = 0; i < f.n_targets; ++i) {
                    const auto& t = f.targets[i];
                    if (t.color < 0 || t.color > 3 || t.shape < 0 || t.shape > 3) continue;  // 未知编码丢弃
                    arr.push_back({ {"id", t.id}, {"color", kColor[t.color]},
                                    {"shape", kShape[t.shape]}, {"x", t.x}, {"y", t.y} });
                }
                j["targets"] = arr;
                find_.ingest(j);
                lf_.ingest(j);

                // ★锁定投掷(LOCK_DROP)用的 dx/dy★：取【第一个目标】的 x/y 当机体系
                //   偏移(前+/左+)。与找图共用同一份 shm 帧，只是找图看 color/shape，
                //   这里看相对量。不经 kColor/kShape 过滤，直接取原始 t.x/t.y。
                //
                //   ★★★ 坐标系约定(已与视觉端逐项核对) ★★★
                //   配套视觉程序 = ws_opencv/github_vison-master/down_vision
                //     · params.yaml: shm_world_coords: false  → ★写机体系 dx/dy★
                //     · x = dx 向前为正、y = dy 向左为正(视觉端 shm_swap_xy: false)
                //     · dx 由 (K_.cy - py)*height/K_.fy 算出，"前为正"，与本段一致
                //     · id/color/shape 恒 0(该程序不发类别)，本段不用这三个字段
                //     · 该程序每帧都写(没目标时 n_targets=0)，所以 seq 会持续推进
                //   ★注意 shm 的 x/y 没有字段标明坐标系★：另一个视觉程序 race_2025 的
                //   shm_world_coords=true 写的是 SLAM 世界绝对坐标，语义相反。
                //   ⇒ ★同时只能跑一个视觉程序★。跑错了的症状：dx/dy 是"飞机在场地里的
                //     绝对坐标"那种数值(几米、且不随对准而减小) → 越飞越偏、永不投掷。
                //   地面自检：手持飞机对着目标移动，看 [锁定] 日志的 dx/dy 是否趋近 0。
                // ★视觉进程"活着"的心跳★：视觉端每拍都写(没看到目标也写 n_targets=0，
                //   见 down_vision_node.cpp 的注释)，所以只要读到新 seq 就说明视觉在跑。
                //   ★与"看到目标"分开记★：否则"视觉活着但暂时没看到目标"和"视觉挂了"
                //   在主控这边表现完全一样(都是 dx/dy 不更新)，日志会误导排查方向。
                lock_cv_alive_time_  = now();
                lock_cv_alive_valid_ = true;

                if (f.n_targets > 0) {
                    const double dx = f.targets[0].x;
                    const double dy = f.targets[0].y;
                    if (std::isfinite(dx) && std::isfinite(dy)) {
                        lock_dx_ = dx;
                        lock_dy_ = dy;
                        lock_cv_time_  = now();     // ★本机时钟★：判视觉数据新鲜度
                        lock_cv_valid_ = true;
                        // ★收到新帧就立刻把机体系偏移换算成 SLAM 绝对目标点并冻结★
                        //   ——绝不能在状态机里每拍用"当前位置+同一个旧 dx/dy"重算：
                        //   视觉写 50Hz、状态机读 50Hz —— ★同频但不同步★，相位漂移会让
                        //   某些拍读不到新帧。那些拍若用【已经移动过的当前位置】加同一个
                        //   旧偏移重算，就会把目标点一路往前推 → 正反馈，飞机越冲越远。
                        //   在收帧这一刻算，用的就是"看到目标时飞机在哪"，物理上正确。
                        if (drone_.has_pose()) {
                            const double yaw = drone_.current_yaw_deg() * M_PI / 180.0;
                            const double c = std::cos(yaw), s = std::sin(yaw);
                            lock_tgt_x_ = drone_.current_x() + c * dx - s * dy;
                            lock_tgt_y_ = drone_.current_y() + s * dx + c * dy;
                            lock_tgt_valid_ = true;
                        }
                    }
                }
                const double age_ms = (shm::ShmMailboxReader::now_mono() - f.stamp) * 1000.0;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                    "[shm] 消费 seq=%lu 数据年龄=%.1fms targets=%d",
                    static_cast<unsigned long>(f.seq), age_ms, f.n_targets);
            }
        }
        
        if (lost_check_active_state(state_))
        {
            constexpr double LOST_DEBOUNCE_S = 0.5;

            if (drone_.is_armed_offboard()) {
                lost_since_valid_ = false;
            } else {
                if (!lost_since_valid_) {
                    lost_since_       = now();
                    lost_since_valid_ = true;
                }
                if ((now() - lost_since_).seconds() >= LOST_DEBOUNCE_S) {
                    RCLCPP_WARN(get_logger(),
                        "飞手接管或失锁（armed=%d mode=\"%s\" 持续 %.2fs）→ 任务中止",
                        drone_.is_armed() ? 1 : 0,
                        drone_.mode_string().c_str(),
                        LOST_DEBOUNCE_S);
                    drone_.stop();
                    state_ = MissionState::FINISHED;
                }
            }
        }

        // ★视觉找图·打断检查★：仅在【正在按航点飞】(wp_loaded_)时才打断——
        //   外部模式悬停等航点期间(未装载)不打断，避免任务还没开始就跑去追图形。
        //   一旦确认到(未拉黑的)图形 → 记下当前状态、打断去 FINDFIGURE。FINDFIGURE 自身不重入。
        if (find_figure_active_state(state_) &&
            wp_loaded_ &&
            state_ != MissionState::FINDFIGURE &&
            find_.has_pending())
        {
            state_before_find_ = state_;
            find_.begin();
            state_ = MissionState::FINDFIGURE;
            RCLCPP_INFO(get_logger(), "[找图] 确认到图形 → 打断当前走线，前往查看");
        }

        switch (state_) {

        // ─── case 1：检测 ─────────────────────────────────────────
        case MissionState::BOOT_CHECK:
        {
            if (!drone_.is_connected()) break;
            if (!check_connected_done_) {
                RCLCPP_INFO(get_logger(), "[检测] 飞控连接：是");
                check_connected_done_ = true;
            }

            if (!drone_.has_pose()) break;
            if (!check_pose_done_) {
                RCLCPP_INFO(get_logger(), "[检测] 雷达里程计：是");
                check_pose_done_ = true;
            }

            // ── BEEP① ──：飞控+雷达都就绪的提示（★只发一次★，本 case 每拍循环，
            //   靠 beep_sent_ 防止等待期间狂发）。没插串口只告警不影响流程。
            //   ★不能每拍调 arduino_send★：一次阻塞几十 ms(5次×20ms)，50Hz 会拖垮主循环。
            if (!beep_sent_) {
                arduino_send("BEEP");
                beep_sent_ = true;
                RCLCPP_INFO(get_logger(),
                    "[启动] 飞控+雷达就绪，已响第一声 → 等启动指令 /mission/start (Int32 data=1)");
            }

            // ── 等启动指令 ──：等 /mission/start 收到 data==1。
            //   ★收到一次即锁定★(start_recv_)，用户会 1s 1 次持续发防丢包，重复发直接忽略。
            //   此时【尚未解锁、也还没到 OFFBOARD 检查】，桨不转，可以安全长时间等。
            if (!start_recv_) {
                if (params::CMD_USE_UDP) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                        "[启动] 等启动指令(UDP 端口 %d)：在另一台机器上跑 "
                        "udp_cmd_send <本机IP> start   (建议 1s 1 次持续发防丢包)",
                        params::CMD_UDP_PORT);
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                        "[启动] 等启动指令：ros2 topic pub -r 1 /mission/start std_msgs/msg/Int32 \"{data: 1}\"");
                }
                break;
            }

            // ── BEEP② ──：收到启动指令的确认音（同样只发一次）。
            //   响完这声就该由飞手拨 OFFBOARD 了。
            if (!beep2_sent_) {
                arduino_send("BEEP");
                beep2_sent_ = true;
                RCLCPP_INFO(get_logger(),
                    "[启动] 已收到启动指令并响第二声 → 请拨 OFFBOARD，之后自动解锁起飞");
            }

            // 等飞手手动切到 OFFBOARD（此时 setpoint 占位流已经在发了）
            if (!drone_.is_offboard()) break;
            if (!check_offboard_done_) {
                RCLCPP_INFO(get_logger(), "[检测] OFFBOARD：是");
                check_offboard_done_ = true;
            }

            // 程序自动请求解锁（每秒重试，最多 5 次）
            if (!drone_.is_armed()) {
                if (!drone_.request_arm()) {
                    RCLCPP_ERROR(get_logger(), "[检测] 解锁失败，任务中止");
                    state_ = MissionState::FINISHED;
                }
                break;
            }
            RCLCPP_INFO(get_logger(), "[检测] 解锁：是");

            drone_.capture_home();
            RCLCPP_INFO(get_logger(), "[检测] 全部通过，开始起飞");
            state_ = MissionState::TAKEOFF;
            break;
        }

        // ─── case 2：起飞 ─────────────────────────────────────────
        case MissionState::TAKEOFF:
            takeoff(1.5);
            if (is_reached()) {
                RCLCPP_INFO(get_logger(), "[起飞] 完成");
                state_ = MissionState::WAIT_AFTER_TAKEOFF;
            }
            break;

        case MissionState::WAIT_AFTER_TAKEOFF:
            wait_time(0.5);
            if (is_reached()) {
                // ★★★ 切换起飞后的任务：只改这一行即可（六选一）★★★
                //   HOVER_3S          = 悬停3s → ★追踪小车雷达★(一直追不退出)  ★当前选中★
                //   ★注意★：HOVER_3S 出口现在进 TRACK_CAR。二次起飞那条链
                //     (GO_FORWARD→LAND_THEN_WAIT→WAIT_TRIGGER→REARM→TAKEOFF_AGAIN→GO_HOME)
                //     代码完整保留但【当前进不去】——要跑二次起飞，把 HOVER_3S 出口的
                //     state_ 改回 MissionState::GO_FORWARD 即可(见该 case 里的注释)。
                //   FOLLOW_LINE       = 视觉寻线(沿黑线飞，机头朝前进方向，丢线超时→降落)
                //   RUN_WAYPOINTS     = 写死航点表(waypoints.hpp)，逐点飞
                //   RUN_EXT_WAYPOINTS = 外部发的航点(/mission/waypoints；未收到悬停等，已收到免等直接飞)
                //   DRILL_RING        = 钻圈(悬停采集环位姿→飞到环前1m对准→穿圈→降落)
                //   CIRCLE_AROUND     = 绕杆(悬停采集杆位姿→飞到杆前1m对准→绕杆一圈→降落)
                const MissionState next = MissionState::HOVER_3S;

                // ★退出起飞段位置环 → 之后全部改回速度环 PD★
                //   放在这里(起飞后悬停已 is_reached，飞机基本静止)切换最平稳；
                //   之后走航点/找图/绕杆/钻圈/寻线全部走原来的 PD，行为与改动前一致。
                //   放在下面各分支初始化【之前】调，保证无论选哪个后续任务都已退出位置环。
                //   (tick() 里 EXTERNAL_VEL 分支本就在位置环分支之前 return，所以寻线/探索
                //    即使忘了清标志也不会被位置环拦住；但其余任务会，故统一在此清。)
                drone_.exit_takeoff_position_mode();

                // 按所选目标各自初始化(互不干扰)：
                if (next == MissionState::FOLLOW_LINE) {
                    drone_.enter_exploration();  // ★锁定当前高度为保持高度★ + 切 EXTERNAL_VEL；
                                                 //   不调则 explore_z_ 保持初值 0 → 保高 PD 把飞机往地面(0)压→越飞越低！
                    lf_.begin();          // 从此刻起算寻线无线超时(不开找图：寻线与找图共用话题)
                } else if (next == MissionState::DRILL_RING) {
                    rd_.begin();          // 起采集：悬停收 N 帧算环位姿(不开找图、不装航点)
                } else if (next == MissionState::CIRCLE_AROUND) {
                    // 绕杆无需在此初始化：description_circle_right() 首次调用时内部自动起采集
                    //   (不开找图、不装航点)。见 DroneController 的绕杆子状态机。
                } else if (next == MissionState::HOVER_3S) {
                    // 悬停3s→追踪小车：不开找图、不装航点，全程用 wait_time /
                    //   target_pose_slam 原语。追踪的初始化(锁高)在 HOVER_3S 出口做——
                    //   要锁的是"悬停结束那一刻"的高度，不是现在这一刻。
                } else {
                    find_.enable();       // 航点走线：启用找图(可被打断)
                    wp_loaded_ = false;   // 走线状态首拍装载自己的航点
                    // 外部航点·预装载：起飞前已收到则免等直接飞(对写死方式无影响)
                    if (next == MissionState::RUN_EXT_WAYPOINTS && recv_.has_waypoints()) {
                        runner_.set_waypoints(recv_.take());
                        wp_loaded_ = true;
                        RCLCPP_INFO(get_logger(), "[外部航点] 起飞前已收到 %zu 个点 → 免等待直接执行",
                                    runner_.total());
                    }
                }
                state_ = next;
            }
            break;

        case MissionState::EXPLORATION:
            exploration(4.0, 1.6);              // 探索终点 (SLAM 系)，改这里即可
            if (explore_done_) {
                RCLCPP_INFO(get_logger(), "[探索] 算法报告完成，准备降落");
                state_ = MissionState::LAND;
            }
            break;
        
        // ─── 【视觉寻线】沿黑线飞：机头朝前进方向；连续丢线超时 → 降落。不开找图。 ───
        case MissionState::FOLLOW_LINE: {
            if (lf_.timed_out()) {
                RCLCPP_INFO(get_logger(), "[寻线] 连续丢线超时 → 线走完/丢线，准备降落");
                state_ = MissionState::LAND;
                break;
            }
            double vf, vl, yr;
            lf_.compute(vf, vl, yr);          // 有效线→算机体系速度；无有效线→全 0(悬停等，超时另判)
            drone_.set_velocity_body(vf, vl, yr);
            break;
        }

        case MissionState::RUN_EXT_WAYPOINTS:
            if (!wp_loaded_) {    
                // 起飞前没收到 → 悬停等外部发来(wait_time 幂等锁位置)；收到即装载
                wait_time(1.0);
                if (recv_.has_waypoints()) {
                    runner_.set_waypoints(recv_.take());   // 装入外部航点，从第 0 个开始
                    wp_loaded_ = true;
                    RCLCPP_INFO(get_logger(), "[外部航点] 收到 %zu 个点，开始执行", runner_.total());
                }
            } else if (runner_.done()) {
                RCLCPP_INFO(get_logger(), "[外部航点] 全部 %zu 个完成，准备降落", runner_.total());
                state_ = MissionState::LAND;
            } else {
                // 逐点飞
                const auto& wp = runner_.current();
                target_xy_slam(wp.x, wp.y);                // 幂等 PD 飞向当前航点，高度不变
                if (is_reached() || waypoint_blocked_arrived(wp.x, wp.y)) {
                    RCLCPP_INFO(get_logger(), "[外部航点] 航点 %zu/%zu (%.2f,%.2f) 到达",
                                runner_.index_1based(), runner_.total(), wp.x, wp.y);
                    runner_.advance();
                }
            }
            break;

        case MissionState::RUN_WAYPOINTS:
            if (!wp_loaded_) {
                runner_.reset_default();   // 装入写死航点表，从第 0 个开始
                wp_loaded_ = true;
                RCLCPP_INFO(get_logger(), "[写死航点] 装入 %zu 个点，开始执行", runner_.total());
            } else if (runner_.done()) {
                RCLCPP_INFO(get_logger(), "[写死航点] 全部 %zu 个完成，准备降落", runner_.total());
                state_ = MissionState::LAND;
            } else {
                const auto& wp = runner_.current();
                target_xy_slam(wp.x, wp.y);          // 幂等 PD 飞向当前航点，高度不变
                if (is_reached() || waypoint_blocked_arrived(wp.x, wp.y)) {
                    RCLCPP_INFO(get_logger(), "[写死航点] 航点 %zu/%zu (%.2f,%.2f) 到达",
                                runner_.index_1based(), runner_.total(), wp.x, wp.y);
                    runner_.advance();               // 推进下一个(下一拍飞新点；走完则上面转 LAND)
                }
            }
            break;
        
        // ─── 视觉找图：飞向确认到的图形中心(实时刷新)，到点悬停后拉黑，回被打断状态 ───
        case MissionState::FINDFIGURE: {
            double tx, ty;
            // ★用找图专用的收紧容差 FF_TOL_XY(0.10)★，不是正常飞行的 TOL_XY(0.15)。
            const auto step = find_.tick(is_reached_find(), tx, ty);
            if (step == FindFigure::Step::FLYING) {
                target_xy_slam(tx, ty);         
            } else {
                // DONE：到点悬停完 / 超时放弃 / 队空 → 回被打断前状态(下一拍其 case 重下原目标，幂等续飞)
                state_ = state_before_find_;
                RCLCPP_INFO(get_logger(), "[找图] 本次结束 → 回到原状态继续走线");
            }
            break;
        }

        // ─── 钻圈：悬停采集环位姿 → 边飞边转升高到环前1m对准 → 悬停5s → 降落 ───
        //   rd_.begin() 在 WAIT_AFTER_TAKEOFF 出口已调；本 case 每拍按 rd_ 的阶段下指令。
        case MissionState::DRILL_RING: {
            double tx, ty, tz, tyaw;
            const auto step = rd_.tick(is_reached(), tx, ty, tz, tyaw);
            switch (step) {
                case RingDriller::Step::COLLECTING:
                    // 原地悬停攒帧：wait_time 幂等锁住当前位姿(承接起飞后 HOLD)，等 ring_detector 攒够
                    wait_time(params::DRILL_COLLECT_TIMEOUT_S);
                    break;
                case RingDriller::Step::APPROACHING:
                    target_pose_slam(tx, ty, tz, tyaw);     // ★边飞边转升高★飞向当前段(环前对准→穿到环后)
                    break;
                case RingDriller::Step::HOVERING:
                    wait_time(params::DRILL_HOVER_SEC);      // 到位悬停(环前/环后各一次，幂等)
                    break;
                case RingDriller::Step::DONE:
                    RCLCPP_INFO(get_logger(), "[钻圈] 环前对准+穿圈到环后 完成 → 降落");
                    state_ = MissionState::LAND;
                    break;
                case RingDriller::Step::FAILED:
                    RCLCPP_WARN(get_logger(), "[钻圈] 未获得足够环检测 → 降落");
                    state_ = MissionState::LAND;
                    break;
            }
            break;
        }

        // ─── 绕杆：悬停采集杆位姿 → 飞到杆前1m对准 → 绕杆一圈(只飞xy) → 降落 ───
        //   description_circle_right() 无参、内部自带 采集→接近→环绕 子状态机；每拍调一次推进。
        //   ★完成判定用 pole_circle_done()，不是 is_reached()★(该原语内部切多个子模式)。
        case MissionState::CIRCLE_AROUND: {
            description_circle_right();               // 每拍推进绕杆子状态机(首次调用内部自动起采集)
            if (pole_circle_done()) {
                if (drone_.pole_circle_failed())
                    RCLCPP_WARN(get_logger(), "[绕杆] 未获得足够杆检测 → 降落");
                else
                    RCLCPP_INFO(get_logger(), "[绕杆] 绕杆完成 → 降落");
                state_ = MissionState::LAND;
            }
            break;
        }


        // ════════════════════════════════════════════════════════════════
        //  【当前流程】起飞(1.5m) → 悬停 3s → 追踪小车雷达(一直追，不退出)
        //
        //  下面 HOVER_3S 之后还保留着一条【二次起飞流程】的完整实现：
        //      GO_FORWARD → LAND_THEN_WAIT → WAIT_TRIGGER → REARM → TAKEOFF_AGAIN
        //      → GO_HOME → LAND → FINISHED
        //  （飞到 SLAM(1,0) → 降落 → 地面等 /mission/takeoff_again → 自己切 OFFBOARD
        //    + 解锁 → 再起飞 → 回原点 → 降落）
        //  ★这条链当前进不去★(HOVER_3S 出口改成进 TRACK_CAR 了)，代码保留备用。
        //  要跑二次起飞：把 HOVER_3S 出口的 state_ 改成 GO_FORWARD、并去掉 car_.begin()。
        //  坐标全是 SLAM/camera_init 绝对系(原点 = SLAM 初始化点 ≈ 第一次起飞点)。
        // ════════════════════════════════════════════════════════════════

        // ─── ① 起飞后悬停 3s → 转入追踪 ──────────────────────────
        case MissionState::HOVER_3S:
            wait_time(3.0);                       // 幂等(同时长重复调不重置计时)；锁住当前位姿悬停
            if (is_reached()) {
                // ★锁定当前高度为追踪高度★(小车在地上跑，它的 z 对飞机没意义)。
                //   ★放在这里而不是 WAIT_AFTER_TAKEOFF★：要锁的是"悬停结束、飞机已稳"
                //   那一刻的高度。CarTracker 不订阅飞机位姿，高度必须由主控传进去。
                car_.begin(drone_.current_z());
                // 重置追上计时(当前流程只进 TRACK_CAR 一次，成员初值已是 0；
                //   显式清零是为了以后若有"回到追踪"的分支时不会带着旧计时直接判追上)
                catch_timer_      = 0.0;
                catch_tick_valid_ = false;
                RCLCPP_INFO(get_logger(),
                    "[流程] 悬停 3s 完成 → 开始追踪小车雷达"
                    "（★标定量 CAR_ORIGIN=(%.2f, %.2f) 是开环的：量错多少就稳定偏多少，"
                    "建议先让小车不动、确认飞机停在它正上方再放跑★）",
                    params::CAR_ORIGIN_X, params::CAR_ORIGIN_Y);
                state_ = MissionState::TRACK_CAR;
            }
            break;

        // ─── 【追踪小车雷达】飞到小车正上方 + 机头与小车同向，★一直追不退出★ ───
        //   每拍问 car_ 要"小车现在在飞机 SLAM 系的哪里"，直接喂 target_pose_slam：
        //   MOVE_POSE 是"一次到位"原语(PD 同拍控制平移+转向)，每拍重下新目标即为实时追踪。
        //   ★不判 is_reached()★：追踪是持续行为，不存在"到位了就下一步"——到位了也要
        //   继续跟着小车动。小车停下时飞机自然停在它上方(PD 误差趋零)。
        //   高度：CAR_TRACK_Z=false 时 car_ 返回的 z 恒为进入时锁定的高度。
        case MissionState::TRACK_CAR: {
            // 丢数据时悬停的时长：本状态不判 is_reached()，所以这个数值本身不影响行为，
            //   只需要是个【固定常量】——wait_time 靠"时长没变"实现幂等(每拍重复调不会
            //   重置计时、不会把锁定位置刷成当前漂移位置)。给大值纯粹表示"一直悬停"。
            constexpr double CAR_LOST_HOLD_SEC = 3600.0;

            double tx, ty, tz, tyaw;
            if (car_.latest(tx, ty, tz, tyaw)) {
                target_pose_slam(tx, ty, tz, tyaw);      // 实时刷新目标(位置+偏航一起)

                // ★追上判定★：飞机与小车的【水平】距离(不含高度差——飞机在小车上方飞，
                //   算上高度永远追不上)。≤CATCH_DIST 就累加时间，累计达 CATCH_HOLD_SEC
                //   → 认为真追上了 → 直接转 LOCK_DROP(★不降高，全程保持当前高度★)。
                const double d_xy = std::hypot(tx - drone_.current_x(),
                                               ty - drone_.current_y());
                const rclcpp::Time now_t = now();
                if (d_xy <= params::CATCH_DIST) {
                    if (catch_tick_valid_) {
                        // 累加本拍时长(用两拍时间差，不假定 50Hz 恒定)
                        catch_timer_ += (now_t - catch_last_tick_).seconds();
                    }
                    catch_last_tick_  = now_t;
                    catch_tick_valid_ = true;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "[追上] 距离 %.2fm ≤ %.2fm，已累计 %.1f/%.1fs",
                        d_xy, params::CATCH_DIST, catch_timer_, params::CATCH_HOLD_SEC);
                } else {
                    // ★跑出追上距离★：按用户选择【只暂停累加、不清零】(允许断断续续凑满)。
                    //   要改成"必须连续保持"就在这里加 catch_timer_ = 0.0;
                    catch_tick_valid_ = false;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                        "[追踪] 小车@飞机系(%.2f, %.2f) yaw=%.1f° 距离 %.2fm(未追上，已累计 %.1fs)",
                        tx, ty, tyaw, d_xy, catch_timer_);
                }

                if (catch_timer_ >= params::CATCH_HOLD_SEC) {
                    // ★直接转视觉锁定★：★不降高★，全程保持追踪高度(2026-08 需求变更)。
                    //   lock_z_ 锁住此刻的高度，之后 LOCK_DROP 全程下发它保持不变。
                    //   ★为什么锁住而不是每拍取 current_z★：每拍取当前高度会让目标 z
                    //   永远等于实际 z → 高度误差恒 0 → 等于不控高度，飞机会随气流/
                    //   PD 耦合慢慢漂高或掉高而没人纠正。锁一个定值才是真正的定高。
                    lock_z_ = drone_.current_z();
                    drop_sent_        = false;
                    drop_hold_valid_  = false;
                    // ★清掉之前阶段(如 FINDFIGURE)残留的视觉状态★：否则进锁定第一拍就会
                    //   拿一个很久以前算出的 lock_tgt_ 当目标飞过去。清掉后必须等新帧。
                    lock_cv_valid_    = false;
                    lock_tgt_valid_   = false;
                    lock_start_       = now();       // 锁定总超时起算
                    lock_start_valid_ = true;
                    state_ = MissionState::LOCK_DROP;
                    RCLCPP_INFO(get_logger(),
                        "[追上] 累计 %.1fs ≥ %.1fs → ★转视觉锁定投掷★"
                        "(★保持当前高度 %.2fm[离起飞点 %.2fm]不下降★；"
                        "对准阈值 %.2fm；★必投倒计时 %.1fs 已开始——到点无论视觉/雷达"
                        "什么状态都会投★)",
                        catch_timer_, params::CATCH_HOLD_SEC,
                        lock_z_, lock_z_ - drone_.home_z(),
                        params::DROP_DIST, params::LOCK_TIMEOUT_SEC);
                }
            } else {
                // 没有新数据(小车雷达没上线 / 话题真的停了)：★必须主动锁住当前位置★。
                //   ★不能"什么都不做"★：那样 action_mode_ 仍是 MOVE_POSE、target_* 仍是
                //   【最后一次收到的小车位置】，PD 会继续朝那个点冲——如果话题是在小车
                //   移动中断掉的，飞机就会奔向一个过期的点，正是要避免的行为。
                //   wait_time 幂等：切 HOLD 并锁死【调用瞬间】的位姿，重复调不重置计时；
                //   数据恢复后上面的 target_pose_slam 会切回 MOVE_POSE 继续追(切换会清
                //   settle 计时，但追踪不判 is_reached()，无影响)。
                //   注意这不是"跟丢"——小车位置本来一直知道，这里只针对话题停了。
                wait_time(CAR_LOST_HOLD_SEC);
            }
            break;
        }

        // ═══════════════════════════════════════════════════════════════
        //  【视觉锁定投掷】LOCK_DROP —— ★新增状态，不改动 TRACK_CAR 的逻辑★
        //
        //  位置：用 shm 信箱的 dx/dy(机体系，前+/左+)换算成 SLAM 绝对目标点。
        //        换算 = 收帧那一刻的飞机位置 + 绕当时 yaw 旋转后的 (dx,dy)，公式与
        //        drone_controller 的 body_to_slam_xy() 一致(dx/dy 归零即正对目标)。
        //        ★换算在 on_timer 的 shm 消费处做完并冻结★，本状态只是取用——
        //        绝不能在这里每拍用"当前位置 + 旧 dx/dy"重算：视觉写 50Hz、状态机读 50Hz
        //        但★不同步★，读不到新帧的那些拍会拿已移动过的位置配旧偏移，把目标点
        //        一路往前推(正反馈)。
        //  偏航：★仍用小车雷达(UDP)的 yaw★——shm 里只有 dx/dy 没有 yaw，而"锁定时
        //        偏航要与目标一致"是需求。雷达数据没了就保持当前机头(不乱转)。
        //
        //  ★视觉数据分三级处理(丢帧不等于放弃)★：
        //    ① 新鲜(≤LOCK_CV_TIMEOUT_S)      用 dx/dy 精确锁定 + 可投掷
        //    ② 短暂丢(~LOCK_CV_FALLBACK_S)    原地锁住等它回来(不投)
        //    ③ 长时间丢(>LOCK_CV_FALLBACK_S)  ★回退小车雷达(UDP)定位★继续跟上，
        //                                     仍不投；视觉恢复自动切回 ①
        //    为什么③要回退：小车会跑，视觉丢了原地干等的话小车早开走、再也看不见 → 死等。
        //  高度：★全程保持不变★——lock_z_ 在判定追上那一刻被锁成"当时的高度"，
        //        本状态每拍都下发它。★不下降★(2026-08 需求：全程一个高度)。
        //  投掷：hypot(dx,dy) ≤ DROP_DIST → 串口发 DIANCI(★非阻塞★，只发一次) →
        //        再锁定 DROP_HOLD_SEC → 转 RETURN_HOME_DROP。
        //  ★两种投掷触发(哪个先到算哪个)★，之后收尾路径完全相同(锁 DROP_HOLD_SEC → 返航)：
        //    · 提前投：视觉对准到 DROP_DIST 以内
        //    · ★必投★：进入本状态起 LOCK_TIMEOUT_SEC 到点 → ★无条件投掷★
        //      —— 视觉看不见/视觉进程死了/雷达接管中，一律照投(纯挂钟计时，
        //         不依赖任何数据源)。否则视觉一死就会永久悬停到电池耗尽。
        // ═══════════════════════════════════════════════════════════════
        case MissionState::LOCK_DROP: {
            const rclcpp::Time now_t = now();

            // ── 1) 视觉数据新鲜度 ──
            const double cv_age = lock_cv_valid_
                                ? (now_t - lock_cv_time_).seconds() : 1e9;
            //   ★必须同时有"冻结好的目标点"★(lock_tgt_valid_)：收帧时若还没位姿，
            //   dx/dy 存下来了但目标点没算出来 —— 那种情况不能算数据可用，
            //   否则会出现"位置在悬停、高度却在下降"的不一致行为。
            const bool cv_ok = lock_cv_valid_ && lock_tgt_valid_
                               && cv_age <= params::LOCK_CV_TIMEOUT_S;

            // ── 2) 高度：★不下降，保持 lock_z_ 不变★ ──
            //   lock_z_ 在 TRACK_CAR 判定追上那一刻锁成"当时的高度"，之后不再改动，
            //   下面每个分支都把它原样下发 → 全程定高。不做任何积分。
            //   (需求 2026-08：去掉下降段，追踪/锁定/投掷全程保持一个高度)

            // ── 3) 偏航：优先小车雷达的 yaw，没有就保持当前机头 ──
            double cx, cy, cz, cyaw;
            const bool car_ok = car_.latest(cx, cy, cz, cyaw);
            const double tgt_yaw_deg = car_ok ? cyaw : drone_.current_yaw_deg();

            if (cv_ok) {
                // ── 4) 位置：用【收视觉帧那一刻算好并冻结】的 SLAM 目标点 ──
                //   ★不在这里用 current_x/y + dx/dy 重算★：视觉与状态机同为 50Hz 但不
                //   同步，重算会把目标点一路往前推(正反馈)。换算已在 shm 消费处做完并冻结。
                target_pose_slam(lock_tgt_x_, lock_tgt_y_, lock_z_, tgt_yaw_deg);

                // ★投掷判定距离要加前置补偿 DROP_LEAD_X★(见 params 说明)：
                //   投掷物带着飞机前向速度、机构可能有前抛角 → 落点系统性偏前。
                //   把 dx 减去补偿量再判距离 = "目标还在前方 DROP_LEAD_X 时就提前投"。
                //   ★注意只用于投掷时机判定★——上面 target_pose_slam 用的仍是未补偿的
                //   lock_tgt_(飞机照旧往目标正上方飞)，所以飞机不会停在偏前的位置。
                const double d_cv = std::hypot(lock_dx_ - params::DROP_LEAD_X, lock_dy_);
                //   未补偿的真实偏差，仅用于日志对照(看飞机实际对得准不准)
                const double d_raw = std::hypot(lock_dx_, lock_dy_);

                // ── 5) 投掷判定 ──
                //   ★用非阻塞版 arduino_send_async★：这是飞行中(OFFBOARD)，阻塞版会
                //   usleep 5×20ms + tcdrain ≈ 80ms，主循环停转、setpoint 流中断，
                //   飞控可能判"没收到 setpoint"退出 OFFBOARD。异步版只排队，纳秒级返回，
                //   实际字节由每拍的 arduino_.pump() 写出。
                //   drop_sent_ 保证★只排队一次★。
                if (!drop_sent_ && d_cv <= params::DROP_DIST) {
                    drop_sent_ = true;                       // 先置位：确保只触发一次
                    arduino_send_async(params::DROP_CMD);
                    drop_hold_until_ = now_t + rclcpp::Duration::from_seconds(params::DROP_HOLD_SEC);
                    drop_hold_valid_ = true;
                    RCLCPP_INFO(get_logger(),
                        "[投掷] 补偿后距离 %.3fm ≤ %.2fm → 已发 \"%s\"；"
                        "(实际偏差 dx=%.3f dy=%.3f 未补偿距离 %.3fm，前置补偿 %.2fm)"
                        "保持高度 %.2fm，再锁定 %.1fs 后返航",
                        d_cv, params::DROP_DIST, params::DROP_CMD,
                        lock_dx_, lock_dy_, d_raw, params::DROP_LEAD_X,
                        lock_z_, params::DROP_HOLD_SEC);
                } else if (!drop_sent_) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "[锁定] dx=%.3f dy=%.3f | 补偿后距离 %.3fm(阈值 %.2fm，"
                        "前置补偿 %.2fm；未补偿 %.3fm) | 高度 %.2fm(离起飞点 %.2fm) "
                        "yaw=%.1f°%s",
                        lock_dx_, lock_dy_, d_cv, params::DROP_DIST,
                        params::DROP_LEAD_X, d_raw,
                        lock_z_, lock_z_ - drone_.home_z(), tgt_yaw_deg,
                        car_ok ? "" : "(雷达yaw无数据，保持机头)");
                }
            } else if (cv_age > params::LOCK_CV_FALLBACK_S && car_ok) {
                // ★视觉长时间丢失(>LOCK_CV_FALLBACK_S) → 回退用小车雷达(UDP)定位★
                //   为什么要回退：小车是会跑的，视觉丢了还原地干等，小车早开走了，
                //   回来也看不见 → 死等。雷达位置精度不如视觉，但足够"继续跟上"，
                //   等飞到小车上方视觉重新看见就自动切回精确锁定(cv_ok 恢复即走上面分支)。
                //   ★仍然不投掷★：雷达精度不足(标定开环)，投了大概率偏，白费一次机会。
                //   高度保持 lock_z_ 不变(本状态本就不改高度)。
                target_pose_slam(cx, cy, lock_z_, tgt_yaw_deg);
                const double alive_age2 = lock_cv_alive_valid_
                    ? (now_t - lock_cv_alive_time_).seconds() : 1e9;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[锁定] 已 %.1fs 没看到目标(>%.1fs) → ★回退雷达定位★ 跟到小车@(%.2f,%.2f)"
                    "；暂不投掷，等视觉恢复(视觉进程%s)",
                    cv_age, params::LOCK_CV_FALLBACK_S, cx, cy,
                    (lock_cv_alive_valid_ && alive_age2 <= 1.0)
                        ? "在跑，只是没识别到" : "★无心跳★");
            } else {
                // 视觉短暂丢帧(LOCK_CV_TIMEOUT_S ~ LOCK_CV_FALLBACK_S)，或雷达也没数据：
                //   ★锁住当前位置悬停等★。高度目标仍然有效(当前水平位置 + 目标高度)，
                //   避免"什么都不做"导致 PD 继续冲向过期目标点。
                target_pose_slam(drone_.current_x(), drone_.current_y(),
                                 lock_z_, tgt_yaw_deg);
                // ★区分"视觉挂了"和"视觉活着但没看到目标"★：视觉端没看到目标也会写
                //   空帧推进 seq，所以 alive 心跳还在 = 程序在跑、只是没识别到。
                const double alive_age = lock_cv_alive_valid_
                    ? (now_t - lock_cv_alive_time_).seconds() : 1e9;
                const bool cv_alive = lock_cv_alive_valid_ && alive_age <= 1.0;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[锁定] 已 %.2fs 没看到目标(>%.2fs) → 原地锁定等；"
                    "视觉进程%s%s",
                    cv_age > 1e8 ? -1.0 : cv_age, params::LOCK_CV_TIMEOUT_S,
                    cv_alive ? "在跑(只是没识别到目标：目标出视野? 太小? 光照?)"
                             : "★无心跳(程序挂了/没启动/shm 没写)★",
                    car_ok ? "；再丢下去将回退雷达定位"
                           : "；★雷达也无数据，无法回退★");
            }

            // ── 6) ★★★ 必投倒计时：到点无条件投掷 ★★★ ──
            //   ★这段刻意放在上面所有 if/else 分支【之外】★，所以它与数据状态完全无关：
            //     视觉看不看得见、视觉进程死没死、雷达有没有接管 —— 一律不影响。
            //     纯挂钟计时(从切入 LOCK_DROP 起算)，到点就投。
            //   ★不要求 cv_ok★：视觉挂了照投，这正是本分支存在的意义。
            //   只在【还没投】时判：已经投了(正常对准投的)就交给下面的 DROP_HOLD_SEC。
            //   投完走与正常投掷完全相同的收尾：追加锁定 DROP_HOLD_SEC → 返航。
            if (!drop_sent_ && params::LOCK_TIMEOUT_SEC > 0.0 && lock_start_valid_ &&
                (now_t - lock_start_).seconds() > params::LOCK_TIMEOUT_SEC) {
                drop_sent_ = true;                       // 先置位：确保只触发一次
                arduino_send_async(params::DROP_CMD);    // 非阻塞(飞行中不能阻塞主循环)
                drop_hold_until_ = now_t + rclcpp::Duration::from_seconds(params::DROP_HOLD_SEC);
                drop_hold_valid_ = true;
                // 注：视觉不可用时 lock_dx_/dy_ 是旧值，打 -1 表示"未知"，
                //   ★不要用 std::to_string(...).c_str()★——临时 string 在 printf 读到
                //   之前就析构了，是悬垂指针(UB)。
                RCLCPP_WARN(get_logger(),
                    "[投掷] ★必投倒计时 %.1fs 到 → 无条件投掷★ 已发 \"%s\"；"
                    "当时视觉%s、雷达%s，补偿后距离 %.3fm(阈值 %.2fm，前置补偿 %.2fm)；"
                    "再锁定 %.1fs 后返航",
                    params::LOCK_TIMEOUT_SEC, params::DROP_CMD,
                    cv_ok ? "可用" : "不可用",
                    car_ok ? "有数据" : "无数据",
                    // 与投掷判定同一口径(补偿后)，便于对照"差多少没投上"
                    cv_ok ? std::hypot(lock_dx_ - params::DROP_LEAD_X, lock_dy_) : -1.0,
                    params::DROP_DIST, params::DROP_LEAD_X, params::DROP_HOLD_SEC);
            }

            // ── 7) 投掷后多锁定 DROP_HOLD_SEC → 返航 ──
            if (drop_sent_ && drop_hold_valid_ && now_t >= drop_hold_until_) {
                return_z_   = drone_.current_z();        // ★以当前高度返航★(不额外爬升)
                return_yaw_ = drone_.current_yaw_deg();  // 冻结朝向(返航途中不主动转头)
                state_ = MissionState::RETURN_HOME_DROP;
                RCLCPP_INFO(get_logger(),
                    "[投掷] 追加锁定 %.1fs 完成 → 以当前高度 %.2fm 返回起飞点 (0,0)",
                    params::DROP_HOLD_SEC, return_z_);
            }
            break;
        }

        // ─── 【投掷后返航】以当前高度飞回 (0,0) → 降落 ───
        //   ★高度保持不变★(用进入时记录的 return_z_)；朝向不强制(保持当前机头)。
        case MissionState::RETURN_HOME_DROP:
            // ★显式锁住返航高度 return_z_★(=投掷完那一刻的高度)，而不是只调
            //   target_xy_slam。后者不碰 target_z_，高度靠"上一次谁设的"延续——
            //   那是隐式依赖：LOCK_DROP 最后一拍设的正好是 lock_z_，能用但很脆弱
            //   (以后谁在中间插一个改高度的动作就悄悄变了)。用 target_pose_slam
            //   把位置+高度+朝向一次写全，语义明确。
            //   ★yaw 用进入本状态时冻结的 return_yaw_★，不能每拍传 current_yaw_deg()：
            //   那样 target_yaw_ 永远等于当前朝向，yaw 误差恒 0 —— 等于不控偏航，
            //   机头会随气流/PD 侧滑慢慢转过去而没人纠正。
            target_pose_slam(0.0, 0.0, return_z_, return_yaw_);
            if (is_reached()) {
                RCLCPP_INFO(get_logger(),
                    "[返航] 已回到起飞点 (0,0)，高度 %.2fm → 降落", drone_.current_z());
                state_ = MissionState::LAND;
            }
            break;

        // ─── ② 飞到 SLAM 系 (1,0)：高度/朝向保持 ─────────────────
        case MissionState::GO_FORWARD:
            target_xy_slam(1.0, 0.0);
            if (is_reached()) {
                RCLCPP_INFO(get_logger(), "[二次起飞] 已到 SLAM (1.00, 0.00) → 第一次降落");
                state_ = MissionState::LAND_THEN_WAIT;
            }
            break;

        // ─── ③ 第一次降落：触底 + 上锁 → 转地面待机(★不结束任务★) ──
        case MissionState::LAND_THEN_WAIT:
            land();
            if (is_reached()) {          // LAND 到位判定 = 高度触底 且 已上锁
                RCLCPP_INFO(get_logger(),
                    "[二次起飞] 第一次降落完成（已上锁）→ 地面待机，等触发命令");
                // 回 IDLE：★彻底停发 setpoint★(地面上最安全)，同时 log_progress 不再刷降落日志。
                //   二次起飞时 REARM 里 takeoff() 会重新把 setpoint 流起来。
                drone_.stop();
                state_ = MissionState::WAIT_TRIGGER;
            }
            break;

        // ─── ④ 地面待机：等 /mission/takeoff_again（收到一次即可）──
        // ★地面待机，等第二段任务的触发命令★
        //   UDP 模式(CMD_USE_UDP=true，当前)：等 ★命令 2★(CMD_TAKEOFF_AGAIN)，
        //   与命令 1(启动)★同一个端口 9871★，对方同样 1s 发一次，收到一次即锁定。
        //   ROS 话题模式：等 /mission/takeoff_again。
        case MissionState::WAIT_TRIGGER:
            if (trigger_recv_) {
                RCLCPP_INFO(get_logger(),
                    "[二段] 收到命令 2 → 切 OFFBOARD + 解锁，起飞后★直接进追踪(不悬停)★");
                state_ = MissionState::REARM;
            } else if (params::CMD_USE_UDP) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                    "[二段] 地面待机中，等 ★UDP 命令 2★(端口 %d，与命令1同端口)："
                    "在另一台机器上跑 udp_cmd_send <本机IP> again",
                    params::CMD_UDP_PORT);
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                    "[二段] 地面待机中，等触发命令："
                    "ros2 topic pub --once /mission/takeoff_again std_msgs/msg/Bool \"{data: true}\"");
            }
            break;

        // ─── ⑤ 二次起飞前置：起 setpoint 流 → 切 OFFBOARD → 解锁 ──
        //   ★顺序与 BOOT_CHECK 一致（先模式后解锁）★，区别是 OFFBOARD 这次由程序自己请求
        //   （第一次是飞手手动拨的，落地后没人再去拨）。切 OFFBOARD 前飞控通常要求已有
        //   setpoint 流，故先 takeoff() 把流起来、预热 REARM_PREHEAT_S 再请求。
        //   ★此期间尚未解锁，桨不转、飞机不会动★；解锁成功的那一拍 PD 已在命令爬升。
        case MissionState::REARM: {
            constexpr double REARM_PREHEAT_S = 0.5;   // setpoint 流预热时长 (s)

            if (!rearm_inited_) {
                // ★必须有雷达位姿才敢重记 home★：位姿丢了 capture_home() 只打告警不改 home，
                //   会退化成拿第一次的 home(原点) 当起飞目标 → 边爬边横拉。所以在这等它回来。
                if (!drone_.has_pose()) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[二次起飞] 无雷达位姿，暂不起飞（等 SLAM 恢复）");
                    break;
                }
                // ★把落地点记为新 home★：takeoff() 飞的是 home 正上方，不重记则会拿第一次的
                //   home(原点) 当目标 → 一边爬升一边横拉回原点，贴地拖行很危险。
                //   注意"飞回原点"用的是 target_xy_slam(0,0) 绝对坐标，不受 home 影响。
                drone_.capture_home();
                drone_.reset_arm_offboard_retry();    // 复位第一次起飞用掉的重试计数
                rearm_start_  = now();
                rearm_inited_ = true;
            }

            // ★与 TAKEOFF_AGAIN 的 takeoff() 必须同一个高度★(这里定目标、那里续飞同一段爬升)。
            //   也建议与第一次起飞 TAKEOFF 的高度一致，改高度记得三处一起改。
            takeoff(1.5);   // 每拍下起飞目标 → drone_.tick() 持续发 setpoint（= 切 OFFBOARD 所需的流）

            if ((now() - rearm_start_).seconds() < REARM_PREHEAT_S) break;   // 等流跑起来再切

            if (!drone_.is_offboard()) {
                if (!drone_.request_offboard()) {
                    RCLCPP_ERROR(get_logger(), "[二次起飞] 切 OFFBOARD 失败，任务中止");
                    drone_.stop();
                    state_ = MissionState::FINISHED;
                }
                break;
            }
            if (!drone_.is_armed()) {
                if (!drone_.request_arm()) {
                    RCLCPP_ERROR(get_logger(), "[二次起飞] 解锁失败，任务中止");
                    drone_.stop();
                    state_ = MissionState::FINISHED;
                }
                break;
            }
            RCLCPP_INFO(get_logger(), "[二次起飞] OFFBOARD + 解锁完成 → 爬升到 1.5m");
            state_ = MissionState::TAKEOFF_AGAIN;
            break;
        }

        // ─── ⑥ 二次起飞：爬升到（新 home 上方）1.5m ──────────────
        case MissionState::TAKEOFF_AGAIN:
            takeoff(1.5);      // ★须与 REARM 里的 takeoff() 同高度★
            if (is_reached()) {
                // 与第一次起飞在 WAIT_AFTER_TAKEOFF 出口的处理对称：退出起飞段位置环，
                //   之后 GO_HOME 走速度环 PD。TAKEOFF_POSITION_MODE=false 时本调用是空操作。
                drone_.exit_takeoff_position_mode();
                // ★分岔★：第二段任务(降落到移动平台)走 TRACK_CAR2；
                //   若第二段已跑完(mission2_done_)或功能关掉了，就按老流程直接回原点。
                if (params::MISSION2_ENABLE && !mission2_done_) {
                    // ★不悬停，直接进追踪★(需求)。锁定追踪高度 = 此刻高度。
                    car_.begin(drone_.current_z());
                    catch_timer_      = 0.0;      // 第二段追上计时独立起算
                    catch_tick_valid_ = false;
                    RCLCPP_INFO(get_logger(),
                        "[二段] 起飞到 1.5m 完成 → ★直接进追踪(不悬停)★；"
                        "追上确认时长 %.0fs(第二段专用)", params::CATCH_HOLD_SEC_2);
                    state_ = MissionState::TRACK_CAR2;
                } else {
                    RCLCPP_INFO(get_logger(), "[平台起飞] 起飞完成 → 飞回 SLAM 原点 (0,0)");
                    state_ = MissionState::GO_HOME;
                }
            }
            break;

        // ─── ⑦ 飞回 SLAM 原点 (0,0) → 降落收尾 ──────────────────
        // ═══════════════════════════════════════════════════════════════
        //  【二段追踪】TRACK_CAR2 —— 与 TRACK_CAR 行为相同，只有两点不同：
        //    ① 追上确认时长用 ★CATCH_HOLD_SEC_2★(21s) 而非 CATCH_HOLD_SEC
        //    ② 追上后转 TRACK_LAND(边追边降)，而不是 LOCK_DROP(投掷)
        //  单独建一个状态而不复用 TRACK_CAR：两段的出口和时长都不同，共用会互相干扰。
        // ═══════════════════════════════════════════════════════════════
        case MissionState::TRACK_CAR2: {
            constexpr double CAR_LOST_HOLD_SEC = 3600.0;   // 同 TRACK_CAR：丢数据就悬停等

            double tx, ty, tz, tyaw;
            if (car_.latest(tx, ty, tz, tyaw)) {
                target_pose_slam(tx, ty, tz, tyaw);

                const double d_xy = std::hypot(tx - drone_.current_x(),
                                               ty - drone_.current_y());
                const rclcpp::Time now_t = now();
                if (d_xy <= params::CATCH_DIST) {
                    if (catch_tick_valid_) catch_timer_ += (now_t - catch_last_tick_).seconds();
                    catch_last_tick_  = now_t;
                    catch_tick_valid_ = true;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                        "[二段追上] 距离 %.2fm ≤ %.2fm，已累计 %.1f/%.1fs",
                        d_xy, params::CATCH_DIST, catch_timer_, params::CATCH_HOLD_SEC_2);
                } else {
                    // 与第一段同口径：跑出距离只暂停累加，不清零
                    catch_tick_valid_ = false;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                        "[二段追踪] 小车@(%.2f,%.2f) 距离 %.2fm(未追上，已累计 %.1fs)",
                        tx, ty, d_xy, catch_timer_);
                }

                if (catch_timer_ >= params::CATCH_HOLD_SEC_2) {
                    plat_z_ = drone_.current_z();      // 下降起点 = 当前高度
                    plat_tick_valid_   = false;        // 下降积分从下一拍起算
                    plat_start_        = now_t;        // 下降段超时起算
                    plat_start_valid_  = true;
                    state_ = MissionState::TRACK_LAND;
                    plat_last_z_       = plat_z_;      // 接触检测的上一拍高度初值
                    plat_touch_valid_  = false;
                    plat_descending_   = false;        // ★必须重置★：要重新确认降起来了
                    plat_z_hist_.clear();              // 清空滑窗(必须重新攒)
                    // ★放开垂直限速★：边追边降用 MOVE_POSE，默认会被平飞档 0.1m/s
                    //   卡死(降极慢 + 降速永远达不到接触检测门槛 → 永不锁桨)。
                    //   离开本状态的每条路径都必须复位(见 TRACK_LAND 的两个出口)。
                    drone_.set_plat_descend_mode(true);
                    RCLCPP_INFO(get_logger(),
                        "[二段追上] 累计 %.1fs ≥ %.1fs → ★边追踪边下降★"
                        "(从 %.2fm[离起飞点 %.2fm] 起降，速率 %.2fm/s；"
                        "★上锁闸门：离起飞点 ≤%.2fm 才允许锁桨★，"
                        "闸门内再看降速<%.2fm/s 持续%.2fs 判接触；超时 %.0fs)",
                        catch_timer_, params::CATCH_HOLD_SEC_2, plat_z_,
                        plat_z_ - drone_.home_z(), params::PLAT_DESCEND_SPD,
                        params::PLAT_DISARM_MAX_H_REL, params::PLAT_TOUCH_VZ,
                        params::PLAT_TOUCH_HOLD_S, params::PLAT_DESCEND_TIMEOUT_S);
                }
            } else {
                wait_time(CAR_LOST_HOLD_SEC);          // 同 TRACK_CAR：锁住当前位姿
            }
            break;
        }

        // ═══════════════════════════════════════════════════════════════
        //  【边追边降】TRACK_LAND —— 水平跟小车 + 高度匀速下降 → 到平台上方主动上锁
        //
        //  方案(用户选定：匀速下降)：目标高度按 PLAT_DESCEND_SPD 匀速往下走，
        //    水平/偏航仍然用小车雷达实时跟踪。★不判水平是否对准★——对准与否都在降。
        //
        //  ★为什么不用 AUTO.LAND★：AUTO.LAND 期间飞控自己控高且不再跟踪移动平台，
        //    平台一跑飞机就落到地上了。所以必须自己控高 + 自己判断落到了。
        //
        //  ★★★ 怎么判断"落到平台上了"：高度闸门 + 接触检测(两道条件都要满足) ★★★
        //    ① ★硬性高度闸门★ PLAT_DISARM_MAX_H_REL：离起飞点高于这条线【绝不上锁】。
        //       比较的是 (current_z - home_z)，所以雷达每次启动原点在哪都无所谓 ——
        //       偏移被 home_z 吸收，判据始终是"离起飞地面多高"。
        //       ★这是安全底线★：速度判定万一误判，只要还在高处就锁不了桨。
        //    ② 闸门以内再看接触：目标高度持续下压，同时看【实际高度还降不降】：
        //       · 还在降(降速 ≥ PLAT_TOUCH_VZ) → 悬在空中，继续压
        //       · ★降不动了★ 连续 PLAT_TOUCH_HOLD_S 秒 → 被平台托住 → 主动上锁
        //    另有 plat_descending_ 门控：必须先真的降起来过，防"刚进本状态还在悬停
        //       (vz≈0)就被判成已接触"。
        //    ★为什么不能只用"降到某个高度就上锁"★：那要靠人工量的平台高度，量错
        //    十几厘米就会"从高处摔"或"撞进平台"；而接触检测测的是"物理上还能不能继续
        //    下降"，免疫平台高度未知。两者结合：闸门保安全，接触检测保精度。
        //
        //  ★两条出口★：
        //    · 正常：检测到接触 → 主动上锁 → PLAT_WAIT
        //    · 兜底：PLAT_DESCEND_TIMEOUT_S 超时 → 放弃降落，直接飞回起点降落
        //      (小车乱跑/水平总追不上时，不能无限期悬着)
        // ═══════════════════════════════════════════════════════════════
        case MissionState::TRACK_LAND: {
            const rclcpp::Time now_t = now();

            // ── 1) 目标高度匀速往下压 + ★接触检测★ ──
            //   接触判据：目标一直在往下压，但【实际高度不再下降】= 已被平台托住。
            //   ★完全不依赖平台高度和 home_z★，因此不受 SLAM 漂移影响(见 params 说明)。
            const double cur_z = drone_.current_z();
            bool touched = false;
            if (plat_tick_valid_) {
                const double dt = (now_t - plat_tick_).seconds();
                if (dt > 0.0 && dt < 0.5) {        // dt 异常(卡顿/时间跳变)时本拍不处理
                    // 目标持续下压。★下限跟着"实际高度"走★，始终保持在实际高度下方
                    //   PLAT_PUSH_LIMIT 处 —— 这样：
                    //   · 空中下降时：目标始终领先实际一点，PD 有稳定的向下动力
                    //   · 接触后不降了：目标被钉在 (接触高度 - PUSH_LIMIT)，不会无限
                    //     往下累积 → 推力有上界，不会把平台压坏或把飞机顶翻
                    //   ★不能只在进入时算一次★：那样下降 1m 后目标会落后实际很多，
                    //   PD 反而在往上拉，降不下去。
                    plat_z_ -= params::PLAT_DESCEND_SPD * dt;
                    const double floor_z = cur_z - params::PLAT_PUSH_LIMIT;
                    if (plat_z_ < floor_z) plat_z_ = floor_z;

                    // ★★★ 实际下降速率：用【滑动窗口】而不是单拍差分 ★★★
                    //   ★为什么不能用单拍差分★(原来的 (last_z-cur_z)/dt，实测导致
                    //   "落到平台了却不锁桨")：dt=0.02s，SLAM 高度噪声哪怕只有 ±1cm，
                    //   算出的假速度就是 0.01/0.02 = 0.5 m/s —— 是阈值 0.05 的【10 倍】。
                    //   于是飞机明明已经停在平台上，每一拍的抖动都让 vz 忽大忽小，
                    //   只要有一拍超过阈值就把接触计时清零 → 永远凑不满 HOLD 时长 →
                    //   永远不锁桨 → 进不了 5 秒倒计时。
                    //   ★滑动窗口★：拿 PLAT_VZ_WIN_S 秒前的高度和现在比，
                    //   噪声被时间跨度稀释(同样 ±1cm 噪声，0.3s 窗口只产生
                    //   0.01/0.3 = 0.033 m/s 假速度，落在阈值以内)。
                    plat_z_hist_.push_back({now_t, cur_z});
                    // 丢掉超出窗口的旧样本(保留至少 2 个才能算速度)
                    while (plat_z_hist_.size() > 2 &&
                           (now_t - plat_z_hist_.front().first).seconds() > params::PLAT_VZ_WIN_S) {
                        plat_z_hist_.pop_front();
                    }
                    double vz_down = 0.0;
                    bool   vz_valid = false;
                    if (plat_z_hist_.size() >= 2) {
                        const double span = (now_t - plat_z_hist_.front().first).seconds();
                        // 窗口还没攒够时间就先不判(否则又退化成短跨度=噪声放大)
                        if (span >= params::PLAT_VZ_WIN_S * 0.5) {
                            vz_down  = (plat_z_hist_.front().second - cur_z) / span;
                            vz_valid = true;
                        }
                    }
                    // ★速度算不出来时只跳过"判定"，绝不 break 出 case★——
                    //   下面还要 target_pose_slam() 发 setpoint，跳掉会中断 OFFBOARD 流。
                    if (!vz_valid) {
                        plat_touch_valid_ = false;   // 不累计接触，等窗口攒满
                    } else {

                    // ★★★ 必须先"真的降起来过"，才允许接触判定 ★★★
                    //   否则：刚进本状态时飞机还在悬停(垂直速度≈0) → vz_down≈0
                    //   → 立刻满足"降不动" → 0.5s 后★在 1.5m 高空上锁★ = 摔机。
                    //   ★判据用独立参数 PLAT_DESCEND_MIN_VZ，不要用 PLAT_DESCEND_SPD*0.5★：
                    //   PLAT_DESCEND_SPD 是"限速/目标压降速率"，不等于实际能降多快
                    //   (受载重、PD 参数、飞控自身限制影响)。若拿它推算门槛，一旦实际
                    //   降速达不到，这道门就永远过不去 → ★接触检测永不启用 → 永不锁桨★。
                    //   用一个明确的小绝对值，只要"确实在往下走"就放行。
                    if (!plat_descending_) {
                        if (vz_down > params::PLAT_DESCEND_MIN_VZ) {
                            plat_descending_ = true;
                            RCLCPP_INFO(get_logger(),
                                "[边追边降] 已确认开始下降(降速 %.3f > %.3f m/s) → 接触检测启用",
                                vz_down, params::PLAT_DESCEND_MIN_VZ);
                        } else {
                            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                "[边追边降] 等待下降建立(当前降速 %.3f，需 >%.3f m/s)"
                                "；★接触检测尚未启用(防高空误判上锁)★"
                                "。若一直卡在这里：垂直限速是否没放开? 载重过大?",
                                vz_down, params::PLAT_DESCEND_MIN_VZ);
                        }
                    }

                    // ★★★ 硬性高度闸门：高于这条线【绝不上锁】★★★
                    //   相对起飞点的高度 = cur_z - home_z()，所以雷达每次启动原点在哪
                    //   都无所谓(偏移被 home_z 吸收)，比较的始终是"离起飞地面多高"。
                    //   闸门之上：即使速度判定说"降不动了"也不上锁，只打日志。
                    //   ⇒ 速度判定误判的最坏后果从"高空摔机"降为"降不下去→超时返航"。
                    const double h_rel = cur_z - drone_.home_z();
                    const bool below_gate = (h_rel <= params::PLAT_DISARM_MAX_H_REL);
                    if (!below_gate && vz_down < params::PLAT_TOUCH_VZ) {
                        // 在闸门之上却"降不动"：不是接触，是真的降不下去(风顶/推力不足/
                        //   目标压不下去)。绝不上锁，只提示——最终由下降超时兜底。
                        plat_touch_valid_ = false;
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                            "[边追边降] 降速 %.3f<%.2f 但★高度 %.2fm 仍高于上锁闸门 %.2fm"
                            "→ 绝不上锁★(风顶? 推力不足? 平台高度超过闸门?)",
                            vz_down, params::PLAT_TOUCH_VZ, h_rel,
                            params::PLAT_DISARM_MAX_H_REL);
                    }
                    if (below_gate && plat_descending_ && vz_down < params::PLAT_TOUCH_VZ) {
                        // 降不动了 → 开始/继续累计接触时长
                        if (!plat_touch_valid_) {
                            plat_touch_start_ = now_t;
                            plat_touch_valid_ = true;
                        }
                        const double held = (now_t - plat_touch_start_).seconds();
                        if (held >= params::PLAT_TOUCH_HOLD_S) touched = true;
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
                            "[边追边降] ★降不动了★(降速 %.3f<%.2f m/s，"
                            "高度 %.2fm 已在闸门 %.2fm 以内) 已持续 %.2f/%.2fs → %s",
                            vz_down, params::PLAT_TOUCH_VZ, h_rel,
                            params::PLAT_DISARM_MAX_H_REL, held,
                            params::PLAT_TOUCH_HOLD_S,
                            touched ? "判定已接触平台" : "继续确认");
                    } else {
                        plat_touch_valid_ = false;   // 又开始降了 → 重新计时
                    }
                    }   // end else(vz_valid)
                }
            }
            plat_last_z_     = cur_z;
            plat_tick_       = now_t;
            plat_tick_valid_ = true;

            // ── 2) 水平/偏航：继续跟小车；高度用上面算的 plat_z_ ──
            double tx, ty, tz, tyaw;
            if (car_.latest(tx, ty, tz, tyaw)) {
                target_pose_slam(tx, ty, plat_z_, tyaw);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "[边追边降] 小车@(%.2f,%.2f) 目标高度 %.2fm 实际 %.2fm"
                    "(离起飞点 %.2fm) 水平差 %.2fm",
                    tx, ty, plat_z_, cur_z, cur_z - drone_.home_z(),
                    std::hypot(tx - drone_.current_x(), ty - drone_.current_y()));
            } else {
                // 雷达没数据：水平锁住当前位置，★高度继续降★(高度不依赖小车数据)
                target_pose_slam(drone_.current_x(), drone_.current_y(),
                                 plat_z_, drone_.current_yaw_deg());
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[边追边降] 小车雷达无数据 → 水平锁住当前位置，高度继续下压"
                    "(目标 %.2fm 实际 %.2fm)", plat_z_, cur_z);
            }

            // ── 3) 判定已接触平台 → ★主动上锁★(桨停，落在平台上) ──
            //   ★第二道独立高度闸门★(与上面那道重复是刻意的)：上锁是不可逆的危险动作，
            //   紧挨着 request_disarm() 再查一次高度，即使以后有人重构上面的判定逻辑、
            //   不小心漏掉闸门，这里也拦得住。宁可多一次比较，不冒高空锁桨的风险。
            if (touched &&
                (drone_.current_z() - drone_.home_z()) <= params::PLAT_DISARM_MAX_H_REL) {
                drone_.request_disarm();               // 每 0.5s 重试直到成功
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[边追边降] ★已接触平台(实际高度 %.2fm 不再下降) → 主动上锁★",
                    cur_z);
                if (!drone_.is_armed()) {              // 确认上锁成功才往下走
                    drone_.set_plat_descend_mode(false);  // ★复位垂直限速★(出口1)
                    drone_.stop();                     // 停发 setpoint(已在平台上)
                    plat_wait_start_       = now_t;
                    plat_wait_start_valid_ = true;
                    state_ = MissionState::PLAT_WAIT;
                    RCLCPP_INFO(get_logger(),
                        "[平台] ★已落在移动平台上并上锁★(高度 %.2fm，离起飞点 %.2fm)"
                        " → 等 %.0fs 后自动重新起飞",
                        cur_z, cur_z - drone_.home_z(), params::PLAT_WAIT_SEC);
                }
                break;
            }

            // ── 4) 下降段超时兜底 → 放弃降落，回起点 ──
            if (params::PLAT_DESCEND_TIMEOUT_S > 0.0 && plat_start_valid_ &&
                (now_t - plat_start_).seconds() > params::PLAT_DESCEND_TIMEOUT_S) {
                drone_.set_plat_descend_mode(false);   // ★复位垂直限速★(出口2：超时返航)
                mission2_done_ = true;                 // 标记第二段已尝试过，别再循环
                RCLCPP_ERROR(get_logger(),
                    "[边追边降] ★超时 %.0fs 未检测到接触平台★"
                    "(实际高度 %.2fm，离起飞点 %.2fm；小车乱跑? 水平追不上? "
                    "PLAT_TOUCH_VZ 设太小?) → 放弃落平台，飞回起点降落",
                    params::PLAT_DESCEND_TIMEOUT_S, cur_z, cur_z - drone_.home_z());
                state_ = MissionState::GO_HOME;
            }
            break;
        }

        // ─── 【平台待机】落在平台上等 PLAT_WAIT_SEC 秒 → 自己重新起飞 ───
        //   ★此时已上锁、桨不转★，所以不发 setpoint(drone_.stop() 已停)。
        //   平台可能还在动，飞机跟着平台一起走——这段时间不做任何控制。
        case MissionState::PLAT_WAIT:
            if (!plat_wait_start_valid_) {             // 防御：正常不会走到
                plat_wait_start_ = now();
                plat_wait_start_valid_ = true;
            }
            if ((now() - plat_wait_start_).seconds() >= params::PLAT_WAIT_SEC) {
                // ★复位二次起飞的一次性标志★：REARM/TAKEOFF_AGAIN 要再走一遍，
                //   不复位的话 rearm_inited_ 还是 true，会跳过 capture_home 等初始化。
                rearm_inited_ = false;
                mission2_done_ = true;                 // ★标记第二段已完成★：
                                                       //   之后 TAKEOFF_AGAIN 走"回原点"，
                                                       //   LAND 完成后进 FINISHED 而不是再等命令
                RCLCPP_INFO(get_logger(),
                    "[平台] 等待 %.0fs 完成 → 重新切 OFFBOARD + 解锁 + 起飞到 1.5m",
                    params::PLAT_WAIT_SEC);
                state_ = MissionState::PLAT_TAKEOFF;
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[平台] 平台上待机中 %.1f/%.0fs（已上锁，桨不转）",
                    (now() - plat_wait_start_).seconds(), params::PLAT_WAIT_SEC);
            }
            break;

        // ─── 【平台起飞】与 REARM 完全同一套流程，只是日志前缀不同 ───
        //   ★不重记 home★(用户选定)：1.5m 仍以【最初起飞点】为基准，所以飞机实际
        //   只会从平台爬升 (1.5 - 平台高度) 那么多。好处是后面"飞回原点"的坐标系一致。
        case MissionState::PLAT_TAKEOFF: {
            constexpr double PREHEAT_S = 0.5;          // setpoint 流预热(同 REARM)

            if (!rearm_inited_) {
                if (!drone_.has_pose()) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[平台起飞] 无雷达位姿，暂不起飞（等 SLAM 恢复）");
                    break;
                }
                // ★这里刻意不调 capture_home()★：保持最初起飞点为基准(见上面说明)
                drone_.reset_arm_offboard_retry();
                rearm_start_  = now();
                rearm_inited_ = true;
            }

            takeoff(1.5);      // 每拍下目标 → 持续发 setpoint(= 切 OFFBOARD 所需的流)

            if ((now() - rearm_start_).seconds() < PREHEAT_S) break;

            if (!drone_.is_offboard()) {
                if (!drone_.request_offboard()) {
                    RCLCPP_ERROR(get_logger(), "[平台起飞] 切 OFFBOARD 失败，任务中止");
                    drone_.stop();
                    state_ = MissionState::FINISHED;
                }
                break;
            }
            if (!drone_.is_armed()) {
                if (!drone_.request_arm()) {
                    RCLCPP_ERROR(get_logger(), "[平台起飞] 解锁失败，任务中止");
                    drone_.stop();
                    state_ = MissionState::FINISHED;
                }
                break;
            }
            RCLCPP_INFO(get_logger(), "[平台起飞] OFFBOARD + 解锁完成 → 爬升到 1.5m");
            state_ = MissionState::TAKEOFF_AGAIN;      // 复用：那里会因 mission2_done_ 走回原点
            break;
        }

        case MissionState::GO_HOME:
            target_xy_slam(0.0, 0.0);
            if (is_reached()) {
                RCLCPP_INFO(get_logger(), "[返航] 已回到 SLAM 原点 (0,0) → 降落");
                state_ = MissionState::LAND;
            }
            break;

        // ─── 最后：降落 ──────────────────────────────────────────
        // ★LAND 被两段任务共用★：靠 mission2_done_ 区分是"第一段降落完(还要等命令2
        //   跑第二段)"还是"第二段也跑完了(真结束)"。
        case MissionState::LAND:
            land();
            if (is_reached()) {          // LAND 到位 = 触底 且 已上锁
                if (params::MISSION2_ENABLE && !mission2_done_) {
                    RCLCPP_INFO(get_logger(),
                        "[降落] 第一段任务完成（已上锁）→ 地面待机，"
                        "★等 UDP 命令 2 开始第二段(降落到移动平台)★");
                    drone_.stop();       // 彻底停发 setpoint(地面上最安全)
                    state_ = MissionState::WAIT_TRIGGER;
                } else {
                    RCLCPP_INFO(get_logger(), "[降落] 完成");
                    state_ = MissionState::FINISHED;
                }
            }
            break;

        case MissionState::FINISHED:
            RCLCPP_INFO_ONCE(get_logger(), "========== 任务结束 ==========");
            break;
        }

        // ★遥测上报★：放在 switch【之后】——这样上报的是本拍状态机刚决定的状态，
        //   而不是上一拍的旧状态(状态切换那一拍的编码才准确)。
        //   自带限频(TLM_RATE_HZ)，非阻塞发送，正常时不打印任何日志。
        telemetry_tick();
    }

    // ── 成员 ──
    DroneController              drone_;
    FindFigure                   find_;
    WaypointReceiver             recv_;
    LineFollower                 lf_;
    RingDriller                  rd_;
    CarTracker                   car_;
    WaypointRunner               runner_;
    rclcpp::TimerBase::SharedPtr timer_;
    MissionState                 state_ = MissionState::BOOT_CHECK;
    MissionState                 state_before_find_ = MissionState::RUN_EXT_WAYPOINTS;  // 进 FINDFIGURE 前的状态(处理完回它)
    bool                         wp_loaded_ = false;   // 当前走线状态是否已装载航点(首拍装一次)

    // BOOT_CHECK 一次性通过的标志（防止重复打 "是"）
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
    // ★收视觉帧那一刻算好并冻结的 SLAM 绝对目标点★(不在状态机每拍重算，见 LOCK_DROP 注释)
    double       lock_tgt_x_ = 0.0, lock_tgt_y_ = 0.0;
    bool         lock_tgt_valid_ = false;
    rclcpp::Time lock_cv_time_;             // 最后一次★看到目标★(n_targets>0)的本机时刻
    bool         lock_cv_valid_ = false;
    // 最后一次读到 shm 新帧的本机时刻 = ★视觉进程还活着★的心跳(与"看到目标"分开)。
    //   视觉端没看到目标也会写空帧，靠这个才能区分"视觉挂了"和"视觉活着但没看到"。
    rclcpp::Time lock_cv_alive_time_;
    bool         lock_cv_alive_valid_ = false;
    // ★锁定/投掷全程保持的高度(SLAM 绝对 z)★：TRACK_CAR 判定追上那一刻锁成当时高度，
    //   之后不再改动。★不能每拍取 current_z★——那样高度误差恒 0 = 等于不控高度。
    double       lock_z_ = 0.0;
    bool         drop_sent_ = false;        // DIANCI 是否已发(★只发一次★)
    rclcpp::Time drop_hold_until_;          // 投掷后追加锁定的截止时刻
    bool         drop_hold_valid_ = false;
    double       return_z_ = 0.0;           // 返航高度(=投掷完那一刻的当前高度)
    double       return_yaw_ = 0.0;         // 返航朝向(度，进入返航时冻结，途中不主动转头)
    rclcpp::Time lock_start_;               // 进入 LOCK_DROP 的时刻(锁定总超时用)
    bool         lock_start_valid_ = false;

    // ---- 第二段任务：降落到移动平台 ----
    bool         mission2_done_ = false;    // ★第二段是否已跑过★：LAND 靠它判断
                                           //   "还要等命令2" 还是 "真结束"；
                                           //   TAKEOFF_AGAIN 靠它判断进追踪还是回原点
    double       plat_z_ = 0.0;             // 边追边降的目标高度(SLAM 绝对 z)
    rclcpp::Time plat_tick_;                // 上一拍时刻(按时间差积分下降量)
    bool         plat_tick_valid_ = false;
    // ---- 接触检测(判断"已落到平台上") ----
    // ★高度历史窗口★(时刻, 高度)：用 PLAT_VZ_WIN_S 秒的跨度算下降速率，
    //   而不是单拍差分 —— 单拍差分会把 ±1cm 的 SLAM 噪声放大成 0.5m/s 假速度，
    //   导致"落到平台了却永远不锁桨"(2026-08 实测 bug)。
    std::deque<std::pair<rclcpp::Time, double>> plat_z_hist_;
    double       plat_last_z_ = 0.0;        // 上一拍的实际高度(仅日志/兼容用)
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

    // OFFBOARD/Armed 丢失去抖：用单帧异常会被 /mavros/state 低频流误判
    bool         lost_since_valid_ = false;
    rclcpp::Time lost_since_;

    // ── 探索：算法速度命令缓存 + 完成标志 + 终点 ──
    double ext_v_fwd_    = 0.0;
    double ext_v_lat_    = 0.0;
    double ext_yaw_rate_ = 0.0;
    bool   ext_cmd_valid_   = false;   // 是否收到过算法速度
    bool   explore_done_    = false;   // 算法报告探索完成
    bool   explore_entered_ = false;   // EXPLORATION 是否已初始化（锁高+发终点）

    // ── 二次起飞（WAIT_TRIGGER / REARM 用）──
    bool         trigger_recv_ = false;   // 已收到触发命令(收到一次即锁定，重复发忽略)
    bool         rearm_inited_ = false;   // REARM 首拍一次性初始化(重记 home + 复位重试)标志
    rclcpp::Time rearm_start_;            // 进 REARM 的时刻(setpoint 流预热计时)

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              finished_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              trigger_sub_;   // 二次起飞触发频道
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr             start_sub_;     // 任务启动指令频道(仅 ROS 模式创建)
    // 跨机命令 UDP 读端(仅 params::CMD_USE_UDP=true 时创建)。★非线程安全★，
    //   只在 on_timer 单线程里 poll() → 安全。收到 CMD_START 置 start_recv_。
    std::unique_ptr<udp_cmd::UdpCmdReceiver>                          cmd_rx_;
    // ---- 遥测上报(飞机 → 监控端，只发不收) ----
    std::unique_ptr<udp_tlm::UdpTelemetrySender> tlm_tx_;
    double   tlm_last_send_ = 0.0;      // 上次发送时刻(steady 秒)，用于限频
    double   tlm_last_warn_ = 0.0;      // 上次告警时刻，用于节流
    uint32_t tlm_fail_run_  = 0;        // 连续失败计数(恢复即清零)
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr    goal_pub_;
    rclcpp::CallbackGroup::SharedPtr                                  cv_cbg_;   // 视觉订阅独立 Reentrant 组
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            cv_sub_;   // /cv/target_info 统一入口(订阅+parse各一次)

    // ── shm 视觉信箱(第3步·并行验证期：只读来对比，数据源仍是 ROS 话题) ──
    shm::ShmMailboxReader shm_cv_;        // 读端(视觉进程建信箱；没建时 read 恒 false 自动回退)
    uint64_t              shm_last_seq_ = 0;   // 上次读到的帧序号(变大=新帧)

    // Arduino 串口(懒打开：第一次 arduino_send 才 open，没插不影响任何功能)
    ArduinoSerial arduino_{params::ARDUINO_DEV, params::ARDUINO_BOOT_WAIT_S};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FlyMissionNode>();
    // 多线程 executor：让视觉找图订阅(独立 Reentrant 组)在后台线程收帧、攒帧，
    //   不被 20Hz 状态机 timer 阻塞(与算法节点 POI/点云订阅同款)。
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
