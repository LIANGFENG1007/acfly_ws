// ============================================================================
//  fly_mission_node.cpp —— 主控节点：构造(话题/UDP/遥测) + 50Hz 主循环派发 + main
//  ---------------------------------------------------------------------------
//  ★本文件【只做派发】★：on_timer 的 switch 里每个 case 只有几行 —— 调一个
//  step_xxx()、判到位、切状态。所有飞行逻辑在 mission_step_*.cpp 里，见
//  fly_mission_node.hpp 顶部的文件分工说明。
//
//  这样 switch 就是一张"全流程目录"：想知道任务怎么走，看这里；想改某个动作的
//  细节，去对应的 step_xxx()。两件事互不干扰。
// ============================================================================

#include "fly_mission/fly_mission_node.hpp"

#include <nlohmann/json.hpp>
#include <chrono>

namespace fly_mission {

FlyMissionNode::FlyMissionNode()
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
            ext_cmd_time_  = now();   // ★收到时刻★：给 exploration() 判新鲜度用(见那里说明)
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
        //   不会误起飞。收包在 on_timer 里 poll_udp_cmd()。
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
    //   放独立 Reentrant 组，后台收，不阻塞主循环 timer。
    //   ★USE_SHM_CV=true 时不订阅★——视觉结果改从 shm 信箱读(consume_shm_cv())，此为回退老路。
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

// ════════════════════════════════════════════════════════════════════════════
//  50Hz 主循环
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::on_timer()
{
    drone_.tick();

    // ★Arduino 非阻塞发送泵★：把 arduino_send_async() 排队的字节往串口写一点。
    //   非阻塞(O_NONBLOCK + 不 tcdrain/不 usleep)，没东西发时直接返回，零开销。
    arduino_.pump();

    // ★小车位姿收包(UDP 模式必须每拍调)★：UDP 没有回调线程，不主动收就永远没数据
    //   → car_.latest() 恒 false → 追踪时飞机只会悬停。
    //   params::CAR_USE_UDP=false(ROS 话题模式)时本调用是空操作，留着无副作用。
    car_.poll();

    poll_udp_cmd();      // 跨机命令收包(启动指令 / 二段触发)
    consume_shm_cv();    // 视觉 shm 信箱消费(USE_SHM_CV=true 时)

    // 失锁/飞手接管 → 已中止任务，本拍不再跑状态机
    if (check_pilot_takeover()) return;

    check_find_figure_interrupt();   // 视觉找图打断(仅走航点时)

    switch (state_) {

    // ─── 检测：连接 + 雷达 → BEEP① → 等启动指令 → BEEP② → OFFBOARD + 解锁 ───
    case MissionState::BOOT_CHECK:
        if (step_boot_check()) {
            RCLCPP_INFO(get_logger(), "[检测] 全部通过，开始起飞");
            state_ = MissionState::TAKEOFF;
        }
        break;

    case MissionState::TAKEOFF:
        takeoff(1.5);
        if (is_reached()) {
            RCLCPP_INFO(get_logger(), "[起飞] 完成");
            state_ = MissionState::WAIT_AFTER_TAKEOFF;
        }
        break;

    // 起飞后悬停 → 到位则退位置环 + 按选定分支初始化并切走(切哪个在 step 里改)
    case MissionState::WAIT_AFTER_TAKEOFF:
        wait_time(0.5);
        if (is_reached()) step_wait_after_takeoff();
        break;

    case MissionState::EXPLORATION:
        exploration(7.0, 4.0);              // 探索终点 (SLAM 系)，改这里即可
        if (explore_done_) {
            RCLCPP_INFO(get_logger(), "[探索] 算法报告完成，准备降落");
            state_ = MissionState::LAND;
        }
        break;

    case MissionState::FOLLOW_LINE:
        if (lf_.timed_out()) {
            RCLCPP_INFO(get_logger(), "[寻线] 连续丢线超时 → 线走完/丢线，准备降落");
            state_ = MissionState::LAND;
            break;
        }
        step_follow_line();
        break;

    case MissionState::RUN_EXT_WAYPOINTS:
        step_run_ext_waypoints();
        break;

    case MissionState::RUN_WAYPOINTS:
        step_run_waypoints();
        break;

    case MissionState::FINDFIGURE:
        step_find_figure();
        break;

    case MissionState::DRILL_RING:
        step_drill_ring();
        break;

    case MissionState::CIRCLE_AROUND:
        step_circle_around();
        break;

    // ═══════════════════════════════════════════════════════════════════
    //  第一段：悬停 3s → 追踪小车 → 投掷(视觉锁定 或 纯雷达链) → 返航降落
    // ═══════════════════════════════════════════════════════════════════
    case MissionState::HOVER_3S:
        wait_time(3.0);                       // 幂等(同时长重复调不重置计时)；锁住当前位姿悬停
        if (is_reached()) {
            // ★锁定当前高度为追踪高度★(小车在地上跑，它的 z 对飞机没意义)。
            //   ★放在这里而不是 WAIT_AFTER_TAKEOFF★：要锁的是"悬停结束、飞机已稳"
            //   那一刻的高度。CarTracker 不订阅飞机位姿，高度必须由主控传进去。
            car_.begin(drone_.current_z());
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

    case MissionState::TRACK_CAR:
        step_track_car();
        break;

    case MissionState::LOCK_DROP:
        step_lock_drop();
        break;

    case MissionState::RADAR_DESCEND:
        step_radar_descend();
        break;

    case MissionState::RADAR_DROP:
        step_radar_drop();
        break;

    case MissionState::RADAR_CLIMB:
        step_radar_climb();
        break;

    case MissionState::RETURN_HOME_DROP:
        step_return_home_drop();
        break;

    // ═══════════════════════════════════════════════════════════════════
    //  第二段：再起飞 → 追踪小车 → 落到移动平台 → 平台待机 → 起飞回原点
    // ═══════════════════════════════════════════════════════════════════
    case MissionState::TRACK_CAR2:
        step_track_car2();
        break;

    case MissionState::TRACK_LAND:
        step_track_land();
        break;

    case MissionState::PLAT_WAIT:
        step_plat_wait();
        break;

    case MissionState::PLAT_TAKEOFF:
        step_plat_takeoff();
        break;

    // ─── 二次起飞流程(飞到前方 → 降落 → 地面等触发 → 再起飞) ───
    case MissionState::GO_FORWARD:
        target_xy_slam(1.0, 0.0);
        if (is_reached()) {
            RCLCPP_INFO(get_logger(), "[二次起飞] 已到 SLAM (1.00, 0.00) → 第一次降落");
            state_ = MissionState::LAND_THEN_WAIT;
        }
        break;

    case MissionState::LAND_THEN_WAIT:
        step_land_then_wait();
        break;

    case MissionState::WAIT_TRIGGER:
        step_wait_trigger();
        break;

    case MissionState::REARM:
        step_rearm();
        break;

    case MissionState::TAKEOFF_AGAIN:
        step_takeoff_again();
        break;

    case MissionState::GO_HOME:
        target_xy_slam(0.0, 0.0);
        if (is_reached()) {
            RCLCPP_INFO(get_logger(), "[返航] 已回到 SLAM 原点 (0,0) → 降落");
            state_ = MissionState::LAND;
        }
        break;

    // ★LAND 被两段任务共用★：靠 mission2_done_ 区分是"第一段降落完(还要等命令2
    //   跑第二段)"还是"第二段也跑完了(真结束)"。
    case MissionState::LAND:
        if (step_land()) {
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
        // ★进终态必须停控制器★：不停的话 action_mode_ 还留在上一个动作(通常是 LAND)，
        //   而 on_timer 开头每拍都调 drone_.tick()，其 LAND 分支里的 log_progress()
        //   会按 0.2s 一直刷 "[降落] 相对高度 -0.02m" —— 任务都结束了还在刷屏。
        //   (只是刷日志：tick() 的 LAND 分支在发 setpoint 之前就 return，
        //    AUTO.LAND 也有 land_requested_ 守卫只请求一次，不会有指令继续下发。)
        //   ★放在这里而不是各个入口★：进 FINISHED 的路径有 7 条，其中 5 条自己调了
        //   stop()、2 条没调。放在终态里 = 所有路径(含以后新加的)一律覆盖，
        //   不用每加一条出口都记得补 stop()。
        //   先判 action_mode() 再调：stop() 内部要拿一次 mutex，没必要每拍重复。
        if (drone_.action_mode() != ActionMode::IDLE) drone_.stop();
        RCLCPP_INFO_ONCE(get_logger(), "========== 任务结束 ==========");
        break;
    }

    // ★遥测上报★：放在 switch【之后】——这样上报的是本拍状态机刚决定的状态，
    //   而不是上一拍的旧状态(状态切换那一拍的编码才准确)。
    //   自带限频(TLM_RATE_HZ)，非阻塞发送，正常时不打印任何日志。
    telemetry_tick();
}

}  // namespace fly_mission

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // ★MultiThreadedExecutor★：外部航点/视觉/雷达等订阅放在独立 Reentrant 回调组，
    //   与 50Hz 主循环 timer 并行，收包不阻塞控制。
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<fly_mission::FlyMissionNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
