#include "fly_mission/drone_controller.hpp"
#include "fly_mission/params.hpp"
#include "fly_mission/find_figure.hpp"
#include "fly_mission/waypoint_runner.hpp"
#include "fly_mission/waypoint_receiver.hpp"
#include "fly_mission/line_follower.hpp"
#include "fly_mission/ring_driller.hpp"
#include "fly_mission/car_tracker.hpp"
#include "fly_mission/shm_mailbox.hpp"   // 视觉shm信箱读端(第3步·并行验证期,只对比不切换)
#include "fly_mission/arduino_serial.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>   // 任务启动指令 /mission/start
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>

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
    TRACK_CAR,            // 【追踪小车雷达】飞到小车正上方、机头与小车同向，★不退出★
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

        // ★任务启动指令频道★：BOOT_CHECK 里响完第一声蜂鸣后等这个话题，收到 data==1
        //   才继续（响第二声 → 等 OFFBOARD → 解锁起飞）。
        //   ★收到一次就锁定★——用户会 1s 1 次持续发防丢包，之后的一律忽略，不会重复触发。
        //   只认 data==1，其他值忽略(防误发)。放默认回调组(与 timer 互斥) → 无需加锁。
        start_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/mission/start", rclcpp::QoS(10),
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                if (msg->data != 1) return;    // 只认 1
                if (start_recv_) return;       // 已锁定：重复发直接丢(不刷日志)
                start_recv_ = true;
                RCLCPP_INFO(get_logger(), "[启动] 收到启动指令 data=1（已锁定，后续重复发将忽略）");
            });

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
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                    "[启动] 等启动指令：ros2 topic pub -r 1 /mission/start std_msgs/msg/Int32 \"{data: 1}\"");
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
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "[追踪] 小车@飞机系(%.2f, %.2f) yaw=%.1f° → 追踪中", tx, ty, tyaw);
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
        case MissionState::WAIT_TRIGGER:
            if (trigger_recv_) {
                RCLCPP_INFO(get_logger(), "[二次起飞] 触发 → 开始切 OFFBOARD + 解锁");
                state_ = MissionState::REARM;
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                    "[二次起飞] 地面待机中，等触发命令："
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
                RCLCPP_INFO(get_logger(), "[二次起飞] 起飞完成 → 飞回 SLAM 原点 (0,0)");
                state_ = MissionState::GO_HOME;
            }
            break;

        // ─── ⑦ 飞回 SLAM 原点 (0,0) → 降落收尾 ──────────────────
        case MissionState::GO_HOME:
            target_xy_slam(0.0, 0.0);
            if (is_reached()) {
                RCLCPP_INFO(get_logger(), "[二次起飞] 已回到 SLAM 原点 (0,0) → 降落");
                state_ = MissionState::LAND;
            }
            break;

        // ─── 最后：降落 ──────────────────────────────────────────
        case MissionState::LAND:
            land();
            if (is_reached()) {
                RCLCPP_INFO(get_logger(), "[降落] 完成");
                state_ = MissionState::FINISHED;
            }
            break;

        case MissionState::FINISHED:
            RCLCPP_INFO_ONCE(get_logger(), "========== 任务结束 ==========");
            break;
        }
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
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr             start_sub_;     // 任务启动指令频道
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
