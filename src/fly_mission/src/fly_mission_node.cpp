#include "fly_mission/drone_controller.hpp"
#include "fly_mission/params.hpp"
#include "fly_mission/find_figure.hpp"
#include "fly_mission/waypoint_runner.hpp"
#include "fly_mission/waypoint_receiver.hpp"
#include "fly_mission/line_follower.hpp"
#include "fly_mission/ring_driller.hpp"
#include "fly_mission/shm_mailbox.hpp"   // 视觉shm信箱读端(第3步·并行验证期,只对比不切换)
#include "fly_mission/arduino_serial.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <chrono>

using namespace std::chrono_literals;
using namespace fly_mission;

// ────────────────────────────────────────────────────────────────────────
//  任务全部状态
// ────────────────────────────────────────────────────────────────────────
enum class MissionState {
    BOOT_CHECK,                 // 检测：连接 + 解锁 + OFFBOARD + 雷达
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
          rd_(this)
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
        
        if (state_ != MissionState::BOOT_CHECK &&
            state_ != MissionState::LAND &&
            state_ != MissionState::FINISHED)
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
            takeoff(1.0);
            if (is_reached()) {
                RCLCPP_INFO(get_logger(), "[起飞] 完成");
                state_ = MissionState::WAIT_AFTER_TAKEOFF;
            }
            break;

        case MissionState::WAIT_AFTER_TAKEOFF:
            wait_time(0.5);
            if (is_reached()) {
                // ★★★ 切换起飞后的任务：只改这一行即可（五选一）★★★
                //   FOLLOW_LINE       = 视觉寻线(沿黑线飞，机头朝前进方向，丢线超时→降落)
                //   RUN_WAYPOINTS     = 写死航点表(waypoints.hpp)，逐点飞
                //   RUN_EXT_WAYPOINTS = 外部发的航点(/mission/waypoints；未收到悬停等，已收到免等直接飞)
                //   DRILL_RING        = 钻圈(悬停采集环位姿→飞到环前1m对准→穿圈→降落)
                //   CIRCLE_AROUND     = 绕杆(悬停采集杆位姿→飞到杆前1m对准→绕杆一圈→降落)  ★当前选中★
                const MissionState next = MissionState::RUN_EXT_WAYPOINTS;    

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
            const auto step = find_.tick(is_reached(), tx, ty);
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
    WaypointRunner               runner_;
    rclcpp::TimerBase::SharedPtr timer_;
    MissionState                 state_ = MissionState::BOOT_CHECK;
    MissionState                 state_before_find_ = MissionState::RUN_EXT_WAYPOINTS;  // 进 FINDFIGURE 前的状态(处理完回它)
    bool                         wp_loaded_ = false;   // 当前走线状态是否已装载航点(首拍装一次)

    // BOOT_CHECK 一次性通过的标志（防止重复打 "是"）
    bool check_connected_done_ = false;
    bool check_pose_done_      = false;
    bool check_offboard_done_  = false;

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

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              finished_sub_;
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
