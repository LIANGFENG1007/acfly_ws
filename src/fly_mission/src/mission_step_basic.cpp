// ============================================================================
//  mission_step_basic.cpp —— 基础流程各状态的实现
//    横切判定(失锁去抖 / 找图打断) + 检测 + 起飞后分岔 + 探索 + 走线 + 找图
//    + 钻圈 + 绕杆 + 降落
// ============================================================================

#include "fly_mission/fly_mission_node.hpp"

namespace fly_mission {

// ════════════════════════════════════════════════════════════════════════════
//  横切判定(每拍在 switch 之前跑)
// ════════════════════════════════════════════════════════════════════════════
bool FlyMissionNode::check_pilot_takeover()
{
    if (!lost_check_active_state(state_)) return false;

    // 用单帧异常会被 /mavros/state 低频流误判，故加去抖
    constexpr double LOST_DEBOUNCE_S = 0.5;

    if (drone_.is_armed_offboard()) {
        lost_since_valid_ = false;
        return false;
    }
    if (!lost_since_valid_) {
        lost_since_       = now();
        lost_since_valid_ = true;
    }
    if ((now() - lost_since_).seconds() < LOST_DEBOUNCE_S) return false;

    RCLCPP_WARN(get_logger(),
        "飞手接管或失锁（armed=%d mode=\"%s\" 持续 %.2fs）→ 任务中止",
        drone_.is_armed() ? 1 : 0, drone_.mode_string().c_str(), LOST_DEBOUNCE_S);
    drone_.stop();
    state_ = MissionState::FINISHED;
    return true;
}

void FlyMissionNode::check_find_figure_interrupt()
{
    // ★仅在【正在按航点飞】(wp_loaded_)时才打断★——外部模式悬停等航点期间(未装载)
    //   不打断，避免任务还没开始就跑去追图形。FINDFIGURE 自身不重入。
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
}

// ════════════════════════════════════════════════════════════════════════════
//  BOOT_CHECK：返回 true = 全部检测通过，可以起飞
// ════════════════════════════════════════════════════════════════════════════
bool FlyMissionNode::step_boot_check()
{
    if (!drone_.is_connected()) return false;
    if (!check_connected_done_) {
        RCLCPP_INFO(get_logger(), "[检测] 飞控连接：是");
        check_connected_done_ = true;
    }

    if (!drone_.has_pose()) return false;
    if (!check_pose_done_) {
        RCLCPP_INFO(get_logger(), "[检测] 雷达里程计：是");
        check_pose_done_ = true;
    }

    // ── BEEP① ──：飞控+雷达都就绪的提示（★只发一次★，本状态每拍循环，
    //   靠 beep_sent_ 防止等待期间狂发）。没插串口只告警不影响流程。
    //   ★不能每拍调 arduino_send★：一次阻塞几十 ms(5次×20ms)，50Hz 会拖垮主循环。
    if (!beep_sent_) {
        arduino_send("BEEP");
        beep_sent_ = true;
        RCLCPP_INFO(get_logger(),
            "[启动] 飞控+雷达就绪，已响第一声 → 等启动指令 /mission/start (Int32 data=1)");
    }

    // ── 等启动指令 ──：★收到一次即锁定★(start_recv_)，用户会 1s 1 次持续发防丢包，
    //   重复发直接忽略。此时【尚未解锁、也还没到 OFFBOARD 检查】，桨不转，可安全长时间等。
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
        return false;
    }

    // ── BEEP② ──：收到启动指令的确认音（同样只发一次）。响完这声就该由飞手拨 OFFBOARD。
    if (!beep2_sent_) {
        arduino_send("BEEP");
        beep2_sent_ = true;
        RCLCPP_INFO(get_logger(),
            "[启动] 已收到启动指令并响第二声 → 请拨 OFFBOARD，之后自动解锁起飞");
    }

    // 等飞手手动切到 OFFBOARD（此时 setpoint 占位流已经在发了）
    if (!drone_.is_offboard()) return false;
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
        return false;
    }
    RCLCPP_INFO(get_logger(), "[检测] 解锁：是");

    drone_.capture_home();
    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  WAIT_AFTER_TAKEOFF 到位后：退位置环 + 按选定分支各自初始化 → 切走
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_wait_after_takeoff()
{
    // ★★★ 切换起飞后的任务：只改这一行即可（六选一）★★★
    //   ★注意★：这行赋值是唯一权威，别信别处注释里的"当前选中"标注。
    //     EXPLORATION       = 自主探索(exploration_planner 给速度，扫完→降落)
    //     HOVER_3S          = 悬停3s → 追踪小车 → 投掷 → 返航(第一段主流程)
    //                         其后可接第二段(落移动平台)，见 MISSION2_ENABLE
    //     FOLLOW_LINE       = 视觉寻线(沿黑线飞，丢线超时→降落)
    //     RUN_WAYPOINTS     = 写死航点表(waypoints.hpp)，逐点飞
    //     RUN_EXT_WAYPOINTS = 外部发的航点(/mission/waypoints；未收到悬停等，已收到免等直接飞)
    //     DRILL_RING        = 钻圈(悬停采集环位姿→飞到环前1m对准→穿圈→降落)
    //     CIRCLE_AROUND     = 绕杆(悬停采集杆位姿→飞到杆前1m对准→绕杆一圈→降落)
    //   ★二次起飞那条链★(GO_FORWARD→LAND_THEN_WAIT→WAIT_TRIGGER→REARM→
    //     TAKEOFF_AGAIN→GO_HOME) 代码完整保留，把下面改成 GO_FORWARD 即可跑。
    const MissionState next = MissionState::EXPLORATION;

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
    } else if (next == MissionState::EXPLORATION) {
        // ★自主探索★：这里【什么都不用做】，但★必须显式列出这个分支★——
        //   否则会掉进下面的 else(那是给"航点走线"兜底的)，被误开找图 +
        //   置 wp_loaded_，探索期间就可能被视觉找图打断。
        //   锁高(enter_exploration)由 exploration() 首拍自己做，同时发一次终点。
        //   不开 find_.enable()：探索不被找图打断(与寻线同理)。
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

// ════════════════════════════════════════════════════════════════════════════
//  探索：首次进入锁高 + 发一次终点；每拍把算法最新速度交给控制器
//  gx,gy = 探索终点 (SLAM 系，原点=起飞点)
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::exploration(double gx, double gy)
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
    // ★只转发【新鲜】的速度★：数据过期就不转发，让 drone_ 的看门狗接管保高悬停。
    //
    //   ★为什么必须判新鲜度(勿改回只判 ext_cmd_valid_)★：
    //   ext_cmd_valid_ 是"收到过速度"的一次性锁存标志，置真后永不清零。若只判它，
    //   规划节点挂掉/停发之后，这里每拍仍会拿【最后那个速度】调 set_velocity_body，
    //   而该函数内部会刷新 ext_cmd_time_(控制器自己那个) → drone_ 里的 0.3s 看门狗
    //   【永远不触发】→ 飞机带着最后一条速度命令一直飞下去(算法死了也没人叫停)。
    //   加上本判断后：算法停发 → 这里停止转发 → 控制器看门狗超时 → 水平/yaw 清零、
    //   只保高悬停，等算法恢复。
    if (!ext_cmd_valid_) {
        // 从未收到过速度：算法节点没起 / 还没算出第一条。悬停等(控制器看门狗保高)。
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[探索] 还没收到过 /exploration/cmd_vel：exploration_planner_node 起了吗? "
            "→ 悬停保高等它");
    } else if ((now() - ext_cmd_time_).seconds() <= params::EXPLORE_CMD_TIMEOUT_S) {
        drone_.set_velocity_body(ext_v_fwd_, ext_v_lat_, ext_yaw_rate_);
    } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[探索] /exploration/cmd_vel 已 %.2fs 无更新(>%.2fs)：算法挂了/停发了? "
            "→ 停止转发，悬停保高等它恢复",
            (now() - ext_cmd_time_).seconds(), params::EXPLORE_CMD_TIMEOUT_S);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  走线三方式
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_follow_line()
{
    double vf, vl, yr;
    lf_.compute(vf, vl, yr);   // 有效线→算机体系速度；无有效线→全 0(悬停等，超时由调用方判)
    drone_.set_velocity_body(vf, vl, yr);
}

void FlyMissionNode::step_run_waypoints()
{
    if (!wp_loaded_) {
        runner_.reset_default();   // 装入写死航点表，从第 0 个开始
        wp_loaded_ = true;
        RCLCPP_INFO(get_logger(), "[写死航点] 装入 %zu 个点，开始执行", runner_.total());
        return;
    }
    if (runner_.done()) {
        RCLCPP_INFO(get_logger(), "[写死航点] 全部 %zu 个完成，准备降落", runner_.total());
        state_ = MissionState::LAND;
        return;
    }
    const auto& wp = runner_.current();
    target_xy_slam(wp.x, wp.y);          // 幂等 PD 飞向当前航点，高度不变
    if (is_reached() || waypoint_blocked_arrived(wp.x, wp.y)) {
        RCLCPP_INFO(get_logger(), "[写死航点] 航点 %zu/%zu (%.2f,%.2f) 到达",
                    runner_.index_1based(), runner_.total(), wp.x, wp.y);
        runner_.advance();               // 推进下一个(下一拍飞新点；走完则上面转 LAND)
    }
}

void FlyMissionNode::step_run_ext_waypoints()
{
    if (!wp_loaded_) {
        // 起飞前没收到 → 悬停等外部发来(wait_time 幂等锁位置)；收到即装载
        wait_time(1.0);
        if (recv_.has_waypoints()) {
            runner_.set_waypoints(recv_.take());   // 装入外部航点，从第 0 个开始
            wp_loaded_ = true;
            RCLCPP_INFO(get_logger(), "[外部航点] 收到 %zu 个点，开始执行", runner_.total());
        }
        return;
    }
    if (runner_.done()) {
        RCLCPP_INFO(get_logger(), "[外部航点] 全部 %zu 个完成，准备降落", runner_.total());
        state_ = MissionState::LAND;
        return;
    }
    const auto& wp = runner_.current();
    target_xy_slam(wp.x, wp.y);                // 幂等 PD 飞向当前航点，高度不变
    if (is_reached() || waypoint_blocked_arrived(wp.x, wp.y)) {
        RCLCPP_INFO(get_logger(), "[外部航点] 航点 %zu/%zu (%.2f,%.2f) 到达",
                    runner_.index_1based(), runner_.total(), wp.x, wp.y);
        runner_.advance();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  视觉找图：飞向确认到的图形中心(实时刷新)，到点悬停后拉黑，回被打断状态
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_find_figure()
{
    double tx, ty;
    // ★用找图专用的收紧容差 FF_TOL_XY(0.10)★，不是正常飞行的 TOL_XY(0.15)。
    const auto step = find_.tick(is_reached_find(), tx, ty);
    if (step == FindFigure::Step::FLYING) {
        target_xy_slam(tx, ty);
    } else {
        // DONE：到点悬停完 / 超时放弃 / 队空 → 回被打断前状态
        //   (下一拍其 case 重下原目标，幂等续飞)
        state_ = state_before_find_;
        RCLCPP_INFO(get_logger(), "[找图] 本次结束 → 回到原状态继续走线");
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  钻圈：悬停采集环位姿 → 边飞边转升高到环前1m对准 → 悬停 → 穿到环后 → 降落
//    rd_.begin() 在 step_wait_after_takeoff() 已调；这里每拍按 rd_ 的阶段下指令。
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_drill_ring()
{
    double tx, ty, tz, tyaw;
    switch (rd_.tick(is_reached(), tx, ty, tz, tyaw)) {
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
}

// ════════════════════════════════════════════════════════════════════════════
//  绕杆：description_circle_right() 无参、内部自带 采集→接近→环绕 子状态机
//    ★完成判定用 pole_circle_done()，不是 is_reached()★(该原语内部切多个子模式)
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_circle_around()
{
    description_circle_right();      // 每拍推进绕杆子状态机(首次调用内部自动起采集)
    if (!pole_circle_done()) return;

    if (drone_.pole_circle_failed())
        RCLCPP_WARN(get_logger(), "[绕杆] 未获得足够杆检测 → 降落");
    else
        RCLCPP_INFO(get_logger(), "[绕杆] 绕杆完成 → 降落");
    state_ = MissionState::LAND;
}

// ════════════════════════════════════════════════════════════════════════════
//  降落：返回 true = 已触底且上锁(去哪由调用方决定)
// ════════════════════════════════════════════════════════════════════════════
bool FlyMissionNode::step_land()
{
    land();
    return is_reached();      // LAND 到位判定 = 高度触底 且 已上锁
}

}  // namespace fly_mission
