// ============================================================================
//  mission_step_track.cpp —— 追踪小车 + 投掷(视觉锁定 / 纯雷达链) + 投后返航
// ============================================================================

#include "fly_mission/fly_mission_node.hpp"

#include <cmath>       // std::hypot / std::fabs

namespace fly_mission {

namespace {
// 丢数据时悬停的时长：追踪状态不判 is_reached()，所以这个数值本身不影响行为，
//   只需要是个【固定常量】——wait_time 靠"时长没变"实现幂等(每拍重复调不会重置
//   计时、不会把锁定位置刷成当前漂移位置)。给大值纯粹表示"一直悬停"。
constexpr double CAR_LOST_HOLD_SEC = 3600.0;
}  // namespace

// ════════════════════════════════════════════════════════════════════════════
//  【追踪小车雷达】飞到小车正上方 + 机头与小车同向，★一直追不退出★
//    每拍问 car_ 要"小车现在在飞机 SLAM 系的哪里"，直接喂 target_pose_slam：
//    MOVE_POSE 是"一次到位"原语(PD 同拍控制平移+转向)，每拍重下新目标即为实时追踪。
//    ★不判 is_reached()★：追踪是持续行为——到位了也要继续跟着小车动。
//    小车停下时飞机自然停在它上方(PD 误差趋零)。
//    高度：CAR_TRACK_Z=false 时 car_ 返回的 z 恒为进入时锁定的高度。
//
//  追上判定：水平距离 ≤CATCH_DIST 累计 CATCH_HOLD_SEC → 按 RADAR_DROP_MODE 分岔
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_track_car()
{
    double tx, ty, tz, tyaw;
    if (!car_.latest(tx, ty, tz, tyaw)) {
        // 没有新数据(小车雷达没上线 / 话题真的停了)：★必须主动锁住当前位置★。
        //   ★不能"什么都不做"★：那样 action_mode_ 仍是 MOVE_POSE、target_* 仍是
        //   【最后一次收到的小车位置】，PD 会继续朝那个点冲——如果话题是在小车
        //   移动中断掉的，飞机就会奔向一个过期的点，正是要避免的行为。
        //   wait_time 幂等：切 HOLD 并锁死【调用瞬间】的位姿，重复调不重置计时；
        //   数据恢复后 target_pose_slam 会切回 MOVE_POSE 继续追。
        //   注意这不是"跟丢"——小车位置本来一直知道，这里只针对话题停了。
        wait_time(CAR_LOST_HOLD_SEC);
        return;
    }

    target_pose_slam(tx, ty, tz, tyaw);      // 实时刷新目标(位置+偏航一起)

    // ★追上判定★：飞机与小车的【水平】距离(不含高度差——飞机在小车上方飞，
    //   算上高度永远追不上)。
    const double d_xy = std::hypot(tx - drone_.current_x(), ty - drone_.current_y());
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

    if (catch_timer_ < params::CATCH_HOLD_SEC) return;

    if (params::RADAR_DROP_MODE) {
        // ★★★ 纯雷达投掷链 ★★★(完全不用视觉)：先边追边降到 RD_DROP_H_REL，再判稳投掷。
        rd_z_ = drone_.current_z();      // 下降起点 = 当前高度
        rd_tick_valid_    = false;       // 下降积分从下一拍起算
        rd_start_         = now_t;       // 下降段超时起算
        rd_start_valid_   = true;
        rd_stable_valid_  = false;
        drop_sent_        = false;
        drop_hold_valid_  = false;
        // ★放开垂直限速★：MOVE_POSE 默认被平飞档 0.1m/s 限死，
        //   不放开的话 RD_DESCEND_SPD 根本跑不起来(与落平台同一坑)。
        //   ★离开纯雷达链的每条路径都必须复位★(见各状态出口 + stop() 兜底)。
        drone_.set_plat_descend_mode(true);
        state_ = MissionState::RADAR_DESCEND;
        RCLCPP_INFO(get_logger(),
            "[追上] 累计 %.1fs ≥ %.1fs → ★纯雷达投掷模式★"
            "(边追边降 %.2fm→离起飞点%.2fm @%.2fm/s；"
            "水平目标=小车后方%.2fm[随车头旋转]，y/yaw 正常跟随；"
            "到高度后稳 %.1fs 即投)",
            catch_timer_, params::CATCH_HOLD_SEC,
            rd_z_ - drone_.home_z(), params::RD_DROP_H_REL,
            params::RD_DESCEND_SPD, -params::RD_OFS_X,
            params::RD_STABLE_SEC);
    } else {
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
        lock_start_       = now_t;       // 锁定总超时起算
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
}

// ════════════════════════════════════════════════════════════════════════════
//  【视觉锁定投掷】LOCK_DROP
//
//  位置：用 shm 信箱的 dx/dy(机体系，前+/左+)换算成 SLAM 绝对目标点。
//        ★换算在 consume_shm_cv() 里做完并冻结★，本状态只是取用——绝不能在这里
//        每拍用"当前位置 + 旧 dx/dy"重算：视觉写 50Hz、状态机读 50Hz 但★不同步★，
//        读不到新帧的那些拍会拿已移动过的位置配旧偏移，把目标点一路往前推(正反馈)。
//  偏航：★仍用小车雷达(UDP)的 yaw★——shm 里只有 dx/dy 没有 yaw，而"锁定时偏航要与
//        目标一致"是需求。雷达数据没了就保持当前机头(不乱转)。
//
//  ★视觉数据分三级处理(丢帧不等于放弃)★：
//    ① 新鲜(≤LOCK_CV_TIMEOUT_S)      用 dx/dy 精确锁定 + 可投掷
//    ② 短暂丢(~LOCK_CV_FALLBACK_S)    原地锁住等它回来(不投)
//    ③ 长时间丢(>LOCK_CV_FALLBACK_S)  ★回退小车雷达(UDP)定位★继续跟上，
//                                     仍不投；视觉恢复自动切回 ①
//    为什么③要回退：小车会跑，视觉丢了原地干等的话小车早开走、再也看不见 → 死等。
//  高度：★全程保持不变★——lock_z_ 在判定追上那一刻被锁成"当时的高度"。
//  ★两种投掷触发(哪个先到算哪个)★，之后收尾路径完全相同(锁 DROP_HOLD_SEC → 返航)：
//    · 提前投：视觉对准到 DROP_DIST 以内
//    · ★必投★：进入本状态起 LOCK_TIMEOUT_SEC 到点 → ★无条件投掷★
//      —— 视觉看不见/视觉进程死了/雷达接管中，一律照投(纯挂钟计时，不依赖任何
//         数据源)。否则视觉一死就会永久悬停到电池耗尽。
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_lock_drop()
{
    const rclcpp::Time now_t = now();

    // ── 1) 视觉数据新鲜度 ──
    const double cv_age = lock_cv_valid_ ? (now_t - lock_cv_time_).seconds() : 1e9;
    //   ★必须同时有"冻结好的目标点"★(lock_tgt_valid_)：收帧时若还没位姿，
    //   dx/dy 存下来了但目标点没算出来 —— 那种情况不能算数据可用，
    //   否则会出现"位置在悬停、高度却在下降"的不一致行为。
    const bool cv_ok = lock_cv_valid_ && lock_tgt_valid_
                       && cv_age <= params::LOCK_CV_TIMEOUT_S;

    // ── 2) 高度：★不下降，保持 lock_z_ 不变★(需求 2026-08) ──
    //   lock_z_ 在 TRACK_CAR 判定追上那一刻锁成"当时的高度"，之后不再改动，
    //   下面每个分支都把它原样下发 → 全程定高。不做任何积分。

    // ── 3) 偏航：优先小车雷达的 yaw，没有就保持当前机头 ──
    double cx, cy, cz, cyaw;
    const bool car_ok = car_.latest(cx, cy, cz, cyaw);
    const double tgt_yaw_deg = car_ok ? cyaw : drone_.current_yaw_deg();

    if (cv_ok) {
        // ── 4) 位置：用【收视觉帧那一刻算好并冻结】的 SLAM 目标点 ──
        target_pose_slam(lock_tgt_x_, lock_tgt_y_, lock_z_, tgt_yaw_deg);

        // ★投掷判定距离要加落点补偿 DROP_LEAD_X / DROP_LEAD_Y★(见 params 说明)：
        //   投掷物带着飞机速度、机构有前抛角/侧向偏置、相机光轴不正 →
        //   落点有系统性偏差。把 dx/dy 各减去对应补偿量再判距离，等价于"把判定
        //   中心从目标正上方挪到 (LEAD_X, LEAD_Y) 处"，使投掷时机提前/推后到刚好
        //   能抵消偏差。
        //   ★注意只用于投掷时机判定★——上面 target_pose_slam 用的仍是未补偿的
        //   lock_tgt_(飞机照旧往目标正上方飞)，所以飞机不会停在偏心的位置。
        const double d_cv = std::hypot(lock_dx_ - params::DROP_LEAD_X,
                                       lock_dy_ - params::DROP_LEAD_Y);
        //   未补偿的真实偏差，仅用于日志对照(看飞机实际对得准不准)
        const double d_raw = std::hypot(lock_dx_, lock_dy_);

        // ── 5) 投掷判定 ──
        //   ★用非阻塞版 arduino_send_async★：这是飞行中(OFFBOARD)，阻塞版会
        //   usleep 5×20ms + tcdrain ≈ 80ms，主循环停转、setpoint 流中断，
        //   飞控可能判"没收到 setpoint"退出 OFFBOARD。
        //   drop_sent_ 保证★只排队一次★。
        if (!drop_sent_ && d_cv <= params::DROP_DIST) {
            drop_sent_ = true;                       // 先置位：确保只触发一次
            arduino_send_async(params::DROP_CMD);
            drop_hold_until_ = now_t + rclcpp::Duration::from_seconds(params::DROP_HOLD_SEC);
            drop_hold_valid_ = true;
            RCLCPP_INFO(get_logger(),
                "[投掷] 补偿后距离 %.3fm ≤ %.2fm → 已发 \"%s\"；"
                "(实际偏差 dx=%.3f dy=%.3f 未补偿距离 %.3fm；"
                "补偿 前后%+.2f 左右%+.2f) 保持高度 %.2fm，再锁定 %.1fs 后返航",
                d_cv, params::DROP_DIST, params::DROP_CMD,
                lock_dx_, lock_dy_, d_raw,
                params::DROP_LEAD_X, params::DROP_LEAD_Y,
                lock_z_, params::DROP_HOLD_SEC);
        } else if (!drop_sent_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "[锁定] dx=%.3f dy=%.3f | 补偿后距离 %.3fm(阈值 %.2fm；"
                "补偿 前后%+.2f 左右%+.2f；未补偿 %.3fm) | "
                "高度 %.2fm(离起飞点 %.2fm) yaw=%.1f°%s",
                lock_dx_, lock_dy_, d_cv, params::DROP_DIST,
                params::DROP_LEAD_X, params::DROP_LEAD_Y, d_raw,
                lock_z_, lock_z_ - drone_.home_z(), tgt_yaw_deg,
                car_ok ? "" : "(雷达yaw无数据，保持机头)");
        }
    } else if (cv_age > params::LOCK_CV_FALLBACK_S && car_ok) {
        // ★视觉长时间丢失 → 回退用小车雷达(UDP)定位★
        //   为什么要回退：小车是会跑的，视觉丢了还原地干等，小车早开走了，
        //   回来也看不见 → 死等。雷达位置精度不如视觉，但足够"继续跟上"，
        //   等飞到小车上方视觉重新看见就自动切回精确锁定。
        //   ★仍然不投掷★：雷达精度不足(标定开环)，投了大概率偏，白费一次机会。
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
        // 视觉短暂丢帧，或雷达也没数据：★锁住当前位置悬停等★。
        //   高度目标仍然有效(当前水平位置 + 目标高度)，避免"什么都不做"导致
        //   PD 继续冲向过期目标点。
        target_pose_slam(drone_.current_x(), drone_.current_y(), lock_z_, tgt_yaw_deg);
        // ★区分"视觉挂了"和"视觉活着但没看到目标"★：视觉端没看到目标也会写
        //   空帧推进 seq，所以 alive 心跳还在 = 程序在跑、只是没识别到。
        const double alive_age = lock_cv_alive_valid_
            ? (now_t - lock_cv_alive_time_).seconds() : 1e9;
        const bool cv_alive = lock_cv_alive_valid_ && alive_age <= 1.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[锁定] 已 %.2fs 没看到目标(>%.2fs) → 原地锁定等；视觉进程%s%s",
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
            cv_ok ? std::hypot(lock_dx_ - params::DROP_LEAD_X,
                               lock_dy_ - params::DROP_LEAD_Y) : -1.0,
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
}

// ════════════════════════════════════════════════════════════════════════════
//  纯雷达投掷的水平目标 = 小车雷达位置 + 车身系偏移(随车头旋转)
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::radar_drop_target(double cx, double cy, double cyaw_deg,
                                       double& out_x, double& out_y) const
{
    const double yaw = cyaw_deg * M_PI / 180.0;
    const double c = std::cos(yaw), s = std::sin(yaw);
    out_x = cx + c * params::RD_OFS_X - s * params::RD_OFS_Y;
    out_y = cy + s * params::RD_OFS_X + c * params::RD_OFS_Y;
}

// ════════════════════════════════════════════════════════════════════════════
//  【纯雷达·边追边降】RADAR_DESCEND  (params::RADAR_DROP_MODE=true 时走这条)
//
//  水平：目标 = 小车雷达位置 + 【车身后方 RD_OFS_X】(随小车 yaw 旋转)。
//        y 不加偏移、yaw 直接跟小车 —— 即需求里的"y/yaw 正常矫正"。
//  高度：从进入时高度按 RD_DESCEND_SPD 匀速降到 (起飞点 + RD_DROP_H_REL)。
//  ★完全不看视觉★：本链一行都不碰 lock_*(视觉)那套变量。
//
//  ★两条出口★：
//    · 正常：目标高度降到底 且 实际高度进 RD_H_TOL → RADAR_DROP
//    · 兜底：RD_DESCEND_TIMEOUT_S 超时 → 也进 RADAR_DROP(高度可能没到位，
//      但不该无限期悬着；判稳阶段仍会正常工作，只是投掷高度偏高)
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_radar_descend()
{
    const rclcpp::Time now_t = now();
    const double floor_z = drone_.home_z() + params::RD_DROP_H_REL;

    // ── 1) 目标高度匀速下降 ──
    if (rd_tick_valid_) {
        const double dt = (now_t - rd_tick_).seconds();
        if (dt > 0.0 && dt < 0.5) {      // dt 异常(卡顿/时间跳变)时本拍不降
            rd_z_ -= params::RD_DESCEND_SPD * dt;
            if (rd_z_ < floor_z) rd_z_ = floor_z;
        }
    }
    rd_tick_ = now_t;
    rd_tick_valid_ = true;

    // ── 2) 水平/偏航：追小车(x 偏后)，高度用上面算的 rd_z_ ──
    double cx, cy, cz, cyaw;
    if (car_.latest(cx, cy, cz, cyaw)) {
        double tx, ty;
        radar_drop_target(cx, cy, cyaw, tx, ty);   // 加车身后方偏移
        target_pose_slam(tx, ty, rd_z_, cyaw);     // yaw 直接跟小车
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[雷达降] 雷达@(%.2f,%.2f)→投掷点@(%.2f,%.2f)[车后%.2fm] "
            "高度 %.2f→%.2fm 实际 %.2fm(离起飞点 %.2fm) 水平差 %.2fm",
            cx, cy, tx, ty, -params::RD_OFS_X,
            rd_z_, floor_z, drone_.current_z(),
            drone_.current_z() - drone_.home_z(),
            std::hypot(tx - drone_.current_x(), ty - drone_.current_y()));
    } else {
        // 雷达没数据：水平锁住当前位置，★高度继续降★(高度不依赖小车数据)
        target_pose_slam(drone_.current_x(), drone_.current_y(),
                         rd_z_, drone_.current_yaw_deg());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[雷达降] 小车雷达无数据 → 水平锁住当前位置，高度继续降到 %.2fm", floor_z);
    }

    // ── 3) 高度到位(或超时) → 转判稳投掷 ──
    const bool tgt_at  = (rd_z_ <= floor_z + 1e-6);
    const bool real_at = std::fabs(drone_.current_z() - floor_z) <= params::RD_H_TOL;
    const bool timeout = params::RD_DESCEND_TIMEOUT_S > 0.0 && rd_start_valid_ &&
                         (now_t - rd_start_).seconds() > params::RD_DESCEND_TIMEOUT_S;
    if (!((tgt_at && real_at) || timeout)) return;

    if (timeout && !(tgt_at && real_at)) {
        RCLCPP_WARN(get_logger(),
            "[雷达降] ★超时 %.0fs 未降到位★(目标 %.2fm 实际 %.2fm)"
            " → 仍进入判稳投掷，投掷高度将偏高",
            params::RD_DESCEND_TIMEOUT_S, floor_z, drone_.current_z());
        rd_z_ = drone_.current_z();   // 钉在当前高度，别再往下追不到的目标压
    }
    rd_stable_valid_ = false;
    rd_start_        = now_t;         // 判稳总超时重新起算
    rd_start_valid_  = true;
    state_ = MissionState::RADAR_DROP;
    RCLCPP_INFO(get_logger(),
        "[雷达降] 高度到位(%.2fm，离起飞点 %.2fm) → ★开始判稳★"
        "(水平≤%.2fm 持续 %.1fs 即投；总超时 %.0fs 无条件投)",
        drone_.current_z(), drone_.current_z() - drone_.home_z(),
        params::RD_STABLE_TOL, params::RD_STABLE_SEC, params::RD_DROP_TIMEOUT_S);
}

// ════════════════════════════════════════════════════════════════════════════
//  【纯雷达·判稳投掷】RADAR_DROP
//
//  ★高度不再变★：保持 rd_z_(= 进入时的投掷高度)。
//  判稳：水平与投掷点距离 ≤RD_STABLE_TOL 且 高度仍在 RD_H_TOL 内，
//        连续 RD_STABLE_SEC 秒 → 投掷。中途跑出容差 → 计时清零重算
//        (这里要求【连续】稳住，与追上判定的"只暂停不清零"不同 ——
//         投掷要的是"此刻确实稳"，断断续续的稳不算)。
//  ★两种投掷触发★，之后收尾相同(锁 DROP_HOLD_SEC → 爬升 → 返航)：
//    · 正常：稳住 RD_STABLE_SEC
//    · 兜底：RD_DROP_TIMEOUT_S 到点 → ★无条件投★(小车乱跑/标定偏也要收场)
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_radar_drop()
{
    const rclcpp::Time now_t = now();
    const double floor_z = drone_.home_z() + params::RD_DROP_H_REL;

    double cx, cy, cz, cyaw;
    const bool car_ok = car_.latest(cx, cy, cz, cyaw);
    double d_xy = 1e9;
    if (car_ok) {
        double tx, ty;
        radar_drop_target(cx, cy, cyaw, tx, ty);
        target_pose_slam(tx, ty, rd_z_, cyaw);
        d_xy = std::hypot(tx - drone_.current_x(), ty - drone_.current_y());
    } else {
        // 雷达没数据：锁住当前位置保持高度(不拿过期目标飞)，判稳自然不满足
        target_pose_slam(drone_.current_x(), drone_.current_y(),
                         rd_z_, drone_.current_yaw_deg());
        rd_stable_valid_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[雷达投] 小车雷达无数据 → 原地保持，判稳暂停(靠总超时兜底)");
    }

    // ── 判稳：水平 + 高度都在容差内才累计 ──
    const bool h_ok  = std::fabs(drone_.current_z() - floor_z) <= params::RD_H_TOL;
    const bool xy_ok = car_ok && d_xy <= params::RD_STABLE_TOL;
    if (!drop_sent_ && xy_ok && h_ok) {
        if (!rd_stable_valid_) {
            rd_stable_start_ = now_t;
            rd_stable_valid_ = true;
        }
        const double held = (now_t - rd_stable_start_).seconds();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
            "[雷达投] 稳住中 水平%.3f≤%.2f 高度%.2f → %.2f/%.1fs",
            d_xy, params::RD_STABLE_TOL, drone_.current_z(), held, params::RD_STABLE_SEC);
        if (held >= params::RD_STABLE_SEC) {
            drop_sent_ = true;                    // 先置位：只触发一次
            arduino_send_async(params::DROP_CMD); // 非阻塞(飞行中不能阻塞)
            drop_hold_until_ = now_t + rclcpp::Duration::from_seconds(params::DROP_HOLD_SEC);
            drop_hold_valid_ = true;
            RCLCPP_INFO(get_logger(),
                "[雷达投] ★稳住 %.1fs → 投掷★ 已发 \"%s\"(水平差 %.3fm，"
                "高度 %.2fm)；锁定 %.1fs 后爬升到离起飞点 %.2fm 再返航",
                held, params::DROP_CMD, d_xy, drone_.current_z(),
                params::DROP_HOLD_SEC, params::RD_RETURN_H_REL);
        }
    } else if (!drop_sent_) {
        rd_stable_valid_ = false;                 // 跑出容差 → 重新计时
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[雷达投] 未稳住(水平%.3f/%.2f%s 高度%.2f%s) 重新计时",
            d_xy, params::RD_STABLE_TOL, xy_ok ? "✓" : "✗",
            drone_.current_z(), h_ok ? "✓" : "✗");
    }

    // ── 兜底：判稳总超时 → 无条件投 ──
    if (!drop_sent_ && params::RD_DROP_TIMEOUT_S > 0.0 && rd_start_valid_ &&
        (now_t - rd_start_).seconds() > params::RD_DROP_TIMEOUT_S) {
        drop_sent_ = true;
        arduino_send_async(params::DROP_CMD);
        drop_hold_until_ = now_t + rclcpp::Duration::from_seconds(params::DROP_HOLD_SEC);
        drop_hold_valid_ = true;
        RCLCPP_WARN(get_logger(),
            "[雷达投] ★总超时 %.0fs 未稳住 → 无条件投掷★ 已发 \"%s\""
            "(当时水平差 %.3fm，雷达%s)；锁定 %.1fs 后爬升返航",
            params::RD_DROP_TIMEOUT_S, params::DROP_CMD,
            car_ok ? d_xy : -1.0, car_ok ? "有数据" : "无数据",
            params::DROP_HOLD_SEC);
    }

    // ── 投掷后锁定 DROP_HOLD_SEC → 转爬升 ──
    if (drop_sent_ && drop_hold_valid_ && now_t >= drop_hold_until_) {
        rd_climb_yaw_ = drone_.current_yaw_deg();   // 冻结朝向，爬升途中不转头
        state_ = MissionState::RADAR_CLIMB;
        RCLCPP_INFO(get_logger(),
            "[雷达投] 追加锁定 %.1fs 完成 → 爬升到离起飞点 %.2fm",
            params::DROP_HOLD_SEC, params::RD_RETURN_H_REL);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  【纯雷达·投后爬升】爬到 RD_RETURN_H_REL 再返航
//    ★为什么要先爬升★：投掷高度只有 1m，低空长距离飞回起点容易刮到东西。
//    水平保持在【当前位置】(不追小车了，投都投完了)，只改高度。
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_radar_climb()
{
    const double target_z = drone_.home_z() + params::RD_RETURN_H_REL;
    // 水平锁住进入时的位置：不能每拍传 current_x/y，否则位置误差恒 0 = 不控水平
    if (!rd_climb_valid_) {
        rd_climb_x_ = drone_.current_x();
        rd_climb_y_ = drone_.current_y();
        rd_climb_valid_ = true;
    }
    target_pose_slam(rd_climb_x_, rd_climb_y_, target_z, rd_climb_yaw_);
    if (!is_reached()) return;

    // ★复位垂直限速★：纯雷达链结束，恢复平飞档(否则后续 MOVE_POSE 会上下窜)
    drone_.set_plat_descend_mode(false);
    return_z_   = drone_.current_z();
    return_yaw_ = drone_.current_yaw_deg();
    RCLCPP_INFO(get_logger(),
        "[雷达投] 已爬到 %.2fm(离起飞点 %.2fm) → 飞回起点 (0,0)",
        drone_.current_z(), drone_.current_z() - drone_.home_z());
    state_ = MissionState::RETURN_HOME_DROP;
}

// ════════════════════════════════════════════════════════════════════════════
//  【投掷后返航】以当前高度飞回 (0,0) → 降落
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_return_home_drop()
{
    // ★显式锁住返航高度 return_z_★(=投掷完那一刻的高度)，而不是只调 target_xy_slam。
    //   后者不碰 target_z_，高度靠"上一次谁设的"延续——那是隐式依赖：LOCK_DROP 最后
    //   一拍设的正好是 lock_z_，能用但很脆弱(以后谁在中间插一个改高度的动作就悄悄
    //   变了)。用 target_pose_slam 把位置+高度+朝向一次写全，语义明确。
    //   ★yaw 用进入本状态时冻结的 return_yaw_★，不能每拍传 current_yaw_deg()：
    //   那样 target_yaw_ 永远等于当前朝向，yaw 误差恒 0 —— 等于不控偏航，
    //   机头会随气流/PD 侧滑慢慢转过去而没人纠正。
    target_pose_slam(0.0, 0.0, return_z_, return_yaw_);
    if (is_reached()) {
        RCLCPP_INFO(get_logger(),
            "[返航] 已回到起飞点 (0,0)，高度 %.2fm → 降落", drone_.current_z());
        state_ = MissionState::LAND;
    }
}

}  // namespace fly_mission
