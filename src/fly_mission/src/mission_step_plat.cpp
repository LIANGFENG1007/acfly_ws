// ============================================================================
//  mission_step_plat.cpp —— 第二段任务：落移动平台 / 平台待机与起飞 / 二次起飞
// ============================================================================

#include "fly_mission/fly_mission_node.hpp"

#include <cmath>       // std::hypot

namespace fly_mission {

namespace {
constexpr double CAR_LOST_HOLD_SEC = 3600.0;   // 同 TRACK_CAR：丢数据就悬停等
constexpr double PREHEAT_S         = 0.5;      // 切 OFFBOARD 前的 setpoint 流预热时长 (s)
}  // namespace

// ════════════════════════════════════════════════════════════════════════════
//  【二段追踪】TRACK_CAR2 —— 与 TRACK_CAR 行为相同，只有两点不同：
//    ① 追上确认时长用 ★CATCH_HOLD_SEC_2★ 而非 CATCH_HOLD_SEC
//    ② 追上后转 TRACK_LAND(边追边降)，而不是 LOCK_DROP(投掷)
//  单独建一个状态而不复用 TRACK_CAR：两段的出口和时长不同，共用会互相干扰。
//  ★不加任何安装偏移★：直接飞到小车雷达正上方(2026-08 回退，偏移逻辑已移除)。
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_track_car2()
{
    double tx, ty, tz, tyaw;
    if (!car_.latest(tx, ty, tz, tyaw)) {
        wait_time(CAR_LOST_HOLD_SEC);          // 同 TRACK_CAR：锁住当前位姿
        return;
    }

    target_pose_slam(tx, ty, tz, tyaw);

    const double d_xy = std::hypot(tx - drone_.current_x(), ty - drone_.current_y());
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

    if (catch_timer_ < params::CATCH_HOLD_SEC_2) return;

    plat_z_ = drone_.current_z();      // 下降起点 = 当前高度
    plat_tick_valid_   = false;        // 下降积分从下一拍起算
    plat_start_        = now_t;        // 下降段超时起算
    plat_start_valid_  = true;
    plat_touch_valid_  = false;
    plat_descending_   = false;        // ★必须重置★：要重新确认降起来了
    // ★★★ 必须放开垂直限速，否则永不锁桨 ★★★
    //   本状态走 MOVE_POSE，默认命中平飞档 MAX_SPEED_Z_LEVEL(0.1m/s)，
    //   而接触检测要求实际降速先超过 PLAT_DESCEND_MIN_VZ 才启用 ——
    //   不放开的话门槛就够到了限速硬上限，这道门永远过不去 →
    //   接触检测永不启用 → 永不上锁 → 走到超时返航(实测踩过)。
    //   两个出口(接触成功 / 超时)都会复位，drone_.stop() 里还有兜底。
    drone_.set_plat_descend_mode(true);
    state_ = MissionState::TRACK_LAND;
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

// ════════════════════════════════════════════════════════════════════════════
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
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_track_land()
{
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

            // 实际下降速率(★向下为正★)。取控制器的低通速度估计，不是单拍裸差分：
            //   50Hz 下 ±1cm 的 SLAM 噪声 = 0.5m/s 假速度，是 PLAT_TOUCH_VZ(0.05)
            //   的 10 倍，裸差分会让接触计时被噪声反复清零 → 永远判不出接触。
            //   vz_est() 向上为正，取负号换成"向下为正"。
            const double vz_down = -drone_.vz_est();

            // ★★★ 必须先"真的降起来过"，才允许接触判定 ★★★
            //   否则：刚进本状态时飞机还在悬停(垂直速度≈0) → vz_down≈0
            //   → 立刻满足"降不动" → 在高空上锁 = 摔机。
            //   判据：实际降速曾经超过 PLAT_DESCEND_MIN_VZ(独立参数，★不从
            //   PLAT_DESCEND_SPD 推算★——推算式会与垂直限速咬死，见 params 说明)。
            if (!plat_descending_) {
                if (vz_down > params::PLAT_DESCEND_MIN_VZ) {
                    plat_descending_ = true;
                    RCLCPP_INFO(get_logger(),
                        "[边追边降] 已确认开始下降(降速 %.3f m/s) → 接触检测启用", vz_down);
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                        "[边追边降] 等待下降建立(当前降速 %.3f，需 >%.3f m/s)"
                        "；★接触检测尚未启用(防高空误判上锁)★",
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
                    vz_down, params::PLAT_TOUCH_VZ, h_rel, params::PLAT_DISARM_MAX_H_REL);
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
                    params::PLAT_DISARM_MAX_H_REL, held, params::PLAT_TOUCH_HOLD_S,
                    touched ? "判定已接触平台" : "继续确认");
            } else {
                plat_touch_valid_ = false;   // 又开始降了 → 重新计时
            }
        }
    }
    plat_tick_       = now_t;
    plat_tick_valid_ = true;

    // ── 2) 水平/偏航：继续跟小车(★无偏移，直接飞雷达正上方★)；高度用上面算的 plat_z_ ──
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
            "[边追边降] ★已接触平台(实际高度 %.2fm 不再下降) → 主动上锁★", cur_z);
        if (!drone_.is_armed()) {              // 确认上锁成功才往下走
            drone_.set_plat_descend_mode(false);  // ★复位垂直限速★(出口1：正常落上)
            drone_.stop();                     // 停发 setpoint(已在平台上)
            plat_wait_start_       = now_t;
            plat_wait_start_valid_ = true;
            state_ = MissionState::PLAT_WAIT;
            RCLCPP_INFO(get_logger(),
                "[平台] ★已落在移动平台上并上锁★(高度 %.2fm，离起飞点 %.2fm)"
                " → 等 %.0fs 后自动重新起飞",
                cur_z, cur_z - drone_.home_z(), params::PLAT_WAIT_SEC);
        }
        return;
    }

    // ── 4) 下降段超时兜底 → 放弃降落，回起点 ──
    if (params::PLAT_DESCEND_TIMEOUT_S > 0.0 && plat_start_valid_ &&
        (now_t - plat_start_).seconds() > params::PLAT_DESCEND_TIMEOUT_S) {
        mission2_done_ = true;                 // 标记第二段已尝试过，别再循环
        drone_.set_plat_descend_mode(false);   // ★复位垂直限速★(出口2：超时返航)
        RCLCPP_ERROR(get_logger(),
            "[边追边降] ★超时 %.0fs 未检测到接触平台★"
            "(实际高度 %.2fm，离起飞点 %.2fm；小车乱跑? 水平追不上? "
            "PLAT_TOUCH_VZ 设太小?) → 放弃落平台，飞回起点降落",
            params::PLAT_DESCEND_TIMEOUT_S, cur_z, cur_z - drone_.home_z());
        state_ = MissionState::GO_HOME;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  【平台待机】落在平台上等 PLAT_WAIT_SEC 秒 → 自己重新起飞
//    ★此时已上锁、桨不转★，所以不发 setpoint(drone_.stop() 已停)。
//    平台可能还在动，飞机跟着平台一起走——这段时间不做任何控制。
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_plat_wait()
{
    if (!plat_wait_start_valid_) {             // 防御：正常不会走到
        plat_wait_start_ = now();
        plat_wait_start_valid_ = true;
    }
    const double waited = (now() - plat_wait_start_).seconds();
    if (waited < params::PLAT_WAIT_SEC) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[平台] 平台上待机中 %.1f/%.0fs（已上锁，桨不转）",
            waited, params::PLAT_WAIT_SEC);
        return;
    }
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
}

// ════════════════════════════════════════════════════════════════════════════
//  ★REARM 与 PLAT_TAKEOFF 共用的"自主解锁起飞"流程★
//    两者原来是两段几乎逐行相同的代码，唯一实质差别：REARM 要 capture_home()
//    (把落地点记为新 home)，PLAT_TAKEOFF 刻意不记(保持最初起飞点为基准)。
//
//  流程：等位姿 → [可选]重记 home → 复位重试计数 → 起 setpoint 流 → 预热
//        → 切 OFFBOARD → 解锁 → TAKEOFF_AGAIN
//  ★顺序与 BOOT_CHECK 一致(先模式后解锁)★，区别是 OFFBOARD 这次由程序自己请求
//    (第一次是飞手手动拨的，落地后没人再去拨)。切 OFFBOARD 前飞控通常要求已有
//    setpoint 流，故先 takeoff() 把流起来、预热 PREHEAT_S 再请求。
//  ★此期间尚未解锁，桨不转、飞机不会动★；解锁成功的那一拍 PD 已在命令爬升。
//
//  tag        = 日志前缀("二次起飞" / "平台起飞")
//  recapture_home = 是否把当前位置记为新 home
// ════════════════════════════════════════════════════════════════════════════
namespace {
// 起飞高度：REARM 与 TAKEOFF_AGAIN 必须同一个值(这里定目标、那里续飞同一段爬升)。
//   也建议与第一次起飞 TAKEOFF 的高度一致。
constexpr double REARM_TAKEOFF_Z = 1.5;
}  // namespace

void FlyMissionNode::step_rearm()
{
    if (!rearm_inited_) {
        // ★必须有雷达位姿才敢重记 home★：位姿丢了 capture_home() 只打告警不改 home，
        //   会退化成拿第一次的 home(原点) 当起飞目标 → 边爬边横拉。所以在这等它回来。
        if (!drone_.has_pose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[二次起飞] 无雷达位姿，暂不起飞（等 SLAM 恢复）");
            return;
        }
        // ★把落地点记为新 home★：takeoff() 飞的是 home 正上方，不重记则会拿第一次的
        //   home(原点) 当目标 → 一边爬升一边横拉回原点，贴地拖行很危险。
        //   注意"飞回原点"用的是 target_xy_slam(0,0) 绝对坐标，不受 home 影响。
        drone_.capture_home();
        drone_.reset_arm_offboard_retry();    // 复位第一次起飞用掉的重试计数
        rearm_start_  = now();
        rearm_inited_ = true;
    }

    takeoff(REARM_TAKEOFF_Z);   // 每拍下起飞目标 → tick() 持续发 setpoint(= 切 OFFBOARD 所需的流)

    if ((now() - rearm_start_).seconds() < PREHEAT_S) return;   // 等流跑起来再切

    if (!drone_.is_offboard()) {
        if (!drone_.request_offboard()) {
            RCLCPP_ERROR(get_logger(), "[二次起飞] 切 OFFBOARD 失败，任务中止");
            drone_.stop();
            state_ = MissionState::FINISHED;
        }
        return;
    }
    if (!drone_.is_armed()) {
        if (!drone_.request_arm()) {
            RCLCPP_ERROR(get_logger(), "[二次起飞] 解锁失败，任务中止");
            drone_.stop();
            state_ = MissionState::FINISHED;
        }
        return;
    }
    RCLCPP_INFO(get_logger(), "[二次起飞] OFFBOARD + 解锁完成 → 爬升到 %.1fm", REARM_TAKEOFF_Z);
    state_ = MissionState::TAKEOFF_AGAIN;
}

// ─── 【平台起飞】与 REARM 同一套流程，两点不同：日志前缀、★不重记 home★ ───
//   ★不重记 home★(用户选定)：1.5m 仍以【最初起飞点】为基准，所以飞机实际
//   只会从平台爬升 (1.5 - 平台高度) 那么多。好处是后面"飞回原点"的坐标系一致。
void FlyMissionNode::step_plat_takeoff()
{
    if (!rearm_inited_) {
        if (!drone_.has_pose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "[平台起飞] 无雷达位姿，暂不起飞（等 SLAM 恢复）");
            return;
        }
        // ★这里刻意不调 capture_home()★：保持最初起飞点为基准(见上面说明)
        drone_.reset_arm_offboard_retry();
        rearm_start_  = now();
        rearm_inited_ = true;
    }

    takeoff(REARM_TAKEOFF_Z);   // 每拍下目标 → 持续发 setpoint(= 切 OFFBOARD 所需的流)

    if ((now() - rearm_start_).seconds() < PREHEAT_S) return;

    if (!drone_.is_offboard()) {
        if (!drone_.request_offboard()) {
            RCLCPP_ERROR(get_logger(), "[平台起飞] 切 OFFBOARD 失败，任务中止");
            drone_.stop();
            state_ = MissionState::FINISHED;
        }
        return;
    }
    if (!drone_.is_armed()) {
        if (!drone_.request_arm()) {
            RCLCPP_ERROR(get_logger(), "[平台起飞] 解锁失败，任务中止");
            drone_.stop();
            state_ = MissionState::FINISHED;
        }
        return;
    }
    RCLCPP_INFO(get_logger(), "[平台起飞] OFFBOARD + 解锁完成 → 爬升到 %.1fm", REARM_TAKEOFF_Z);
    state_ = MissionState::TAKEOFF_AGAIN;      // 复用：那里会因 mission2_done_ 走回原点
}

// ════════════════════════════════════════════════════════════════════════════
//  【二次起飞爬升】爬到(新 home 上方) REARM_TAKEOFF_Z → 按 mission2_done_ 分岔
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_takeoff_again()
{
    takeoff(REARM_TAKEOFF_Z);      // ★须与 step_rearm/step_plat_takeoff 里同高度★
    if (!is_reached()) return;

    // 与第一次起飞在 step_wait_after_takeoff() 的处理对称：退出起飞段位置环，
    //   之后走速度环 PD。TAKEOFF_POSITION_MODE=false 时本调用是空操作。
    drone_.exit_takeoff_position_mode();

    // ★分岔★：第二段任务(降落到移动平台)走 TRACK_CAR2；
    //   若第二段已跑完(mission2_done_)或功能关掉了，就按老流程直接回原点。
    if (params::MISSION2_ENABLE && !mission2_done_) {
        // ★不悬停，直接进追踪★(需求)。锁定追踪高度 = 此刻高度。
        car_.begin(drone_.current_z());
        catch_timer_      = 0.0;      // 第二段追上计时独立起算
        catch_tick_valid_ = false;
        RCLCPP_INFO(get_logger(),
            "[二段] 起飞到 %.1fm 完成 → ★直接进追踪(不悬停)★；"
            "追上确认时长 %.0fs(第二段专用)", REARM_TAKEOFF_Z, params::CATCH_HOLD_SEC_2);
        state_ = MissionState::TRACK_CAR2;
    } else {
        RCLCPP_INFO(get_logger(), "[平台起飞] 起飞完成 → 飞回 SLAM 原点 (0,0)");
        state_ = MissionState::GO_HOME;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  【第一次降落】触底 + 上锁 → 转地面待机(★不结束任务★)
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_land_then_wait()
{
    land();
    if (!is_reached()) return;     // LAND 到位判定 = 高度触底 且 已上锁

    RCLCPP_INFO(get_logger(),
        "[二次起飞] 第一次降落完成（已上锁）→ 地面待机，等触发命令");
    // 回 IDLE：★彻底停发 setpoint★(地面上最安全)，同时 log_progress 不再刷降落日志。
    //   二次起飞时 step_rearm() 里 takeoff() 会重新把 setpoint 流起来。
    drone_.stop();
    state_ = MissionState::WAIT_TRIGGER;
}

// ════════════════════════════════════════════════════════════════════════════
//  【地面待机】等二次起飞触发命令
//    UDP 模式(CMD_USE_UDP=true，当前)：等 ★命令 2★(CMD_TAKEOFF_AGAIN)，
//    与命令 1(启动)★同一个端口★，对方同样 1s 发一次，收到一次即锁定。
//    ROS 话题模式：等 /mission/takeoff_again。
// ════════════════════════════════════════════════════════════════════════════
void FlyMissionNode::step_wait_trigger()
{
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
}

}  // namespace fly_mission
