#pragma once
// ============================================================================
//  任务状态枚举 + 状态名 / 遥测状态码映射
//  ---------------------------------------------------------------------------
//  单独成文件的原因：状态枚举被状态机的每个 .cpp 都要用，放在 fly_mission_node.hpp
//  里会让所有 case 文件都拖上整个节点类的依赖(ROS/json/shm/udp 全套头)。
//  这里只依赖 udp_telemetry.hpp 的几个状态码常量。
// ============================================================================

#include "fly_mission/udp_telemetry.hpp"

namespace fly_mission {

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

    // ───── 纯雷达投掷链(params::RADAR_DROP_MODE=true 时走这条，不用视觉) ─────
    RADAR_DESCEND,        // 【纯雷达·边追边降】追小车(x 偏后 RD_OFS_X)同时降到 RD_DROP_H_REL
    RADAR_DROP,           // 【纯雷达·判稳投掷】水平稳住 RD_STABLE_SEC 秒 → 投掷 → 爬升
    RADAR_CLIMB,          // 【纯雷达·投后爬升】爬到 RD_RETURN_H_REL 再返航
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

// 状态名(诊断日志用)。加新状态时补一行；漏了会显示 "?" 而不是崩溃。
const char* state_name(MissionState s);

// ★状态机状态 → 遥测状态码★(需求：1=起飞 2=追踪 3=投掷 4=降落 0=其它)
//   把内部十几个状态归并成监控端关心的四类。加新状态时记得在这里归类，
//   漏了会落到 default(0=其它)——不会出错，只是监控端看不出在干什么。
int32_t telemetry_status(MissionState s);

// ★该状态是否要做"失锁/飞手接管"判定★。
//   下面这些状态【本来就处于 未解锁 或 非 OFFBOARD】，必须放行，否则 0.5s 去抖一到就被
//   误判成"飞手接管"→ stop() + FINISHED，二次起飞永远走不到：
//     BOOT_CHECK      —— 还没解锁、等飞手切 OFFBOARD
//     LAND / LAND_THEN_WAIT —— 降落触底会上锁(LAND 到位判定本身就要求 !armed)
//     WAIT_TRIGGER    —— 已落地上锁，在地面等触发命令(可能等很久)
//     REARM           —— 正在自己切 OFFBOARD + 解锁，成功前必然不满足
//     PLAT_WAIT       —— 已落在平台上并主动上锁，桨不转、必然 !armed
//     PLAT_TAKEOFF    —— 正在自己切 OFFBOARD + 解锁(与 REARM 同理)
//     TRACK_LAND      —— 到上锁高度后会主动上锁，disarm 到状态切走之间有几拍 !armed
//     FINISHED        —— 任务已结束
bool lost_check_active_state(MissionState s);

// 该状态是否允许被"视觉找图"打断：只有【正在按航点走线】时才打断——
//   RUN_EXT_WAYPOINTS(外部航点，当前主用) / RUN_WAYPOINTS(写死表，备用)。
//   起飞前后、悬停等航点、降落、找图态本身都不打断。
bool find_figure_active_state(MissionState s);

}  // namespace fly_mission
