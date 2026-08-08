// ============================================================================
//  mission_state.cpp —— 状态枚举的三张映射表(状态名 / 遥测码 / 两个白名单)
//  ---------------------------------------------------------------------------
//  这里全是【纯函数】，不碰节点成员、不发指令，加新状态时只改这一个文件。
// ============================================================================

#include "fly_mission/mission_state.hpp"

namespace fly_mission {

const char* state_name(MissionState s)
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
    case MissionState::RADAR_DESCEND:      return "RADAR_DESCEND";
    case MissionState::RADAR_DROP:         return "RADAR_DROP";
    case MissionState::RADAR_CLIMB:        return "RADAR_CLIMB";
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

int32_t telemetry_status(MissionState s)
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

    // ── 3 投掷：视觉锁定 + 投掷；纯雷达链的三个阶段也都算"投掷中" ──
    case MissionState::LOCK_DROP:
    case MissionState::RADAR_DESCEND:
    case MissionState::RADAR_DROP:
    case MissionState::RADAR_CLIMB:
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

bool lost_check_active_state(MissionState s)
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

bool find_figure_active_state(MissionState s)
{
    return s == MissionState::RUN_EXT_WAYPOINTS ||
           s == MissionState::RUN_WAYPOINTS;
}

}  // namespace fly_mission
