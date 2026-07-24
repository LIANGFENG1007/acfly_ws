#pragma once

// ============================================================================
//  waypoint_runner.hpp  ── 按航点顺序走线的进度管理
//
//  持有一组航点 + "当前走到第几个"的进度。主控走线状态每拍问它"该飞哪"、到位后叫它
//  "推进下一个"。航点来源两种：① 写死表 waypoints.hpp 的 WAYPOINTS(reset_default)；
//  ② 运行时外部发来的一组(set_waypoints，如 /mission/waypoints)。逻辑很轻但按项目
//  分工单独成文件，主控只简单调用。
//
//  ★被找图打断也不丢进度★：进度存在本对象里，FINDFIGURE 期间主控不碰它；打断回来
//    current() 仍是被打断前那个航点，target_xy_slam 幂等续飞 → 接着走剩下的点。
// ============================================================================

#include "fly_mission/waypoints.hpp"

#include <cstddef>
#include <vector>

namespace fly_mission {

class WaypointRunner
{
public:
    WaypointRunner() = default;

    // 装入一组运行时航点(外部发来的)，并从第 0 个开始走。
    void set_waypoints(const std::vector<Waypoint>& wps) { wps_ = wps; idx_ = 0; }

    // 用写死表 waypoints.hpp 的 WAYPOINTS 作为航点，从头开始(原 RUN_WAYPOINTS 用法)。
    void reset_default() { wps_.assign(WAYPOINTS.begin(), WAYPOINTS.end()); idx_ = 0; }

    // 只把进度归零(航点不变，进入走线状态时调一次)。
    void reset() { idx_ = 0; }

    // 航点是否已走完(全部到位过)。空表也视为已完成。
    bool done() const { return idx_ >= wps_.size(); }

    // 当前应飞向的航点。done()/空表时返回一个安全的零点(正常主控在 done 前不会再调)。
    const Waypoint& current() const {
        if (wps_.empty()) { static const Waypoint kZero{}; return kZero; }
        const std::size_t i = done() ? wps_.size() - 1 : idx_;
        return wps_[i];
    }

    // 当前航点已到位 → 推进到下一个。返回推进后的序号(从 1 起，便于打印)。
    std::size_t advance() { ++idx_; return idx_; }

    // 当前航点序号(从 1 起，给日志用) / 总航点数。
    std::size_t index_1based() const { return idx_ + 1; }
    std::size_t total() const { return wps_.size(); }

private:
    std::vector<Waypoint> wps_;   // 当前这组航点(写死表或外部发来的)
    std::size_t           idx_ = 0;   // 当前航点下标(0 起)
};

}  // namespace fly_mission
