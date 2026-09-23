#!/usr/bin/env python3
"""把指令突变(前进骤降/角速度尖峰)对齐到当时规划器 diagnostics 状态，定位抖动来源。

用法：event_timeline.py TRIAL [--drop 0.05] [--yaw 0.5]
输出每个事件：时间(相对 goal)、位置、突变量、replan_reason、各计数器在事件前后 0.3 s 的增量。
"""
import argparse
import json
from pathlib import Path

COUNTERS = ("adoptions", "turn_entries", "recovery_entries", "explore_guard_rejections",
            "explore_guard_lateral_trims", "cone_bypass_adoptions", "smooth_fallbacks", "astar_searches")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trial", type=Path)
    parser.add_argument("--drop", type=float, default=0.05, help="单拍前进指令下降阈值(m/s)")
    parser.add_argument("--yaw", type=float, default=0.5, help="角速度指令尖峰阈值(rad/s)")
    args = parser.parse_args()
    cmds, diags, slam, goal_t = [], [], [], None
    for line in (args.trial / "events.jsonl").open():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        k = d.get("kind")
        if k == "cmd":
            cmds.append(d)
        elif k == "diagnostics":
            diags.append(d)
        elif k == "slam":
            slam.append(d)
        elif k == "goal" and goal_t is None:
            goal_t = d["t_wall"]
    goal_t = goal_t or cmds[0]["t_wall"]

    def diag_at(t):
        best = None
        for d in diags:
            if d["t_wall"] > t:
                break
            best = d
        return best["values"] if best else {}

    def pose_at(t):
        best = slam[0] if slam else {}
        for s in slam:
            if s["t_wall"] > t:
                break
            best = s
        return best

    events = []
    for a, b in zip(cmds, cmds[1:]):
        if b["t_wall"] < goal_t:
            continue
        drop = a["vx"] - b["vx"]
        if drop >= args.drop:
            events.append((b["t_wall"], f"fwd {a['vx']:.2f}->{b['vx']:.2f}"))
        if abs(b["yaw_rate"]) >= args.yaw and abs(a["yaw_rate"]) < args.yaw:
            events.append((b["t_wall"], f"yaw spike {b['yaw_rate']:+.2f}"))
    last = -1e9
    for t, text in sorted(events):
        if t - last < 0.25:
            continue
        last = t
        before, after = diag_at(t - 0.3), diag_at(t + 0.3)
        delta = {c: float(after.get(c, 0)) - float(before.get(c, 0)) for c in COUNTERS}
        delta = {c: int(v) for c, v in delta.items() if v}
        p = pose_at(t)
        print(f"{t - goal_t:7.2f}s ({p.get('x', 0):5.2f},{p.get('y', 0):5.2f}) v={p.get('vx', 0):+.2f},{p.get('vy', 0):+.2f} "
              f"{text:22s} mode={after.get('mode', '')} reason={after.get('replan_reason', '')} "
              f"head_err={float(after.get('heading_error', 0)):+.2f} curv={float(after.get('reference_curvature', 0)):.2f} "
              f"corner={float(after.get('next_corner_distance', -1)):.2f} rem={float(after.get('remaining_s', 0)):.2f} {delta}")


if __name__ == "__main__":
    main()
