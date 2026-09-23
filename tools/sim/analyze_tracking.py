#!/usr/bin/env python3
"""离线轨迹跟踪质量分析：贴线误差、拐弯平滑度、卡顿段。

输入 run_trial.py 产生的试验目录(events.jsonl)，输出 tracking_summary.json、
tracking_report.md 与 tracking.png。只读，不启动 ROS。

口径：
- 贴线误差 = SLAM 位姿到当时规划器发布的 active_path 折线的最近距离(m)，
  只统计探索/归航阶段且路径点数>=2 的样本；路径刚切换后 0.3 s 内的样本不计
  (旧路线到新路线的几何跳变不是跟踪误差)。
- 卡顿 = 起飞后、任务结束前，水平指令 <0.05 m/s 连续 >=0.4 s 的段；真值速度
  <0.05 m/s 连续 >=0.5 s 的段另列，两者都排除起飞前和降落。
- 拐弯段 = SLAM 航向角速度 |ω|>0.30 rad/s 连续 >=0.3 s；记录段内最低真值速度。
- 平滑度 = 指令合速度差分加速度的 RMS、真值速度差分加速度 RMS。
"""
import argparse
import json
import math
import sys
from pathlib import Path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def point_polyline_distance(x, y, pts):
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts[:-1], pts[1:]):
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            d = math.hypot(x - ax, y - ay)
        else:
            u = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
            d = math.hypot(x - (ax + u * dx), y - (ay + u * dy))
        best = min(best, d)
    return best


def segments(times, flags, min_duration):
    """连续 flags 为真且时长>=min_duration 的段 [(t0,t1)]。"""
    out, start = [], None
    for t, f in zip(times, flags):
        if f and start is None:
            start = t
        elif not f and start is not None:
            if t - start >= min_duration:
                out.append((start, t))
            start = None
    if start is not None and times and times[-1] - start >= min_duration:
        out.append((start, times[-1]))
    return out


def percentile(values, q):
    if not values:
        return None
    s = sorted(values)
    k = (len(s) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (k - lo)


def analyze(trial: Path):
    events = trial / "events.jsonl"
    if not events.exists():
        raise FileNotFoundError(events)
    path, path_t, mode = None, None, None
    goal_t, finished_t, corridor_t = None, None, None
    airborne_t = None
    slam, cmd, truth, dev = [], [], [], []
    path_switches = 0
    with events.open() as f:
        for line in f:
            e = json.loads(line)
            t = e.get("t_sim")
            if t is None:
                continue
            k = e["kind"]
            if k == "active_path":
                pts = [(p[0], p[1]) for p in e["points"]]
                if pts != path:
                    path_switches += 1 if path is not None else 0
                    path, path_t = pts, t
            elif k == "diagnostics":
                mode = e.get("values", {}).get("mode", mode)
            elif k == "goal":
                goal_t = t if goal_t is None else goal_t
            elif k == "finished" and e.get("value"):
                finished_t = t if finished_t is None else finished_t
            elif k == "corridor_active" and e.get("value"):
                corridor_t = t if corridor_t is None else corridor_t
            elif k == "truth":
                if airborne_t is None and e["z"] > 0.5:
                    airborne_t = t
                truth.append((t, e["x"], e["y"], e["z"], e["yaw"], e.get("speed_mps")))
            elif k == "cmd":
                cmd.append((t, math.hypot(e["vx"], e["vy"]), e["yaw_rate"], mode))
            elif k == "slam":
                slam.append((t, e["x"], e["y"], e["yaw"]))
                if (path and len(path) >= 2 and mode in ("exploration", "homing")
                        and path_t is not None and t - path_t >= 0.3
                        and airborne_t is not None and finished_t is None):
                    dev.append((t, point_polyline_distance(e["x"], e["y"], path), mode))
    end_t = finished_t if finished_t else (slam[-1][0] if slam else None)
    explore_end = corridor_t if corridor_t else end_t

    def in_window(t):
        return airborne_t is not None and t > airborne_t + 3.0 and (end_t is None or t < end_t)

    # ---- 指令平滑度与零速段 ----
    cmd_win = [c for c in cmd if in_window(c[0]) and (c[3] in (None, "exploration", "homing"))]
    cmd_acc = [(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(cmd_win[:-1], cmd_win[1:]) if b[0] > a[0] + 1e-4]
    cmd_yaw_acc = [(b[2] - a[2]) / (b[0] - a[0]) for a, b in zip(cmd_win[:-1], cmd_win[1:]) if b[0] > a[0] + 1e-4]
    zero_cmd = segments([c[0] for c in cmd_win], [c[1] < 0.05 for c in cmd_win], 0.4)
    # ---- 真值速度 ----
    truth_win = [x for x in truth if in_window(x[0]) and x[5] is not None
                 and (explore_end is None or x[0] < explore_end)]
    spd = [x[5] for x in truth_win]
    # 真值速度由记录器对 Gazebo 位姿差分得到，时间戳偶有 <10 ms 的抖动产生单点尖峰(seed43 见 1.1 m/s 孤点)，
    # 只用间隔 >=15 ms 且 |a|<=5 m/s² 的样本，避免一个坏点主导 RMS。
    truth_acc = [(b[5] - a[5]) / (b[0] - a[0]) for a, b in zip(truth_win[:-1], truth_win[1:])
                 if b[0] > a[0] + 0.015 and abs((b[5] - a[5]) / (b[0] - a[0])) <= 5.0]
    slow_truth = segments([x[0] for x in truth_win], [x[5] < 0.05 for x in truth_win], 0.5)
    # ---- 拐弯段 (SLAM yaw 差分，低通) ----
    slam_win = [s for s in slam if in_window(s[0]) and (explore_end is None or s[0] < explore_end)]
    rates, rt = [], []
    for a, b in zip(slam_win[:-1], slam_win[1:]):
        if b[0] > a[0] + 1e-4:
            rt.append(b[0]); rates.append(wrap(b[3] - a[3]) / (b[0] - a[0]))
    filt, prev = [], 0.0
    for r in rates:
        prev += 0.3 * (r - prev); filt.append(prev)
    turns = segments(rt, [abs(r) > 0.30 for r in filt], 0.3)
    turn_rows = []
    for t0, t1 in turns:
        v = [x[5] for x in truth_win if t0 <= x[0] <= t1]
        turn_rows.append({"t0": round(t0, 2), "t1": round(t1, 2), "duration_s": round(t1 - t0, 2),
                          "min_truth_speed": round(min(v), 3) if v else None,
                          "mean_truth_speed": round(sum(v) / len(v), 3) if v else None})
    stalled_turns = sum(1 for r in turn_rows if r["min_truth_speed"] is not None and r["min_truth_speed"] < 0.05)
    # ---- 爬行段：真值速度 < 0.10 m/s 持续 >= 2 s(含停顿)，是"卡顿感"的直接指标 ----
    crawl = segments([x[0] for x in truth_win], [x[5] < 0.10 for x in truth_win], 2.0)
    # ---- 贴线误差 ----
    dev_explore = [d[1] for d in dev if d[2] == "exploration"]
    dev_homing = [d[1] for d in dev if d[2] == "homing"]

    def stats(v):
        return None if not v else {"n": len(v), "mean": round(sum(v) / len(v), 4),
                                   "p50": round(percentile(v, .5), 4), "p95": round(percentile(v, .95), 4),
                                   "max": round(max(v), 4)}
    rms = lambda v: round(math.sqrt(sum(x * x for x in v) / len(v)), 4) if v else None
    summary = {
        "trial": str(trial),
        "timeline_sim_s": {"airborne": airborne_t, "goal": goal_t, "corridor": corridor_t, "finished": finished_t,
                           "explore_duration": (explore_end - goal_t) if (goal_t and explore_end) else None,
                           "total_duration": (finished_t - airborne_t) if (finished_t and airborne_t) else None},
        "cross_track_exploration_m": stats(dev_explore),
        "cross_track_homing_m": stats(dev_homing),
        "cross_track_over_0p10_fraction": round(sum(1 for d in dev if d[1] > .10) / len(dev), 4) if dev else None,
        "path_switches": path_switches,
        "cmd_speed_accel_rms": rms(cmd_acc), "cmd_yaw_accel_rms": rms(cmd_yaw_acc),
        "cmd_speed_accel_p99": round(percentile([abs(a) for a in cmd_acc], .99), 3) if cmd_acc else None,
        "truth_speed": stats(spd), "truth_accel_rms": rms(truth_acc),
        "zero_cmd_segments": [{"t0": round(a, 2), "t1": round(b, 2), "dur": round(b - a, 2)} for a, b in zero_cmd],
        "zero_cmd_total_s": round(sum(b - a for a, b in zero_cmd), 2),
        "slow_truth_segments": [{"t0": round(a, 2), "t1": round(b, 2), "dur": round(b - a, 2)} for a, b in slow_truth],
        "slow_truth_total_s": round(sum(b - a for a, b in slow_truth), 2),
        "turns": turn_rows, "turn_count": len(turn_rows), "turns_with_stop": stalled_turns,
        "crawl_segments": [{"t0": round(a, 2), "t1": round(b, 2), "dur": round(b - a, 2)} for a, b in crawl],
        "crawl_total_s": round(sum(b - a for a, b in crawl), 2),
    }
    return summary, dev, cmd_win, truth_win, slam_win, filt, rt


def report(s):
    tl = s["timeline_sim_s"]
    ct = s["cross_track_exploration_m"] or {}
    lines = [f"# 跟踪质量：{Path(s['trial']).name}", "",
             f"- 探索用时 {tl['explore_duration'] and round(tl['explore_duration'],1)} s，全程 {tl['total_duration'] and round(tl['total_duration'],1)} s，任务完成: {tl['finished'] is not None}",
             f"- 贴线误差(探索)：均值 {ct.get('mean')} m，p95 {ct.get('p95')} m，最大 {ct.get('max')} m，>0.10 m 占比 {s['cross_track_over_0p10_fraction']}",
             f"- 路线切换 {s['path_switches']} 次",
             f"- 指令加速度 RMS {s['cmd_speed_accel_rms']} m/s²，p99 {s['cmd_speed_accel_p99']}；yaw 指令加速度 RMS {s['cmd_yaw_accel_rms']} rad/s²",
             f"- 真值速度均值 {(s['truth_speed'] or {}).get('mean')} m/s，真值加速度 RMS {s['truth_accel_rms']} m/s²",
             f"- 零指令段 {len(s['zero_cmd_segments'])} 段共 {s['zero_cmd_total_s']} s；真值近零速段 {len(s['slow_truth_segments'])} 段共 {s['slow_truth_total_s']} s",
             f"- 拐弯段 {s['turn_count']} 段，其中停车 {s['turns_with_stop']} 段",
             f"- 爬行段(真值<0.10 m/s 持续>=2 s) {len(s['crawl_segments'])} 段共 {s['crawl_total_s']} s", ""]
    if s["turns"]:
        lines.append("| 拐弯 t0 | 时长 s | 段内最低速 | 段内均速 |")
        lines.append("|---|---|---|---|")
        for r in s["turns"]:
            lines.append(f"| {r['t0']} | {r['duration_s']} | {r['min_truth_speed']} | {r['mean_truth_speed']} |")
    return "\n".join(lines) + "\n"


def plot(trial, s, dev, cmd_win, truth_win, slam_win, rates, rt):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "Noto Sans CJK TC", "DejaVu Sans"]
        matplotlib.rcParams["axes.unicode_minus"] = False
    except Exception as e:  # noqa
        print("matplotlib 不可用，跳过绘图:", e, file=sys.stderr)
        return
    fig, ax = plt.subplots(3, 1, figsize=(12, 12), gridspec_kw={"height_ratios": [2, 1, 1]})
    # 轨迹 + 所有 active_path
    paths = []
    with (trial / "events.jsonl").open() as f:
        for line in f:
            if '"kind": "active_path"' in line:
                e = json.loads(line)
                if len(e["points"]) >= 2:
                    paths.append(e["points"])
    for p in paths:
        ax[0].plot([q[0] for q in p], [q[1] for q in p], color="#9aa5b1", lw=0.6, alpha=0.5)
    xs = [q[1] for q in slam_win]; ys = [q[2] for q in slam_win]
    ax[0].plot(xs, ys, color="#1f77b4", lw=1.2, label="SLAM 轨迹(探索段)")
    if dev:
        sc = ax[0].scatter([0] * 0, [0] * 0)
        dt = {round(d[0], 2): d[1] for d in dev}
        cx = [q[1] for q in slam_win if round(q[0], 2) in dt]
        cy = [q[2] for q in slam_win if round(q[0], 2) in dt]
        cv = [dt[round(q[0], 2)] for q in slam_win if round(q[0], 2) in dt]
        sc = ax[0].scatter(cx, cy, c=cv, cmap="viridis", s=6, vmin=0, vmax=0.2)
        fig.colorbar(sc, ax=ax[0], label="贴线误差 m")
    ax[0].set_aspect("equal"); ax[0].grid(alpha=.3); ax[0].legend(loc="upper left")
    ax[0].set_title(f"{trial.name}: 灰=历次规划路线, 蓝=实际轨迹")
    ax[1].plot([c[0] for c in cmd_win], [c[1] for c in cmd_win], lw=.8, label="指令 |v|")
    ax[1].plot([x[0] for x in truth_win], [x[5] for x in truth_win], lw=.8, label="真值速度")
    for a, b in [(z["t0"], z["t1"]) for z in s["zero_cmd_segments"]]:
        ax[1].axvspan(a, b, color="red", alpha=.2)
    ax[1].set_ylabel("m/s"); ax[1].legend(); ax[1].grid(alpha=.3)
    ax[2].plot(rt, rates, lw=.8, label="SLAM 航向角速度(滤波)")
    ax[2].plot([c[0] for c in cmd_win], [c[2] for c in cmd_win], lw=.8, alpha=.7, label="yaw_rate 指令")
    if dev:
        ax2 = ax[2].twinx(); ax2.plot([d[0] for d in dev], [d[1] for d in dev], color="green", lw=.6, alpha=.6)
        ax2.set_ylabel("贴线误差 m", color="green"); ax2.set_ylim(0, .4)
    ax[2].set_ylabel("rad/s"); ax[2].set_xlabel("仿真时间 s"); ax[2].legend(loc="upper left"); ax[2].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(trial / "tracking.png", dpi=110); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trials", nargs="+", type=Path)
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    for trial in a.trials:
        s, dev, cmd_win, truth_win, slam_win, rates, rt = analyze(trial)
        (trial / "tracking_summary.json").write_text(json.dumps(s, indent=2, ensure_ascii=False))
        md = report(s)
        (trial / "tracking_report.md").write_text(md)
        print(md)
        if not a.no_plot:
            plot(trial, s, dev, cmd_win, truth_win, slam_win, rates, rt)


if __name__ == "__main__":
    main()
