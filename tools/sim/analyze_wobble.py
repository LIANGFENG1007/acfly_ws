#!/usr/bin/env python3
"""晃动/平稳性分析：量化规划器指令的高频抖动与真值姿态的高频摆动。

输入 run_trial.py 的试验目录(events.jsonl)，输出 wobble.json 与 wobble.png。只读。

口径(按 diagnostics 的 mode 分段：exploration / homing / corridor)：
- 指令(/exploration/cmd_vel，机体系，50 Hz)：
  d_v_rms     相邻两拍水平指令向量差的 RMS (m/s/拍)，拍间跳变越大越"抖"；
  lat_flips   横向指令 |v_lat|>0.02 时的符号翻转次数/分钟(左右来回纠偏)；
  yaw_flips   |yaw_rate|>0.05 时的符号翻转次数/分钟(机头左右摆)；
  fwd_reversals 前进指令的局部极值(幅度≥0.03 m/s)次数/分钟(加减速来回)；
  d_yaw_rms   相邻两拍角速度指令差的 RMS (rad/s/拍)。
- 真值(Gazebo，约 27 Hz，重采样到 25 Hz)：
  tilt_hf_deg 倾角(roll/pitch)减去 1 s 滑动平均后的 RMS(度)，即去掉机动所需倾角后剩下的摆动；
  acc_rms     水平加速度 RMS；jerk_rms 水平加加速度 RMS；
  yaw_acc_rms 航向角加速度 RMS；
  tilt_peak_hz 高频倾角的主频(0.3~6 Hz)。
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load(trial):
    cmds, truth, modes = [], [], []
    goal_t = finished_t = None
    for line in (trial / "events.jsonl").open():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        k, t = d.get("kind"), d.get("t_wall")
        if k == "cmd":
            cmds.append((t, d["vx"], d["vy"], d["yaw_rate"]))
        elif k == "truth" and d.get("sim_at_receive") is not None:
            truth.append((t, d["x"], d["y"], d["z"], d["yaw"], d["roll"], d["pitch"]))
        elif k == "diagnostics":
            modes.append((t, d.get("values", {}).get("mode", "")))
        elif k == "goal" and goal_t is None:
            goal_t = t
        elif k == "finished" and d.get("value") and finished_t is None:
            finished_t = t
    return np.array(cmds), np.array(truth), modes, goal_t, finished_t


def mode_at(modes, times):
    out = np.empty(len(times), dtype=object)
    j, cur = 0, ""
    for i, t in enumerate(times):
        while j < len(modes) and modes[j][0] <= t:
            cur = modes[j][1]
            j += 1
        out[i] = cur
    return out


def flips(values, threshold):
    sign = np.sign(values) * (np.abs(values) > threshold)
    sign = sign[sign != 0]
    return int(np.sum(sign[1:] != sign[:-1])) if len(sign) > 1 else 0


def reversals(values, amplitude):
    count, direction, extreme = 0, 0, values[0] if len(values) else 0.0
    for v in values[1:]:
        if direction >= 0 and v < extreme - amplitude:
            count += direction > 0
            direction, extreme = -1, v
        elif direction <= 0 and v > extreme + amplitude:
            count += direction < 0
            direction, extreme = 1, v
        elif (direction >= 0 and v > extreme) or (direction <= 0 and v < extreme):
            extreme = v
    return count


def command_metrics(c):
    if len(c) < 10:
        return {}
    minutes = max(1e-6, (c[-1, 0] - c[0, 0]) / 60.0)
    dv = np.hypot(np.diff(c[:, 1]), np.diff(c[:, 2]))
    dyaw = np.diff(c[:, 3])
    return {
        "samples": int(len(c)),
        "d_v_rms": float(np.sqrt(np.mean(dv ** 2))),
        "d_v_p99": float(np.percentile(dv, 99)),
        "d_yaw_rms": float(np.sqrt(np.mean(dyaw ** 2))),
        "lat_flips_per_min": flips(c[:, 2], 0.02) / minutes,
        "yaw_flips_per_min": flips(c[:, 3], 0.05) / minutes,
        "fwd_reversals_per_min": reversals(c[:, 1], 0.03) / minutes,
        "lat_rms": float(np.sqrt(np.mean(c[:, 2] ** 2))),
    }


def truth_metrics(tr, rate=25.0):
    if len(tr) < 50:
        return {}, None
    t = tr[:, 0]
    keep = np.concatenate([[True], np.diff(t) > 0.01])
    tr, t = tr[keep], t[keep]
    grid = np.arange(t[0], t[-1], 1.0 / rate)
    if len(grid) < 50:
        return {}, None
    x, y = np.interp(grid, t, tr[:, 1]), np.interp(grid, t, tr[:, 2])
    yaw = np.interp(grid, t, np.unwrap(tr[:, 4]))
    roll, pitch = np.interp(grid, t, tr[:, 5]), np.interp(grid, t, tr[:, 6])
    win = int(rate)
    kernel = np.ones(win) / win
    hf = []
    for a in (roll, pitch):
        smooth = np.convolve(a, kernel, mode="same")
        hf.append((a - smooth)[win:-win])
    tilt_hf = np.hypot(hf[0], hf[1])
    dt = 1.0 / rate
    # 位置先做 0.2 s 平滑再差分，压住真值量化噪声
    k5 = np.ones(5) / 5
    xs, ys = np.convolve(x, k5, "same"), np.convolve(y, k5, "same")
    vx, vy = np.gradient(xs, dt), np.gradient(ys, dt)
    ax, ay = np.gradient(vx, dt), np.gradient(vy, dt)
    jx, jy = np.gradient(ax, dt), np.gradient(ay, dt)
    yaw_rate = np.gradient(np.convolve(yaw, k5, "same"), dt)
    yaw_acc = np.gradient(yaw_rate, dt)
    core = slice(8, -8)
    peak = None
    signal = hf[0] + 1j * hf[1]
    if len(signal) > 64:
        spectrum = np.abs(np.fft.fft(signal * np.hanning(len(signal)))) ** 2
        freqs = np.abs(np.fft.fftfreq(len(signal), dt))
        band = (freqs >= 0.3) & (freqs <= 6.0)
        if band.any():
            peak = float(freqs[band][np.argmax(spectrum[band])])
    return {
        "tilt_hf_deg_rms": float(np.degrees(np.sqrt(np.mean(tilt_hf ** 2)))),
        "tilt_hf_deg_p95": float(np.degrees(np.percentile(tilt_hf, 95))),
        "tilt_deg_rms": float(np.degrees(np.sqrt(np.mean(roll ** 2 + pitch ** 2)))),
        "acc_rms": float(np.sqrt(np.mean(ax[core] ** 2 + ay[core] ** 2))),
        "jerk_rms": float(np.sqrt(np.mean(jx[core] ** 2 + jy[core] ** 2))),
        "yaw_acc_rms": float(np.sqrt(np.mean(yaw_acc[core] ** 2))),
        "tilt_peak_hz": peak,
    }, (grid, np.degrees(roll), np.degrees(pitch))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trial", type=Path)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()
    cmds, truth, modes, goal_t, finished_t = load(args.trial)
    start = goal_t if goal_t is not None else cmds[0, 0]
    end = finished_t if finished_t is not None else cmds[-1, 0]
    result = {"window_s": [start, end]}
    cmode = mode_at(modes, cmds[:, 0])
    tmode = mode_at(modes, truth[:, 0]) if len(truth) else np.array([])
    for name in ("exploration", "homing", "corridor", "all"):
        cm = (cmds[:, 0] >= start) & (cmds[:, 0] <= end)
        tm = (truth[:, 0] >= start) & (truth[:, 0] <= end) if len(truth) else np.array([], bool)
        if name != "all":
            cm &= cmode == name
            tm &= tmode == name
        entry = command_metrics(cmds[cm])
        tmetrics, series = truth_metrics(truth[tm]) if tm.any() else ({}, None)
        entry.update(tmetrics)
        if entry:
            entry["duration_s"] = float(cm.sum() / 50.0)
            result[name] = entry
        if name == "all" and args.plot and series is not None:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
            c = cmds[cm]
            axes[0].plot(c[:, 0] - start, c[:, 1], lw=.7, label="v_fwd")
            axes[0].plot(c[:, 0] - start, c[:, 2], lw=.7, label="v_lat")
            axes[0].legend(); axes[0].set_ylabel("cmd m/s")
            axes[1].plot(c[:, 0] - start, c[:, 3], lw=.7, color="C2"); axes[1].set_ylabel("cmd yaw_rate")
            g, r, p = series
            axes[2].plot(g - start, r, lw=.7, label="roll"); axes[2].plot(g - start, p, lw=.7, label="pitch")
            axes[2].legend(); axes[2].set_ylabel("truth deg"); axes[2].set_xlabel("s since goal")
            fig.tight_layout(); fig.savefig(args.trial / "wobble.png", dpi=110)
    (args.trial / "wobble.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
    for name in ("exploration", "homing", "corridor", "all"):
        if name in result:
            e = result[name]
            print(f"{name:12s} {e.get('duration_s', 0):6.1f}s dV={e.get('d_v_rms', 0):.4f} "
                  f"dYaw={e.get('d_yaw_rms', 0):.4f} latFlip={e.get('lat_flips_per_min', 0):5.1f}/min "
                  f"yawFlip={e.get('yaw_flips_per_min', 0):5.1f}/min fwdRev={e.get('fwd_reversals_per_min', 0):5.1f}/min "
                  f"tiltHF={e.get('tilt_hf_deg_rms', float('nan')):.2f}° acc={e.get('acc_rms', float('nan')):.3f} "
                  f"jerk={e.get('jerk_rms', float('nan')):.2f} yawAcc={e.get('yaw_acc_rms', float('nan')):.2f} "
                  f"peak={e.get('tilt_peak_hz')}")


if __name__ == "__main__":
    main()
