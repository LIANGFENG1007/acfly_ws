#!/usr/bin/env python3
"""拟合"规划器指令 → 飞机实际运动"的响应滞后(纯延迟 + 一阶时间常数)，并给出 Smith 预估参数建议。

输入二选一：
  ros2 bag 目录(实机)：需要 /exploration/cmd_vel(TwistStamped，机体系) 与 /aft_mapped_to_init(Odometry)。
      录制：ros2 bag record /exploration/cmd_vel /aft_mapped_to_init
  run_trial 试验目录(仿真)：读 events.jsonl 的 cmd 与 truth。
只用探索巡航期间的数据：取指令非零的时间段。

用法：/usr/bin/python3 tools/sim/fit_response.py <bag目录或试验目录> [--start S] [--end S]
参考(2026-09-23 仿真 r01)：前进速度 延迟 0.38 s + τ 0.30 s；角速度 延迟 0.16 s + τ 0.22 s；
    当前默认 carlike_smith_s=0.30、carlike_smith_yaw_s=0.20 就是按这组值定的。
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

SIM_REF = {"fwd": (0.38, 0.30), "yaw": (0.16, 0.22)}
DEFAULT_SMITH = {"fwd": 0.30, "yaw": 0.20}


def load_trial(path):
    cmds, poses = [], []
    for line in (path / "events.jsonl").open():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if d.get("kind") == "cmd":
            cmds.append((d["t_wall"], d["vx"], d["yaw_rate"]))
        elif d.get("kind") == "truth" and d.get("sim_at_receive") is not None:
            poses.append((d["t_wall"], d["x"], d["y"], d["yaw"]))
    return np.array(cmds), np.array(poses)


def load_bag(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    storage = "mcap" if any(path.glob("*.mcap")) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cmds, poses = [], []
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        t = stamp * 1e-9
        if topic == "/exploration/cmd_vel":
            m = deserialize_message(raw, get_message(types[topic]))
            cmds.append((t, m.twist.linear.x, m.twist.angular.z))
        elif topic == "/aft_mapped_to_init":
            m = deserialize_message(raw, get_message(types[topic]))
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            poses.append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    return np.array(cmds), np.array(poses)


def fit(cmd, meas, dt):
    best = None
    for delay_steps in range(0, int(1.2 / dt)):
        for tau in np.arange(0.02, 1.2, 0.02):
            model = np.zeros_like(cmd)
            state = 0.0
            for i in range(len(cmd)):
                u = cmd[i - delay_steps] if i >= delay_steps else 0.0
                state += (u - state) * dt / tau
                model[i] = state
            err = float(np.mean((model[25:-25] - meas[25:-25]) ** 2))
            if best is None or err < best[0]:
                best = (err, delay_steps * dt, tau)
    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--start", type=float, default=None, help="相对首条指令的起始秒")
    parser.add_argument("--end", type=float, default=None)
    args = parser.parse_args()
    cmds, poses = load_trial(args.source) if (args.source / "events.jsonl").exists() else load_bag(args.source)
    if len(cmds) < 100 or len(poses) < 50:
        raise SystemExit("数据不足：需要指令与位姿各至少几十秒")
    _, unique = np.unique(poses[:, 0], return_index=True)
    poses = poses[unique]
    active = cmds[np.abs(cmds[:, 1]) + np.abs(cmds[:, 2]) > 1e-3]
    t0 = active[0, 0] + (args.start or 0.0)
    t1 = active[-1, 0] if args.end is None else active[0, 0] + args.end
    dt = 0.02
    t = np.arange(max(t0, poses[0, 0]) + 0.5, min(t1, poses[-1, 0]) - 0.5, dt)
    if len(t) < 500:
        raise SystemExit("有效时段太短(<10 s)")
    k = np.ones(5) / 5
    x = np.convolve(np.interp(t, poses[:, 0], poses[:, 1]), k, "same")
    y = np.convolve(np.interp(t, poses[:, 0], poses[:, 2]), k, "same")
    yaw = np.convolve(np.interp(t, poses[:, 0], np.unwrap(poses[:, 3])), k, "same")
    vx, vy, yaw_rate = np.gradient(x, dt), np.gradient(y, dt), np.gradient(yaw, dt)
    forward = np.cos(yaw) * vx + np.sin(yaw) * vy
    cmd_fwd = np.interp(t, cmds[:, 0], cmds[:, 1])
    cmd_yaw = np.interp(t, cmds[:, 0], cmds[:, 2])
    print(f"数据 {t[-1] - t[0]:.1f} s")
    result = {}
    for name, c, m in (("fwd", cmd_fwd, forward), ("yaw", cmd_yaw, yaw_rate)):
        err, delay, tau = fit(c, m, dt)
        ref_d, ref_tau = SIM_REF[name]
        scale = (delay + tau) / (ref_d + ref_tau)
        result[name] = (delay, tau, scale)
        label = "前进速度" if name == "fwd" else "航向角速度"
        print(f"{label}: 延迟 {delay:.2f} s + 时间常数 {tau:.2f} s  (仿真参考 {ref_d:.2f}+{ref_tau:.2f}，总滞后 ×{scale:.2f}，残差 {err:.2e})")
    s_fwd = DEFAULT_SMITH["fwd"] * result["fwd"][2]
    s_yaw = DEFAULT_SMITH["yaw"] * result["yaw"][2]
    print(f"建议：-p carlike_smith_s:={s_fwd:.2f} -p carlike_smith_yaw_s:={s_yaw:.2f}")
    worst = max(result["fwd"][2], result["yaw"][2])
    if worst > 1.4:
        print("滞后明显大于仿真(>×1.4)：离线台架显示此时横向纠偏偏弱，另建议 -p carlike_drift_kd:=0.8")


if __name__ == "__main__":
    main()
