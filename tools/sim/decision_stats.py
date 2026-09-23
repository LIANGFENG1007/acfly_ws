#!/usr/bin/env python3
"""探索决策明确性统计：换路次数/原因、保持旧路、原地转身、钩子拒绝、目标反复、前进限速来源。

用法：decision_stats.py TRIAL [TRIAL ...]
- 换路原因取自 stack.log 的"探索换路[原因]"；
- 目标反复 = 新目标距 20 s 内任一更早目标 < 0.6 m 但中间换过别的目标(来回改主意)；
- 限速来源(v5 起)：探索段 diagnostics 的 tracker_limiter 占比。
"""
import collections
import json
import math
import re
import sys
from pathlib import Path

LIMITERS = {0: "非巡航", 1: "前瞻窗曲率", 2: "速度剖面", 3: "停车律", 4: "航向门", 5: "横向加速度", 6: "实测角速度", 7: "横向速度因子"}


def analyze(trial):
    log = (trial / "stack.log").read_text(errors="replace") if (trial / "stack.log").exists() else ""
    reasons = collections.Counter()
    targets = []
    for m in re.finditer(r"\[(\d+\.\d+)\] \[exploration_planner_node\]: 探索换路\[(\w+)\] → \(([-\d.]+), ([-\d.]+)\)", log):
        t, reason, x, y = float(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4))
        reasons[reason] += 1
        targets.append((t, x, y))
    flipflops = 0
    for i, (t, x, y) in enumerate(targets):
        for j in range(i - 2, -1, -1):
            tj, xj, yj = targets[j]
            if t - tj > 20:
                break
            if math.hypot(x - xj, y - yj) < 0.6:
                flipflops += 1
                break
    kept = log.count("换路评估→保持旧路")
    turns = log.count("原地转身找解成功")
    hooks = log.count("视为锥内无解")
    bypass = log.count("直接采纳并由跟踪器转向")
    unreachable = log.count("暂无路径") + log.count("暂缓观察点")
    limiter = collections.Counter()
    explore_time = None
    ev = trial / "events.jsonl"
    if ev.exists():
        first = last = None
        for line in ev.open():
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            if d.get("kind") != "diagnostics":
                continue
            v = d.get("values", {})
            if v.get("mode") != "exploration":
                continue
            first = first or d["t_wall"]
            last = d["t_wall"]
            if "tracker_limiter" in v:
                limiter[int(float(v["tracker_limiter"]))] += 1
        if first and last:
            explore_time = last - first
    total = sum(limiter.values())
    lim = " ".join(f"{LIMITERS.get(k, k)}{100 * n / total:.0f}%" for k, n in limiter.most_common()) if total else "-"
    minutes = (explore_time or 60) / 60
    print(f"{trial.name:14s} 探索{explore_time or 0:5.1f}s 换路{sum(reasons.values()):3d}({sum(reasons.values()) / minutes:4.1f}/min) "
          f"保持旧路{kept:3d} 转身{turns:2d} 钩子{hooks:2d} 锥外直采{bypass:2d} 暂缓{unreachable:2d} 目标反复{flipflops:2d}")
    print(f"{'':14s} 原因 {dict(reasons.most_common())}")
    print(f"{'':14s} 限速 {lim}")


for arg in sys.argv[1:]:
    analyze(Path(arg))
