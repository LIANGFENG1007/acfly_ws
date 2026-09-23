#!/usr/bin/env python3
"""一轮试验结束后的统一报告：跑 analyze_tracking / analyze_wobble，汇总成一行追加到 sim_runs/LOG.md。

用法：trial_report.py TRIAL_DIR ["本轮改动说明"]
"""
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
trial = Path(sys.argv[1]).resolve()
note = sys.argv[2] if len(sys.argv) > 2 else ""
py = "/usr/bin/python3"
subprocess.run([py, str(HERE / "analyze_tracking.py"), str(trial)], capture_output=True)
subprocess.run([py, str(HERE / "analyze_wobble.py"), str(trial), "--plot"], capture_output=True)
summary = json.loads((trial / "summary.json").read_text()) if (trial / "summary.json").exists() else {}
run = json.loads((trial / "run.json").read_text()) if (trial / "run.json").exists() else {}
track = json.loads((trial / "tracking_summary.json").read_text()) if (trial / "tracking_summary.json").exists() else {}
wob = json.loads((trial / "wobble.json").read_text()) if (trial / "wobble.json").exists() else {}
metrics = summary.get("metrics", {})
ex = wob.get("exploration", {})
layout = Path(run.get("layout", "")).stem or f"seed{run.get('seed')}"


def f(v, fmt):
    return format(v, fmt) if isinstance(v, (int, float)) else "-"


row = (f"| {trial.name} | {layout} | {run.get('status', '?')} | {f(track.get('explore_duration'), '.1f')} | "
       f"{f(summary.get('mission_duration_sim_s'), '.1f')} | {f(track.get('cross_track_exploration_m', {}).get('p95'), '.3f')} | "
       f"{f(metrics.get('minimum_surface_clearance_m'), '.3f')} | {f(ex.get('yaw_flips_per_min'), '.1f')} | "
       f"{f(ex.get('fwd_reversals_per_min'), '.1f')} | {f(ex.get('d_yaw_rms'), '.4f')} | {f(ex.get('jerk_rms'), '.2f')} | "
       f"{f(ex.get('tilt_hf_deg_rms'), '.2f')} | {f(track.get('crawl_total_s'), '.1f')} | {note} |")
log = trial.parent / "LOG.md"
if not log.exists():
    log.write_text("# 仿真试验日志\n\n探索段晃动指标见 analyze_wobble.py；贴线/爬行见 analyze_tracking.py。\n\n"
                   "| 试验 | 布局 | 状态 | 探索s | 全程s | 贴线p95 | 最小间距 | 航向翻转/min | 前进反复/min | dYaw | jerk | 倾角HF° | 爬行s | 改动 |\n"
                   "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n")
with log.open("a") as stream:
    stream.write(row + "\n")
print(row)
