#!/usr/bin/env python3
"""Analyze a snapshot of a running or finished trial without ROS dependencies."""

import argparse
from bisect import bisect_right
from collections import Counter
import csv
import io
import json
import math
from pathlib import Path

from record_trial import Obstacle, TrialRecorder, atomic_json, clearance_at, planner_metrics


def read_rows(path):
    text = path.read_text(encoding="utf-8")
    # The writer may currently be appending its last row.
    if not text.endswith("\n"):
        text = text.rsplit("\n", 1)[0] + "\n"
    rows = []
    string_keys = {"phase", "state_mode", "nearest_obstacle", "planner_mode", "planner_gate_reason"}
    for raw in csv.DictReader(io.StringIO(text)):
        if None in raw or any(value is None for value in raw.values()):
            continue
        row = {}
        for key, value in raw.items():
            if value == "":
                row[key] = None
            elif key in string_keys:
                row[key] = value
            elif value in ("True", "False"):
                row[key] = value == "True"
            else:
                row[key] = float(value)
        rows.append(row)
    return rows


def read_events(path, cutoff):
    events = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event["t_wall"] <= cutoff:
                events.append(event)
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, help="Existing recorder trial directory")
    parser.add_argument("--robot-radius", "--body-radius", dest="robot_radius", type=float)
    parser.add_argument("--robot-half-height", type=float)
    parser.add_argument("--world-origin-x", type=float)
    parser.add_argument("--world-origin-y", type=float)
    parser.add_argument("--plot-active-path", action="store_true", default=None)
    args = parser.parse_args()
    directory = Path(args.output_dir)
    old_summary_path = directory / "summary.json"
    old_summary = json.loads(old_summary_path.read_text()) if old_summary_path.exists() else {}
    config = dict(max_pose_age=.5, stop_speed=.035, stop_min_duration=.4, yaw_reversal_threshold=.15,
                  robot_radius=.32, robot_half_height=.15, world_origin_x=-4.0, world_origin_y=0.0,
                  plot_active_path=False)
    config_path = directory / "recorder_config.json"
    if config_path.exists():
        config.update(json.loads(config_path.read_text()))
    config.update(old_summary.get("configuration", {}))
    config.update({key: value for key, value in vars(args).items() if value is not None})
    recorder = TrialRecorder.__new__(TrialRecorder)
    recorder.args = argparse.Namespace(**config)
    recorder.output = directory
    geometry = json.loads((directory / "world_geometry.json").read_text())
    recorder.obstacles = [Obstacle(**item) for item in geometry["obstacles"]]
    rows = read_rows(directory / "samples.csv")
    if not rows:
        raise SystemExit("No complete samples available yet")
    events = read_events(directory / "events.jsonl", rows[-1]["t_wall"])
    truths = [event for event in events if event["kind"] == "truth"]
    truth_times = [event["t_wall"] for event in truths]
    for row in rows:
        index = bisect_right(truth_times, row["t_wall"]) - 1
        truth_event = truths[index] if index >= 0 else None
        if "truth_sim_at_receive" not in row:
            row["truth_sim_at_receive"] = truth_event.get("sim_at_receive", truth_event["t_sim"]) if truth_event else None
        if row["truth_age_s"] is not None and row["truth_age_s"] < config["max_pose_age"]:
            clearance, obstacle = clearance_at(recorder.obstacles, row["truth_x"], row["truth_y"], row["truth_z"],
                                               config["robot_radius"], config["robot_half_height"])
            row["surface_clearance_m"], row["nearest_obstacle"] = clearance, obstacle
            row["collision_proxy"] = clearance < 0 if clearance is not None else None
    recorder.rows = rows
    recorder.started_flying_at = None
    started_flying_sim = None
    for row in rows:
        altitude = row["truth_z"] if row["truth_z"] is not None else row["mavros_z"]
        if row["state_armed"] and altitude is not None and altitude > .30:
            recorder.started_flying_at, started_flying_sim = row["t_wall"], row["t_sim"]
            break
    recorder.route = []
    recorder.active_path = []
    goal = None
    finished = None
    for event in events:
        if event["kind"] == "corridor_route":
            recorder.route = event["points"]
        elif event["kind"] == "active_path":
            recorder.active_path = event["points"]
        elif event["kind"] == "goal":
            goal = event["point"]
        elif event["kind"] == "finished" and event["value"] and finished is None:
            finished = event
    all_metrics = recorder.metrics(rows)
    flight_rows = [row for row in rows if recorder.started_flying_at is not None and row["t_wall"] >= recorder.started_flying_at]
    coverage = sum(row["truth_age_s"] is not None and row["truth_age_s"] < config["max_pose_age"] for row in flight_rows) / len(flight_rows) if flight_rows else None
    clock_resets = sum(event["kind"] == "clock_reset" for event in events)
    mission_complete = finished is not None or any(row["mission_complete"] for row in rows)
    landed_after_mission = any(row["landed_after_mission"] for row in rows)
    phases = {phase: recorder.metrics([row for row in rows if row["phase"] == phase]) for phase in ("startup", "exploration", "corridor", "landing")}
    min_row = min((row for row in rows if row["surface_clearance_m"] is not None),
                  key=lambda row: row["surface_clearance_m"], default=None)
    summary = {
        "source": "Read-only snapshot replay; does not overwrite recorder summary or progress",
        "wall_elapsed_s": rows[-1]["t_wall"], "snapshot_sample_count": len(rows),
        "mission_complete": mission_complete, "landed_after_mission": landed_after_mission,
        "success": (mission_complete and landed_after_mission and all_metrics["collision_proxy_count"] == 0)
                   if coverage is not None and coverage >= .99 and clock_resets == 0 else None,
        "ground_truth_status": "observed" if truths else "unknown",
        "ground_truth_sample_count": len(truths), "ground_truth_flight_coverage": coverage,
        "collision_proxy_count": all_metrics["collision_proxy_count"],
        "clock_resets": clock_resets, "configuration": config,
        "finished_at_wall_s": finished["t_wall"] if finished else None,
        "finished_at_sim_s": finished["t_sim"] if finished else None,
        "started_flying_wall_s": recorder.started_flying_at,
        "mission_duration_wall_s": finished["t_wall"] - recorder.started_flying_at if finished and recorder.started_flying_at is not None else None,
        "mission_duration_sim_s": finished["t_sim"] - started_flying_sim if finished and started_flying_sim is not None and not clock_resets else None,
        "topic_message_counts": dict(Counter(event["kind"] for event in events)),
        "goal_slam": goal, "corridor_route_slam": recorder.route,
        "planner": planner_metrics(rows), "latest_active_path_slam": recorder.active_path,
        "metrics": all_metrics, "phases": phases,
        "recent_windows": {str(seconds) + "s": recorder.metrics([row for row in rows if row["t_wall"] >= rows[-1]["t_wall"] - seconds])
                           for seconds in (10, 30, 60)},
        "current": {key: rows[-1].get(key) for key in ("t_wall", "t_sim", "phase", "truth_x", "truth_y", "truth_z", "truth_yaw", "slam_x", "slam_y", "cmd_vx", "cmd_vy", "cmd_yaw_rate", "surface_clearance_m", "nearest_obstacle", "coverage_ratio", "planner_mode", "planner_target_x", "planner_target_y", "planner_gate_center_x", "planner_gate_center_y", "planner_gate_reason")},
        "minimum_clearance_sample": {key: min_row[key] for key in ("t_wall", "t_sim", "phase", "truth_x", "truth_y", "truth_z", "surface_clearance_m", "nearest_obstacle")} if min_row else None,
        "metric_notes": {
            "actual_yaw_rate": "Uses truth header stamps, with truth-event bridged /clock fallback when stamps are zero.",
            "collision": "Geometric aircraft cylinder-envelope overlap with vertical SDF collisions; not a contact sensor. Ground and pad disks excluded.",
            "yaw_reversals": "Includes intended alternating turns; inspect trajectory to distinguish oscillation.",
        },
    }
    fresh = [row for row in rows if row["truth_age_s"] is not None and row["truth_age_s"] < config["max_pose_age"]]
    if fresh:
        recent = [row for row in fresh if row["t_wall"] >= fresh[-1]["t_wall"] - 1.0]
        if len(recent) >= 2 and recent[-1]["truth_sim_at_receive"] is not None and recent[0]["truth_sim_at_receive"] is not None:
            elapsed = recent[-1]["truth_sim_at_receive"] - recent[0]["truth_sim_at_receive"]
            if elapsed > 0:
                summary["current"]["truth_horizontal_speed_1s_mps"] = math.hypot(recent[-1]["truth_x"] - recent[0]["truth_x"], recent[-1]["truth_y"] - recent[0]["truth_y"]) / elapsed
    atomic_json(directory / "summary_replay.json", summary)
    recorder.plot(summary, "trajectory_live.png")
    print(json.dumps({"plot": str(directory / "trajectory_live.png"), "summary": str(directory / "summary_replay.json"),
                      "current": summary["current"],
                      "metrics": {key: value for key, value in all_metrics.items() if key != "zero_command_segments"}}, indent=2))


if __name__ == "__main__":
    main()
