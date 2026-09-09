#!/usr/bin/env python3
"""Compare completed trial artifacts without ROS, simulation, or log parsing."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re


DEFAULT_ROOT = Path("/home/yk/acfly_ws/sim_runs")
LEGACY_V1_TRIALS = {
    "20260909_seed13_dense_corridor",
    "20260909_seed14_dense_corridor",
    "20260909_seed15_dense_corridor",
}
COUNTERS = ("replan_checks", "astar_searches", "adoptions", "smooth_fallbacks", "recovery_entries")
ACTIVE_STATUSES = {"running", "starting", "initializing", "pending"}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def read_events(path):
    result = dict(goal=None, coverage90=None, coverage90_source=None, corridor=None,
                  finished=None, mission_finished=None, landed=None, post_landing_disarm=None,
                  counter_last={}, counter_max={}, event_counts={})
    if not path.exists():
        result["status"] = "events_missing"
        return result
    coverage_first = diagnostic_first = None
    coverage_messages = 0
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            kind, now = event.get("kind"), event.get("t_wall")
            if kind is None or now is None:
                continue
            result["event_counts"][kind] = result["event_counts"].get(kind, 0) + 1
            if kind == "goal" and result["goal"] is None:
                result["goal"] = now
            elif kind == "coverage":
                coverage_messages += 1
                if result["goal"] is not None and now >= result["goal"] and event.get("ratio", 0) >= .9:
                    if coverage_first is None:
                        coverage_first = now
            elif kind == "corridor_active" and event.get("value") and result["corridor"] is None:
                result["corridor"] = now
            elif kind == "finished" and event.get("value"):
                if result["mission_finished"] is None:
                    result["mission_finished"] = now
                if result["corridor"] is not None and now >= result["corridor"] and result["finished"] is None:
                    result["finished"] = now
            elif kind == "extended_state" and event.get("landed_state") == 1:
                if result["mission_finished"] is not None and now >= result["mission_finished"] and result["landed"] is None:
                    result["landed"] = now
            elif kind == "state" and event.get("armed") is False:
                if result["landed"] is not None and now >= result["landed"] and result["post_landing_disarm"] is None:
                    result["post_landing_disarm"] = now
            elif kind == "diagnostics":
                values = event.get("values", {})
                ratio = values.get("coverage_ratio")
                if isinstance(ratio, (int, float)) and ratio >= .9 and result["goal"] is not None and now >= result["goal"]:
                    if diagnostic_first is None:
                        diagnostic_first = now
                for key in COUNTERS:
                    value = values.get(key)
                    if isinstance(value, (int, float)):
                        result["counter_last"][key] = value
                        result["counter_max"][key] = max(value, result["counter_max"].get(key, value))
    result["coverage90"] = coverage_first if coverage_messages else diagnostic_first
    result["coverage90_source"] = "coverage" if coverage_messages else "diagnostics" if diagnostic_first is not None else None
    result["status"] = "parsed"
    return result


def binary_info(directory, run, root, package, executable):
    metadata = run.get("binaries", {}).get(package, {})
    expected = metadata.get("sha256")
    path = directory / "snapshots/bin" / executable
    source = "trial_binary_snapshot"
    if not path.exists() and metadata.get("executable_path"):
        candidate = Path(metadata["executable_path"])
        # Installed executables may have changed since flight; only immutable
        # trial snapshots are evidence for an already completed trial.
        if "snapshots" in candidate.parts and candidate.exists():
            path = candidate
    if path.exists():
        actual = sha256(path)
        return dict(sha256=actual, short_hash=actual[:10], source=source, path=str(path.resolve()),
                    recorded_sha256=expected, metadata_matches=actual == expected if expected else None)
    if expected:
        return dict(sha256=expected, short_hash=expected[:10], source="run_binary_metadata",
                    path=metadata.get("executable_path"), metadata_matches=None)
    if directory.name in LEGACY_V1_TRIALS:
        path = root / "builds/dense_corridor_v1" / executable
        if path.exists():
            actual = sha256(path)
            return dict(sha256=actual, short_hash=actual[:10], source="documented_dense_v1_mapping",
                        path=str(path.resolve()), metadata_matches=None,
                        evidence="tools/sim/EXPERIMENTS.md: seeds 13/14/15 dense used these saved v1 binaries")
    return dict(sha256=None, short_hash="unknown", source="unknown", path=None, metadata_matches=None)


def audit_index(root):
    by_path = {}
    for path in sorted(root.rglob("audit_footprint*.json"), key=lambda item: (item.stat().st_mtime_ns, str(item))):
        try:
            report = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        for trial in report.get("trials", []):
            if trial.get("trial_dir"):
                by_path[str(Path(trial["trial_dir"]).resolve())] = (path, trial)
    return by_path


def elapsed(start, end):
    return end - start if start is not None and end is not None and end >= start else None


def trial_summary(directory, run, summary, root, audits):
    timing = read_events(directory / "events.jsonl")
    world = directory / "world.sdf"
    planner = binary_info(directory, run, root, "exploration_planner", "exploration_planner_node")
    mission = binary_info(directory, run, root, "fly_mission", "fly_mission_node")
    seed = run.get("seed")
    if seed is None:
        match = re.search(r"(?:^|_)seed(\d+)(?:_|$)", directory.name)
        seed = int(match.group(1)) if match else None
    phases = summary.get("phases", {})
    exploration = phases.get("exploration", {})
    monitor = {phase: phases.get(phase, {}).get("minimum_surface_clearance_m")
               for phase in ("exploration", "corridor", "landing")}
    monitor["all"] = summary.get("metrics", {}).get("minimum_surface_clearance_m")
    counters = {}
    for key in COUNTERS:
        recorded = summary.get("planner", {}).get("counters", {}).get(key, {})
        counters[key] = dict(last=timing["counter_last"].get(key, recorded.get("last")),
                             max=timing["counter_max"].get(key, recorded.get("max")))
    auto = run.get("auto_disarm", {})
    automatic = auto.get("auto_disarm_observed")
    if automatic is not True:
        automatic = False if auto.get("timed_out") else None
    quality = summary.get("success")
    # Early runners stopped immediately on landing. Their missing auto-disarm
    # observation is unknown, not proof that automatic disarm failed.
    overall = False if quality is False or automatic is False or run.get("success") is False else \
              True if quality is True and automatic is True else None
    audit = None
    if str(directory.resolve()) in audits:
        source, audited = audits[str(directory.resolve())]
        method = audited.get("methods", {}).get("attitude_projected_enclosures", {})
        audit = dict(source=str(source.resolve()), method="attitude_projected_enclosures",
                     minimum_clearance_m={phase: method.get("phases", {}).get(phase, {}).get("minimum_clearance_m")
                                          for phase in ("exploration", "corridor", "landing")},
                     all_minimum_clearance_m=method.get("all", {}).get("minimum_clearance_m"),
                     projected_overlap_episode_count=method.get("all", {}).get("overlap_episode_count"))
    overrides = {}
    for name in ("planner_override.yaml", "mission_override.yaml"):
        path = directory / "snapshots" / name
        if path.exists():
            overrides[name] = sha256(path)
    runner_status = run.get("status", "manual_recording")
    blocked = runner_status == "mission_blocked"
    outcome_evidence = run.get("mission_blocked", {}) if blocked else run.get("flight_failure", {})
    return dict(trial=directory.name, trial_dir=str(directory.resolve()), seed=seed,
                runner_status=runner_status, runner_reason=outcome_evidence.get("reason"),
                mission_blocked=blocked, started_at=run.get("started_at"),
                planner_binary=planner, mission_binary=mission,
                world_sha256=sha256(world) if world.exists() else None, runtime_override_sha256=overrides,
                quality_success=quality, runner_success=run.get("success"), overall_success=overall,
                automatic_disarm_observed=automatic,
                post_landing_disarm_observed=timing["post_landing_disarm"] is not None,
                automatic_disarm_source="runner_auto_disarm" if auto else "not_recorded_by_early_runner",
                mission_complete=summary.get("mission_complete"), landed_after_mission=summary.get("landed_after_mission"),
                goal_to_coverage90_s=elapsed(timing["goal"], timing["coverage90"]),
                goal_to_handoff_s=elapsed(timing["goal"], timing["corridor"]),
                handoff_to_H_s=elapsed(timing["corridor"], timing["finished"]),
                exploration_truth_length_m=exploration.get("trajectory_length_m", {}).get("truth"),
                exploration_near_zero_duration_s=exploration.get("zero_command_duration_s"),
                exploration_near_zero_count=exploration.get("zero_command_count"),
                planner_counters=counters,
                cylindrical_monitor=dict(radius_m=summary.get("configuration", {}).get("robot_radius"),
                                         minimum_clearance_m=monitor, overlap_count=summary.get("collision_proxy_count")),
                footprint_audit=audit, event_timing=timing,
                binary_metadata_mismatch=planner.get("metadata_matches") is False or mission.get("metadata_matches") is False)


def number(value, decimals=2):
    return f"{value:.{decimals}f}" if isinstance(value, (float, int)) else "unknown"


def flag(value):
    return "yes" if value is True else "no" if value is False else "unknown"


def markdown(report):
    lines = ["# Iteration Summary", "", f"Generated: {report['generated_at']}", "",
             "Only finalized trial artifacts are included. Goal/coverage/handoff/H times come from structured events, not log text. Different maps, binaries, or parameter overrides are not controlled before/after comparisons.", "",
             "| Trial | Seed | Planner ELF | World | Runner status / reason | Quality / Auto-disarm | Goal to 90% s | Goal to handoff s | Handoff to H s | Explore truth m | Near-zero s / count | A* / adopt / fallback / recovery | Monitor min m | Part audit min E / C m |",
             "| --- | ---: | --- | --- | --- | --- | ---: | ---: | ---: | ---: | --- | --- | ---: | --- |"]
    for row in report["trials"]:
        audit = row.get("footprint_audit")
        audit_values = audit["minimum_clearance_m"] if audit else {}
        counter_text = " / ".join(number(row["planner_counters"][key]["last"], 0)
                                  for key in ("astar_searches", "adoptions", "smooth_fallbacks", "recovery_entries"))
        lines.append("| " + " | ".join([
            row["trial"], str(row["seed"]) if row["seed"] is not None else "unknown",
            row["planner_binary"]["short_hash"], row["world_sha256"][:10] if row["world_sha256"] else "unknown",
            row["runner_status"] + (" / " + row["runner_reason"] if row["runner_reason"] else ""),
            ("blocked" if row["mission_blocked"] else flag(row["quality_success"])) + " / " + flag(row["automatic_disarm_observed"]),
            number(row["goal_to_coverage90_s"]), number(row["goal_to_handoff_s"]), number(row["handoff_to_H_s"]),
            number(row["exploration_truth_length_m"]), number(row["exploration_near_zero_duration_s"]) + " / " + number(row["exploration_near_zero_count"], 0),
            counter_text, number(row["cylindrical_monitor"]["minimum_clearance_m"]["all"], 4),
            number(audit_values.get("exploration"), 4) + " / " + number(audit_values.get("corridor"), 4),
        ]) + " |")
    lines.extend(["", "## Definitions", ""])
    lines.extend("- " + note for note in report["definitions"])
    mismatches = [row["trial"] for row in report["trials"] if row["binary_metadata_mismatch"]]
    if mismatches:
        lines.extend(["", "## Binary Mismatches", "", "Copied ELF hashes disagree with run metadata: " + ", ".join(mismatches)])
    if report["skipped"]:
        lines.extend(["", "## Skipped", ""])
        lines.extend(f"- {item['trial']}: {item['reason']}" for item in report["skipped"])
        lines.append("Skipped/in-progress artifacts are not counted as failed trials.")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", nargs="*", type=Path, help="Explicit trial directories; finalized manual recordings may lack run.json")
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT, help="Scan immediate child directories when no explicit trials are supplied")
    parser.add_argument("--output-dir", type=Path, help="Defaults to ROOT/comparisons")
    args = parser.parse_args()
    root = args.root.resolve()
    explicit = bool(args.trials)
    directories = [path.resolve() for path in args.trials] if explicit else sorted(
        path for path in root.iterdir() if path.is_dir() and
        ((path / "summary.json").exists() or (path / "run.json").exists()))
    audits = audit_index(root)
    report = dict(generated_at=datetime.now(timezone.utc).isoformat(), root=str(root), trials=[], skipped=[],
                  definitions=[
                      "Planner ELF is the first 10 SHA-256 characters of the actual copied executable, or its saved run metadata. Mission ELF and runtime YAML override hashes are retained in JSON. Installed binaries and source diffs are never used to infer a previous runtime.",
                      "The exact documented seeds 13/14/15 dense trial names map to builds/dense_corridor_v1. Earlier trials without binary evidence remain unknown; a seed number alone never selects a version.",
                      "Quality is the recorder's completion/landing/cylindrical-monitor result. Automatic disarm requires the runner's explicit observation. Early runners that stopped at touchdown retain unknown auto-disarm, not a false failure. Overall success in JSON requires both.",
                      "mission_blocked is shown as blocked with its structured runner reason. An invalid-goal safety rejection is neither full mission completion nor a collision failure; its original quality_success=false stays unchanged in JSON, and blocked cases are counted separately.",
                      "First 90% coverage uses Float64 coverage events, with diagnostic coverage only as a fallback when that topic is absent. Missing coverage events remain unknown. H means the first finished=true after corridor activation.",
                      "Exploration means goal receipt to corridor activation, including homing to the handoff point. Truth length is the recorder's sampled Gazebo XY length. Near-zero commands are below 0.035 m/s for at least 0.4 s under the recorded configuration.",
                      "Monitor min is the original cylindrical monitoring envelope minimum across phases. Its radius is recorded per trial in JSON; the usual 0.32 m single-axis radius is not an all-yaw conservative vehicle footprint.",
                      "Part audit E/C is separately reported exploration/corridor clearance from optional audit_footprint*.json files under ROOT; the newest matching audit is used. It never replaces or changes the cylindrical monitor. Neither metric is a contact-sensor observation or continuous-time guarantee.",
                  ])
    for directory in dict.fromkeys(directories):
        summary_path, run_path = directory / "summary.json", directory / "run.json"
        if not summary_path.exists() or (not explicit and not run_path.exists()):
            report["skipped"].append(dict(trial=directory.name, reason="missing finalized summary.json or run.json"))
            continue
        try:
            summary = json.loads(summary_path.read_text())
            run = json.loads(run_path.read_text()) if run_path.exists() else {}
        except (OSError, json.JSONDecodeError) as error:
            report["skipped"].append(dict(trial=directory.name, reason="artifact read incomplete: " + str(error)))
            continue
        if run.get("status") in ACTIVE_STATUSES or (run and not run.get("ended_at")):
            report["skipped"].append(dict(trial=directory.name, reason="run not finalized"))
            continue
        if "termination" not in summary:
            report["skipped"].append(dict(trial=directory.name, reason="summary not finalized"))
            continue
        report["trials"].append(trial_summary(directory, run, summary, root, audits))
    report["trials"].sort(key=lambda row: (row.get("started_at") or row["trial"], row["trial"]))
    report["counts"] = dict(finalized=len(report["trials"]), skipped=len(report["skipped"]),
                          quality_successes=sum(row["quality_success"] is True for row in report["trials"]),
                          quality_failures=sum(row["quality_success"] is False and not row["mission_blocked"] for row in report["trials"]),
                          blocked=sum(row["mission_blocked"] for row in report["trials"]),
                          quality_unknown=sum(row["quality_success"] is None for row in report["trials"]),
                          success_with_automatic_disarm=sum(row["overall_success"] is True for row in report["trials"]))
    output = args.output_dir or root / "comparisons"
    output.mkdir(parents=True, exist_ok=True)
    atomic_json(output / "iteration_summary.json", report)
    temporary = output / "iteration_summary.md.tmp"
    temporary.write_text(markdown(report), encoding="utf-8")
    temporary.replace(output / "iteration_summary.md")
    print(json.dumps(dict(output_dir=str(output.resolve()), **report["counts"]), indent=2))


if __name__ == "__main__":
    main()
