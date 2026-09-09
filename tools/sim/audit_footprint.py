#!/usr/bin/env python3
"""Offline SDF-derived footprint audit; never connects to ROS or commands flight.

The original recorder's cylindrical monitoring metrics are left unchanged.
Rotor collision boxes are swept about their SDF joint axes. Full recorded
attitude rotates their centers; a circle encloses each tilted sweep's XY
projection. Static collision boxes use the convex hull of all eight projected
vertices. Vertical interval overlap gates XY checks, not a full 3-D contact test.
"""

import argparse
from dataclasses import dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np
import sdformat14 as sdf
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from record_trial import Obstacle, atomic_json


def pose_matrix(pose):
    q = pose.rot()
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat([q.x(), q.y(), q.z(), q.w()]).as_matrix()
    result[:3, 3] = [pose.pos().x(), pose.pos().y(), pose.pos().z()]
    return result


@dataclass
class Part:
    name: str
    kind: str
    transform: np.ndarray
    size: np.ndarray
    radius: float = 0.0
    half_height: float = 0.0

    def describe(self):
        return dict(name=self.name, kind=self.kind, transform_model=self.transform.tolist(),
                    source_box_size_m=self.size.tolist(), sweep_radius_m=self.radius or None,
                    sweep_half_height_m=self.half_height or None)


def load_vehicle(model_path, models_dir):
    config = sdf.ParserConfig()
    config.add_uri_path("model://", str(models_dir))
    config.set_unrecognized_elements_policy(sdf.EnforcementPolicy.LOG)

    def find(uri):
        path = models_dir / uri.removeprefix("model://")
        return str(path / "model.sdf" if path.is_dir() else path)

    config.set_find_callback(find)
    root = sdf.Root()
    root.load(str(model_path), config)
    model = root.model()
    if model is None:
        raise ValueError("Expected a vehicle model SDF")
    rotors = {}
    for i in range(model.joint_count()):
        joint = model.joint_by_index(i)
        if joint.type() != sdf.JointType.REVOLUTE:
            continue
        axis = joint.axis(0).xyz()
        if abs(axis.x()) > 1e-9 or abs(axis.y()) > 1e-9 or abs(abs(axis.z()) - 1) > 1e-9:
            raise ValueError("Only local-Z rotor joint axes are supported: " + joint.name())
        rotors[joint.child_name()] = pose_matrix(joint.semantic_pose().resolve("__model__"))
    parts = []
    for i in range(model.link_count()):
        link = model.link_by_index(i)
        for j in range(link.collision_count()):
            collision = link.collision_by_index(j)
            geometry = collision.geometry()
            if geometry.type() != sdf.GeometryType.BOX:
                raise ValueError("Unsupported vehicle collision geometry: " + link.name() + "/" + collision.name())
            box = geometry.box_shape().size()
            size = np.array([box.x(), box.y(), box.z()])
            transform = pose_matrix(collision.semantic_pose().resolve("__model__"))
            name = link.name() + "/" + collision.name()
            if link.name() in rotors:
                joint_transform = rotors[link.name()]
                local = np.linalg.inv(joint_transform) @ transform
                corners = box_vertices(size) @ local[:3, :3].T + local[:3, 3]
                radius = float(np.max(np.linalg.norm(corners[:, :2], axis=1)))
                low, high = float(corners[:, 2].min()), float(corners[:, 2].max())
                sweep_transform = joint_transform.copy()
                sweep_transform[:3, 3] += joint_transform[:3, 2] * ((low + high) / 2)
                parts.append(Part(name, "rotor_swept_cylinder", sweep_transform, size,
                                  radius, (high - low) / 2))
            else:
                parts.append(Part(name, "box", transform, size))
    if len([part for part in parts if part.kind == "rotor_swept_cylinder"]) != 4:
        raise ValueError("Expected exactly four rotor collision sweeps")
    source_paths = [model_path, models_dir / "x500/model.sdf", models_dir / "x500_base/model.sdf"]
    return parts, {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in source_paths}


def box_vertices(size):
    return np.array(list(itertools.product((-1, 1), repeat=3)), dtype=float) * size / 2


def obstacle_distance(obstacle, centers):
    delta = centers[:, :2] - [obstacle.x, obstacle.y]
    if obstacle.kind == "cylinder":
        return np.linalg.norm(delta, axis=1) - obstacle.size_x
    c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
    local = delta @ np.array([[c, -s], [s, c]])
    q = np.abs(local) - [obstacle.size_x / 2, obstacle.size_y / 2]
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(np.max(q, axis=1), 0)


def rectangle(obstacle):
    local = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], dtype=float)
    local *= [obstacle.size_x / 2, obstacle.size_y / 2]
    c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
    return local @ np.array([[c, s], [-s, c]]) + [obstacle.x, obstacle.y]


def point_polygon_distance(point, polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    relative = point - polygon
    t = np.clip(np.sum(relative * edges, axis=1) / np.sum(edges * edges, axis=1), 0, 1)
    distance = float(np.min(np.linalg.norm(relative - t[:, None] * edges, axis=1)))
    cross = edges[:, 0] * relative[:, 1] - edges[:, 1] * relative[:, 0]
    return -distance if np.all(cross >= -1e-12) else distance


def polygon_distance(first, second):
    axes = []
    for polygon in (first, second):
        edges = np.roll(polygon, -1, axis=0) - polygon
        axes.extend(np.array([-edge[1], edge[0]]) / np.linalg.norm(edge) for edge in edges)
    separation = False
    penetration = float("inf")
    for axis in axes:
        a, b = first @ axis, second @ axis
        if a.max() < b.min() or b.max() < a.min():
            separation = True
        penetration = min(penetration, a.max() - b.min(), b.max() - a.min())
    if not separation:
        return -float(penetration)
    return min(min(abs(point_polygon_distance(point, second)) for point in first),
               min(abs(point_polygon_distance(point, first)) for point in second))


def evaluate(parts, obstacles, positions, rotations):
    count = len(positions)
    best = np.full(count, np.inf)
    best_part = np.full(count, -1, dtype=int)
    best_obstacle = np.full(count, -1, dtype=int)
    projections = []
    for part in parts:
        centers = positions + np.einsum("nij,j->ni", rotations, part.transform[:3, 3])
        orientation = rotations @ part.transform[:3, :3]
        if part.kind == "rotor_swept_cylinder":
            axis = orientation[:, :, 2]
            tilt_sin = np.sqrt(np.maximum(0, 1 - axis[:, 2] ** 2))
            radius_xy = part.radius + part.half_height * tilt_sin
            z_extent = part.radius * tilt_sin + part.half_height * np.abs(axis[:, 2])
            projections.append((centers, radius_xy, centers[:, 2] - z_extent, centers[:, 2] + z_extent, None))
        else:
            corners = np.einsum("nij,kj->nki", orientation, box_vertices(part.size)) + centers[:, None, :]
            radius_xy = np.max(np.linalg.norm(corners[:, :, :2] - centers[:, None, :2], axis=2), axis=1)
            projections.append((centers, radius_xy, corners[:, :, 2].min(axis=1), corners[:, :, 2].max(axis=1), corners))
    # Circle checks are vectorized; their bounds cull nearly all slower box pairs.
    order = sorted(range(len(parts)), key=lambda index: parts[index].kind != "rotor_swept_cylinder")
    for part_index in order:
        centers, radii, low, high, corners = projections[part_index]
        for obstacle_index, obstacle in enumerate(obstacles):
            overlap_z = (high >= obstacle.z - obstacle.height / 2) & (low <= obstacle.z + obstacle.height / 2)
            lower_bound = obstacle_distance(obstacle, centers) - radii
            candidates = overlap_z & (lower_bound < best)
            if corners is None:
                best[candidates] = lower_bound[candidates]
                best_part[candidates] = part_index
                best_obstacle[candidates] = obstacle_index
                continue
            other = rectangle(obstacle) if obstacle.kind == "box" else None
            for index in np.flatnonzero(candidates):
                points = corners[index, :, :2]
                polygon = points[ConvexHull(points).vertices]
                distance = (point_polygon_distance(np.array([obstacle.x, obstacle.y]), polygon) - obstacle.size_x
                            if other is None else polygon_distance(polygon, other))
                if distance < best[index]:
                    best[index], best_part[index], best_obstacle[index] = distance, part_index, obstacle_index
    return best, best_part, best_obstacle


def summarize(indices, values, part_indices, obstacle_indices, truths, phases, parts, obstacles):
    finite = [int(index) for index in indices if np.isfinite(values[index])]
    if not finite:
        return dict(sample_count=len(indices), minimum_clearance_m=None)
    minimum = min(finite, key=lambda index: values[index])
    event = truths[minimum]
    negative = values[finite] < 0
    episodes = int(np.sum(negative & ~np.r_[False, negative[:-1]]))
    return dict(sample_count=len(indices), evaluated_count=len(finite), minimum_clearance_m=float(values[minimum]),
                negative_sample_count=int(negative.sum()), overlap_episode_count=episodes,
                minimum_sample=dict(t_wall_s=event["t_wall"], t_sim_s=event.get("t_sim"), phase=phases[minimum],
                                    truth_xyz=[event["x"], event["y"], event["z"]],
                                    yaw_deg=math.degrees(event["yaw"]), roll_deg=math.degrees(event.get("roll", 0)),
                                    pitch_deg=math.degrees(event.get("pitch", 0)),
                                    vehicle_collision_part=parts[part_indices[minimum]].name,
                                    obstacle=obstacles[obstacle_indices[minimum]].name))


def audit_trial(directory, parts):
    events = []
    with (directory / "events.jsonl").open() as stream:
        for line in stream:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    truths = [event for event in events if event["kind"] == "truth"]
    if not truths:
        raise ValueError("No ground truth: " + str(directory))
    if any(event.get("child", "").endswith("::base_link") for event in truths):
        raise ValueError("Audit requires Gazebo model poses, not base_link poses")
    goal_time = next((event["t_wall"] for event in events if event["kind"] == "goal"), float("inf"))
    corridor_time = next((event["t_wall"] for event in events if event["kind"] == "corridor_active" and event["value"]), float("inf"))
    finished_time = next((event["t_wall"] for event in events if event["kind"] == "finished" and event["value"]), float("inf"))
    phases = ["startup" if event["t_wall"] < goal_time else "exploration" if event["t_wall"] < corridor_time else
              "corridor" if event["t_wall"] < finished_time else "landing" for event in truths]
    positions = np.array([[event[key] for key in ("x", "y", "z")] for event in truths])
    angles = np.array([[event["yaw"], event.get("pitch", 0), event.get("roll", 0)] for event in truths])
    missing_attitude = sum("roll" not in event or "pitch" not in event for event in truths)
    yaw_only = angles.copy()
    yaw_only[:, 1:] = 0
    obstacles = [Obstacle(**item) for item in json.loads((directory / "world_geometry.json").read_text())["obstacles"]]
    output = dict(trial_dir=str(directory), truth_sample_count=len(truths), missing_roll_pitch_samples=missing_attitude,
                  maximum_absolute_roll_deg=float(np.rad2deg(np.abs(angles[:, 2])).max()),
                  maximum_absolute_pitch_deg=float(np.rad2deg(np.abs(angles[:, 1])).max()),
                  maximum_truth_wall_gap_s=max(b["t_wall"] - a["t_wall"] for a, b in zip(truths, truths[1:])),
                  maximum_truth_wall_gap_by_phase_s={
                      phase: max((truths[i + 1]["t_wall"] - truths[i]["t_wall"] for i in range(len(truths) - 1)
                                  if phases[i] == phase and phases[i + 1] == phase), default=None)
                      for phase in ("startup", "exploration", "corridor", "landing")},
                  methods={})
    for method, method_angles in (("level_yaw_sweeps", yaw_only), ("attitude_projected_enclosures", angles)):
        if method == "attitude_projected_enclosures" and missing_attitude:
            output["methods"][method] = dict(status="unavailable", reason="missing recorded roll/pitch")
            continue
        values, part_indices, obstacle_indices = evaluate(parts, obstacles, positions,
                                                         Rotation.from_euler("ZYX", method_angles).as_matrix())
        output["methods"][method] = dict(
            all=summarize(range(len(truths)), values, part_indices, obstacle_indices, truths, phases, parts, obstacles),
            phases={phase: summarize([i for i, value in enumerate(phases) if value == phase], values,
                                     part_indices, obstacle_indices, truths, phases, parts, obstacles)
                    for phase in ("startup", "exploration", "corridor", "landing")})
    old_summary_path = directory / "summary.json"
    if old_summary_path.exists():
        old = json.loads(old_summary_path.read_text())
        output["original_cylindrical_monitor"] = dict(
            radius_m=old["configuration"]["robot_radius"], sample_hz=old["configuration"]["sample_hz"],
            phases={phase: old["phases"][phase]["minimum_surface_clearance_m"] for phase in old["phases"]})
    return output


def markdown(report):
    def clearance(value):
        return f"{value:.4f}" if value is not None else "unknown"

    lines = ["# SDF Rotor-Sweep Footprint Audit", "",
             "This is an offline geometric audit. The original recorder and its 0.32 m cylindrical monitoring metrics are unchanged.", "",
             "Reproduce using the compatible distribution NumPy/SciPy runtime: `PYTHONPATH=/usr/lib/python3/dist-packages /usr/bin/python3 tools/sim/audit_footprint.py`. No ROS environment or simulator is needed.", "",
             "libsdformat resolves the actual include/merge and relative-pose frames. All collision boxes are parsed; four revolute rotor links become full-revolution swept circles, and the five fixed body/landing-gear boxes retain their projected geometry. Visual meshes do not replace collision shapes.", "",
             "| Trial | Original exploration monitor (m) | Yaw-only exploration (m) | Attitude-projected exploration (m) | Attitude-projected corridor (m) | Projected overlap episodes |",
             "| --- | ---: | ---: | ---: | ---: | ---: |"]
    for trial in report["trials"]:
        old = trial.get("original_cylindrical_monitor", {}).get("phases", {}).get("exploration")
        yaw = trial["methods"]["level_yaw_sweeps"]["phases"]["exploration"]["minimum_clearance_m"]
        attitude = trial["methods"]["attitude_projected_enclosures"]
        if "phases" not in attitude:
            continue
        explore = attitude["phases"]["exploration"]["minimum_clearance_m"]
        corridor = attitude["phases"]["corridor"]["minimum_clearance_m"]
        lines.append(f"| {Path(trial['trial_dir']).name} | {clearance(old)} | {clearance(yaw)} | {clearance(explore)} | {clearance(corridor)} | {attitude['all']['overlap_episode_count']} |")
    rotor = next(part for part in report["vehicle_parts"] if part["kind"] == "rotor_swept_cylinder")
    lines.extend(["", "## Parsed Geometry", "",
                  f"Rotor collision box dimensions: {rotor['source_box_size_m']} m. Sweeping its full box corners, not just half its length, gives radius {rotor['sweep_radius_m']:.9f} m.",
                  f"Parsed level-footprint single-axis full width: {report['level_axis_full_width_m']:.6f} m. Rotor-tip circumradius about the model XY origin: {report['level_rotor_circumradius_m']:.6f} m.", "",
                  "The included x500_base frame is 0.24 m above the outer model origin, so resolved rotor centers are at model Z=0.30 m. This offset is included when full attitude rotates component centers; ignoring it can alter wall-clearance estimates during tilt.", "",
                  "## Interpretation", ""])
    lines.extend("- " + note for note in report["limitations"])
    lines.extend(["", "## Minimum Samples", ""])
    for trial in report["trials"]:
        attitude = trial["methods"]["attitude_projected_enclosures"]
        if "phases" not in attitude:
            continue
        for phase in ("exploration", "corridor"):
            data = attitude["phases"][phase]
            sample = data.get("minimum_sample")
            if sample is None:
                continue
            lines.append(f"- {Path(trial['trial_dir']).name}, {phase}: {data['minimum_clearance_m']:.4f} m at recorder t={sample['t_wall_s']:.3f} s, yaw={sample['yaw_deg']:.2f} deg, roll={sample['roll_deg']:.2f} deg, pitch={sample['pitch_deg']:.2f} deg; {sample['vehicle_collision_part']} versus {sample['obstacle']}.")
    return "\n".join(lines) + "\n"


def self_test():
    square = np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])
    assert abs(point_polygon_distance(np.array([2., 0.]), square) - 1) < 1e-9
    assert abs(point_polygon_distance(np.array([0., 0.]), square) + 1) < 1e-9
    assert abs(polygon_distance(square, square + [3., 0.]) - 1) < 1e-9
    assert abs(polygon_distance(square, square + [1.5, 0.]) + .5) < 1e-9
    wall = Obstacle("wall", "box", 1., 0., 1., 0., .1, 5., 2.)
    transform = np.eye(4)
    transform[:3, 3] = [.174, .174, 0.]
    part = Part("rotor", "rotor_swept_cylinder", transform, np.array([.28, .02, .001]), .14, .0005)
    positions = np.array([[0., 0., 1.], [0., 0., 1.]])
    rotations = Rotation.from_euler("z", [0., math.pi / 4]).as_matrix()
    values, _, _ = evaluate([part], [wall], positions, rotations)
    assert abs(values[0] - (.95 - .174 - .14)) < 1e-9
    assert abs(values[1] - (.95 - .14)) < 1e-9
    box = Part("body", "box", np.eye(4), np.array([.4, .2, .1]))
    for wall_yaw in (0., .37, 1.2):
        rotated_wall = Obstacle("wall", "box", math.cos(wall_yaw), math.sin(wall_yaw),
                                1., wall_yaw, .1, 5., 2.)
        yaw = .8
        values, _, _ = evaluate([box], [rotated_wall], positions[:1],
                                 Rotation.from_euler("z", [yaw]).as_matrix())
        support = .2 * abs(math.cos(yaw - wall_yaw)) + .1 * abs(math.sin(yaw - wall_yaw))
        assert abs(values[0] - (.95 - support)) < 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim-runs", type=Path, default=Path("/home/yk/acfly_ws/sim_runs"))
    parser.add_argument("--models-dir", type=Path,
                        default=Path("/home/yk/uav_sim_env/PX4-Autopilot/Tools/simulation/gz/models"))
    parser.add_argument("--model", default="x500_lidar_3d")
    parser.add_argument("--seeds", nargs="+", type=int, default=list(range(13, 19)))
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    self_test()
    output_dir = args.output_dir or args.sim_runs / "comparisons"
    output_dir.mkdir(parents=True, exist_ok=True)
    parts, hashes = load_vehicle(args.models_dir / args.model / "model.sdf", args.models_dir)
    rotors = [part for part in parts if part.kind == "rotor_swept_cylinder"]
    axis_radius = max(max(abs(part.transform[0, 3]), abs(part.transform[1, 3])) + part.radius for part in rotors)
    circumradius = max(np.linalg.norm(part.transform[:2, 3]) + part.radius for part in rotors)
    report = dict(model=args.model, current_model_source_sha256=hashes,
                  numpy_version=np.__version__, sdformat_module=sdf.__file__,
                  vehicle_parts=[part.describe() for part in parts],
                  level_axis_full_width_m=2 * axis_radius, level_rotor_circumradius_m=float(circumradius),
                  limitations=[
                      "A 0.32 m cylindrical monitoring radius is based on single-axis width; it does not enclose the four swept rotors at every yaw. Its original results remain separate for experiment comparability.",
                      "Full attitude is reconstructed from recorded Gazebo model yaw/pitch/roll. The yaw-only result assumes a level aircraft. Tilted rotor projections use enclosing circles; each body box uses its exact projected vertex hull.",
                      "Negative values mean overlap of XY projected enclosures with overlapping vertical intervals. This can overreport true 3-D overlap and is not a physics-engine contact report or penetration-depth measurement.",
                      "Rotor joint phases were not recorded. Full-revolution sweeps cover all phases but do not establish instantaneous blade contact, especially after motors stop.",
                      "All recorded truth frames are checked, usually 50 Hz. No continuous-time guarantee is made between samples; maximum per-trial gaps are in JSON.",
                      "Ground and landing-pad collisions are excluded, consistently with the recorder. This audit checks lateral wall/pole clearance, not terrain contact or landing mechanics.",
                      "Geometry is parsed from current actual model SDFs and identified by SHA-256. Historical root model snapshots exist, but included x500_base collision files were not independently snapshotted for every early trial.",
                  ], trials=[])
    for seed in args.seeds:
        directories = sorted(args.sim_runs.glob(f"*seed{seed}_*"))
        if not directories:
            raise ValueError(f"No recorded trial for seed {seed}")
        for directory in directories:
            trial = audit_trial(directory, parts)
            report["trials"].append(trial)
            method = trial["methods"]["attitude_projected_enclosures"]
            print(json.dumps(dict(seed=seed, trial=directory.name,
                                  minimums={phase: data.get("minimum_clearance_m") for phase, data in method.get("phases", {}).items()},
                                  projected_overlap_episodes=method.get("all", {}).get("overlap_episode_count"))), flush=True)
    atomic_json(output_dir / "audit_footprint.json", report)
    (output_dir / "audit_footprint.md").write_text(markdown(report), encoding="utf-8")


if __name__ == "__main__":
    main()
