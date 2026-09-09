#!/usr/bin/env python3
"""Record one ROS/Gazebo trial without commanding the aircraft.

Ground-truth coordinates always come from the Gazebo pose bridge. The fixed
SLAM/world origin is used only for the explicitly labelled trajectory overlay.
Clearance is a geometric cylindrical aircraft-envelope estimate, not a Gazebo
contact-sensor measurement.
"""

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as ET

import numpy as np


PLANNER_COUNTERS = ("replan_checks", "astar_searches", "adoptions", "smooth_fallbacks",
                    "turn_entries", "recovery_entries", "gates_passed")
PLANNER_VALUES = ("mode", "coverage_ratio", "target_x", "target_y", "lookahead_x", "lookahead_y",
                  "gate_center_x", "gate_center_y", "gate_width", "gate_reason") + PLANNER_COUNTERS


def planner_metrics(rows):
    def extrema(key):
        values = [row[key] for row in rows if row.get(key) is not None]
        return {"last": values[-1] if values else None, "max": max(values) if values else None}

    return {"coverage_ratio": extrema("coverage_ratio"),
            "counters": {key: extrema("planner_" + key) for key in PLANNER_COUNTERS},
            "last": {key: next((row["planner_" + key] for row in reversed(rows)
                                 if row.get("planner_" + key) is not None), None)
                     for key in PLANNER_VALUES}}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def pose_matrix(element):
    values = [float(v) for v in element.findtext("pose", "0 0 0 0 0 0").split()]
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result[:3, 3] = (x, y, z)
    return result


@dataclass
class Obstacle:
    name: str
    kind: str
    x: float
    y: float
    z: float
    yaw: float
    size_x: float
    size_y: float
    height: float

    def center_distance(self, x, y):
        dx, dy = x - self.x, y - self.y
        if self.kind == "cylinder":
            return math.hypot(dx, dy) - self.size_x
        local_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        qx, qy = abs(local_x) - self.size_x / 2, abs(local_y) - self.size_y / 2
        return math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0)


def read_world(path):
    obstacles = []
    ignored = []
    root = ET.parse(path).getroot()

    def visit_model(model, parent, prefix):
        transform = parent @ pose_matrix(model)
        name = prefix + model.get("name", "model")
        for link in model.findall("link"):
            link_transform = transform @ pose_matrix(link)
            for collision in link.findall("collision"):
                full_name = name + "/" + link.get("name", "link") + "/" + collision.get("name", "collision")
                matrix = link_transform @ pose_matrix(collision)
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                if any(pose is not None and pose.get("relative_to")
                       for pose in (model.find("pose"), link.find("pose"), collision.find("pose"))):
                    ignored.append({"name": full_name, "reason": "relative_to frames unsupported"})
                    continue
                if abs(matrix[2, 0]) > 1e-6 or abs(matrix[2, 1]) > 1e-6:
                    ignored.append({"name": full_name, "reason": "tilted collision unsupported"})
                    continue
                box, cylinder = geometry.find("box"), geometry.find("cylinder")
                if box is not None:
                    sx, sy, height = map(float, box.findtext("size").split())
                    kind = "box"
                elif cylinder is not None:
                    sx = sy = float(cylinder.findtext("radius"))
                    height = float(cylinder.findtext("length"))
                    kind = "cylinder"
                else:
                    ignored.append({"name": full_name, "reason": "unsupported geometry"})
                    continue
                if matrix[2, 3] + height / 2 < 0.06:
                    ignored.append({"name": full_name, "reason": "floor/landing pad excluded"})
                    continue
                obstacles.append(Obstacle(full_name, kind, *matrix[:3, 3].tolist(),
                                          math.atan2(matrix[1, 0], matrix[0, 0]), sx, sy, height))
        for child in model.findall("model"):
            visit_model(child, transform, name + "/")

    world = root.find("world")
    for model in world.findall("model"):
        visit_model(model, np.eye(4), "")
    return obstacles, ignored


def clearance_at(obstacles, x, y, z, radius, half_height):
    candidates = [(obstacle.center_distance(x, y) - radius, obstacle.name)
                  for obstacle in obstacles
                  if z + half_height >= obstacle.z - obstacle.height / 2
                  and z - half_height <= obstacle.z + obstacle.height / 2]
    return min(candidates) if candidates else (None, None)


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


class TrialRecorder:
    def __init__(self, args):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
        from mavros_msgs.msg import ExtendedState, State
        from nav_msgs.msg import Odometry, Path as RosPath
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Bool, Float64
        from tf2_msgs.msg import TFMessage

        self.args = args
        self.output = Path(args.output_dir)
        self.output.mkdir(parents=True, exist_ok=True)
        atomic_json(self.output / "recorder_config.json", vars(args))
        self.obstacles, ignored = read_world(args.world)
        atomic_json(self.output / "world_geometry.json", {
            "source": str(Path(args.world).resolve()), "frame": "Gazebo world",
            "obstacles": [asdict(obstacle) for obstacle in self.obstacles], "ignored": ignored,
        })
        self.node = rclpy.create_node("sim_trial_recorder")
        self.started = time.monotonic()
        self.latest = {}
        self.counts = {}
        self.rows = []
        self.finished_at = None
        self.finished_sim = None
        self.started_flying_at = None
        self.started_flying_sim = None
        self.corridor = False
        self.mission_complete = False
        self.landed_after_mission = False
        self.goal = None
        self.route = []
        self.active_path = []
        self.collision_count = 0
        self.collision_active = False
        self.minimum_clearance = None
        self.last_progress = -10.0
        self.last_sim = None
        self.clock_resets = 0
        self.tf_children = set()
        self.event_file = (self.output / "events.jsonl").open("w", encoding="utf-8")
        self.sample_file = (self.output / "samples.csv").open("w", newline="", encoding="utf-8")
        self.writer = None
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = qos_profile_sensor_data
        subscriptions = [
            (Odometry, "/aft_mapped_to_init", lambda m: self.pose("slam", m.pose.pose, m.header, m.twist.twist), sensor),
            (PoseStamped, "/mavros/local_position/pose", lambda m: self.pose("mavros", m.pose, m.header), sensor),
            (State, "/mavros/state", self.state, sensor),
            (ExtendedState, "/mavros/extended_state", self.extended_state, sensor),
            (TwistStamped, "/exploration/cmd_vel", self.command, sensor),
            (PoseStamped, "/exploration/target_pose", lambda m: self.pose("target", m.pose, m.header), sensor),
            (Bool, "/exploration/finished", self.finished, latched),
            (Bool, "/exploration/corridor_active", self.corridor_active, latched),
            (PointStamped, "/exploration/goal", self.set_goal, latched),
            (RosPath, "/exploration/corridor_route", self.set_route, latched),
            (Float64, "/exploration/coverage", self.coverage, sensor),
            (RosPath, "/exploration/active_path", self.set_active_path, sensor),
            (DiagnosticArray, "/exploration/diagnostics", self.diagnostics, sensor),
            (Clock, "/clock", self.clock, sensor),
            (TFMessage, args.ground_truth_topic, self.ground_truth, sensor),
        ]
        self.subscriptions = [self.node.create_subscription(kind, topic, callback, qos)
                              for kind, topic, callback, qos in subscriptions]
        self.timer = self.node.create_timer(1.0 / args.sample_hz, self.sample)
        self.node.get_logger().info("Recording trial to " + str(self.output))

    def now(self):
        return time.monotonic() - self.started

    def event(self, kind, value):
        elapsed = self.now()
        data = {"t_wall": elapsed, "t_sim": self.last_sim, "kind": kind, **value}
        self.event_file.write(json.dumps(data, allow_nan=False) + "\n")
        self.latest[kind] = (elapsed, value)
        self.counts[kind] = self.counts.get(kind, 0) + 1

    def pose(self, kind, pose, header, twist=None):
        values = {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
                  "yaw": quaternion_yaw(pose.orientation), "stamp": stamp_seconds(header.stamp),
                  "frame": header.frame_id}
        if twist is not None:
            values.update(vx=twist.linear.x, vy=twist.linear.y, vz=twist.linear.z,
                          yaw_rate=twist.angular.z)
        if all(math.isfinite(value) for value in values.values() if isinstance(value, (int, float))):
            self.event(kind, values)

    def state(self, message):
        self.event("state", {"connected": message.connected, "armed": message.armed, "mode": message.mode})

    def extended_state(self, message):
        self.event("extended_state", {"landed_state": message.landed_state})
        if self.mission_complete and message.landed_state == 1:
            self.landed_after_mission = True

    def command(self, message):
        self.event("cmd", {"vx": message.twist.linear.x, "vy": message.twist.linear.y,
                           "vz": message.twist.linear.z, "yaw_rate": message.twist.angular.z})

    def finished(self, message):
        self.event("finished", {"value": message.data})
        if message.data and not self.mission_complete:
            self.mission_complete = True
            self.finished_at, self.finished_sim = self.now(), self.last_sim

    def corridor_active(self, message):
        if message.data != self.corridor:
            self.event("corridor_active", {"value": message.data})
        self.corridor = message.data

    def set_goal(self, message):
        self.goal = [message.point.x, message.point.y, message.point.z]
        self.event("goal", {"point": self.goal, "frame": message.header.frame_id})

    def set_route(self, message):
        self.route = [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                      for pose in message.poses]
        self.event("corridor_route", {"points": self.route, "frame": message.header.frame_id})

    def coverage(self, message):
        if math.isfinite(message.data):
            self.event("coverage", {"ratio": message.data})

    def set_active_path(self, message):
        points = [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                  for pose in message.poses]
        if all(math.isfinite(coordinate) for point in points for coordinate in point):
            self.active_path = points
            self.event("active_path", {"points": points, "frame": message.header.frame_id})

    def diagnostics(self, message):
        for status in message.status:
            if status.name != "exploration/planner":
                continue
            values = {}
            for item in status.values:
                if item.key in ("mode", "gate_reason"):
                    values[item.key] = item.value
                else:
                    try:
                        value = float(item.value)
                        values[item.key] = value if math.isfinite(value) else None
                    except ValueError:
                        values[item.key] = None if item.key in PLANNER_VALUES else item.value
            level = status.level[0] if isinstance(status.level, bytes) else int(status.level)
            self.event("diagnostics", {"level": level, "message": status.message, "values": values})

    def clock(self, message):
        current = stamp_seconds(message.clock)
        if self.last_sim is not None and current < self.last_sim:
            self.clock_resets += 1
            self.event("clock_reset", {"previous": self.last_sim, "current": current})
        self.last_sim = current
        self.counts["clock"] = self.counts.get("clock", 0) + 1

    def ground_truth(self, message):
        self.counts["ground_truth_messages"] = self.counts.get("ground_truth_messages", 0) + 1
        selected = None
        for transform in message.transforms:
            child = transform.child_frame_id
            self.tf_children.add(child)
            if child == self.args.model_name or child == self.args.model_name + "::base_link":
                selected = transform
                if child == self.args.model_name:
                    break
        if selected is None:
            return
        tr = selected.transform.translation
        q = selected.transform.rotation
        values = {"x": tr.x, "y": tr.y, "z": tr.z,
                  "yaw": quaternion_yaw(q),
                  "roll": math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                  "pitch": math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
                  "stamp": stamp_seconds(selected.header.stamp),
                  "sim_at_receive": self.last_sim,
                  "frame": selected.header.frame_id, "child": selected.child_frame_id}
        previous = self.latest.get("truth", (None, {}))[1]
        values["speed_mps"] = None
        if previous:
            dt = values["stamp"] - previous["stamp"]
            if dt <= 0 and values["sim_at_receive"] is not None and previous["sim_at_receive"] is not None:
                dt = values["sim_at_receive"] - previous["sim_at_receive"]
            if dt > .001:
                values["speed_mps"] = math.sqrt(sum((values[key] - previous[key]) ** 2 for key in ("x", "y", "z"))) / dt
        if all(math.isfinite(value) for value in values.values() if isinstance(value, (int, float))):
            self.event("truth", values)

    def sample(self):
        elapsed = self.now()
        row = {"t_wall": elapsed, "t_sim": self.last_sim,
               "phase": "landing" if self.mission_complete else
                        "corridor" if self.corridor else "exploration" if self.goal else "startup",
               "mission_complete": self.mission_complete, "landed_after_mission": self.landed_after_mission}
        for kind in ("slam", "mavros", "truth", "target", "cmd", "state", "extended_state"):
            received, value = self.latest.get(kind, (None, {}))
            row[kind + "_age_s"] = elapsed - received if received is not None else None
            keys = {"slam": ("x", "y", "z", "yaw", "yaw_rate"),
                    "mavros": ("x", "y", "z", "yaw"), "truth": ("x", "y", "z", "yaw", "roll", "pitch", "speed_mps", "stamp", "sim_at_receive"),
                    "target": ("x", "y", "z", "yaw"), "cmd": ("vx", "vy", "vz", "yaw_rate"),
                    "state": ("connected", "armed", "mode"), "extended_state": ("landed_state",)}[kind]
            for key in keys:
                row[kind + "_" + key] = value.get(key)
        diagnostics_received, diagnostics = self.latest.get("diagnostics", (None, {}))
        coverage_received, coverage = self.latest.get("coverage", (None, {}))
        path_received, path = self.latest.get("active_path", (None, {}))
        row["diagnostics_age_s"] = elapsed - diagnostics_received if diagnostics_received is not None else None
        row["coverage_age_s"] = elapsed - coverage_received if coverage_received is not None else None
        row["active_path_age_s"] = elapsed - path_received if path_received is not None else None
        row["active_path_point_count"] = len(path["points"]) if path else None
        values = diagnostics.get("values", {})
        row["coverage_ratio"] = coverage.get("ratio", values.get("coverage_ratio"))
        for key in PLANNER_VALUES:
            row["planner_" + key] = values.get(key)
        row["surface_clearance_m"] = None
        row["nearest_obstacle"] = None
        row["collision_proxy"] = None
        row["slam_truth_xy_error_m"] = None
        if row["truth_age_s"] is not None and row["truth_age_s"] < self.args.max_pose_age:
            clearance, obstacle = clearance_at(self.obstacles, row["truth_x"], row["truth_y"],
                                               row["truth_z"], self.args.robot_radius, self.args.robot_half_height)
            row["surface_clearance_m"], row["nearest_obstacle"] = clearance, obstacle
            row["collision_proxy"] = clearance < 0 if clearance is not None else None
            if clearance is not None:
                self.minimum_clearance = clearance if self.minimum_clearance is None else min(self.minimum_clearance, clearance)
                self.collision_count += int(row["collision_proxy"] and not self.collision_active)
                self.collision_active = row["collision_proxy"]
            if row["slam_age_s"] is not None and row["slam_age_s"] < self.args.max_pose_age:
                row["slam_truth_xy_error_m"] = math.hypot(row["slam_x"] + self.args.world_origin_x - row["truth_x"],
                                                         row["slam_y"] + self.args.world_origin_y - row["truth_y"])
        altitude = row["truth_z"] if row["truth_z"] is not None else row["mavros_z"]
        if self.started_flying_at is None and row["state_armed"] and altitude is not None and altitude > 0.30:
            self.started_flying_at, self.started_flying_sim = elapsed, self.last_sim
        if self.writer is None:
            self.writer = csv.DictWriter(self.sample_file, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.rows.append(row)
        if elapsed - self.last_progress >= self.args.progress_interval:
            atomic_json(self.output / "progress.json", {
                "wall_elapsed_s": elapsed, "sim_time_s": self.last_sim,
                "phase": row["phase"], "mission_complete": self.mission_complete,
                "landed_after_mission": self.landed_after_mission,
                "ground_truth_sample_count": self.counts.get("truth", 0),
                "pose": {key: row["truth_" + key] for key in ("x", "y", "z", "yaw")},
                "ground_truth_age_s": row["truth_age_s"], "ground_truth_z": row["truth_z"],
                "ground_truth_roll_rad": row["truth_roll"], "ground_truth_pitch_rad": row["truth_pitch"],
                "ground_truth_speed_mps": row["truth_speed_mps"],
                "slam_truth_xy_error_m": row["slam_truth_xy_error_m"],
                "collision_proxy_active": row["collision_proxy"], "collision_proxy_count": self.collision_count,
                "surface_clearance_m": row["surface_clearance_m"], "minimum_surface_clearance_m": self.minimum_clearance,
                "coverage_ratio": row["coverage_ratio"], "planner_mode": row["planner_mode"],
                "planner_diagnostics": values,
                "mavros_mode": row["state_mode"], "armed": row["state_armed"],
            })
            self.event_file.flush()
            self.sample_file.flush()
            self.last_progress = elapsed

    def metrics(self, rows):
        distance = {source: 0.0 for source in ("truth", "slam", "mavros")}
        samples = {source: 0 for source in distance}
        yaw_reversals, last_yaw_sign, max_yaw_rate = 0, 0, 0.0
        zero_segments, zero_start, previous = [], None, None
        max_actual_yaw_rate = None
        min_clearance = None
        collisions = 0
        in_collision = False
        for row in rows:
            for source in distance:
                age = row[source + "_age_s"]
                if age is None or age >= self.args.max_pose_age:
                    continue
                samples[source] += 1
                if previous is not None and previous[source + "_age_s"] is not None and previous[source + "_age_s"] < self.args.max_pose_age:
                    distance[source] += math.hypot(row[source + "_x"] - previous[source + "_x"],
                                                  row[source + "_y"] - previous[source + "_y"])
            command_fresh = row["cmd_age_s"] is not None and row["cmd_age_s"] < self.args.max_pose_age
            flying = (self.started_flying_at is not None and row["t_wall"] >= self.started_flying_at
                      and row["phase"] in ("exploration", "corridor"))
            stopped = command_fresh and flying and math.hypot(row["cmd_vx"], row["cmd_vy"]) < self.args.stop_speed
            if stopped and zero_start is None:
                zero_start = row["t_wall"]
            elif not stopped and zero_start is not None:
                if row["t_wall"] - zero_start >= self.args.stop_min_duration:
                    zero_segments.append({"start_wall_s": zero_start, "duration_s": row["t_wall"] - zero_start})
                zero_start = None
            if command_fresh and flying:
                yaw_rate = row["cmd_yaw_rate"]
                max_yaw_rate = max(max_yaw_rate, abs(yaw_rate))
                sign = 1 if yaw_rate > self.args.yaw_reversal_threshold else -1 if yaw_rate < -self.args.yaw_reversal_threshold else 0
                if sign:
                    yaw_reversals += int(last_yaw_sign != 0 and sign != last_yaw_sign)
                    last_yaw_sign = sign
            if previous is not None and row["truth_age_s"] is not None and row["truth_age_s"] < self.args.max_pose_age and previous["truth_age_s"] is not None and previous["truth_age_s"] < self.args.max_pose_age:
                dt = row["truth_stamp"] - previous["truth_stamp"]
                if dt <= 0 and row["truth_sim_at_receive"] is not None and previous["truth_sim_at_receive"] is not None:
                    dt = row["truth_sim_at_receive"] - previous["truth_sim_at_receive"]
                if dt > 0.001:
                    value = abs(wrap(row["truth_yaw"] - previous["truth_yaw"]) / dt)
                    max_actual_yaw_rate = max(max_actual_yaw_rate or 0, value)
            clearance = row["surface_clearance_m"]
            if clearance is not None:
                min_clearance = clearance if min_clearance is None else min(min_clearance, clearance)
                collision = row["collision_proxy"]
                collisions += int(collision and not in_collision)
                in_collision = collision
            previous = row
        if zero_start is not None and rows and rows[-1]["t_wall"] - zero_start >= self.args.stop_min_duration:
            zero_segments.append({"start_wall_s": zero_start, "duration_s": rows[-1]["t_wall"] - zero_start})
        return {"duration_wall_s": rows[-1]["t_wall"] - rows[0]["t_wall"] if rows else 0,
                "trajectory_length_m": {source: distance[source] if samples[source] else None for source in distance},
                "sample_count": samples, "zero_command_segments": zero_segments,
                "zero_command_count": len(zero_segments),
                "zero_command_duration_s": sum(segment["duration_s"] for segment in zero_segments),
                "yaw_command_reversals": yaw_reversals, "max_yaw_command_rad_s": max_yaw_rate,
                "max_ground_truth_yaw_rate_rad_s": max_actual_yaw_rate,
                "minimum_surface_clearance_m": min_clearance,
                "collision_proxy_count": collisions if samples["truth"] else None}

    def close(self, reason):
        self.sample()
        self.event_file.close()
        self.sample_file.close()
        metrics = self.metrics(self.rows)
        flight_rows = [row for row in self.rows if self.started_flying_at is not None
                       and row["t_wall"] >= self.started_flying_at]
        truth_coverage = (sum(row["truth_age_s"] is not None and row["truth_age_s"] < self.args.max_pose_age
                              for row in flight_rows) / len(flight_rows)) if flight_rows else None
        complete_truth = truth_coverage is not None and truth_coverage >= .99 and not self.clock_resets
        summary = {"termination": reason, "mission_complete": self.mission_complete,
                   "landed_after_mission": self.landed_after_mission,
                   "success": (self.mission_complete and self.landed_after_mission and metrics["collision_proxy_count"] == 0)
                              if complete_truth else None,
                   "wall_elapsed_s": self.now(), "finished_at_wall_s": self.finished_at,
                   "finished_at_sim_s": self.finished_sim, "started_flying_wall_s": self.started_flying_at,
                   "mission_duration_wall_s": self.finished_at - self.started_flying_at
                                              if self.finished_at is not None and self.started_flying_at is not None else None,
                   "mission_duration_sim_s": self.finished_sim - self.started_flying_sim
                                             if self.finished_sim is not None and self.started_flying_sim is not None and not self.clock_resets else None,
                   "ground_truth_status": "observed" if metrics["sample_count"]["truth"] else "unknown",
                   "ground_truth_sample_count": self.counts.get("truth", 0),
                   "ground_truth_flight_coverage": truth_coverage,
                   "collision_proxy_count": metrics["collision_proxy_count"],
                   "ground_truth_tf_children": sorted(self.tf_children), "clock_resets": self.clock_resets,
                   "topic_message_counts": self.counts, "goal_slam": self.goal, "corridor_route_slam": self.route,
                   "planner": planner_metrics(self.rows), "latest_active_path_slam": self.active_path,
                   "metric_definitions": {
                       "trajectory_length": "Horizontal length from sampled poses; SLAM and MAVROS are estimates, truth is Gazebo world pose.",
                       "clearance": "Signed horizontal distance from Gazebo world aircraft center to SDF collision surfaces minus aircraft radius, restricted to overlapping vertical extents.",
                       "collision_proxy": "Conservative XY circular-envelope overlap; not a contact-sensor observation. Default radius 0.32 m for 0.627 m model width. Ground and landing disks excluded.",
                       "yaw_reversals": "Command sign changes crossing the configured threshold, including intended alternating turns. Actual yaw rate uses truth stamps or bridged simulation clock if stamps are zero.",
                       "zero_commands": "Horizontal exploration commands below threshold for minimum duration after takeoff, excluding landing.",
                       "success": "Planner finished, on-ground confirmation after finish, and zero geometric overlaps; null unless truth covers at least 99% of flight samples and simulation clock never resets.",
                   }, "configuration": vars(self.args), "metrics": metrics,
                   "phases": {phase: self.metrics([row for row in self.rows if row["phase"] == phase])
                              for phase in ("startup", "exploration", "corridor", "landing")}}
        atomic_json(self.output / "summary.json", summary)
        try:
            self.plot(summary)
        except Exception as error:
            summary["plot_error"] = str(error)
            atomic_json(self.output / "summary.json", summary)
        print(json.dumps({"output_dir": str(self.output), "mission_complete": self.mission_complete,
                          "success": summary["success"], "ground_truth_status": summary["ground_truth_status"]}), flush=True)
        self.node.destroy_node()

    def plot(self, summary, filename="trajectory.png"):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon

        fig, axes = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": [1.3, 1]})
        axis, signals = axes
        for obstacle in self.obstacles:
            color = "#956337" if "rand_obs" in obstacle.name else "#787f83"
            if obstacle.kind == "cylinder":
                patch = Circle((obstacle.x, obstacle.y), obstacle.size_x, color=color, alpha=.8)
            else:
                c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
                corners = [(obstacle.x + c * x - s * y, obstacle.y + s * x + c * y)
                           for x, y in ((-obstacle.size_x / 2, -obstacle.size_y / 2),
                                        (obstacle.size_x / 2, -obstacle.size_y / 2),
                                        (obstacle.size_x / 2, obstacle.size_y / 2),
                                        (-obstacle.size_x / 2, obstacle.size_y / 2))]
                patch = Polygon(corners, color=color, alpha=.8)
            axis.add_patch(patch)
        colors = {"startup": "#92999e", "exploration": "#1870bc", "corridor": "#d1527d", "landing": "#19906d"}
        for phase, color in colors.items():
            points = [row for row in self.rows if row["phase"] == phase and row["truth_age_s"] is not None and row["truth_age_s"] < self.args.max_pose_age]
            if points:
                axis.plot([row["truth_x"] for row in points], [row["truth_y"] for row in points], color=color, lw=1.7, label=phase + " (truth)")
        truth_rows = [row for row in self.rows if row["truth_age_s"] is not None and row["truth_age_s"] < self.args.max_pose_age]
        if truth_rows:
            last = truth_rows[-1]
            axis.plot(last["truth_x"], last["truth_y"], "o", color="#191c1e", ms=5, zorder=5)
            axis.arrow(last["truth_x"], last["truth_y"], .45 * math.cos(last["truth_yaw"]),
                       .45 * math.sin(last["truth_yaw"]), width=.018, head_width=.15,
                       length_includes_head=True, color="#191c1e", zorder=6)
            collisions = [row for row in truth_rows if row["collision_proxy"]]
            if collisions:
                axis.scatter([row["truth_x"] for row in collisions], [row["truth_y"] for row in collisions],
                             s=14, marker="x", color="#dd2828", label="Envelope overlap", zorder=7)
        slam = [row for row in self.rows if row["slam_x"] is not None]
        if slam:
            axis.plot([row["slam_x"] + self.args.world_origin_x for row in slam],
                      [row["slam_y"] + self.args.world_origin_y for row in slam],
                      color="#42a9ac", lw=.7, alpha=.5, label="SLAM + configured origin")
        if getattr(self.args, "plot_active_path", False) and self.active_path:
            axis.plot([point[0] + self.args.world_origin_x for point in self.active_path],
                      [point[1] + self.args.world_origin_y for point in self.active_path],
                      "--", color="#dd9e1a", lw=1.6, label="Latest planned path (SLAM)")
        for index, point in enumerate(self.route):
            x, y = point[0] + self.args.world_origin_x, point[1] + self.args.world_origin_y
            axis.plot(x, y, "x", color="#b73066", ms=8)
            axis.annotate("E" if index == 0 else "H" if index == len(self.route) - 1 else str(index), (x, y), xytext=(5, 5), textcoords="offset points")
        axis.set(xlabel="Gazebo world X (m)", ylabel="Gazebo world Y (m)", title="Flight trajectory and actual world geometry")
        axis.autoscale_view()
        if self.obstacles:
            bounds = []
            for obstacle in self.obstacles:
                if obstacle.kind == "cylinder":
                    ex = ey = obstacle.size_x
                else:
                    c, s = abs(math.cos(obstacle.yaw)), abs(math.sin(obstacle.yaw))
                    ex = (c * obstacle.size_x + s * obstacle.size_y) / 2
                    ey = (s * obstacle.size_x + c * obstacle.size_y) / 2
                bounds.append((obstacle.x - ex, obstacle.x + ex, obstacle.y - ey, obstacle.y + ey))
            axis.set_xlim(min(item[0] for item in bounds) - .4, max(item[1] for item in bounds) + .4)
            axis.set_ylim(min(item[2] for item in bounds) - .4, max(item[3] for item in bounds) + .4)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=.15)
        if axis.get_legend_handles_labels()[0]:
            axis.legend(loc="lower left", fontsize=8)
        cmd = [row for row in self.rows if row["cmd_vx"] is not None]
        if cmd:
            signals.plot([row["t_wall"] for row in cmd], [math.hypot(row["cmd_vx"], row["cmd_vy"]) for row in cmd], label="Horizontal command (m/s)", color="#1870bc")
            signals.plot([row["t_wall"] for row in cmd], [row["cmd_yaw_rate"] for row in cmd], label="Yaw command (rad/s)", color="#d1527d", alpha=.8)
        clearance = [row for row in self.rows if row["surface_clearance_m"] is not None]
        if clearance:
            signals.plot([row["t_wall"] for row in clearance], [row["surface_clearance_m"] for row in clearance], label="Envelope clearance (m)", color="#19906d", lw=1)
        signals.axhline(0, color="#4b5053", lw=.6)
        signals.set(xlabel="Recorder wall time (s)", title="Commands and measured geometric clearance")
        signals.grid(alpha=.15)
        if signals.get_legend_handles_labels()[0]:
            signals.legend(fontsize=8)
        fig.suptitle("Completed: {} | Landed: {} | Truth: {}".format(summary["mission_complete"], summary["landed_after_mission"], summary["ground_truth_status"]))
        fig.tight_layout()
        fig.savefig(self.output / filename, dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--duration", type=float, default=600, help="Maximum recorder wall seconds")
    parser.add_argument("--ground-truth-topic", default="/sim/ground_truth")
    parser.add_argument("--model-name", default="x500_lidar_3d_0")
    parser.add_argument("--world-origin-x", type=float, default=-4.0)
    parser.add_argument("--world-origin-y", type=float, default=0.0)
    parser.add_argument("--robot-radius", "--body-radius", dest="robot_radius", type=float, default=.32)
    parser.add_argument("--robot-half-height", type=float, default=.15)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--progress-interval", type=float, default=.5)
    parser.add_argument("--max-pose-age", type=float, default=.5)
    parser.add_argument("--stop-speed", type=float, default=.035)
    parser.add_argument("--stop-min-duration", type=float, default=.4)
    parser.add_argument("--yaw-reversal-threshold", type=float, default=.15)
    parser.add_argument("--plot-active-path", action="store_true", help="Overlay only the latest planned path")
    args = parser.parse_args()
    if args.duration <= 0 or args.sample_hz <= 0 or args.robot_radius <= 0 or args.progress_interval <= 0:
        parser.error("duration, sample-hz, robot-radius and progress-interval must be positive")
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append("SIGINT"))
    signal.signal(signal.SIGTERM, lambda *_: stop.append("SIGTERM"))
    recorder = TrialRecorder(args)
    reason = "duration"
    try:
        while not stop and recorder.now() < args.duration and rclpy.ok():
            rclpy.spin_once(recorder.node, timeout_sec=.1)
        if stop:
            reason = stop[0]
    except Exception:
        reason = "exception"
        raise
    finally:
        recorder.close(reason)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
