#!/usr/bin/env python3
"""Run one isolated, reproducible PX4/Gazebo exploration trial."""

import argparse
import ast
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_SIM_ROOT = Path.home() / "uav_sim_env"
AUTO_DISARM_WAIT_S = 15.0


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--seed", type=int, required=True)
    result.add_argument("--output", "--output-dir", type=Path, required=True)
    result.add_argument("--duration", type=float, default=360.0,
                        help="Maximum flight wall time after OFFBOARD (seconds).")
    result.add_argument("--startup-timeout", type=float, default=180.0)
    result.add_argument("--offboard-timeout", type=float, default=45.0)
    result.add_argument("--preheat", type=float, default=3.0)
    result.add_argument("--domain", type=int, default=87, help="Dedicated ROS_DOMAIN_ID.")
    result.add_argument("--partition", help="GZ_PARTITION (default: unique per trial).")
    result.add_argument("--sim-root", type=Path, default=DEFAULT_SIM_ROOT)
    result.add_argument("--python", default="/usr/bin/python3", help="ROS-compatible Python.")
    display = result.add_mutually_exclusive_group()
    display.add_argument("--headless", dest="headless", action="store_true")
    display.add_argument("--gui", dest="headless", action="store_false")
    result.set_defaults(headless=True)
    result.add_argument("--viz", action="store_true", help="Show planner visualization.")
    result.add_argument("--dense-corridor-cloud", action="store_true",
                        help="Transform dense Point-LIO body cloud for corridor perception.")
    result.add_argument("--stack-launch", action="store_true",
                        help="Use installed competition_sim.launch.py for the ROS stack; enables dense corridor clouds.")
    result.add_argument("--no-offboard", action="store_true",
                        help="Leave activation to a manual OFFBOARD request.")
    result.add_argument("--no-recorder", action="store_true")
    result.add_argument("--no-failure-abort", action="store_true",
                        help="Record through flight anomalies until completion or timeout.")
    result.add_argument("--abort-hold-s", type=float, default=2.0,
                        help="Require repeated anomaly evidence over this wall time.")
    result.add_argument("--abort-slam-error-m", type=float, default=1.2)
    result.add_argument("--abort-penetration-m", type=float, default=0.08,
                        help="Sustained geometric obstacle-envelope penetration threshold.")
    result.add_argument("--abort-tilt-deg", type=float, default=60.0)
    result.add_argument("--abort-crash-z", type=float, default=0.18,
                        help="Unexpected world altitude after confirmed takeoff (metres).")
    result.add_argument("--airborne-z", type=float, default=0.45)
    result.add_argument("--planner-params", type=Path, help="ROS parameter YAML override.")
    result.add_argument("--mission-params", type=Path, help="ROS parameter YAML override.")
    result.add_argument("--planner-binary", type=Path,
                        help="Use a specific planner executable, such as an older trial snapshot.")
    result.add_argument("--mission-binary", type=Path,
                        help="Use a specific mission executable, such as an older trial snapshot.")
    return result


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n")
    temporary.replace(path)


def snapshot_executable(source, destination):
    source_realpath = source.resolve(strict=True)
    digest = hashlib.sha256()
    with source.open("rb") as input_file, destination.open("xb") as output_file:
        before = os.fstat(input_file.fileno())
        while True:
            block = input_file.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
            output_file.write(block)
        after = os.fstat(input_file.fileno())
    current = source.stat()
    identity = lambda value: (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns)
    if identity(before) != identity(after) or identity(before) != identity(current):
        raise RuntimeError(f"Executable changed during snapshot; retry after build finishes: {source}")
    os.chmod(destination, before.st_mode & 0o777)
    os.utime(destination, ns=(before.st_atime_ns, before.st_mtime_ns))
    return {"source_path": str(source), "source_realpath": str(source_realpath),
            "executable_path": str(destination), "sha256": digest.hexdigest(),
            "size_bytes": before.st_size, "mtime_ns": before.st_mtime_ns,
            "mtime_utc": datetime.datetime.fromtimestamp(before.st_mtime,
                                                         datetime.timezone.utc).isoformat()}


class FlightFailureMonitor:
    """Require fresh, repeated measurements; a single proxy overlap is not a crash."""

    def __init__(self, args):
        self.args = args
        self.airborne = False
        self.last_sample = None
        self.since = {}

    @staticmethod
    def number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    def update(self, progress):
        stamp = progress.get("wall_elapsed_s")
        if not self.number(stamp) or stamp == self.last_sample:
            return None
        if self.last_sample is not None and (stamp < self.last_sample or stamp - self.last_sample > 5.0):
            self.since.clear()
        self.last_sample = stamp
        age = progress.get("ground_truth_age_s")
        fresh = self.number(age) and 0.0 <= age < 0.5
        if not fresh:
            self.since.clear()
            return None
        z = progress.get("ground_truth_z")
        armed = progress.get("armed") is True
        if armed and self.number(z) and z >= self.args.airborne_z:
            self.airborne = True
        active = armed or (self.airborne and not progress.get("mission_complete"))
        if not active:
            self.since.clear()
            return None
        roll = progress.get("ground_truth_roll_rad")
        pitch = progress.get("ground_truth_pitch_rad")
        tilt = None
        if self.number(roll) and self.number(pitch):
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
        slam_error = progress.get("slam_truth_xy_error_m")
        clearance = progress.get("surface_clearance_m")
        diagnostics = progress.get("planner_diagnostics", {})
        required_goal_blocked = (isinstance(diagnostics, dict) and
                                 diagnostics.get("required_goal_blocked") == 1)
        conditions = {
            "required_goal_blocked": required_goal_blocked and not progress.get("mission_complete"),
            "unexpected_ground_contact": self.airborne and not progress.get("mission_complete") and
                                         self.number(z) and z <= self.args.abort_crash_z,
            "excessive_tilt": self.number(tilt) and tilt >= self.args.abort_tilt_deg,
            "slam_truth_divergence": self.number(slam_error) and slam_error >= self.args.abort_slam_error_m,
            "sustained_obstacle_penetration": progress.get("collision_proxy_active") is True and
                                               self.number(clearance) and clearance <= -self.args.abort_penetration_m,
        }
        for reason, condition in conditions.items():
            if not condition:
                self.since.pop(reason, None)
                continue
            began = self.since.setdefault(reason, stamp)
            if stamp - began >= self.args.abort_hold_s:
                return {"reason": reason, "sustained_wall_s": stamp - began,
                        "ground_truth_z": z, "tilt_deg": tilt,
                        "slam_truth_xy_error_m": slam_error,
                        "surface_clearance_m": clearance, "progress": progress}
        return None


class LandingCompletionMonitor:
    def __init__(self):
        self.started_at = None
        self.armed = None
        self.status = None

    def update(self, progress, now):
        if self.started_at is None:
            if not (progress.get("mission_complete") and progress.get("landed_after_mission")):
                return None
            self.started_at = now
        if isinstance(progress.get("armed"), bool):
            self.armed = progress["armed"]
        if self.armed is False:
            self.status = "completed"
        elif now - self.started_at >= AUTO_DISARM_WAIT_S:
            self.status = "landed_armed" if self.armed is True else "landed_disarm_unknown"
        else:
            self.status = "landed_waiting_disarm"
        return self.status

    def evidence(self, now):
        return {"auto_disarm_observed": self.status == "completed", "last_armed": self.armed,
                "waited_wall_s": now - self.started_at, "timeout_s": AUTO_DISARM_WAIT_S,
                "timed_out": self.status in ("landed_armed", "landed_disarm_unknown")}


def source_environment(sim_root):
    setups = [Path("/opt/ros/humble/setup.bash"),
              Path.home() / "ros_gz_ws/install/setup.bash",
              Path.home() / "ws_point/install/setup.bash",
              sim_root / "colcon_ws/install/setup.bash",
              WORKSPACE / "install/setup.bash"]
    for setup in setups:
        if not setup.is_file():
            raise RuntimeError(f"Missing ROS setup: {setup}")
    command = " && ".join(f"source {shlex.quote(str(path))} >/dev/null" for path in setups)
    raw = subprocess.check_output(["/bin/bash", "-c", command + " && env -0"], timeout=30)
    return {item.split(b"=", 1)[0].decode(): item.split(b"=", 1)[1].decode()
            for item in raw.split(b"\0") if b"=" in item}


def check_existing_simulation():
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            executable = (entry / "exe").resolve()
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode()
        except (OSError, UnicodeError):
            continue
        arguments = command.split()
        gazebo_cli = any(Path(arg).name == "gz" for arg in arguments[:2]) and "sim" in arguments
        if executable.name == "px4" or gazebo_cli or "gz-sim-server" in executable.name:
            raise RuntimeError(f"Existing simulator process PID {entry.name}; stop its owning "
                               "trial before starting another. No process was killed.")
    for port in (14540, 14557):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            try:
                probe.bind(("0.0.0.0", port))
            except OSError as error:
                raise RuntimeError(f"UDP port {port} is already occupied") from error


def ros_probe(stage, timeout):
    """Runs in sourced ROS environment through a private subprocess entry point."""
    import rclpy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import SetMode
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("acfly_trial_" + stage)
    state = {"connected": False, "mode": "", "odom_at": 0.0, "stamp": None,
             "odom_count": 0}

    def on_state(message):
        state["connected"] = message.connected
        state["mode"] = message.mode

    def on_odom(message):
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        position = message.pose.pose.position
        if stamp != state["stamp"] and all(math.isfinite(x) for x in
                                            (position.x, position.y, position.z)):
            state["odom_at"] = time.monotonic()
            state["odom_count"] += 1
            state["stamp"] = stamp

    node.create_subscription(State, "/mavros/state", on_state, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/aft_mapped_to_init", on_odom, qos_profile_sensor_data)
    client = node.create_client(SetMode, "/mavros/set_mode")
    deadline = time.monotonic() + timeout
    next_request = 0.0
    next_log = 0.0
    pending = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            fresh = state["odom_count"] >= 3 and now - state["odom_at"] < 1.0
            ready = state["connected"] and fresh and client.service_is_ready()
            if now >= next_log:
                print(json.dumps({"stage": stage, "connected": state["connected"],
                                  "mode": state["mode"], "fresh_odom": fresh,
                                  "set_mode_service": client.service_is_ready()}), flush=True)
                next_log = now + 5.0
            if not ready:
                continue
            if stage == "ready" or state["mode"] == "OFFBOARD":
                print(f"{stage}: ready", flush=True)
                return 0
            if pending is not None and pending.done():
                try:
                    print(f"OFFBOARD request accepted: {pending.result().mode_sent}", flush=True)
                except Exception as error:
                    print(f"OFFBOARD service failed: {error}", flush=True)
                pending = None
            if pending is None and now >= next_request:
                request = SetMode.Request()
                request.custom_mode = "OFFBOARD"
                pending = client.call_async(request)
                next_request = now + 2.0
        print(f"{stage}: timed out", flush=True)
        return 124
    finally:
        node.destroy_node()
        rclpy.shutdown()


class Trial:
    def __init__(self, args):
        self.args = args
        self.dense_corridor_cloud = args.dense_corridor_cloud or args.stack_launch
        self.stack_profile = None
        self.stack_launch_file = None
        self.stack_adapter = None
        self.output = args.output.expanduser().resolve()
        self.sim_root = args.sim_root.expanduser().resolve()
        self.px4_root = self.sim_root / "PX4-Autopilot"
        self.processes = []
        self.stopped = False
        self.lock = None
        self.output_owned = False
        self.metadata = {"seed": args.seed, "obstacle_count": 4,
                         "started_at": datetime.datetime.now().astimezone().isoformat(),
                         "status": "starting", "processes": []}

    def log(self, message):
        print(f"[{time.strftime('%H:%M:%S')}] {message}", flush=True)
        with (self.output / "runner.log").open("a") as stream:
            stream.write(f"[{time.strftime('%H:%M:%S')}] {message}\n")

    def save(self):
        write_json(self.output / "run.json", self.metadata)

    def start(self, name, command, cwd=None, critical=True):
        stream = (self.output / f"{name}.log").open("w")
        process = subprocess.Popen(command, cwd=cwd or WORKSPACE, env=self.env,
                                   stdin=subprocess.PIPE, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        item = {"name": name, "pid": process.pid, "command": command,
                "cwd": str(cwd or WORKSPACE), "log": f"{name}.log"}
        self.metadata["processes"].append(item)
        self.processes.append((process, stream, item, critical))
        self.save()
        self.log(f"Started {name}: PID {process.pid}")
        return process

    def ensure_alive(self):
        if self.stopped:
            raise KeyboardInterrupt
        for process, _, item, critical in self.processes:
            code = process.poll()
            if critical and code is not None:
                raise RuntimeError(f"{item['name']} exited with {code}; see {item['log']}")

    def wait(self, process, timeout):
        deadline = time.monotonic() + timeout
        while process.poll() is None and time.monotonic() < deadline:
            self.ensure_alive()
            time.sleep(0.2)
        if process.poll() is None:
            raise RuntimeError(f"Process PID {process.pid} timed out after {timeout:.1f}s")
        if process.returncode:
            raise RuntimeError(f"Process PID {process.pid} exited with {process.returncode}")

    def snapshot(self):
        snapshots = self.output / "snapshots"
        snapshots.mkdir()
        point_lio_share = Path(self.metadata["point_lio_prefix"]) / "share/point_lio"
        files = {
            "exploration_params.hpp": WORKSPACE / "src/exploration_planner/include/exploration_planner/params.hpp",
            "mission_params.hpp": WORKSPACE / "src/fly_mission/include/fly_mission/params.hpp",
            "arena_base.sdf": self.sim_root / "edc_arena_base.sdf",
            "randomize_arena.py": self.sim_root / "randomize_arena.py",
            "point_lio_sim.yaml": point_lio_share / "config/sim.yaml",
            "mapping_sim.launch.py": point_lio_share / "launch/mapping_sim.launch.py",
            "px4_parameters_before.bson": self.px4_root / "build/px4_sitl_default/rootfs/parameters.bson",
            "vehicle_model.sdf": self.px4_root / "Tools/simulation/gz/models/x500_lidar_3d/model.sdf",
        }
        if self.dense_corridor_cloud:
            files["dense_cloud_bridge.py"] = self.stack_adapter if self.args.stack_launch else (
                Path(__file__).with_name("dense_cloud_bridge.py"))
        if self.args.stack_launch:
            files["competition_sim.launch.py"] = self.stack_launch_file
            files["planner_override.yaml"] = self.stack_profile
        elif self.args.planner_params:
            files["planner_override.yaml"] = self.args.planner_params.resolve()
        if self.args.mission_params:
            files["mission_override.yaml"] = self.args.mission_params.resolve()
        for name, path in files.items():
            if path.exists():
                shutil.copy2(path, snapshots / name)
        if self.args.stack_launch:
            self.metadata["stack_resources"] = {
                name: {"source_path": str(files[name]), "snapshot_path": str(snapshots / name),
                       "sha256": hashlib.sha256((snapshots / name).read_bytes()).hexdigest()}
                for name in ("competition_sim.launch.py", "dense_cloud_bridge.py", "planner_override.yaml")}
        binary_dir = snapshots / "bin"
        binary_dir.mkdir()
        self.metadata["binaries"] = {}
        overrides = {"exploration_planner": self.args.planner_binary,
                     "fly_mission": self.args.mission_binary}
        for package in ("exploration_planner", "fly_mission"):
            executable_name = package + "_node"
            override = overrides[package]
            source = override.expanduser().resolve() if override else (
                Path(self.metadata[package + "_prefix"]) / "lib" / package / executable_name)
            self.metadata["binaries"][package] = snapshot_executable(source, binary_dir / executable_name)
            self.metadata["binaries"][package]["selection"] = "override" if override else "installed_package"
        self.metadata["source_worktree_snapshot"] = {
            "path": str(snapshots), "may_contain_unbuilt_changes": True,
            "note": "Sources and header defaults may include edits not compiled into this trial. "
                    "The copied executables in binaries are the authoritative compiled versions; "
                    "ROS parameter overrides are recorded separately.",
        }
        for name, repo in (("acfly", WORKSPACE), ("px4", self.px4_root)):
            for suffix, command in (("diff", ["git", "diff", "HEAD", "--", "."]),
                                    ("status.txt", ["git", "status", "--short"]),
                                    ("head.txt", ["git", "rev-parse", "HEAD"])):
                with (snapshots / f"{name}.{suffix}").open("w") as stream:
                    subprocess.run(command, cwd=repo, stdout=stream,
                                   stderr=subprocess.STDOUT, timeout=30, check=False)

    def prepare(self):
        if self.output.exists() and any(self.output.iterdir()):
            raise RuntimeError(f"Output directory must be empty: {self.output}")
        self.output.mkdir(parents=True, exist_ok=True)
        self.output_owned = True
        self.lock = open("/tmp/acfly-sim-trial.lock", "a+")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("Another run_trial.py owns the simulator lock") from error
        check_existing_simulation()
        if self.args.dense_corridor_cloud and not self.args.stack_launch and not Path(__file__).with_name("dense_cloud_bridge.py").is_file():
            raise RuntimeError("Dense corridor adapter not found: tools/sim/dense_cloud_bridge.py")
        self.env = source_environment(self.sim_root)
        partition = self.args.partition or f"acfly-trial-{self.args.seed}-{os.getpid()}"
        self.env.update(ROS_DOMAIN_ID=str(self.args.domain), GZ_PARTITION=partition,
                        IGN_PARTITION=partition, GZ_IP="127.0.0.1", IGN_IP="127.0.0.1",
                        ROS_LOG_DIR=str(self.output / "ros_logs"),
                        PYTHONUNBUFFERED="1", PX4_GZ_MODEL_POSE="-4,0,0,0,0,0",
                        PX4_GZ_WORLD="edc_arena")
        if self.args.headless:
            self.env["HEADLESS"] = "1"
        else:
            self.env.pop("HEADLESS", None)
        self.metadata.update(domain=self.args.domain, partition=partition,
                             duration=self.args.duration, headless=self.args.headless,
                             dense_corridor_cloud=self.dense_corridor_cloud,
                             stack_launch=self.args.stack_launch,
                             workspace=str(WORKSPACE), sim_root=str(self.sim_root))
        self.metadata["flight_failure_monitor"] = {
            "enabled": not self.args.no_failure_abort and not self.args.no_recorder,
            **{name: getattr(self.args, name) for name in
               ("abort_hold_s", "abort_slam_error_m", "abort_penetration_m", "abort_tilt_deg",
                "abort_crash_z", "airborne_z")},
        }
        for package in ("fly_mission", "exploration_planner"):
            prefix = subprocess.check_output(["ros2", "pkg", "prefix", package],
                                             env=self.env, timeout=15, text=True).strip()
            expected = WORKSPACE / "install" / package
            if Path(prefix).resolve() != expected.resolve():
                raise RuntimeError(f"Wrong {package} overlay: {prefix}; expected {expected}")
            self.metadata[package + "_prefix"] = prefix
        self.metadata["point_lio_prefix"] = subprocess.check_output(
            ["ros2", "pkg", "prefix", "point_lio"], env=self.env, timeout=15, text=True).strip()
        if self.args.stack_launch:
            prefix = Path(self.metadata["exploration_planner_prefix"])
            share = prefix / "share/exploration_planner"
            self.stack_profile = (self.args.planner_params.resolve() if self.args.planner_params else
                                  share / "config/physical_corridor.yaml")
            self.stack_launch_file = share / "launch/competition_sim.launch.py"
            self.stack_adapter = prefix / "lib/exploration_planner/dense_cloud_bridge"
            for resource in (self.stack_profile, self.stack_launch_file, self.stack_adapter):
                if not resource.is_file():
                    raise RuntimeError(f"Missing installed stack resource: {resource}; rebuild exploration_planner")
            declarations = {
                call.args[0].value for call in ast.walk(ast.parse(self.stack_launch_file.read_text()))
                if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
                and call.func.id == "DeclareLaunchArgument" and call.args
                and isinstance(call.args[0], ast.Constant)}
            if not {"planner_executable", "mission_executable", "mission_param_file"} <= declarations:
                raise RuntimeError("Installed competition_sim.launch.py lacks snapshot arguments; rebuild exploration_planner")
        self.save()
        self.snapshot()
        randomizer = self.start("randomize", [self.args.python,
                               str(self.sim_root / "randomize_arena.py"), "4", str(self.args.seed)],
                                cwd=self.sim_root, critical=False)
        self.wait(randomizer, 20)
        world = self.px4_root / "Tools/simulation/gz/worlds/edc_arena.sdf"
        shutil.copy2(world, self.output / "world.sdf")
        self.log(f"Saved randomized world, seed {self.args.seed}; starting simulation")

    def run(self):
        self.prepare()
        self.start("px4", ["make", "px4_sitl", "gz_x500_lidar_3d"], cwd=self.px4_root)
        if self.args.stack_launch:
            stack_arguments = {
                "param_file": str(self.output / "snapshots/planner_override.yaml"),
                "planner_executable": self.metadata["binaries"]["exploration_planner"]["executable_path"],
                "mission_executable": self.metadata["binaries"]["fly_mission"]["executable_path"],
                "viz": str(self.args.viz).lower(), "rviz": "false",
                "fcu_url": "udp://:14540@127.0.0.1:14557"}
            if self.args.mission_params:
                stack_arguments["mission_param_file"] = str(self.output / "snapshots/mission_override.yaml")
            self.metadata["stack_arguments"] = stack_arguments
            self.start("stack", ["ros2", "launch", "exploration_planner", "competition_sim.launch.py"] +
                       [f"{key}:={value}" for key, value in stack_arguments.items()])
        else:
            point_lio = ["ros2", "launch", "point_lio", "mapping_sim.launch.py", "rviz:=false",
                         "dense_cloud_adapter:=false",
                         f"dense_body_cloud:={str(self.dense_corridor_cloud).lower()}"]
            self.start("point_lio", point_lio)
            if self.dense_corridor_cloud:
                self.start("dense_cloud", [self.args.python,
                                            str(Path(__file__).with_name("dense_cloud_bridge.py"))])
            self.start("mavros", ["ros2", "launch", "mavros", "px4.launch",
                                  "fcu_url:=udp://:14540@127.0.0.1:14557"])
        truth_topic = "/world/edc_arena/dynamic_pose/info"
        self.start("ground_truth", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                   truth_topic + "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                                   "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                                   "--ros-args", "-r", truth_topic + ":=/sim/ground_truth"])
        if not self.args.no_recorder:
            recorder = Path(__file__).with_name("record_trial.py")
            if not recorder.exists():
                raise RuntimeError(f"Recorder not found: {recorder}")
            self.start("recorder", [self.args.python, str(recorder), "--output-dir", str(self.output),
                                    "--world", str(self.output / "world.sdf"), "--duration",
                                    str(self.args.duration + self.args.startup_timeout +
                                        self.args.offboard_timeout + self.args.preheat + AUTO_DISARM_WAIT_S + 30.0)])
        ready = self.start("ready", [self.args.python, str(Path(__file__).resolve()),
                                    "_probe", "ready", str(self.args.startup_timeout)], critical=False)
        self.wait(ready, self.args.startup_timeout + 10.0)
        if not self.args.stack_launch:
            planner = [self.metadata["binaries"]["exploration_planner"]["executable_path"], "--ros-args"]
            if self.args.planner_params:
                planner += ["--params-file", str(self.output / "snapshots/planner_override.yaml")]
            planner += ["-p", f"viz:={str(self.args.viz).lower()}"]
            if self.dense_corridor_cloud:
                planner += ["-p", "corridor_cloud_topic:=/corridor/cloud_registered_dense"]
            else:
                planner += ["-p", "corridor_cloud_topic:=/cloud_registered"]
            self.start("planner", planner)
            mission = [self.metadata["binaries"]["fly_mission"]["executable_path"]]
            if self.args.mission_params:
                mission += ["--ros-args", "--params-file", str(self.output / "snapshots/mission_override.yaml")]
            self.start("mission", mission)
        warmup = time.monotonic() + self.args.preheat
        while time.monotonic() < warmup:
            self.ensure_alive()
            time.sleep(0.2)
        if not self.args.no_offboard:
            offboard = self.start("offboard", [self.args.python, str(Path(__file__).resolve()),
                                               "_probe", "offboard", str(self.args.offboard_timeout)],
                                  critical=False)
            self.wait(offboard, self.args.offboard_timeout + 10.0)
        self.metadata["status"] = "flying"
        self.metadata["flight_started_at"] = datetime.datetime.now().astimezone().isoformat()
        self.save()
        self.log("Flight observation started")
        deadline = time.monotonic() + self.args.duration
        next_log = 0.0
        failure_monitor = FlightFailureMonitor(self.args)
        landing_monitor = LandingCompletionMonitor()
        self.metadata["status"] = "timeout"
        while True:
            self.ensure_alive()
            progress = {}
            try:
                progress = json.loads((self.output / "progress.json").read_text())
            except (OSError, json.JSONDecodeError):
                pass
            now = time.monotonic()
            landing_status = landing_monitor.update(progress, now)
            if landing_status is not None:
                self.metadata["auto_disarm"] = landing_monitor.evidence(now)
                if landing_status != self.metadata["status"]:
                    self.metadata["status"] = landing_status
                    self.save()
                    if landing_status == "landed_waiting_disarm":
                        self.log(f"Landing observed; waiting up to {AUTO_DISARM_WAIT_S:.0f}s for automatic disarm")
                if landing_status == "completed":
                    self.log("Exploration and corridor complete; landing and automatic disarm observed")
                    break
                if landing_status in ("landed_armed", "landed_disarm_unknown"):
                    self.log(f"Automatic disarm was not observed within {AUTO_DISARM_WAIT_S:.0f}s: {landing_status}")
                    return 3
            elif now >= deadline:
                break
            if not self.args.no_failure_abort:
                failure = failure_monitor.update(progress)
                if failure is not None:
                    blocked = failure["reason"] == "required_goal_blocked"
                    if blocked:
                        self.metadata.update(status="mission_blocked", mission_blocked=failure)
                    else:
                        self.metadata.update(status="flight_failed", flight_failure=failure)
                    self.save()
                    outcome = "blocked mission" if blocked else "failed flight"
                    self.log(f"Ending {outcome}: {failure['reason']} "
                             f"sustained {failure['sustained_wall_s']:.1f}s")
                    return 2
            if time.monotonic() >= next_log:
                remaining = deadline - now if landing_monitor.started_at is None else (
                    landing_monitor.started_at + AUTO_DISARM_WAIT_S - now)
                self.log(f"Observing flight: {max(0.0, remaining):.0f}s remaining; "
                         f"mission_complete={progress.get('mission_complete', False)}")
                next_log = time.monotonic() + 15.0
            time.sleep(0.25)
        return 0 if self.metadata["status"] == "completed" else 124

    def cleanup(self):
        # Child launchers sometimes exit before their descendants. Kill by the
        # session IDs created above, never by a system-wide process-name pattern.
        sessions = {process.pid for process, _, _, _ in self.processes}
        for sig, grace in ((signal.SIGINT, 12.0), (signal.SIGTERM, 4.0), (signal.SIGKILL, 2.0)):
            groups = set()
            for entry in Path("/proc").iterdir():
                if not entry.name.isdigit():
                    continue
                try:
                    pid = int(entry.name)
                    if os.getsid(pid) in sessions:
                        groups.add(os.getpgid(pid))
                except (ProcessLookupError, PermissionError):
                    pass
            if not groups:
                break
            for group in groups:
                try:
                    os.killpg(group, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                for process, _, _, _ in self.processes:
                    process.poll()
                remaining = False
                for group in groups:
                    try:
                        os.killpg(group, 0)
                        remaining = True
                    except ProcessLookupError:
                        pass
                if not remaining:
                    break
                time.sleep(0.1)
        for process, stream, item, _ in self.processes:
            item["returncode"] = process.poll()
            if process.stdin is not None:
                process.stdin.close()
            stream.close()
        if self.output_owned:
            self.metadata["ended_at"] = datetime.datetime.now().astimezone().isoformat()
            self.metadata["success"] = False if self.metadata["status"] != "completed" else None
            parameters = self.px4_root / "build/px4_sitl_default/rootfs/parameters.bson"
            if self.processes and parameters.exists() and (self.output / "snapshots").exists():
                shutil.copy2(parameters, self.output / "snapshots/px4_parameters_after.bson")
            try:
                summary = json.loads((self.output / "summary.json").read_text())
                self.metadata["metrics"] = {key: summary.get(key) for key in
                                            ("success", "mission_complete", "landed_after_mission",
                                             "collision_proxy_count", "ground_truth_sample_count")}
                if self.metadata["status"] == "completed":
                    self.metadata["success"] = summary.get("success")
            except (OSError, json.JSONDecodeError):
                pass
            self.save()
        if self.lock is not None:
            self.lock.close()


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "_probe":
        return ros_probe(sys.argv[2], float(sys.argv[3]))
    arguments = parser().parse_args()
    for name in ("duration", "startup_timeout", "offboard_timeout", "abort_hold_s",
                 "abort_slam_error_m", "abort_penetration_m", "abort_tilt_deg", "airborne_z"):
        if not math.isfinite(getattr(arguments, name)) or getattr(arguments, name) <= 0:
            parser().error(f"--{name.replace('_', '-')} must be positive and finite")
    if not math.isfinite(arguments.preheat) or arguments.preheat < 0:
        parser().error("--preheat must be nonnegative and finite")
    if not math.isfinite(arguments.abort_crash_z) or arguments.abort_crash_z < 0:
        parser().error("--abort-crash-z must be nonnegative and finite")
    if arguments.airborne_z <= arguments.abort_crash_z:
        parser().error("--airborne-z must exceed --abort-crash-z")
    if arguments.abort_tilt_deg > 180:
        parser().error("--abort-tilt-deg must be at most 180")
    if not 0 <= arguments.domain <= 232:
        parser().error("--domain must be in [0, 232]")
    for path in (arguments.planner_params, arguments.mission_params):
        if path is not None and not path.is_file():
            parser().error(f"Parameter file does not exist: {path}")
    for path in (arguments.planner_binary, arguments.mission_binary):
        if path is not None and (not path.expanduser().is_file() or not os.access(path.expanduser(), os.X_OK)):
            parser().error(f"Binary must be an existing executable file: {path}")
    trial = Trial(arguments)

    def interrupted(signum, _frame):
        trial.stopped = True
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        return trial.run()
    except KeyboardInterrupt:
        trial.metadata["status"] = "interrupted"
        return 130
    except Exception as error:
        trial.metadata.update(status="failed", error=str(error))
        print(f"Trial failed: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        trial.cleanup()


if __name__ == "__main__":
    sys.exit(main())
