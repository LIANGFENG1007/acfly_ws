# Reproducible simulation trials

Run from any directory. The runner sources ROS Humble and the installed workspaces
in dependency order, with `acfly_ws/install` last. It verifies that both flight
packages resolve to that workspace before launching anything.

```bash
/usr/bin/python3 /home/yk/acfly_ws/tools/sim/run_trial.py \
  --seed 42 --duration 360 --domain 87 --headless --stack-launch \
  --output /tmp/acfly-trials/seed42-baseline
```

The output directory must be empty. One simulator may run at a time because PX4's
MAVLink ports and generated world file are shared. A runner lock, existing-process
check, and UDP port checks reject conflicting trials without terminating them.
Use `--gui --viz` to show Gazebo and the planner. `--headless` is the default and
leaves the planner visualization disabled.

Each trial performs these steps:

1. Save parameter files, Git revision/diff/status, and simulation inputs.
2. Run `randomize_arena.py 4 SEED`, then save the exact resulting `world.sdf`.
3. Launch PX4 at `-4,0,0,0,0,0` in `edc_arena`, Point-LIO, MAVROS, ground-truth
   bridging, and the telemetry recorder in a dedicated ROS domain/Gazebo partition.
4. Wait for MAVROS connection, advancing odometry, and the mode service. Start the
   planner and mission nodes, allow preheating, then request OFFBOARD through the
   simulation's loopback MAVLink endpoint. The mission node handles arming.
5. Observe until mission completion, landing, and automatic disarm, or the requested flight duration.
   Interrupt the recorder to finalize its metrics and close this trial's process
   groups. Ctrl-C follows the same cleanup path.

`--duration` is wall-clock flight time after OFFBOARD is observed. Startup and
OFFBOARD have separate limits (`--startup-timeout`, `--offboard-timeout`). With
`--no-offboard`, the duration starts after mission preheating and the user can
activate OFFBOARD manually in the specified ROS domain. A completed flight exits
with code 0, a flight timeout with 124, an interruption with 130, and a startup or
process failure with 1. A sustained flight anomaly exits with 2 and stores
`flight_failure` evidence in `run.json`. Completion is distinct from the recorder's quality and
collision checks in `summary.json`.

After landing, the runner waits up to 15 additional wall-clock seconds for PX4 to
disarm automatically, including when the flight duration expires during that wait.
It never sends a disarm request. If the vehicle remains armed, `run.json.status`
is `landed_armed`; missing arm-state evidence produces `landed_disarm_unknown`.
Both exit with code 3, with wait details in `auto_disarm`. `run.json.success` can
only be true after automatic disarm and successful recorder quality checks;
`metrics.success` preserves the recorder's separate completion/geometry result.

The failure monitor uses fresh Gazebo truth and requires repeated measurements
over two seconds. Defaults stop an experiment after unexpected descent below
0.18 m following confirmed takeoff above 0.45 m, tilt beyond 60 degrees, SLAM
horizontal error above 1.2 m, or obstacle-envelope penetration beyond 0.08 m.
A single geometric overlap does not trigger termination. Expected landing after
mission completion is exempt from the unexpected-descent check. These limits can
be changed with `--abort-*` options; `--no-failure-abort` leaves termination to
completion or timeout. No truth data means these checks cannot establish a failure.

Parameter experiments can use ROS YAML overrides without changing header defaults:

```bash
/usr/bin/python3 /home/yk/acfly_ws/tools/sim/run_trial.py \
  --seed 42 --output /tmp/acfly-trials/seed42-candidate \
  --planner-params /tmp/candidate-planner.yaml
```

Use `--mission-params` for mission overrides. The runner copies the YAML into the
trial's snapshots and launches nodes against those copies. Source edits still
require rebuilding the corresponding packages before starting a trial.

Each trial copies the two resolved planner/mission executables to `snapshots/bin/`
and directly runs those copies with the sourced ROS environment. `run.json.binaries`
records each original path, resolved path, SHA-256, modification time, size, and
actual executable path. This preserves the running compiled version even when the
workspace is edited or rebuilt for the next trial. The source/header/Git snapshots
may contain changes not yet compiled; `source_worktree_snapshot` explicitly records
that limitation. Binary snapshots identify compiled behavior, while YAML overrides
and recorded launch commands identify runtime parameter changes.

For a same-seed comparison against an older compiled version, pass
`--planner-binary /path/to/old/snapshots/bin/exploration_planner_node` and/or
`--mission-binary /path/to/old/snapshots/bin/fly_mission_node`. Selected files are
copied and hashed again in the new trial, with `selection: override` in their
binary metadata. The normal ROS overlay checks still run. Omitting these options
uses the installed workspace executables.

`--dense-corridor-cloud` enables Point-LIO's `dense_body_cloud` launch argument,
starts `dense_cloud_bridge.py`, and directs the planner's `corridor_cloud_topic`
to `/corridor/cloud_registered_dense`. The adapter combines `/cloud_registered_body`
with full odometry poses, preserving dense door-edge returns for corridor perception.
SLAM's map filtering remains configured by its original parameters. Adapter output
is saved in `dense_cloud.log`, and its source is included in `snapshots/`.

## Results

- `run.json`: seed, domain, partition, resolved packages, process IDs/commands,
  runner outcome, and selected final recorder metrics.
- `world.sdf`: the exact randomized world loaded for this trial.
- `snapshots/`: header defaults, override YAML, Git information, world generator,
  Point-LIO configuration and `mapping_sim.launch.py`, vehicle model, and PX4's persisted parameters.
- `snapshots/bin/`: exact planner/mission executables launched for this trial.
- `px4.log`, `point_lio.log`, `mavros.log`, `planner.log`, `mission.log`,
  `ground_truth.log`, `recorder.log`: individual process logs.
- `ready.log`, `offboard.log`, `runner.log`: readiness, activation, and lifecycle.
- `progress.json`: live mission/landing status from the recorder.
- `summary.json`, `samples.csv`, `events.jsonl`, `trajectory.png`: final telemetry,
  event timeline, trajectory, and measured quality (see recorder `--help`).

The Gazebo bridge maps `/world/edc_arena/dynamic_pose/info` to `/sim/ground_truth`
as `tf2_msgs/msg/TFMessage`, and bridges `/clock` as `rosgraph_msgs/msg/Clock`
for simulation-time speed, yaw-rate, and duration measurements. Ground-truth availability is reported explicitly;
an odometry-only run cannot establish collision clearance against the saved world.
The runner does not reset PX4 tuning. Before/after parameter snapshots preserve
the persisted simulator configuration used by each experiment.

## Compare iterations

Refresh a table of finalized trials without starting ROS or simulation:

```bash
/usr/bin/python3 /home/yk/acfly_ws/tools/sim/compare_trials.py \
  --root /home/yk/acfly_ws/sim_runs
```

This writes `sim_runs/comparisons/iteration_summary.md` and `.json`. Automatic
discovery requires both a finalized `summary.json` and a completed `run.json`;
unfinished runs and partial outputs are skipped, not counted as failures.
The output separates recorder quality success from automatic-disarm observation.
Early runners stopped on landing, so their missing disarm observation stays unknown.
Runner status and structured failure/block reason are displayed as well. Expected
`mission_blocked` invalid-goal rejections are labeled `blocked`, counted separately
from collision/failure cases, and are never promoted to full mission success.

Pass explicit directories to compare a subset or include a finalized manual
recording that predates `run.json`:

```bash
/usr/bin/python3 /home/yk/acfly_ws/tools/sim/compare_trials.py \
  /home/yk/acfly_ws/sim_runs/20260909_seed11_baseline \
  /home/yk/acfly_ws/sim_runs/20260909_seed11_escape_validation \
  --output-dir /tmp/acfly-seed11-comparison
```

Coverage and handoff timings come from structured `events.jsonl`; unavailable
legacy telemetry remains unknown. Actual copied ELF hashes, world hashes,
mission hashes and runtime YAML hashes identify reproducible inputs. The documented
Seed 13/14/15 dense runs use the preserved `dense_corridor_v1` binary mapping;
other old runs without executable evidence remain unknown. Source snapshots do
not identify which compiled version was running.

When an `audit_footprint*.json` anywhere under the selected root contains a matching
trial, the newest matching audit's SDF-derived rotor/body clearance is shown in
separate columns. It does not replace the original
cylindrical monitoring clearance or imply contact-sensor verification.

For a live trajectory snapshot, run `plot_trial.py --output-dir TRIAL_DIR`;
`--plot-active-path` overlays only the latest planned path. This writes
`trajectory_live.png` and `summary_replay.json` without changing the recorder's
live progress or final summary. The separate SDF footprint audit can be refreshed
with the distribution NumPy/SciPy runtime:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages /usr/bin/python3 \
  /home/yk/acfly_ws/tools/sim/audit_footprint.py --seeds 13 14 15 16 17 18
```

## User startup commands

Use the original commands below, in separate terminals. No replacement launch
command or extra parameter file is required.

PX4 / Gazebo:

```bash
cd ~/uav_sim_env/PX4-Autopilot && PX4_GZ_MODEL_POSE="-4,0,0,0,0,0" PX4_GZ_WORLD=edc_arena make px4_sitl gz_x500_lidar_3d
```

Randomize four obstacles:

```bash
cd ~/uav_sim_env
python3 randomize_arena.py 4
```

Changes to the generated world file take effect the next time Gazebo loads it.

Point-LIO:

```bash
ros2 launch point_lio mapping_sim.launch.py
```

MAVROS:

```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

Mission node:

```bash
ros2 run fly_mission fly_mission_node
```

Exploration planner:

```bash
ros2 run exploration_planner exploration_planner_node
```

OFFBOARD:

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

The existing Point-LIO `mapping_sim.launch.py` enables the dense body cloud and
its corridor adapter by default. The planner's header defaults select that dense
corridor topic and the validated simulation settings, including 0.64 m aircraft
width, 0.015 m center tolerance, 0.30 m/s gate speed, and continuous alignment with
a 0.40 s drift prediction. The normal commands above load these defaults directly.
The planner's existing visualization includes the corridor.

Mission coordinates are in
`src/fly_mission/include/fly_mission/params.hpp`; planner and corridor settings are
in `src/exploration_planner/include/exploration_planner/params.hpp`. Rebuild the
corresponding package after editing a header. ROS parameter overrides and
`tools/sim/profiles/physical_corridor.yaml` remain optional tools for experiments;
the user startup sequence does not require them.

## Internal stack-launch test mode

`competition_sim.launch.py` and the runner's `--stack-launch` option remain as
internal test tools and preserve the startup method used by completed historical
trials. They are not the user's normal startup entry point. This optional launch
starts Point-LIO, MAVROS, the dense adapter, planner and mission together; do not
combine it with separate instances of those same nodes.

In this test mode the runner retains randomization, PX4 startup, recording,
readiness, OFFBOARD, failure monitoring and cleanup. Planner and mission run their
copied ELF snapshots. ROS stack output is combined in `stack.log`, and
`run.json.stack_arguments` plus `stack_resources` record installed launch/adapter/
profile paths, snapshots and hashes. `--planner-params` and `--mission-params`
select snapshot YAML overrides. Reinstall a changed launch before using this
optional test mode; the runner rejects older installed launches missing its
snapshot executable arguments.
