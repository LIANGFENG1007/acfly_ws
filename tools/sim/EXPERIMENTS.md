# Iteration Log

User authorization: repeatedly randomize four obstacles, run the complete PX4,
Gazebo, Point-LIO, MAVROS and acfly mission, and iterate until the user stops work.
No real-flight execution is part of these trials.

## Baseline: Seed 11

- Source baseline: `50e3df52a977b2fae43edbbf7cb4ddd4231267d2`.
- Output: `sim_runs/20260909_seed11_baseline`.
- Startup pose: world `(-4, 0, 0, 0, 0, 0)`; four cylinders generated before world load.
- Current user settings retained: exploration x `[0, 7.5]`, y `[-5, 5]`,
  entry `(8.1, 4.25)`, H `(8.1, -4.0)`, coverage threshold 90%, speed 0.8 m/s.
- Flight reached 90% in approximately 167.6 s after exploration goal receipt.
- Exploration path about 36 m; frequent fallback, route replacement and turning
  episodes. Position estimation remained close to truth during exploration.
- Trial failed in the second corridor opening. Before the disturbance the
  vehicle was about 7.5 cm left of the true opening center. Its configured
  corridor width was 0.50 m, but x500 rotor swept width is approximately 0.627 m.
  Rapid rotation, impact and SLAM divergence followed. Do not count this as success.
- PX4 stdin must remain open (PTY or PIPE), otherwise its console floods logs
  on EOF. The failed startup log was reduced to its useful startup prefix.

## Next Trial: Seed 12

- Keep exploration behavior unchanged for comparison, add numerical diagnostics.
- Apply `profiles/physical_corridor.yaml`: actual aircraft envelope 0.64 m and
  tighter center tolerance 0.015 m, retaining 0.30 m/s crossing speed.
- Validate on a different four-obstacle map; the corridor itself is unchanged.
- Follow safety correction with targeted replan and smoothing experiments based
  on recorded counters, curvature and true trajectory, then retest earlier seeds.

## Seed 12: Physical Envelope

- Output: `sim_runs/20260909_seed12_physical_corridor`.
- Reached 90% about 78.5 s after the exploration goal, but failed again near door B.
- Estimated B opening center was approximately SLAM X=8.483, versus physical
  X=8.575. Estimated gap 0.834 m exceeded the actual 0.75 m: voxelized endpoint
  returns cannot locate this tight opening reliably. Tightening footprint alone failed.
- Diagnostics: 949 replan checks, 507 A* calls, 197 adopted routes, 188 fallbacks.
- Harness aborted sustained SLAM/truth divergence rather than recording a long crash.
- The missing harness /clock bridge has been added for following trials.

## Next Trial: Seed 13

- Enable Point-LIO's existing full body-frame scan and transform it with matched
  source-time odometry, using dense data only for corridor perception.
- Keep Point-LIO registration/map downsampling at 0.3 m.
- Include continuous segment progress for old-route collision checks and scoring;
  preserve A* margins and search behavior in this step.

## Seed 13 Result

- Output: `sim_runs/20260909_seed13_dense_corridor`.
- Complete: both doors, H, and on-ground landing observed. No geometric-envelope
  collision; minimum clearance using the 0.32 m monitoring radius was 0.049 m.
- Total recorded horizontal truth path 34.50 m. Diagnostics: 285 A* calls,
  124 adoptions, 112 smoothing fallbacks; substantial exploration waste remains.
- Door sensing used dense body scans with exact source-time pose matches and
  no dropped frames. Final heading remained approximately -90 degrees.
- Seed 14 is now validating the same configuration on another four-obstacle layout.

## Seed 14 Result

- Output: `sim_runs/20260909_seed14_dense_corridor`.
- Complete: both doors, H and landing; no geometric-envelope collisions.
- Recorded truth path 35.11 m; minimum monitored clearance 0.044 m.
- Diagnostics: 332 A* calls, 149 adoptions, 137 fallbacks. Both successful seeds
  still show redundant exploration replanning and fallback corner stops.
- Seed 15 is validating the same dense-corridor configuration while the strictly
  outward escape validation fix is prepared and unit tested for later trials.
- The exact binaries used by seeds 13/14/15 are preserved under
  `sim_runs/builds/dense_corridor_v1`, with SHA-256 hashes. Source snapshots can
  include edits prepared for the following build; binary snapshots are authoritative.

## Prepared: Consistent Escape Validation

- Keep nominal margins, but allow a start already within the extra margin to
  move strictly outward while the full physical body stays clear. This makes
  result simplification consistent with the existing A* start-relax behavior.
- A destination still inside the margin, inward motion, or physical overlap is
  rejected. Analytic segment distances replace sampled clearance checks.
- Apply the same monotonic entry rule to an initial pose outside the virtual
  field inset, and distinguish field versus obstacle smoothing failures.
- Target seed 16 and then repeat seed 11 for a comparable efficiency check.

## Seed 15 Result

- Same dense-corridor v1 executable as seeds 13/14. Complete mission and landing,
  no envelope overlap, minimum monitored clearance 0.043 m.
- 556 A* calls, 229 adoptions, 222 fallbacks, 15 recovery entries. At least 203
  adoptions repeated the same target. This map still exposes substantial waste.
- A 4.55 cm initial path segment caused a 5.8 s near-zero episode while the
  tracker continued aiming at that short segment behind the vehicle.

## Seed 16 Result

- Output: `sim_runs/20260909_seed16_escape_validation`.
- First run of the consistent escape/field-entry validation build. Complete
  mission and landing, no envelope overlap, minimum monitored clearance 0.035 m.
- 196 A* calls, 49 adoptions, 37 fallbacks and 2 recovery entries. Map differs
  from seeds 13-15, so this is a successful validation, not a controlled speedup.
- Repeating seed 11 before the next change to compare the same obstacle layout.

## Prepared: Strict Exploration Path Contract

- Recorded repeated same-target adoptions also arise when A* appends a blocked
  original target after finding a free proxy, or relaxes the initial search
  neighborhood through the aircraft envelope. Raw fallback was not validated.
- Add an exploration-only strict search mode: valid physical start connections,
  monotonic margin escape, free observation endpoint, and final full validation.
- Validate raw paths before adoption; reaching a projected observation point
  triggers another viewpoint instead of trying to enter the blocked target.
- Required red-point/POI behavior remains on its existing search mode in this
  experiment. The configured red point lies outside the virtual field inset.

## Escape Validation: Seed 11 Repeat and Seed 17

- Both completed exploration, two doors, H, landing and automatic disarm.
- Seed 11 repeat: 762 A* calls, 232 adoptions, 215 fallbacks, 19 recoveries;
  minimum cylindrical monitoring clearance 0.044 m. Remaining invalid endpoints
  still cause repeated resets despite fixing some initial escape stubs.
- Seed 17: 329 A* calls, 42 adoptions, 28 fallbacks, 1 recovery; minimum
  monitoring clearance 0.044 m. Same escape-validation executable as seed 16.
- From the seed 11 repeat onward, the harness waits for automatic disarm after
  landing and explicitly distinguishes landing while still armed from success.

## Seed 18: Strict Exploration Path Contract

- Output: `sim_runs/20260909_seed18_strict_paths`.
- Complete mission and automatic disarm; no cylindrical monitoring overlaps.
- 29 adoptions, 2 smoothing fallbacks, 1 recovery and zero rejected paths at
  adoption. A* still ran 608 times, so nonurgent candidate search frequency is
  the next separate issue. Minimum monitoring clearance 0.048 m.
- Seven CTest cases passed: path progress, escape, strict safety, trajectory
  overshoot and flowing turns with standard and production gains.
- Seed 15 now repeats with this exact executable for a same-map comparison.
- The 0.32 m monitoring cylinder is based on the x500 single-axis half-width.
  It is useful in the fixed-heading corridor but is not a conservative envelope
  for every exploration heading. A separate yaw-aware footprint audit is planned.

## Controlled Repeat: Seed 15 Strict Paths

- World SDF SHA-256 is identical for the old and new seed 15 trials:
  `a4cfb045c16f0a68f29bb96b231e869dff4bcca1c851245d478d2e00217e3f82`.
- Compared with dense v1, the strict-path build completes goal-to-corridor
  handoff in 58.562 s instead of 98.600 s (40.6% shorter).
- Exploration truth path: 28.818 -> 24.639 m. Near-zero command time:
  33.601 -> 4.500 s; episodes 17 -> 3.
- A* calls 556 -> 205; path adoptions 229 -> 21; smoothing fallbacks
  222 -> 0; recovery entries 15 -> 1. No invalid path passed adoption.
- Both versions completed both doors and H. New run additionally observed
  landing and automatic disarm. Corridor duration stayed about 49 s.
- New minimum monitoring clearance: exploration 0.326 m, corridor 0.045 m.
  These remain the same cylindrical monitoring metric, not direct contacts.
- This is one controlled repeat, with normal estimator/simulation variability;
  continue new maps and repeat difficult layouts before increasing speed.

## Seed 19: Candidate Timing

- Output: `sim_runs/20260909_seed19_candidate_timing`.
- Search candidates no more than every 0.6 s on a safe, useful remaining route.
  Explicit requests, unsafe routes, exhausted routes and stale targets bypass
  that delay. Deviation alone no longer causes safe routes to replan at 50 Hz.
- Route safety and scoring use the tracker's actual remaining path; coverage
  band changes commit only with adoption or an explicit unreachable decision.
- Complete mission and automatic disarm: 270 decision checks, 171 candidate
  skips, 68 A* searches, 25 adoptions, zero fallbacks and zero recoveries.
  Two short cone-search entries; minimum cylindrical clearance 0.046 m.
- Twelve path, tracking and existing corridor regressions passed. Separate
  actual-node checks cover urgent bypass and delayed band commitment.

## Prepared: Continuous Door Alignment

- Enable only with `profiles/continuous_corridor.yaml`; default remains false
  pending real-flight simulation validation. Corridor width, center tolerance
  and gate speed stay at 0.64 m, 0.015 m and 0.30 m/s for the x500 trials.
- Keep the complete 1 m straight exit. During approach, aim along the next
  measured opening toward its exit, limiting forward speed by the lateral
  alignment time and the remaining distance to a body-clear stop line.
- Time estimate includes lateral speed saturation, PD convergence, current
  lateral momentum, and a configurable extra 0.5 s. Center reserve is 0.15 m
  beyond the aircraft radius and perception resolution.
- Existing center/predicted-center release tests, cloud freshness and geometric
  motion checks remain active; insufficient alignment room may still stop.

## Continuous Alignment: Seeds 20 and 21

- Both completed both doors, H, landing and automatic disarm. Seed 20's second
  approach had zero near-zero command samples and minimum command speed
  0.14177 m/s; actual speed remained above 0.1336 m/s there.
- Seed 20 total corridor duration was 46.18 s. Its smallest monitoring clearance
  (0.02851 m; separate part audit 0.03510 m) occurred at the inside entrance-wall
  endpoint before door A, not during the second lateral transition.
- Lock-time SLAM/truth yaw correction shows similar gate-center measurement
  residuals to earlier trials. Do not change wall estimation based on this alone.
- Early release into the long first-door target retains lateral momentum and
  weakens lateral braking after total-vector speed limiting. Prepared an isolated
  `continuous_corridor_prediction.yaml` experiment changing prediction .25 -> .40 s.

## Seed 22 Failure: Occupied Required Goal

- Output: `sim_runs/20260909_seed22_continuous_corridor`.
- Failed during homing before corridor activation. The configured handoff
  `(7.00, 4.25)` lies inside `rand_obs_2`: SLAM center approximately
  `(6.9900, 4.0970)`, cylinder radius 0.20 m. Exact handoff is physically impossible
  in this generated map; do not change the saved world or silently move the goal.
- Old homing code appended the occupied goal and followed it. Observed overlap
  and SLAM divergence followed; the harness stopped the trial.
- Added a required-goal occupancy guard: zero translation, explicit blocked
  diagnostic, no retreat or false completion. Replaying seed 22 with this guard
  is a negative regression; expected outcome is mission_blocked without collision.
- Harness records persistent required_goal_blocked separately as mission_blocked,
  not flight_failure. The full required-path/terminal-connector and execution
  validation changes are being prepared before resuming general optimization.

## Seed 22 Guard Regression

- `20260909_seed22_blocked_goal_guard` uses the unchanged seed 22 world and target.
- The aircraft stopped safely after detecting the occupied handoff. Runner status
  is `mission_blocked`, reason `required_goal_blocked`, rather than success.
- Zero monitoring overlaps; minimum cylinder clearance 0.531 m, independent
  component clearance approximately 0.522 m, final SLAM/truth XY error 0.0067 m.
- Original failed-run evidence shows dangerous approach/physical instability
  preceding SLAM divergence. At t=77.980 s component clearance was only 2.57 mm
  with XY error 11.7 mm; first recorded swept-component overlap was t=78.080 s.

## Longer Prediction and Formal Stack

- `20260909_seed20_prediction_040` completed with the .40 s horizon. Compared with
  .25 s, first-Cross lateral release speed changed from -0.0570 to -0.00158 m/s;
  subsequent leftward travel changed from 3.54 cm to 0.256 cm.
- Cylinder minimum 0.02851 -> 0.04062 m; component minimum 0.03510 -> 0.04722 m.
  Corridor time 46.180 -> 46.939 s. Both approaches retain no >=0.4 s near-zero
  segment, and second Approach has no near-zero command samples in either run.
- `20260909_seed23_formal_stack` completed through the installed
  `competition_sim.launch.py` with the standard physical profile. Exact snapshot
  planner and mission executables ran through launch arguments, with matched
  dense clouds, recorder and automatic-disarm confirmation.
- `20260909_seed24_stack_prediction` completed through the same launch with the
  .40 s continuous profile; minimum monitoring clearance 0.04468 m.

## Prepared: Full Required-Path Execution

- Required A* uses a virtual exact terminal node, a bounded verified final
  connector, strict normal edges and an unchanged field inset for the prefix.
- Homing and POI follow a dedicated instance of the existing trajectory tracker;
  the old Euclidean-near-goal shortcut is removed. Prefix smoothing and any raw
  fallback must satisfy the full required-path contract.
- The reference remainder excludes off-reference current-position samples.
  Actual terminal motion is limited to the approved connector's existing goal
  tolerance capsule, with physical field and full obstacle checks maintained.
- Commanded and measured stopping projections are checked before translation;
  blocked translation resets the downstream acceleration state, and no unverified
  retreat is used for a required-path failure.
- Node closed-loop checks passed straight and obstacle-detour tasks, terminal
  tracking errors, blocked targets, short recovery steps and velocity braking.
  Next real-simulation validation: new seed 25, then known difficult maps.

## Seed 25 and Seed 26 Required-Tracker Validation

- Seed 25 was safely reported `mission_blocked`. Its requested handoff is only
  0.266 m from a cylinder surface, insufficient for the x500 body. Zero overlap;
  this is another preserved unavailable-goal case, not a completed mission.
- First seed 26 run stopped safely but could not resume homing: the reference
  remained valid while full planning-margin checks rejected instantaneous
  pure-pursuit motion. It was interrupted after documenting the persistent state.
  Minimum independent component clearance was 0.416 m; no overlap or localization
  failure occurred. This was a progress regression and is not counted as success.
- Motion checking now has its own 0.15 m additional margin; reference planning
  and goal acceptance retain 0.30 m. Desired motion is always checked; measured
  velocity below the existing stopped threshold cannot independently latch noise.
- A stationary block lasting 0.50 s invalidates the route for a validated retry.
  New diagnostics distinguish command and measured-motion checks. The recorded
  geometry is represented in a persistent required_node_test regression.
- Corrected coverage blacklist checks to use the clamped executable waypoint
  in both normal and fallback selection. This closes the repeated corner-target
  preimage bug observed in seed 21; scoring and band selection stay unchanged.
- `20260909_seed26_motion_recovery` completed the whole mission and automatic
  disarm, with 51 total A* searches, 7 required-path searches, 13 exploration
  adoptions, zero smoothing fallbacks and zero required velocity brakes.
  Minimum cylindrical clearance was 0.04978 m.
- Ten relevant CTest targets passed, including the persisted actual-node and
  blacklist regressions. The footprint report now handles unexecuted phases.
- The installed physical_corridor profile now enables continuous alignment and
  the validated .40 s prediction. New seed 27 is testing those launch defaults.

## User-Requested Closeout

- The user ended continuous iteration and requested completion of work already
  in progress. No new broad tuning sweep is scheduled.
- Near-reversal Bezier blends above 120 degrees now retain the validated raw
  route. Matched seed21 geometry/initial-state closed-loop: 27.54 -> 14.88 s;
  actual speed below .05 m/s for 16.62 -> 4.06 s. Ordinary 45/90 degree geometry
  remains identical. Corrected a test comparing 12-sample displacement (7.22 cm)
  against the continuous cubic extremum (7.60 cm); both are documented.
- Seed27 and the later seed21 blacklist-fixed repeat completed full missions
  and automatic disarm. The seed21 repeat used 156 A* calls versus earlier395,
  with a different flown path; do not attribute all timing differences to one fix.
- The first closeout seed28 held at launch because strict field-entry expansion
  was still truncated by the legacy .6 m start-relax box. The trial was interrupted
  with no collision. Strict edges already enforce monotonic field/obstacle escape,
  so the arbitrary box restriction was removed while preserving edge checks.
- Added a seed28-like field-entry detour regression; empty waypoint candidates
  now hold/retry at the planning period instead of A* to the current point.
- All ten affected path/node/tracking targets pass after this correction.
  Seed28 is rerunning as the final installed-stack check.

## Final Validation Complete

- `20260909_seed28_final_recheck` completed exploration, both doors, H, landing
  and automatic disarm. Duration from takeoff to H: 135.15 s. No monitored or
  component-audit overlaps. Component clearance minima: exploration 0.39367 m,
  corridor 0.05260 m. Required-path search count1; braking recovered normally.
- Final installed planner and this trial's executable match SHA-256
  `843291239cc9e12c4080604030944a9e42877a54077105135986dcbe24fb941d`.
- All19 functional CTest targets and6 dense-adapter tests pass. Python syntax,
  flake8, cppcheck, pep257, xmllint and source-only CMake lint pass. Full-package
  CMake lint scans historical generated files; uncrustify style differences
  remain explicitly documented without reformatting the whole codebase.
- All simulation and ROS flight-stack processes from this work are closed.
  No further iteration or automation is scheduled. Chinese handoff instructions
  and final limitations are in `tools/sim/FINAL_RESULTS.md`.

## Restore User's Original Commands

- User requested the original separate-terminal commands, without the combined
  launch replacing them. Documentation now repeats those commands unchanged.
- Existing Point-LIO mapping_sim launch defaults to full body scans and starts
  the installed dense adapter. Planner defaults now match the validated x500
  configuration directly from its parameter header, including dense cloud topic.
- Internal combined-launch and trial modes explicitly disable the mapping-owned
  adapter when they already start one, preventing duplicate publishers.
- Rebuilt and verified the original mapping/planner commands in an isolated ROS
  domain with no flight mission: body publication enabled, expected six planner
  parameters, exactly one dense publisher, and matched pose/cloud transformation.
  All test processes were closed. Five targeted CTest checks and six adapter
  tests passed; Python syntax and diff checks passed.

## Next Optimization Evidence

Seed 12 diagnostics attribute 24.85 s of 29.06 s near-zero exploration commands
to reorientation. Recorded paths contain 4.9-6.2 cm initial grid-centre stubs;
after sliding past a stub, the tracker still targets it behind the aircraft.
Fix only safety-validated start connections and repeated same-target adoption
before experimenting with larger speed or curvature limits.
