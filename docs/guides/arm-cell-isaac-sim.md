# [SF-Twin] ARM Cell Isaac Sim Development Procedure

- **Document ID:** `GUIDE-ARM-CELL-ISAAC-SIM`
- **Document Type:** `Guide`
- **Scope:** `ARM Cell / Isaac Sim development planning and RGB-D verification`
- **Version:** `1.1.0`
- **Status:** `Active`
- **Owner:** `ARM Cell Simulation Runtime` (Isaac Simulation and ROS Bringup)
- **Applicable Environment:** `Ubuntu / ROS 2 Humble / Isaac Sim 5.1`
- **Related Design:** `Architecture Record 0001`, `Architecture Record 0002`, `Architecture Record 0003`,
  `docs/design/arm-cell/interfaces/icd-sim-state-time-tf.md`,
  `docs/design/arm-cell/interfaces/icd-sim-rgbd.md`

## 1. Purpose

This Guide describes the repeatable operator procedure for building, starting, resetting, and
verifying the supported ARM Cell Isaac Sim development profiles.

It does not define model/state/TF/time/static-scene/RGB-D contract semantics. Refer to the
related Architecture Records/design documents for those contracts. Historical PASS results belong in Acceptance
Reports, not in this Guide.

This Guide is the operator procedure for Isaac runtime setup and operation.

## 2. Preconditions

- Ubuntu environment with ROS 2 Humble installed.
- Isaac Sim 5.1 with the existing M0609/Robotiq ARM Cell workflow available.
- repository checkout with symlinks preserved.
- matching ROS domain/DDS configuration between Isaac and ROS terminals.
- required ROS dependencies installed:
  - `robot_state_publisher`
  - `tf2_ros`
  - Xacro
  - MoveIt `move_group`
  - KDL kinematics
  - OMPL
- Isaac Script Editor access for the role-based ARM Cell runtime scripts.
- no competing `/clock`, canonical `/joint_states`, or `world -> base_link` publisher in the
  selected ROS domain.

The workflow uses both host-terminal commands and Isaac Script Editor actions. Use the
repository-provided commands and role entrypoints described below.

## 3. Procedure

### 3.1 Build the ROS profile

From the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to arm_cell_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

Expected observable:

- build completes without package build failure;
- the ROS overlay can resolve `arm_cell_bringup`.

### 3.2 Prepare generated URDF when rebuilding robot assets

Run from the repository root only when the robot asset must be rebuilt:

```bash
mkdir -p infra/isaac_sim/assets/urdf/generated
xacro "$PWD/infra/isaac_sim/assets/urdf/assemblies/m0609_robotiq_2f85.xacro" \
  -o infra/isaac_sim/assets/urdf/generated/m0609_robotiq_2f85.urdf
check_urdf infra/isaac_sim/assets/urdf/generated/m0609_robotiq_2f85.urdf
```

Expected observable:

- Xacro expansion succeeds;
- `check_urdf` accepts the generated URDF.

### 3.3 Prepare the Isaac Script Editor session

Set `SFTWIN_PROJECT_ROOT` in the Isaac process to the extracted repository root before running
the role entrypoints. Leave `PROJECT_ROOT = None` unless a deliberate local override is needed.
`1_before_play.py` validates that configured root, configures the Script Editor module path and
ROS domain, and selects production runtime mode. The diagnostic PnP profile is not required.

For optional component capture in the independently started Isaac UI, set the same component
selection in that process before running `1_before_play.py`, for example:

```bash
export SFTWIN_DIAGNOSTICS=motion
```

The accepted values are `none` (default), `all`, or a comma-separated list of `motion`, `vision`,
`safety`, and `orchestration`. The Script Editor validates the selection and its Gripper Capture
Diagnostic Record/PhysX Observer are created when `motion` is selected. `clock` and `gripper` are
not independent selection values. ROS Launch settings do not reach a separate already-running
Isaac UI process; set `SFTWIN_DIAGNOSTICS=motion` in the Isaac process before running
`1_before_play.py`. The ROS launch summary explicitly reports this producer as process-local and
does not claim the UI runtime was activated.
The component paths and producer behavior are documented in
[ARM Cell Observability Records ICD](../design/arm-cell/interfaces/icd-observability-records.md).

### 3.4 Construct the Isaac ARM Cell baseline

After the ROS workspace build is complete, run the role-based entrypoints in Isaac Script
Editor. Run `0_pre_build.py` only when generated robot assets need rebuilding; for a normal live
run, start with `1_before_play.py`:

```text
Asset rebuild (when needed): 0_pre_build.py
Normal live run:             1_before_play.py -> Play -> 2_after_play.py
```

`0_pre_build.py` prepares the generated robot asset. `1_before_play.py` authors the robot and scene,
exports the composed camera geometry snapshot, and builds the ROS ActionGraph while the stage
is stopped. `2_after_play.py` initializes
the gripper physics runtime and loads the Simulation UI Hub after Play. Keep the Script Editor
session alive while the simulator runs.

Expected observable:

- the active stage contains the production ARM Cell scene and ROS ActionGraph before Play;
- after Play, gripper runtime reports ready and the Hub observes the selected ROS domain.

The implementation steps live under `scripts/m0609_cell/runtime_steps/`; operators run only the
role entrypoints above. Re-running a role reads the current checkout source into the persistent
Script Editor process.

### 3.4.1 Live development iteration

- After ROS Python/C++ code or launch/configuration changes, stop the existing Bringup,
  rebuild the affected packages, and source the updated overlay. Keep Isaac Kit open and
  use Timeline Stop to reset the old experiment while retaining the Stage unless
  Stage content itself changed. Start a fresh Bringup and verify its ROS endpoints. Then
  Play and prepare the post-Play runtime as required by the existing Script Editor/ROS
  Bridge initialization order; verify fresh simulation readiness/state before publishing
  a new Delivery or running Retry Detection. The new ROS process starts with fresh Delivery accounting, Mission state,
  Retry budget, and pending readiness; none is restored from the prior process. Stop-only
  reset while retaining Bringup is also supported for another experiment in the same ROS
  session. Neither path exits Kit or implies physical Safety Stop/Gripper release.

- For idle-mutable Motion tuning, use `ros2 param set`; accepted updates apply to the next
  Motion action without restarting ROS. Updates are rejected while a goal is pending or active.
- After Isaac Python runtime edits, rerun the existing runtime entrypoint in the Script Editor.
  Reload the edited module before replacing its runtime instance, and use the existing teardown
  path to remove its prior callbacks, ROS node, graph bindings, and robot/camera handles before
  rebuilding. `1_before_play.py` and `2_after_play.py` must be repeatable in the same Kit process;
  a repeated run leaves exactly one active runtime instance. The current gripper bootstrap raises
  an explicit error if the rebuilt runtime still lacks its required `feedback_generation` contract;
  Resolve the runtime incompatibility or use a Kit API/tool reload procedure only when that
  specific API requires a process restart; ordinary module and Extension edits use in-process
  reload.
- For Hub Extension Python edits, disable the extension first (the existing Script Editor
  entrypoint leaves an already-enabled extension instance untouched), then re-enable it, or use the
  documented Kit Extension reload operation for the installed Kit version. Verify that the prior
  callbacks and ROS resources are released before the new instance starts.
- For Stage or ActionGraph edits, keep Kit open, stop Timeline if the edited stage configuration
  requires a stopped simulation, clean up the affected authored/runtime objects, and rerun the
  relevant `1_before_play.py` and/or `2_after_play.py` entrypoint. Play again without restarting
  Kit. Isaac Python edits do not require ROS rebuild or Bringup restart. A fresh Bringup
  session uses Stop→Play to start a clean experiment; ordinary Isaac Python/Extension
  reload does not.
- After an actual Isaac Kit process restart, reconstruct with `1_before_play.py` → Play →
  `2_after_play.py`, then start Bringup. A stage/ActionGraph rebuild performed in the open Kit
  process does not by itself require restarting the ROS profile.

These are the intended procedures for repeated in-process tests. Stop reset and repeatable-runtime contracts require runtime realization and live verification before they can be treated as currently available behavior. This guide change does
not claim that implementation or live acceptance has already passed.

The Stop→Play procedure preserves the Kit process. Simulation reset on Timeline Stop ends the prior experiment and restores the
Isaac-owned robot, simulated Gripper attachment, generated material, validation
fixtures, and temporary simulation fault state. It does not invoke Safety
StopMotion, reset a Safety latch, command physical hardware, or require a
physical release/recovery procedure. Valid configuration, Recipe, Calibration,
Snapshots, logs, and diagnostic history remain available. Pause→Play resumes
the same experiment; Stop→Play prepares a new one. The Hub does not gate Isaac's
Timeline controls. A short ROS admission fence protects against old handoff or
Mission state while owner reset and cancellation are being reconciled; if reset
fails, the owner diagnostic remains visible and ROS admission stays isolated
until a later Timeline lifecycle callback completes it. The simulation reset,
repeatable Publish Trigger, and development-only Retry Detection behavior are
design contracts in
the [Operator UI Hub FDS](../design/arm-cell/components/ui-hub/fds.md),
[Mission Cycle FDS](../design/arm-cell/components/orchestration/fds-mission-cycle.md),
and [ExecuteCycle ICD](../design/arm-cell/interfaces/icd-orchestration-execute-cycle.md).
Retry Detection is a new
Vision-only `/vision/detect_target` request with a 30-second timeout; it does
not supply material or start/resume a Mission. Publish Trigger creates a new
Delivery ID and adds its quantity; Orchestration uses the next sequential
ExecuteCycle to process aggregate outstanding supplied work. Isaac's standard Timeline UI
remains the only Play/Pause/Stop control surface. Use the full reconstruction
procedure in Section 3.9 only when a Kit API or tool specifically requires a
fresh process, or when the Stage/ROS profile cannot be reconstructed through
the in-process lifecycle above; record that concrete constraint. It is not the
normal edit/retest path.

`operational_profile.yaml` remains the startup default and reproducible source. ROS parameters
are per-run overrides. The current Motion parameter classes are:

| Class | Examples | Application |
|---|---|---|
| Static / restart-required | backend mode, arm joint names, topics, robot description, kinematics, transforms, mission target/pose, Safety thresholds | Restart the owning ROS node or rebuild/relaunch the Isaac stage |
| Idle-mutable | approach/retract distances, approach-entry and tracking tolerances, progress stall timeout, holding confirmation timeout | Change with `ros2 param set` while Motion is idle; active or pending goals reject the update |
| Runtime-safe | planning time and PLACE planning budgets, velocity/acceleration scaling | Updates queue for the next Motion action; the current PICK/PLACE/GO_HOME keeps its immutable snapshot |

Example:

```bash
ros2 param set /motion_node planning_time_s 20.0
ros2 param set /motion_node place_total_planning_time_s 75.0
```

Motion parameter updates validate type, finite value and supported range. A rejected parameter
update returns a reason; keep the existing effective value or restart with an edited profile.

### 3.5.1 Run the M09 combined mocked composition

Build/source the ROS workspace as in 3.1, then construct the Isaac cell as in
3.3–3.5. In a ROS Humble terminal, launch the combined M09 profile:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch arm_cell_bringup arm_cell_m09_combined.launch.py
```

In the same Isaac Script Editor session, run `2_after_play.py` after `1_before_play.py` and Play.
This loads the repository Hub extension into the active
Kit process and enables it. Keep the simulator playing and the ROS launch
alive. Register the fixed perception target using the profile's
`runtime_runner_entrypoint` in Isaac and `registration_helper_entrypoint` in
the ROS terminal, as described by the existing PnP validation procedure.

Expected observations:

- ROS advertises `/external/handoff_trigger`,
  `/integration/material_readiness`, `/orchestration/execute_cycle`, and
  `/vision/detect_target` from their owning components;
- the Isaac Hub provides camera/Vision views and request controls; inspect
  owner ROS interfaces and diagnostics for AMR, supply, Safety, Motion, and
  Orchestration outcomes;
- Publish Trigger sends the selected material type, quantity, and optional
  random seed as an External Handoff Trigger; the Hub reports local
  publication while Gateway acceptance, readiness, and mission progress
  remain available through owner interfaces and diagnostic records;
- the fixed target is consumed by the existing Vision seam and the mission
  reports its canonical terminal result.

This profile is an M09 mock/fixed integration composition. It does not qualify
production perception, hardware behavior, or Final Demo acceptance.

### 3.5.2 Run the normal production composition

Complete the build in 3.1, then prepare and play Isaac using 3.3–3.4. Rebuild generated robot
assets with `0_pre_build.py` only when they need updating. For a normal live run:

1. Run `1_before_play.py`.
2. Press Play and wait for a physics frame.
3. Run `2_after_play.py`.
4. From a ROS Humble terminal, start the public production entrypoint:

   ```bash
   ros2 launch arm_cell_bringup arm_cell.launch.py
   ```

   Select optional component diagnostics when needed:

   ```bash
   ros2 launch arm_cell_bringup arm_cell_operational.launch.py \
     log_level:=info diagnostics:=motion,vision
   ```

   The launch reports selected areas and effective ROS-side producers separately, along with the
   artifact root. When `diagnostics` is omitted, `log_level:=debug` selects `all` for ROS
   Bringup-managed diagnostic producers; other log levels select `none`. An explicit
   `diagnostics` value takes precedence, so `log_level:=debug diagnostics:=none` enables DEBUG
   logs without optional capture. `diagnostics` selects optional structured capture and does not
   change logger levels; actual logging level and capture control remain independent. Explicit
   `diagnostics:=motion` enables
   Motion Clock/Watchdog and MotionProfileTrace capture; `diagnostics:=vision` adds bounded RGB-D
   detail to the existing Snapshot. Set `SFTWIN_DIAGNOSTICS=motion` separately in Isaac UI to
   enable Gripper Capture/PhysX observation. Isaac Gripper diagnostics remain process-local; ROS
   Bringup does not pass its log level or diagnostics selection to the Isaac UI.
   `diagnostics:=safety` and `diagnostics:=orchestration` write bounded event JSONL under
   `.local_artifacts/observability/safety/` and `.local_artifacts/observability/orchestration/`.
   `log_level:=debug diagnostics:=none` enables human-readable DEBUG logs while optional capture
   remains off. Pick Execution Record remains required with diagnostics off.

The launch uses `config/operational_profile.yaml`, preserves the established production
composition and includes `isaac_planning.launch.py`. A deployment profile can be selected with
`profile:=/absolute/path/to/profile.yaml`. The Simulation UI Hub is the demo/development trigger
and fault control surface after this launch; Publish Trigger models an external supplier.

Expected ROS observables include production `/vision/detect_target`,
`/external/handoff_trigger`, `/integration/material_readiness`, Safety,
Motion, and Orchestration owner endpoints. Confirm that the Hub subscribes to
the current owner-published producer topics. This runtime setup does not itself
establish production live acceptance. Production mission, Hub nominal PnP, Safety fault, and
E-stop behavior require their own live acceptance checks.

### 3.6 Launch the robot-only planning profile

From a sourced ROS terminal:

```bash
ros2 launch arm_cell_bringup isaac_planning.launch.py use_rviz:=false
```

Use `use_rviz:=true` when measured-robot/planned-path visualization is needed.

Expected observable:

- the ROS planning profile remains running;
- canonical state/TF/planning services become available.

For the validation profile, MoveIt applies the planning/time-parameterization value
`planning_max_acceleration_rad_s2: 1.0`. This is a project-defined tuning seed, not a
measured M0609 hardware or Isaac acceleration ceiling. Isaac execution samples the validated
trajectory with `LinearTrajectorySampler`: it preserves waypoint geometry and
`time_from_start`, but does not claim to reproduce TOTG's continuous velocity or acceleration
interpolation.

### 3.7 Launch the Milestone 1B bounded static-scene profile

Use:

```bash
ros2 launch arm_cell_bringup isaac_static_scene.launch.py use_rviz:=false
```

Expected observable:

- the profile remains running after static-scene load/readback;
- selected static objects are present in the MoveIt PlanningScene.

After changing canonical static geometry or MoveIt selection, regenerate the installed static
scene using the repository package tooling before launching this profile.

### 3.8 Use a non-default fixed ARM Cell placement

Copy the profile YAML, update the `world_to_base` values to the loaded stage, and launch with:

```bash
ros2 launch arm_cell_bringup isaac_planning.launch.py \
  use_rviz:=false \
  profile:=/absolute/path/to/profile.yaml
```

Do not run a competing static TF publisher.

Expected observable:

- `world -> base_link` resolves from the selected profile only.

### 3.9 Reset or replace the Isaac stage

For a new experiment while keeping Kit open:

1. Stop the complete old ROS Bringup with `Ctrl-C`; verify its nodes/endpoints
   have disappeared. This ends its process-local Mission, child Actions,
   readiness, retry budget, and workload accounting.
2. Issue Isaac Timeline Stop. Check owner reset diagnostics: robot and
   simulated attachment/actuator state reset; generated material, validation
   fixtures, pending supply, and injected simulation stimuli are cleared.
   Resolve any failed reset before starting another experiment.
3. Rebuild and source the ROS overlay as in §3.1 when code changes require it.
   Reopen/rebuild Stage or graph using the existing setup roles only when
   those authored assets changed; ordinary reset keeps Kit and Stage open.
4. Start the selected Bringup manually using its entrypoint above and verify
   its ROS endpoints. Then issue Timeline Play, wait for the required physics
   initialization, and run `2_after_play.py` for runtime preparation. Respect
   the existing Script Editor/ROS Bridge initialization dependencies.
5. Verify fresh post-Play clock, measured state, TF, camera and owner readiness
   before publishing a new Delivery. Confirm distinct ROS process/request
   identities and empty new-process Delivery/readiness/workload accounting.

Expected observables and acceptance remain owned by the
[Simulation Session FDS](../design/arm-cell/components/simulation/fds-sim-session.md)
and [demo definition](../product/arm-cell/acceptance.md).
Calibration, Recipe, valid configuration, logs, and Snapshots remain available;
old residual workload does not enter the new experiment's executable quantity.
Timeline Stop does not issue Safety StopMotion, E-stop, Safety reset, or a
physical Gripper release. Bringup replacement transfers no Safety latch/cause
memory; new Safety evaluates fresh inputs under its existing contract.
Pause→Play remains the same experiment. Keeping the same Bringup through
Stop→Play does not establish new-experiment isolation.

If backward time/state is observed while ROS is still running, stop and
restart the **entire** ROS launch using this reset sequence, rather than
restarting only the adapter or rewriting timestamps.

### 3.9.1 Reload runtime Python or the Hub Extension

Keep Kit and the active Stage open. For runtime Python changes, tear down the
existing owned runtime, reload the changed module, then rerun its existing
role entrypoint. For Hub Extension changes, disable/re-enable the Extension.
Neither operation alone rebuilds Stage or clears material. Use §3.9 separately
when an authored Stage/ActionGraph change requires Timeline Stop and a new
experiment.

Verify that the changed module revision is active and that prior callbacks,
ROS nodes, subscriptions, and runtime objects have been released, leaving
exactly one active resource set. Kit and ROS Bringup remain running. Record a
specific documented exception if a Kit-only API cannot support reload; do not
silently claim the reload scenario passed. Extension stimulus cleanup remains
owned by the [UI Hub FDS](../design/arm-cell/components/ui-hub/fds.md).

### 3.10 Run component checks

From `ros2_ws` after build and overlay sourcing:

```bash
colcon test --packages-select arm_cell_description arm_cell_sim_adapter \
  arm_cell_moveit_config arm_cell_bringup
colcon test-result --verbose
```

Expected observable:

- test output clearly reports pass/fail/error/skip state.

### 3.11 Run synthetic integration checks

From the repository root after the preflight has selected the active ROS domain:

```bash
python3 -m pytest \
  tests/integration/arm_cell \
  --state-source=synthetic \
  -q
```

Expected observable:

- real ROS adapter/TF/MoveIt processes start under the test harness;
- failures remain visible rather than being converted into a nominal pass.

### 3.12 Run live Isaac integration checks

Stop a separately launched ROS profile, keep Isaac playing at a stationary collision-free arm
pose, source the ROS overlay, and use the same ROS domain as Isaac:

```bash
python3 -m pytest tests/integration/arm_cell --state-source=isaac -q
```

Expected observable:

- the test harness launches the ROS profile;
- the result explicitly reports pass/fail/skip state.

Do not weaken collision checking to force a pass when the loaded pose/goal is invalid.

### 3.13 Run the deterministic PnP validation preflight

Run the preflight from the same repository checkout used to build and source the ROS overlay.
It accepts any branch or worktree; the checkout must be clean, and its ROS overlay and Isaac
process must resolve to that same checkout.

```bash
python3 ros2_ws/src/arm_cell/arm_cell_bringup/scripts/pnp_validation_preflight.py \
  --repo-root "$PWD"
```

It fails before live testing when the working tree is not clean, the required files
or executable bits are missing, the validation profile paths do not resolve, the ROS overlay
prefix is not the current checkout, or `ros2 pkg executables` cannot discover the registration
helper. It prints the effective `ROS_DOMAIN_ID`; the canonical default is `0` and an explicitly
configured domain is preserved.

After the ROS validation composition is running, check live discovery before opening the Isaac
validation runner:

```bash
python3 ros2_ws/src/arm_cell/arm_cell_bringup/scripts/pnp_validation_preflight.py \
  --repo-root "$PWD" \
  --live
```

The live preflight is fail-closed for Fixed Vision validation ownership. It requires exactly
one `/fixed_detect_target_node`; zero, duplicate, or ambiguous validation instances stop the
flow before registration. Before starting a new deterministic diagnostic composition, stop the complete
previous validation launch with `Ctrl-C`, wait for its nodes to disappear, and start one fresh
launch. Do not manually start another Fixed Vision node and do not kill unrelated processes.
Run the `--live` preflight again and proceed to registration only after it reports one
authoritative instance. Registration and immediate DetectTarget readback must then be performed
against that same composition; a readback failure is a validation blocker, not a reason to retry
registration through another node.

`1_before_play.py` prepares `SFTWIN_PROJECT_ROOT` and the effective `ROS_DOMAIN_ID`. The default
domain is `0`; an
explicitly configured valid domain is preserved. After both preflight phases pass, prepare with `0_pre_build.py`, construct the Isaac stage with `1_before_play.py`,
press Play, and run `2_after_play.py`.
Then run `pnp_validation/entrypoints/start_validation.py`. It switches the gripper runtime to
the diagnostic profile, creates the
deterministic cube fixture, and writes the registration handoff:

```text
Isaac: 0_pre_build.py -> 1_before_play.py -> Play -> 2_after_play.py
       -> pnp_validation/entrypoints/start_validation.py
ROS:   arm_cell_m09_combined.launch.py -> register_pnp_fixture.py
       -> /vision/detect_target verification
```

This route uses cube + Fixed Vision to isolate Motion/PnP behavior. It is not a production mission
entrypoint and cannot replace production acceptance.

If you switch checkouts, rebuild and source that checkout's ROS overlay, set
`SFTWIN_PROJECT_ROOT` to its root in the Isaac process, and rerun both preflight phases before
continuing. The role entrypoints are Script Editor Python files selected for the current Isaac/Kit
execution model; they do not require a shell to attach to the running Kit process.

### 3.14 Perform read-only runtime diagnostics

With the normal profile running:

```bash
ros2 topic info /isaac/joint_states --verbose
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base_link sf_grasp_tcp
```

Expected observable:

- the expected channels/TF edges are inspectable;
- canonical joint state includes the independent ARM Cell joints defined by the current design.

To inspect the actual Isaac stage base transform, run this read-only snippet in Isaac Script
Editor:

```python
import omni.usd
from pxr import UsdGeom

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/SF_Twin_Cell/m0609/base_link")
assert prim.IsValid()
print(UsdGeom.GetStageMetersPerUnit(stage))
print(UsdGeom.XformCache().GetLocalToWorldTransform(prim))
```

### 3.15 Inspect the composed camera snapshot

`1_before_play.py` exports this snapshot automatically after scene composition. For a manual
read-only refresh or inspection, run:

```text
infra/isaac_sim/scripts/m0609_cell/camera_tooling/entrypoints/inspect_camera.py
```

in Isaac Script Editor.

The stable local default output is:

```text
.local_artifacts/m0609_cell/camera_inspection/camera_snapshot.json
```

Use `SFTWIN_CAMERA_SNAPSHOT_PATH` or the script's explicit output argument when a distinct
snapshot location is required.

Expected observable:

- the snapshot path is printed;
- non-rigid composed camera transforms fail snapshot export instead of being silently accepted.

### 3.16 Inspect camera TF and RGB-D channels

From a sourced ROS terminal:

```bash
ros2 topic info /camera/color/image_raw --verbose
ros2 topic info /camera/aligned_depth_to_color/image_raw --verbose
ros2 topic info /camera/color/camera_info --verbose
ros2 topic echo /camera/color/camera_info --once
ros2 run tf2_ros tf2_echo world camera_link
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

Expected observable:

- the canonical operational launch owns camera TF composition;
- RGB, depth, CameraInfo, and camera TF are inspectable.

### 3.17 Prepare and invoke RGB-D live-acceptance tooling

Load both ROS environments:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

Then verify the live-acceptance entry point:

```bash
PYTHONPATH=infra/isaac_sim/scripts/m0609_cell/tests/support:$PYTHONPATH \
python3 infra/isaac_sim/scripts/m0609_cell/tests/acceptance/live_geometry_acceptance.py --help
```

Expected observable:

- the acceptance CLI starts successfully and displays supported arguments.

Use the accepted procedure/arguments for the specific RGB-D acceptance run. Do not treat
`--help` as acceptance evidence.

### 3.18 Author temporary renderer-depth calibration targets when required

Use:

```text
infra/isaac_sim/scripts/m0609_cell/camera_tooling/entrypoints/renderer_depth_calibration.py
```

in Isaac Script Editor with the current-session camera snapshot.

Use the tool's operator-editable `SNAPSHOT_PATH`, `--snapshot`, or
`SFTWIN_DEPTH_CALIBRATION_SNAPSHOT` as applicable.

Expected observable:

- the tool reports derived target transforms;
- only the temporary acceptance target root is replaced;
- the stage is not saved by the calibration authoring step.

### 3.19 Capture a future RGB-D regression fixture

Capture the required source channels together for the accepted RGB-D fixture procedure,
including RGB, depth, CameraInfo, `/clock`, and required static TF.

Store large source captures outside version control or under `.local_artifacts/`.
When a derived small fixture is promoted, preserve the source capture URI and SHA-256 in its
manifest.

Do not overwrite historical captures whose recorded paths are provenance.

## 4. Expected Observables

| Operation | Expected Observable | Abnormal Signal |
|---|---|---|
| ROS build | required packages build and overlay resolves | package/build failure |
| Isaac construction | stage and ROS graph construct | missing prim/graph/script failure |
| robot-only launch | canonical state/TF/planning available | duplicate authority, missing state/clock |
| static-scene launch | selected scene objects loaded/readable | loader exit/readback failure |
| reset/relaunch | current epoch state resumes | latched adapter or stale cached state |
| synthetic/live tests | explicit test result | hidden/ignored failure |
| camera snapshot | rigid snapshot exported | non-rigid rejection or missing source prim |
| camera overlay | camera TF/topics inspectable | missing TF/topic/publisher |
| RGB-D acceptance tooling | CLI/run emits explicit result/evidence | missing ROS environment or contract failure |

## 5. Failure and Recovery

### ROS Python packages unavailable

**Symptom**

```text
ModuleNotFoundError: No module named 'rclpy'
```

**Recovery**

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
```

Then rerun the command from the intended repository location.

### Backward simulation time observed while ROS is running

**Symptom**

- canonical state publication stops;
- adapter reports/latches an error.

**Recovery**

- stop the entire ROS profile;
- restart/rebuild the simulator session as required;
- press Play;
- relaunch the complete ROS profile.

### Gripper subscriber exists but gripper does not move

**Diagnostic**

Confirm `2_after_play.py` was executed **after Play** in the current simulator session.

**Recovery**

Run the runtime setup after Play and keep its runtime callback objects alive.

### Static-scene profile exits during launch

**Diagnostic**

Inspect the static-scene loader error and generated artifact validity.

**Recovery**

Correct the canonical manifest/selection or regenerate the artifact as appropriate. Do not
bypass readback validation merely to keep the profile running.

### Live planning check fails because the current pose/goal is invalid

**Recovery**

Inspect the actual state and collision/contact result. Move to a valid stationary
pose or correct the environment; do not disable collision checking to obtain a pass.

### Camera snapshot export rejects the composed transform

**Recovery**

Inspect the composed camera transform/asset construction. Do not normalize meaningful
scale/shear/reflection into an apparently valid rigid pose.

## 6. Cleanup / Reset

- stop ROS launch processes with `Ctrl-C`;
- remove only temporary acceptance/calibration targets created by the procedure;
- keep `.local_artifacts/` as local diagnostic output only;
- do not promote temporary snapshots or large raw captures into canonical design/configuration
  by copying them into the repository without an explicit artifact decision;
- when changing simulator session/stage, follow the complete reset/relaunch procedure above.

## 7. References

- `docs/record/arm-cell/architecture-decision/model-state-and-time.md`
- `docs/record/arm-cell/architecture-decision/static-planning-scene.md`
- `docs/record/arm-cell/architecture-decision/rgbd-sensor-contract.md`
- `docs/design/arm-cell/interfaces/icd-sim-state-time-tf.md`
- `docs/design/arm-cell/interfaces/icd-sim-rgbd.md`
- `docs/design/arm-cell/components/motion/cds.md`
- `docs/reports/acceptance/arm-cell-milestone-1a.md`
- `docs/reports/acceptance/arm-cell-milestone-1b-static-scene.md`

### PLACE WAYPOINT 접근 경로 생성과 적용

운영 profile과 Planning Runtime을 준비한 뒤 다음 명령으로 고정 HOME→PLACE
접근 경로를 명시적으로 생성한다.

```bash
ros2 service call /motion/generate_place_waypoint_trajectory std_srvs/srv/Trigger '{}'
```

출력은 운영 profile의 `motion.place_waypoint_trajectory_file`에 저장된다.
기존 파일이 있으면 생성은 실패하며 덮어쓰지 않는다. 새 후보를 만들 때는
별도 profile 복사본의 해당 값을 다른 파일 경로로 지정하여 Runtime을 시작한다.
생성 결과와 진단을 확인한 운영자는 기존 운영 파일을 별도로 보관한 뒤 새 파일을
운영 경로로 명시적으로 복사하거나, 운영 profile이 새 파일을 참조하도록 변경한다.
적용 후 Motion Runtime을 재시작한다. Runtime은 시작 시 파일을 한 번 로드하므로
실행 중 파일 교체는 현재 캐시에 영향을 주지 않는다. 실제 PnP 실행은 저장된
경로를 재사용하며 파일 누락 시 자동 생성하지 않는다.
