# [SF-Twin] ARM Cell Isaac Sim Development Procedure

- **Document ID:** `GUIDE-ARM-CELL-ISAAC-SIM`
- **Document Type:** `Guide`
- **Scope:** `ARM Cell / Isaac Sim development planning and RGB-D verification`
- **Version:** `1.0.0`
- **Status:** `Active`
- **Owner:** `ARM Cell Integration`
- **Applicable Environment:** `Ubuntu / ROS 2 Humble / Isaac Sim 5.1`
- **Related Design:** `ADR 0001`, `ADR 0002`, `ADR 0003`,
  `docs/interfaces/arm-cell/icd-sim-state-time-tf.md`,
  `docs/interfaces/arm-cell/icd-sim-rgbd.md`

## 1. Purpose

This Guide describes the repeatable operator procedure for building, starting, resetting, and
verifying the supported ARM Cell Isaac Sim development profiles.

It does not define model/state/TF/time/static-scene/RGB-D contract semantics. Refer to the
related ADR/design documents for those contracts. Historical PASS results belong in Acceptance
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

- After ROS/C++ code changes, rebuild the ROS workspace and restart the production ROS launch.
- For idle-mutable Motion tuning, use `ros2 param set`; accepted updates apply to the next
  Motion action without restarting ROS. Updates are rejected while a goal is pending or active.
- Rerun `2_after_play.py` to rebuild the gripper runtime after runtime-module edits; restart Isaac
  Kit to load edits to an already-enabled extension.
- After scene or ActionGraph changes, stop simulation and run `1_before_play.py` again, then press
  Play and run `2_after_play.py`.
- A normal Stop followed by Play keeps the current Isaac Kit runtime alive. After an Isaac Kit
  restart or stage/ActionGraph rebuild, use a fresh sequence: `1_before_play.py` → Play →
  `2_after_play.py`, then start or restart `ros2 launch arm_cell_bringup arm_cell.launch.py` as
  needed.

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

- ROS advertises `/integration/request_material`,
  `/integration/material_readiness`, `/orchestration/execute_cycle`, and
  `/vision/detect_target` from their owning components;
- the Isaac Hub shows current owner-published AMR, handoff, readiness, Safety,
  Motion, and Orchestration feedback;
- requesting material in the Hub reports only the Integration response first;
  handoff/readiness and mission progress appear from owner feeds;
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

The launch uses `config/operational_profile.yaml`, preserves the established production
composition and includes `isaac_planning.launch.py`. A deployment profile can be selected with
`profile:=/absolute/path/to/profile.yaml`. The Simulation UI Hub is the production mission and
fault control surface after this launch; it routes requests to canonical owners.

Expected ROS observables include production `/vision/detect_target`,
`/integration/request_material`, `/integration/material_readiness`, Safety,
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

Before Stop/reset/stage replacement:

1. stop the complete ROS launch with `Ctrl-C`;
2. reset/reopen/rebuild the Isaac graph as required;
3. press Play;
4. rerun `2_after_play.py` after Play when gripper runtime behavior is needed;
5. relaunch the complete ROS profile.

If reset occurs while ROS is still running and backward time/state is observed, restart the
**entire** ROS launch rather than only the adapter.

Expected observable after restart:

- canonical state publication resumes from the new simulator session;
- ROS consumers use the current simulation epoch.

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

- `docs/adr/0001-arm-cell-model-state-and-time.md`
- `docs/adr/0002-arm-cell-static-planning-scene.md`
- `docs/adr/0003-arm-cell-rgbd-sensor-contract.md`
- `docs/interfaces/arm-cell/icd-sim-state-time-tf.md`
- `docs/interfaces/arm-cell/icd-sim-rgbd.md`
- `docs/components/arm-cell/motion/cds-static-scene.md`
- `docs/reports/acceptance/arm-cell-milestone-1a.md`
- `docs/reports/acceptance/arm-cell-milestone-1b-static-scene.md`
