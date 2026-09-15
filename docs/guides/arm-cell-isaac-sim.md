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
- Isaac Script Editor access for the numbered ARM Cell setup scripts.
- no competing `/clock`, canonical `/joint_states`, or `world -> base_link` publisher in the
  selected ROS domain.

The current workflow spans both host-terminal commands and Isaac Script Editor actions.
Where repository-owned executable scripts already exist, use them. Multi-command host-terminal
bootstrap remains a candidate for future wrapper-script extraction; do not invent a wrapper
outside an approved implementation task.

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

### 3.3 Configure the Isaac Script Editor session

Before running numbered scripts, edit the operator value in:

```text
infra/isaac_sim/scripts/m0609_cell/0_setup_sftwin_env.py
```

Set `PROJECT_ROOT` to the absolute repository path and run the script once in Isaac Script
Editor.

If `SFTWIN_PROJECT_ROOT` is already configured for the current Isaac process, leave
`PROJECT_ROOT = None` and run the setup script to validate it.

Expected observable:

- the repository root is accepted;
- the M0609 module root is available to the Script Editor process.

### 3.4 Construct the Isaac ARM Cell baseline

Run these scripts in Isaac Script Editor in order:

```text
01_build_robot_usd.py
02_setup_scene.py
03_build_actiongraph.py
```

Then start/Play the simulation and allow at least one physics frame.

Expected observable:

- the ARM Cell stage is constructed;
- the ROS graph publishes the expected Isaac-side state/clock channels for the selected profile.

### 3.5 Initialize the runtime gripper when required

After the simulation is playing, run:

```text
04_set_gripper.py
```

Keep the created runtime objects and retained physics callback alive for the simulation session.

Expected observable:

- runtime gripper setup completes without initialization failure.

The existence of a `/gripper/command` subscriber alone is not sufficient proof that runtime
gripper actuation is initialized.

Until the Motion task backend owns gripper lifecycle and grasp-target registration, this is a
**provisional simulator-backend bootstrap**. The retained Script Editor session and
`/gripper/command` are not canonical Motion or mission interfaces; this Guide owns the operator
procedure only.

### 3.6 Launch the robot-only planning profile

From a sourced ROS terminal:

```bash
ros2 launch arm_cell_bringup isaac_planning.launch.py use_rviz:=false
```

Use `use_rviz:=true` when measured-robot/planned-path visualization is needed.

Expected observable:

- the ROS planning profile remains running;
- canonical state/TF/planning services become available.

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
4. rerun `04_set_gripper.py` after Play when gripper runtime behavior is needed;
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

From repository root using an unused ROS domain:

```bash
ROS_DOMAIN_ID=84 python3 -m pytest \
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

### 3.13 Perform read-only runtime diagnostics

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

### 3.14 Export the Milestone 1C camera snapshot

After loading the scene and starting Play, run:

```text
infra/isaac_sim/scripts/m0609_cell/inspect_camera.py
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

### 3.15 Launch the camera TF overlay and inspect RGB-D channels

From a sourced ROS terminal:

```bash
ros2 launch arm_cell_bringup isaac_camera.launch.py \
  snapshot:=.local_artifacts/m0609_cell/camera_inspection/camera_snapshot.json

ros2 topic info /camera/color/image_raw --verbose
ros2 topic info /camera/aligned_depth_to_color/image_raw --verbose
ros2 topic info /camera/color/camera_info --verbose
ros2 topic echo /camera/color/camera_info --once
ros2 run tf2_ros tf2_echo world camera_link
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

Expected observable:

- the camera overlay remains running;
- RGB, depth, CameraInfo, and camera TF are inspectable.

### 3.16 Prepare and invoke RGB-D live-acceptance tooling

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

### 3.17 Author temporary renderer-depth calibration targets when required

Use:

```text
infra/isaac_sim/scripts/m0609_cell/renderer_depth_calibration.py
```

in Isaac Script Editor with the current-session camera snapshot.

Use the tool's operator-editable `SNAPSHOT_PATH`, `--snapshot`, or
`SFTWIN_DEPTH_CALIBRATION_SNAPSHOT` as applicable.

Expected observable:

- the tool reports derived target transforms;
- only the temporary acceptance target root is replaced;
- the stage is not saved by the calibration authoring step.

### 3.18 Capture a future RGB-D regression fixture

Capture the required source channels together for the accepted RGB-D fixture procedure,
including RGB, depth, CameraInfo, `/clock`, and required static TF.

Store large source captures outside version control or under the approved artifact location.
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

Confirm `04_set_gripper.py` was executed **after Play** in the current simulator session.

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

Inspect the actual state and collision/contact result. Move to an approved valid stationary
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
