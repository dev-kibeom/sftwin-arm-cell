# [SF-Twin] ARM Cell Orchestration ↔ Motion Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-ORCHESTRATION-MOTION_v1.3.0`
- **Document Type:** `ICD`
- **Scope:** `Orchestration ↔ Motion Core`
- **Version:** `1.3.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoint and Identity

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/motion/execute_task` | `arm_cell_interfaces/action/ExecuteTask` | Motion | Orchestration |

The ROS Action Goal UUID is the canonical execution identity. It is carried by `MotionStatus.active_execution_id` while the task is active; no caller-assigned task identifier is required.

## 2. ExecuteTask Goal

The executable realization is [`ExecuteTask.action`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/action/ExecuteTask.action). Its goal fields are:

| Field | Type | Meaning |
|---|---|---|
| `task_type` | `MotionTaskType` | `PICK`, `PLACE`, `GO_HOME`, or `RETRACT` |
| `target_pose` / `has_target_pose` | `geometry_msgs/PoseStamped` / `bool` | PICK Observation Pose or PLACE Desired Object Pose, as selected by `task_type`; PLACE semantics are owned by the Mission Cycle FDS |
| `estimated_object_height_m` / `has_estimated_object_height` | `float32` / `bool` | Vision-supplied object estimate where required |
| `grasp_width_mm` / `has_grasp_width` | `float32` / `bool` | logical finger opening width where required |
| `place_position_tolerance_m` | `float64` | Recipe-owned PLACE process position tolerance for Desired Object Pose |
| `place_orientation_tolerance_rad` | `float64` | Recipe-owned PLACE process orientation tolerance for Desired Object Pose |
| `place_orientation_constraint` | `uint8` | ExecuteTask constants `PLACE_ORIENTATION_FIXED=0`, `PLACE_ORIENTATION_BOUNDED=1`, `PLACE_ORIENTATION_FREE=2`; recipe-owned object orientation process constraint mode |
| `place_approach_direction_object` | `geometry_msgs/Vector3` | Unit approach/retract direction expressed in the Desired Object Pose frame |
| `place_approach_distance_m` | `float64` | Recipe-owned PLACE approach distance from the Desired Object Pose |
| `place_retract_distance_m` | `float64` | Recipe-owned PLACE retract distance from the Desired Object Pose |
| `has_place_tool_orientation_preference` / `place_tool_orientation_preference` | `bool` / `geometry_msgs/Quaternion` | Optional `preferred` TCP ranking input in `base_link`, applicable only for fixed/bounded object orientation constraints |

The exact `MotionTaskType.value` constants are `TASK_TYPE_UNSPECIFIED`, `TASK_TYPE_PICK`, `TASK_TYPE_PLACE`, `TASK_TYPE_GO_HOME`, and `TASK_TYPE_RETRACT`.

For `PICK`, `target_pose` is the detector-profile-defined Observation Pose resolved by Orchestration. For `PLACE`, it carries the selected recipe's Desired Object Pose, process tolerances, and object-frame approach/retract geometry defined by the [Mission Cycle FDS](../../components/arm-cell/orchestration/fds-mission-cycle.md); it is never a final TCP pose. The approach direction SHALL be finite and unit length, and both distances SHALL be finite and positive. Motion owns conversion of these logical object-centric values through the configured object-to-grasp-TCP relation to TCP/tool/approach/retract poses and selection only within the process tolerances. PICK geometry remains independent and is not carried in these PLACE fields. `grasp_width_mm` is logical width from `0.0` mm fully closed through `85.0` mm fully open. Backend actuator topology remains below Motion.

The PLACE Desired Object Pose and tolerances are expressed in `base_link` in this W06 interface. The optional TCP orientation preference is independently expressed in `base_link`; it ranks only candidates that already satisfy the object process tolerances and pass collision/IK feasibility. It does not alter or replace the Desired Object Pose. A fixture-relative recipe frame may be introduced later without changing the Desired Object Pose meaning.

At the JSON recipe boundary, `orientation_constraint` accepts `fixed`,
`bounded`, or `free`; the parser converts this string to its typed domain value
and Orchestration maps it to the ExecuteTask constants above. `fixed` requires
the nominal object orientation; `bounded` admits candidates within
`place_orientation_tolerance_rad`; `free` declares no object-orientation
process constraint. For schema compatibility, `orientation_tolerance_rad`
remains required but is ignored when mode is `free`. For free, Motion searches
a small deterministic representative set including side-on poses, then selects
feasible candidates by joint travel and trajectory duration. This search is
bounded and does not claim exhaustive/global optimization. Tool orientation
preference is ignored for free. Position and position-tolerance meanings are
unchanged.

Core invariant: **Recipe expresses process constraints. Free orientation
means there is no process orientation constraint; Motion selects a feasible
pose with low joint travel and trajectory duration within a bounded,
deterministic candidate search.**

### 2.1 PICK input resolution ownership

For a PICK goal, Orchestration SHALL resolve Vision and recipe inputs before
submission:

- `target_pose.position.x/y/z` SHALL be the valid Vision object position;
- yaw SHALL use Vision yaw exactly when `has_target_yaw=true`, otherwise it
  SHALL use the target-specific recipe yaw;
- `grasp_width_mm` SHALL come from the target recipe;
- `estimated_object_height_m` SHALL be treated as non-authoritative and unused
  for PICK geometry; it SHALL NOT modify, repair, offset, replace, or otherwise
  participate in object-reference to TCP pose generation.

Vision owns validity filtering for its published perception result. Downstream
components MUST NOT reinterpret raw Vision confidence thresholds. Orchestration
MUST NOT perform TCP/tool geometry calculations. Motion owns the subsequent
object-reference to executable TCP/approach/retract conversion.

The wire fields remain those in `ExecuteTask.action`; this section defines
resolution ownership and semantics. The `DetectTarget` response now includes
the approved explicit `has_target_yaw` field.
If neither a valid Vision yaw nor the required recipe yaw is available,
Orchestration SHALL fail closed as `MISSION_EXIT_CONFIGURATION_ERROR` (or the
equivalent input/configuration failure) and SHALL NOT invent an orientation.

### 2.2 PICK retry boundary

This contract does not own retry budget or policy. The approved
[Mission Cycle FDS](../../components/arm-cell/orchestration/fds-mission-cycle.md)
defines a bounded target-episode retry policy. Every retry MUST obtain fresh
perception and a newly resolved PICK input, and MUST NOT replay the same stale
pose or goal. This is distinct from Safety recovery. Safety interruption and
recovery MUST retain the invariant that the interrupted task is never
automatically retried or resumed.

### VR-ICD-ORCH-MOT-04 — Yaw resolution
Orchestration SHALL give precedence to an explicitly present valid Vision yaw,
including zero, and SHALL otherwise use the target-specific recipe yaw. Missing
values from both sources SHALL fail closed rather than create an implicit yaw.

`GO_HOME` and `RETRACT` do not require an object target in the current contract. `GO_HOME` has no implicit gripper side effect. `RETRACT` remains a capability-gated constrained recovery task.

## 3. Feedback, Completion, and Cancellation

`MotionTaskPhase.value` reports the public phase: `TASK_PHASE_VALIDATING`, `TASK_PHASE_PLANNING`, `TASK_PHASE_APPROACHING`, `TASK_PHASE_EXECUTING`, `TASK_PHASE_GRIPPER`, `TASK_PHASE_ATTACHING`, `TASK_PHASE_DETACHING`, `TASK_PHASE_RETRACTING`, or `TASK_PHASE_COMPLETING`; `TASK_PHASE_UNKNOWN` is the fail-safe unset value. It describes task progress without exposing a backend or planner implementation.

The result contains `MotionTaskResultCode result_code` and non-normative `diagnostic_detail`. Exact result constants are:

- `TASK_RESULT_SUCCESS`;
- `TASK_RESULT_INVALID_GOAL`;
- `TASK_RESULT_IK_UNREACHABLE`;
- `TASK_RESULT_PATH_OBSTRUCTED`;
- `TASK_RESULT_GRASP_FAILURE`;
- `TASK_RESULT_CANCELED`;
- `TASK_RESULT_SAFETY_PREEMPTED`;
- `TASK_RESULT_BACKEND_UNAVAILABLE`;
- `TASK_RESULT_PERMISSION_DENIED`;
- `TASK_RESULT_BACKEND_ERROR`.

Client cancel is coordination intent and maps to `TASK_RESULT_CANCELED` only when Safety did not preempt the task. Safety preemption is independently initiated through direct StopMotion and maps to `TASK_RESULT_SAFETY_PREEMPTED`. A rejected goal due to current capability maps to `TASK_RESULT_PERMISSION_DENIED`.

## 4. Verification Requirements

### VR-ICD-ORCH-MOT-01 — Logical task contract
Orchestration SHALL use task semantics rather than backend-specific robot commands.

### VR-ICD-ORCH-MOT-02 — Logical gripper width
Upper-layer gripper command SHALL remain logical width, not simulator articulation command.

### VR-ICD-ORCH-MOT-03 — Error distinction
Safety preemption, cancel, planning failure, and backend failure SHALL remain distinguishable.
