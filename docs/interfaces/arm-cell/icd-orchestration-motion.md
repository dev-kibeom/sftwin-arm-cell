# [SF-Twin] ARM Cell Orchestration ↔ Motion Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-ORCHESTRATION-MOTION_v1.1.0`
- **Document Type:** `ICD`
- **Scope:** `Orchestration ↔ Motion Core`
- **Version:** `1.1.0`
- **Status:** `Draft`
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
| `target_pose` / `has_target_pose` | `geometry_msgs/PoseStamped` / `bool` | object-centric reference where the task requires one |
| `estimated_object_height_m` / `has_estimated_object_height` | `float32` / `bool` | Vision-supplied object estimate where required |
| `grasp_width_mm` / `has_grasp_width` | `float32` / `bool` | logical finger opening width where required |

The exact `MotionTaskType.value` constants are `TASK_TYPE_UNSPECIFIED`, `TASK_TYPE_PICK`, `TASK_TYPE_PLACE`, `TASK_TYPE_GO_HOME`, and `TASK_TYPE_RETRACT`.

For `PICK` and `PLACE`, a supplied target pose SHALL be an object-centric reference in `base_link`; it is not a final TCP pose. Motion owns TCP/tool/gripper/approach/retract conversion. `grasp_width_mm` is logical width from `0.0` mm fully closed through `85.0` mm fully open. Backend actuator topology remains below Motion.

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
