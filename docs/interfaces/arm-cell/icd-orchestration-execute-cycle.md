# [SF-Twin] ARM Cell ExecuteCycle Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-ORCHESTRATION-EXECUTE-CYCLE_v1.2.0`
- **Document Type:** `ICD`
- **Scope:** `Orchestration Batch Admission ↔ Batch Mission Execution`
- **Version:** `1.2.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoint and Identity

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/orchestration/execute_cycle` | `arm_cell_interfaces/action/ExecuteCycle` | Orchestration batch admission controller | Orchestration batch mission executor |

The goal contains the configured `target_id` logical work/profile key and the Integration `delivery_id` that was admitted. The `delivery_id` field correlates internally admitted batches; `target_id` is not a perception algorithm parameter. The ROS Action Goal UUID is the execution identity. For Final Demo this Action is invoked internally only after Orchestration has auto-admitted a ready delivery; it is not exposed as an operator start/admission action.

## 2. Feedback

The executable realization is [`ExecuteCycle.action`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/action/ExecuteCycle.action). `MissionPhase.value` uses `MISSION_PHASE_ENSURING_HOME`, `MISSION_PHASE_DETECTING_TARGET`, `MISSION_PHASE_PICKING`, `MISSION_PHASE_PLACING`, `MISSION_PHASE_RETURNING_HOME`, `MISSION_PHASE_WAITING_RECOVERY`, `MISSION_PHASE_RETRACTING`, and `MISSION_PHASE_FINISHED`; `MISSION_PHASE_UNKNOWN` is the unset value.

These phases communicate public mission progress only. They do not expose BehaviorTree implementation details.

For Final Demo observability, the Action feedback contract includes the
following additive fields so the Hub can observe the real batch and current
iteration without becoming an authority. These fields are realized in
[`ExecuteCycle.action`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/action/ExecuteCycle.action):

| Field | Type | Meaning |
|---|---|---|
| `delivery_id` | `unique_identifier_msgs/UUID` | Integration delivery UUID associated with this admitted batch |
| `batch_processed_count` | `uint64` | completed PLACE/RELEASED iterations in this batch |
| `retry_count` | `uint8` | retries already used in the target episode, 0 through 3 |
| `selected_target_pose` / `has_selected_target_pose` | `geometry_msgs/PoseStamped` / `bool` | object-centric pose actually used by the active iteration |
| `target_valid` | `bool` | selected target validity at the latest Vision observation |
| `target_observation_stamp` | `builtin_interfaces/Time` | stamp of the selected Vision observation |
| `interruption_count` | `uint32` | Safety/task interruptions observed in this batch |
| `first_interruption_reason` | `MissionExitReason` | first canonical interruption classification, retained for batch lifetime |
| `first_interruption_causes` | `SafetyCauseSet` | Safety cause set captured at the first Safety interruption |
| `first_interruption_stop_mode` | `StopMode` | selected severity captured at the first Safety interruption |

The Action goal UUID is the execution identity. The Integration delivery UUID
is the batch correlation identity. In Final Demo, Orchestration starts this
execution only after its independent `MATERIAL_READY`/readiness/Safety admission
decision. During eligible recovery the
Action remains active at `MISSION_PHASE_WAITING_RECOVERY`; its interrupted
task is terminal and never resumes. The first interruption fields are
immutable for the batch lifetime and the count may increase for later
interruptions.

## 3. Result and Cancellation

For Final Demo, a successful Integration `MATERIAL_READY` fact causes
Orchestration to evaluate current general readiness and Safety capability and
automatically admit one batch when permitted. No external ExecuteCycle goal,
Hub request, or operator batch-start action is required for admission. Within
a running Orchestration instance, one delivery identity may be admitted at
most once. The ExecuteCycle Action is an Orchestration-internal execution
boundary for Final Demo. Its feedback supplies the existing mission
observation surface to the Hub. The Hub cannot invoke, cancel, or
authoritatively modify the batch. Other external mission-caller use is outside
this Final Demo contract.

The result contains `MissionExitReason exit_reason` and non-normative `diagnostic_detail`. Exact exit constants are:

| Constant | Meaning |
|---|---|
| `MISSION_EXIT_NONE` | no terminal reason has been established |
| `MISSION_EXIT_DEPLETED` | valid target depletion confirmed and the single terminal GO_HOME succeeded; normal batch completion |
| `MISSION_EXIT_VISION_ERROR` | a non-depletion Vision outcome interrupted the mission |
| `MISSION_EXIT_MOTION_ERROR` | Motion task failure interrupted the mission |
| `MISSION_EXIT_SAFETY_PREEMPTED` | Safety preempted the mission |
| `MISSION_EXIT_CANCELED` | caller cancellation interrupted the mission |
| `MISSION_EXIT_CONFIGURATION_ERROR` | Orchestration could not load or validate the requested mission recipe/configuration |

At every local non-Safety terminal-decision boundary, Orchestration SHALL
re-observe current Safety state before committing caller cancellation, a Vision
result, or a Motion result. If current Safety state no longer permits normal
motion, the result SHALL be `MISSION_EXIT_SAFETY_PREEMPTED`; otherwise the
applicable non-Safety result may be committed. This is a deterministic local
revalidation rule, not a claim of atomic observation across distributed
components. Safety preemption remains distinguishable and does not depend on
action cancellation for robot stopping.

`MISSION_EXIT_DEPLETED` is the only normal successful batch completion. For
the Final Demo, it means valid-target depletion was confirmed and the single
terminal GO_HOME succeeded. In batch use, retryable iteration failures and a
recoverable Safety interruption do not terminate the batch Action. An
interrupted task/iteration is terminal and never resumes; its first
interruption provenance remains in batch feedback through eventual batch
completion or abort. If recovery is denied or the operator terminates the
batch, its terminal result is `MISSION_EXIT_SAFETY_PREEMPTED`. Recovery
operation failure has no separate public `MissionExitReason` and remains in
the approved recovery/evidence channel. `diagnostic_detail` remains
non-normative and is not a hidden machine-readable decision contract.

`MISSION_EXIT_CONFIGURATION_ERROR` is an Orchestration-owned result. It applies
when no recipe exists for the requested `target_id`, or a required grasp width
or PLACE pose/frame is missing or invalid. It is not a Vision or Motion
failure.

The ROS Action terminal state is transport/lifecycle semantics separate from
the canonical `MissionExitReason` payload: `MISSION_EXIT_DEPLETED` SHALL map to
SUCCEEDED, caller cancellation (`MISSION_EXIT_CANCELED`) SHALL map to CANCELED,
and Vision, Motion, Safety, or configuration interruption SHALL map to ABORTED. The result
payload SHALL preserve the corresponding `MissionExitReason` in every case.

## 4. Verification Requirements

### VR-ICD-ORCH-CYCLE-01 — Depletion-only success
Only `MISSION_EXIT_DEPLETED` SHALL represent normal mission completion.

### VR-ICD-ORCH-CYCLE-02 — Result and provenance preservation
Recovery completion SHALL not erase or rewrite the batch's first interruption
provenance. A batch terminal result SHALL describe the final batch outcome;
the interrupted task/iteration outcome remains independently observable.

### VR-ICD-ORCH-CYCLE-03 — Preemption distinction
Safety preemption and caller cancellation SHALL remain distinguishable.

### VR-ICD-ORCH-CYCLE-04 — Terminal Safety revalidation
Given a caller, Vision, or Motion non-Safety terminal outcome, when current
Safety state preempts normal motion at Orchestration's terminal-decision
boundary, the public result SHALL be `MISSION_EXIT_SAFETY_PREEMPTED`.

### VR-ICD-ORCH-CYCLE-05 — Configuration result mapping
Given an Orchestration-owned invalid or missing mission recipe, ExecuteCycle
SHALL return `MISSION_EXIT_CONFIGURATION_ERROR` and the ROS Action terminal
state SHALL be ABORTED.
