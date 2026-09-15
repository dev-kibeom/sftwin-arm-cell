# [SF-Twin] ARM Cell ExecuteCycle Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-ORCHESTRATION-EXECUTE-CYCLE_v1.0.0`
- **Document Type:** `ICD`
- **Scope:** `External Mission Caller ↔ Orchestration`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoint and Identity

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/orchestration/execute_cycle` | `arm_cell_interfaces/action/ExecuteCycle` | Orchestration | external mission caller |

The goal contains exactly `string target_id`, the caller-visible logical work/target selection key. It is not a perception algorithm parameter. The ROS Action Goal UUID is the canonical execution identity.

## 2. Feedback

The executable realization is [`ExecuteCycle.action`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/action/ExecuteCycle.action). `MissionPhase.value` uses `MISSION_PHASE_ENSURING_HOME`, `MISSION_PHASE_DETECTING_TARGET`, `MISSION_PHASE_PICKING`, `MISSION_PHASE_PLACING`, `MISSION_PHASE_RETURNING_HOME`, `MISSION_PHASE_WAITING_RECOVERY`, `MISSION_PHASE_RETRACTING`, and `MISSION_PHASE_FINISHED`; `MISSION_PHASE_UNKNOWN` is the unset value.

These phases communicate public mission progress only. They do not expose BehaviorTree implementation details.

## 3. Result and Cancellation

The result contains `MissionExitReason exit_reason` and non-normative `diagnostic_detail`. Exact exit constants are:

| Constant | Meaning |
|---|---|
| `MISSION_EXIT_NONE` | no terminal reason has been established |
| `MISSION_EXIT_DEPLETED` | Vision returned `OBJECT_NOT_FOUND`; normal successful completion |
| `MISSION_EXIT_VISION_ERROR` | a non-depletion Vision outcome interrupted the mission |
| `MISSION_EXIT_MOTION_ERROR` | Motion task failure interrupted the mission |
| `MISSION_EXIT_SAFETY_PREEMPTED` | Safety preempted the mission |
| `MISSION_EXIT_CANCELED` | caller cancellation interrupted the mission |

Client cancellation maps to `MISSION_EXIT_CANCELED` unless Safety has preempted the mission. Safety preemption remains distinguishable as `MISSION_EXIT_SAFETY_PREEMPTED` and does not depend on action cancellation for robot stopping.

`MISSION_EXIT_DEPLETED` is the only normal successful mission completion. A successful controlled recovery does not overwrite the original `VISION_ERROR`, `MOTION_ERROR`, `SAFETY_PREEMPTED`, or `CANCELED` exit reason and does not automatically retry the mission.

## 4. Verification Requirements

### VR-ICD-ORCH-CYCLE-01 — Depletion-only success
Only `MISSION_EXIT_DEPLETED` SHALL represent normal mission completion.

### VR-ICD-ORCH-CYCLE-02 — Result preservation
Recovery completion SHALL not overwrite the interrupted mission exit reason.

### VR-ICD-ORCH-CYCLE-03 — Preemption distinction
Safety preemption and caller cancellation SHALL remain distinguishable.
