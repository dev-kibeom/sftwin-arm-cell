# [SF-Twin] ARM Cell Safety ↔ Motion Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-SAFETY-MOTION_v1.1.0`
- **Document Type:** `ICD`
- **Scope:** `Safety Supervisor ↔ Motion Core`
- **Version:** `1.1.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Shared Interface`

## 1. Contract Realization

This ICD is the normative semantic and wire contract. The linked files under [`arm_cell_interfaces`](../../../ros2_ws/src/interfaces/arm_cell_interfaces) are its executable ROS 2 IDL realization; generated language bindings do not independently define semantics.

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/safety/stop_motion` | `arm_cell_interfaces/srv/StopMotion` | Safety | Motion |
| `/motion/state` | `arm_cell_interfaces/msg/MotionStatus` | Motion | Safety, Orchestration |
| `/safety/state` capability field | `arm_cell_interfaces/msg/SafetyState` | Safety | Motion, Orchestration |

## 2. Motion Capability

`MotionCapability.value` is Safety-owned and uses these exact constants:

| Constant | Normal task | RETRACT | Meaning |
|---|---:|---:|---|
| `MOTION_NONE` | prohibited | prohibited | no trajectory task is authorized |
| `MOTION_RECOVERY_ONLY` | prohibited | permitted | bounded recovery only |
| `MOTION_NORMAL` | permitted | permitted | normal execution allowed |

Safety is the sole capability authority. Motion SHALL independently enforce the current value when it accepts a trajectory-producing goal.

## 3. StopMotion

`StopMotion` request fields are:

| Field | Type | Meaning |
|---|---|---|
| `request_id` | `unique_identifier_msgs/UUID` | Safety-assigned stop request correlation identity |
| `stop_mode` | `arm_cell_interfaces/StopMode` | requested stop method |
| `causes` | `arm_cell_interfaces/SafetyCauseSet` | active known-cause bitset motivating this request |

`StopMode.value` SHALL use exactly one of:

- `STOP_MODE_CONTROLLED` — configured controlled/decelerated stop;
- `STOP_MODE_IMMEDIATE` — active trajectory cancellation with hold/current-position behavior;
- `STOP_MODE_EMERGENCY` — best-effort software cancellation/rejection synchronized with independent hardware E-Stop/STO.

The response fields are `bool accepted` and non-normative `string diagnostic_detail`. `accepted=true` confirms only that Motion accepted the request for processing. It is not a completion signal and does not establish that the robot or execution is stopped.

## 4. MotionStatus

`MotionStatus` fields are:

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Motion publication time in the active ROS clock |
| `execution_state` | `uint8` | canonical Motion execution state |
| `active_execution_id` / `has_active_execution` | `UUID` / `bool` | active ExecuteTask Action Goal UUID, if any |
| `active_task_type` | `MotionTaskType` | active task class, or `TASK_TYPE_UNSPECIFIED` |
| `execution_active` | `bool` | backend execution remains active |
| `backend_inactivity_confirmed` | `bool` | backend has explicitly confirmed inactivity |
| `last_stop_request_id` / `has_last_stop_request` | `UUID` / `bool` | most recently accepted direct stop request, if any |
| `diagnostic_detail` | `string` | non-normative diagnostic text |

Exact `execution_state` constants are `MOTION_STATE_UNKNOWN`, `MOTION_STATE_IDLE`, `MOTION_STATE_EXECUTING`, `MOTION_STATE_STOPPING`, `MOTION_STATE_STOPPED`, `MOTION_STATE_EMERGENCY_STOPPED`, and `MOTION_STATE_FAULTED`.

`MOTION_STATE_STOPPING` means stop dispatch has begun but inactivity is not yet confirmed. `MOTION_STATE_STOPPED` requires `execution_active=false` and `backend_inactivity_confirmed=true`. `MOTION_STATE_EMERGENCY_STOPPED` additionally represents Motion software synchronized with an active E-Stop/STO condition; it does not replace independent safety-rated hardware behavior.

## 5. Direct Path and Recovery

Safety→Motion direct stop is the primary software stop path. Orchestration cancellation may occur in parallel but is not required for stop initiation.

Safety SHALL wait for `MotionStatus` stop-completion evidence before granting recovery capability. Motion SHALL re-check current Safety capability when it receives `RETRACT`.

## 6. Verification Requirements

### VR-ICD-SAFE-MOT-01 — Safety authority
Only Safety determines canonical motion capability.

### VR-ICD-SAFE-MOT-02 — Motion enforcement
Motion SHALL reject goals inconsistent with capability.

### VR-ICD-SAFE-MOT-03 — ACK semantics
StopMotion acceptance SHALL not mean stop completion.

### VR-ICD-SAFE-MOT-04 — Stop completion
Normal stop completion SHALL require backend inactivity confirmation.

### VR-ICD-SAFE-MOT-05 — Direct path
Safety SHALL stop Motion without relying on Orchestration cancel.

### VR-ICD-SAFE-MOT-06 — Recovery gate
RETRACT SHALL require valid recovery-capable authority.

### VR-ICD-SAFE-MOT-07 — Hardware boundary
Emergency-stop software semantics SHALL not replace safety-rated hardware functions.
