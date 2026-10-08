# [SF-Twin] ARM Cell Safety ↔ Motion Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-SAFETY-MOTION_v1.3.0`
- **Document Type:** `ICD`
- **Scope:** `Safety Supervisor ↔ Motion Core`
- **Version:** `1.3.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Shared Interface`

## 1. Contract Realization

This ICD is the normative semantic and wire contract. The linked files under [`arm_cell_interfaces`](../../../ros2_ws/src/interfaces/arm_cell_interfaces) are its executable ROS 2 IDL realization; generated language bindings do not independently define semantics.

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/safety/stop_motion` | `arm_cell_interfaces/srv/StopMotion` | Safety | Motion |
| `/motion/state` | `arm_cell_interfaces/msg/MotionStatus` | Motion | Safety, Orchestration, UI Hub observer |
| `/safety/state` capability, selected-stop-severity, and permitted-envelope fields | `arm_cell_interfaces/msg/SafetyState` | Safety | Motion, Orchestration |

## 2. Motion Capability

`MotionCapability.value` is Safety-owned and uses these exact constants:

| Constant | Normal task | RETRACT | Meaning |
|---|---:|---:|---|
| `MOTION_NONE` | prohibited | prohibited | no trajectory task is authorized |
| `MOTION_RECOVERY_ONLY` | prohibited | permitted | bounded recovery only |
| `MOTION_NORMAL` | permitted | permitted | normal execution allowed |

Safety is the sole capability authority. Motion SHALL independently enforce the current value when it accepts a trajectory-producing goal.

Capability authorization is not a reusable lease. Motion SHALL re-check the
current valid Safety capability immediately before backend trajectory
execution. If that capability is no longer sufficient at that boundary, the
backend trajectory execution SHALL NOT start. A capability downgrade alone
SHALL NOT cause Motion to invent its own stop policy for an already-active
backend trajectory. Stopping an already-active trajectory is owned by the
Safety → Motion `StopMotion` path; Safety is the sole authority for stop
initiation and severity, while Motion enforces and executes the command.

## 2.1 Permitted Motion Envelope

Safety is also the sole authority for the currently permitted motion
envelope. The existing `SafetyState` contract SHALL be extended with a valid
envelope containing maximum velocity and acceleration scale factors relative
to the active calibrated Motion profile. Each factor is finite and in `(0, 1]`;
`1.0` means no restriction from that dimension. `MOTION_NONE` remains the
way to prohibit all trajectory motion. Envelope scale values and the mapping
from condition/severity to those values are configuration/calibration
authority, not values selected by the demo.

The `SafetyState` message includes these fields:

| Field | Type | Meaning |
|---|---|---|
| `motion_envelope_valid` | `bool` | envelope is valid for current Safety state |
| `max_velocity_scale` | `float32` | upper velocity scale relative to calibrated nominal profile |
| `max_acceleration_scale` | `float32` | upper acceleration scale relative to calibrated nominal profile |
| `diagnostic_detail` | `string` | non-normative diagnostic context; consumers must use typed state/capability fields for decisions |

The existing SafetyState header supplies source timestamp; consumers still use
their local receipt-time freshness policy. No new message or enum is needed.

Motion SHALL consume the current envelope without interpreting active causes,
condition class, or severity. Motion SHALL constrain every newly planned and
executed trajectory to the current velocity and acceleration caps. A
restriction update applies to a trajectory already in progress. For an active trajectory, the backend may continue under a tighter envelope
only when it can safely apply the restriction and provide evidence of the
applied limits. If it cannot, Safety SHALL use its authorized StopMotion path,
await confirmed inactivity, and allow motion only through a newly planned
task under the current envelope and capability. Motion SHALL NOT select stop
severity. The contract does not prescribe a backend-specific live-update or
stop implementation.
If an envelope is missing, invalid, or stale, Motion SHALL not start new
trajectory execution; Safety remains fail-closed.

This semantic extension is realized in the executable ROS IDL and generated
bindings: [`SafetyState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyState.msg)
and [`MotionStatus.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/MotionStatus.msg).

## 3. StopMotion

`StopMotion` request fields are:

| Field | Type | Meaning |
|---|---|---|
| `request_id` | `unique_identifier_msgs/UUID` | Safety-assigned stop request correlation identity |
| `stop_mode` | `arm_cell_interfaces/StopMode` | requested stop method |
| `causes` | `arm_cell_interfaces/SafetyCauseSet` | active known-cause bitset motivating this request |

`StopMode.value` SHALL use exactly one of:

- `STOP_MODE_CONTROLLED` — configured ramp-down / controlled-deceleration stop;
- `STOP_MODE_IMMEDIATE` — active trajectory cancellation with hold/current-position behavior;
- `STOP_MODE_EMERGENCY` — best-effort software cancellation/rejection synchronized with independent hardware E-Stop/STO.

The response fields are `bool accepted` and non-normative `string diagnostic_detail`. `accepted=true` confirms only that Motion accepted the request for processing. It is not a completion signal and does not establish that the robot or execution is stopped.

Safety is the sole stop initiation and severity authority. Stop severity is
monotonic: `CONTROLLED < IMMEDIATE < EMERGENCY`; a higher severity supersedes a
lower one regardless of source, and a lower-severity source SHALL NOT downgrade
an active higher-severity hazard. This ICD does not define numeric deceleration
limits, timing, or backend ramp implementation for `STOP_MODE_CONTROLLED`.

The canonical currently selected severity is also published by Safety as
`SafetyState.selected_stop_mode`. This is an observable Safety-owned field;
Motion and Orchestration consume it and do not derive it from causes or local
policy.

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
| `holding_state` | `uint8` | current Motion-observed gripper/object disposition: `HOLDING_UNKNOWN=0`, `HOLDING_HELD=1`, `HOLDING_RELEASED=2` |
| `applied_velocity_scale` / `applied_acceleration_scale` | `float32` / `float32` | backend-attested active execution limits relative to calibrated Motion profile |
| `applied_envelope_valid` | `bool` | active execution evidence is fresh and confirms the reported applied limits |
| `diagnostic_detail` | `string` | non-normative diagnostic text |

Final Demo requires these holding and applied-envelope fields as additive
extensions to MotionStatus. For a tighter envelope during active execution,
Safety may permit continuation only when fresh MotionStatus/backend evidence
confirms both applied scales are within the current Safety limits. A stale or missing observation SHALL be published as
`HOLDING_UNKNOWN`; consumers SHALL NOT infer a fresh state from a previous
sample. The exact observation freshness bound remains owned by the Motion
backend contract/configuration.

Exact `execution_state` constants are `MOTION_STATE_UNKNOWN`, `MOTION_STATE_IDLE`, `MOTION_STATE_EXECUTING`, `MOTION_STATE_STOPPING`, `MOTION_STATE_STOPPED`, `MOTION_STATE_EMERGENCY_STOPPED`, and `MOTION_STATE_FAULTED`.

`MOTION_STATE_STOPPING` means stop dispatch has begun but inactivity is not yet confirmed. `MOTION_STATE_STOPPED` requires `execution_active=false` and `backend_inactivity_confirmed=true`. `MOTION_STATE_EMERGENCY_STOPPED` additionally represents Motion software synchronized with an active E-Stop/STO condition; it does not replace independent safety-rated hardware behavior.

## 5. Direct Path and Recovery

Safety→Motion direct stop is the primary software stop path. Orchestration cancellation may occur in parallel but is not required for stop initiation.

Safety SHALL wait for `MotionStatus` stop-completion evidence before granting recovery capability. Motion SHALL re-check current Safety capability when it receives `RETRACT`.

## 6. Verification Requirements

### VR-ICD-SAFE-MOT-01 — Safety authority
Only Safety determines canonical motion capability.

### VR-ICD-SAFE-MOT-02 — Motion enforcement
Given a trajectory-producing goal, when the current valid Safety capability is
insufficient at acceptance or immediately before backend execution, Motion
SHALL reject the goal or prevent backend execution from starting. A capability
authorization SHALL NOT be reused as a lease.

### VR-ICD-SAFE-MOT-02A — Active execution ownership
Given an already-active backend trajectory, when Safety capability downgrades
without a new `StopMotion` request, Motion SHALL NOT invent a stop policy or
initiate a stop solely because of that downgrade. Active execution stopping
SHALL be initiated through Safety → Motion `StopMotion`.

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

### VR-ICD-SAFE-MOT-08 — Envelope enforcement
Given a valid Safety envelope, Motion SHALL constrain trajectory velocity and
acceleration to its current limits. If a tighter envelope arrives during
active execution, Motion may continue only when the backend safely applies and
attests the new limits. Otherwise Safety SHALL stop through StopMotion; no
continuation trajectory starts until inactivity is confirmed and a new task is
planned under the current envelope. The oracle is envelope application evidence
or stop state, backend inactivity confirmation when stopped, and subsequent
task limits.

### VR-ICD-SAFE-MOT-09 — Holding observability
MotionStatus SHALL expose the current `HELD`, `RELEASED`, or `UNKNOWN`
observation to Orchestration and the Hub. Motion SHALL publish stale/missing
holding data as UNKNOWN, and consumers SHALL NOT interpret a prior fresh value
as current after freshness expires. The existing MotionStatus header timestamps
the publication; backend-observation freshness remains Motion's responsibility.
