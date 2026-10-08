# [SF-Twin] ARM Cell Safety ↔ Orchestration Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-SAFETY-ORCHESTRATION_v1.2.0`
- **Document Type:** `ICD`
- **Scope:** `Safety Supervisor ↔ Orchestration`
- **Version:** `1.2.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoint and Authority

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/safety/state` | `arm_cell_interfaces/msg/SafetyState` | Safety | Orchestration, Motion |

Safety is the sole authority for `MotionCapability` and `selected_stop_mode`. Orchestration consumes these fields and SHALL NOT infer, create, or elevate capability or stop severity from active causes, mission state, or local policy.

## 2. SafetyState Wire Contract

The executable realization is [`SafetyState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyState.msg), with shared domains [`MotionCapability.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/MotionCapability.msg) and [`SafetyCauseSet.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyCauseSet.msg).

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Safety evaluation/publication time in the active ROS clock |
| `safety_state` | `uint8` | `SAFETY_STATE_UNKNOWN`, `SAFETY_STATE_SAFE`, `SAFETY_STATE_INTERLOCKED`, `SAFETY_STATE_STOPPING`, `SAFETY_STATE_STOPPED`, `SAFETY_STATE_RECOVERY_REQUIRED`, or `SAFETY_STATE_EMERGENCY_STOPPED` |
| `motion_capability` | `MotionCapability` | Safety-owned permission domain |
| `selected_stop_mode` | `StopMode` | Safety-owned canonical currently selected stop severity; `STOP_MODE_CONTROLLED` is the lowest baseline when no higher-severity stop is selected |
| `active_causes` | `SafetyCauseSet` | causes currently present |
| `latched_causes` | `SafetyCauseSet` | causes retained until Safety reset policy clears them |
| `valid` | `bool` | Safety has formed a valid supervision state from its required inputs |
| `required_inputs_fresh` | `bool` | all required input classes are currently fresh under Safety watchdog policy |
| `diagnostic_detail` | `string` | optional non-normative diagnostic text |

`SafetyCauseSet.value` is a bitset. Its exact known causes are `E_STOP`, `STO`, `COMMUNICATION_LOSS`, `COLLISION`, `PREMATURE_UNDOCK`, `PACKML_ABORT`, `GRIPPER_FAILURE`, `OTHER`, and `REQUIRED_INPUT_INVALID`. `COMMUNICATION_LOSS` denotes required inputs that were valid but timed out; received invalid or source-implausible required samples use `REQUIRED_INPUT_INVALID`. `OTHER` preserves fail-closed behavior for a non-enumerated cause; its diagnostic text is not a machine-readable decision authority.

The message timestamp does not replace Safety's local receipt-time watchdog evaluation. Missing, stale, or invalid required input remains fail-closed even if an older `SafetyState` message is present.

`selected_stop_mode` is written only by Safety. It uses the shared `StopMode`
domain and is valid only with `valid=true` and `required_inputs_fresh=true`.
Consumers compare its ordering (`CONTROLLED < IMMEDIATE < EMERGENCY`) but do not
recalculate it from `active_causes`. The controlled value is the lowest shared
domain value and is also the no-higher-severity baseline; it is not a hidden
diagnostic encoding.

## 3. Orchestration Behavior

| Capability | Orchestration permission |
|---|---|
| `MOTION_NONE` | no normal or recovery trajectory request |
| `MOTION_RECOVERY_ONLY` | may coordinate `RETRACT` only |
| `MOTION_NORMAL` | may advance the normal mission branch |

Capability loss during a mission causes coordination halt/cancel behavior. That coordination is secondary to Safety's direct StopMotion path. Orchestration waits for explicit recovery-capable Safety state before requesting `RETRACT`; it cannot infer authorization merely from its own mission state.

## 4. Verification Requirements

### VR-ICD-SAFE-ORCH-01 — Capability consumption
Orchestration SHALL consume, not redefine, Safety capability.

### VR-ICD-SAFE-ORCH-02 — Normal gate
Normal mission motion SHALL require `MOTION_NORMAL`.

### VR-ICD-SAFE-ORCH-03 — Recovery gate
Recovery request SHALL require current recovery-capable Safety authority.

### VR-ICD-SAFE-ORCH-04 — Selected stop severity authority
Safety SHALL publish the canonical selected stop severity through
`SafetyState.selected_stop_mode`; Orchestration SHALL consume and compare that
field without deriving severity from `active_causes`, mission state, or local
policy.
