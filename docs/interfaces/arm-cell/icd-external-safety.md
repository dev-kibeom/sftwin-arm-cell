# [SF-Twin] ARM Cell External State ↔ Safety Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-EXTERNAL-SAFETY_v1.1.0`
- **Document Type:** `ICD`
- **Scope:** `AMR / PackML / Safety-HW Adapters ↔ Safety Supervisor`
- **Version:** `1.1.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoints and Authority

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/amr/docking_report` | `arm_cell_interfaces/msg/AMRDockingState` | external AMR adapter | Safety |
| `/packml/state` | `arm_cell_interfaces/msg/PackMLState` | external PackML adapter | Safety |
| `/safety/hardware_state` | `arm_cell_interfaces/msg/SafetyHardwareState` | external Safety-HW adapter | Safety |

External adapters publish state and freshness evidence only. They SHALL NOT determine stop severity, Motion capability, Safety latch/reset, or mission result.

## 2. Shared Freshness Semantics

Every external state message contains `std_msgs/Header header` and `bool valid`. `header.stamp` is the timestamp assigned by the source adapter; `valid=false` means the sample cannot establish a valid external state.

Safety evaluates source plausibility and freshness using the message semantics together with local receipt time. A publisher becoming silent is therefore observable as watchdog expiry; communication loss is not represented solely by an in-band fault flag. The threshold and rate remain profile configuration, not message constants.

## 3. AMR Docking State

[`AMRDockingState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/AMRDockingState.msg) fields are `header`, `amr_id`, `current_node_id`, `docking_state`, `driving`, and `valid`.

`docking_state` uses exactly `AMR_DOCKING_UNKNOWN`, `AMR_DOCKING_DOCKED`, or `AMR_DOCKING_UNDOCKED`. This is the supported ARM Cell docking subset; it is not a full AMR/VDA state model. Safety interprets `UNDOCKED` and `driving` contextually. Neither value independently selects a stop severity.

## 4. PackML State

[`PackMLState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/PackMLState.msg) fields are `header`, `state`, and `valid`.

The supported canonical subset is exactly `PACKML_STATE_UNKNOWN`, `PACKML_STATE_IDLE`, `PACKML_STATE_STARTING`, `PACKML_STATE_EXECUTE`, `PACKML_STATE_COMPLETE`, and `PACKML_STATE_ABORTED`. It is not a declaration of the full PackML state machine. `IDLE`, `STARTING`, and `COMPLETE` may gate normal motion without being stop-level faults; Safety evaluates `EXECUTE` and `ABORTED` contextually.

## 5. Safety Hardware State

[`SafetyHardwareState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyHardwareState.msg) fields are `header`, `e_stop_active`, `sto_active`, and `valid`.

This message communicates only the currently approved supervisory inputs. It does not include an unapproved protective-stop domain and does not replace the independent safety-rated E-Stop/STO path.

## 6. Adapter Substitutability

Mock and real adapters may differ internally, but SHALL publish this canonical subset without opaque vendor codes becoming Safety-facing semantics. Clearing an external mock condition clears only the published external condition; it does not bypass Safety's separate latched-cause/reset policy.

## 7. Verification Requirements

### VR-ICD-EXT-SAFE-01 — Contextual AMR semantics
Safety SHALL not treat all undocked states as identical regardless of process context.

### VR-ICD-EXT-SAFE-02 — Fresh-sample requirement
Safety SHALL require fresh valid external-state samples before granting motion.

### VR-ICD-EXT-SAFE-03 — Hardware-state boundary
SafetyHardwareState SHALL remain synchronization evidence, not a replacement safety-rated channel.

### VR-ICD-EXT-SAFE-04 — Adapter substitutability
Real/mock adapters SHALL preserve the shared Safety-facing semantic contract.
