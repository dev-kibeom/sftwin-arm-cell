# [SF-Twin] ARM Cell Vision ↔ Orchestration Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-VISION-ORCHESTRATION_v1.1.0`
- **Document Type:** `ICD`
- **Scope:** `Vision ↔ Orchestration`
- **Version:** `1.1.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoint and Request

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/vision/detect_target` | `arm_cell_interfaces/srv/DetectTarget` | Vision | Orchestration |

The executable realization is [`DetectTarget.srv`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/srv/DetectTarget.srv).

| Request field | Type | Meaning |
|---|---|---|
| `target_id` | `string` | caller-visible logical work/target selection key |
| `timeout` | `builtin_interfaces/Duration` | relative maximum wait for one valid frame set/result |

On request receipt, Vision starts a local steady monotonic timer for `timeout`. This is neither a ROS-time absolute deadline nor a cross-process wall/steady-clock timestamp. Simulation `/clock` pause or rollback SHALL NOT suspend this timer.

## 2. Response and Target Pose

| Response field | Type | Meaning |
|---|---|---|
| `result_code` | `DetectTargetResultCode` | canonical outcome |
| `target_pose` / `has_target_pose` | `geometry_msgs/PoseStamped` / `bool` | valid only for a successful target result |
| `estimated_height_m` / `has_estimated_height` | `float32` / `bool` | optional successful-object estimate |
| `diagnostic_detail` | `string` | non-normative diagnostic text |

Exact `DetectTargetResultCode.value` constants are `DETECT_RESULT_SUCCESS`, `DETECT_RESULT_OBJECT_NOT_FOUND`, `DETECT_RESULT_TIMEOUT`, `DETECT_RESULT_SENSOR_ERROR`, `DETECT_RESULT_TF_ERROR`, `DETECT_RESULT_GEOMETRY_ERROR`, and `DETECT_RESULT_INVALID_RESULT`.

`DETECT_RESULT_OBJECT_NOT_FOUND` is the only normal depletion outcome. `TIMEOUT`, `SENSOR_ERROR`, `TF_ERROR`, `GEOMETRY_ERROR`, and `INVALID_RESULT` are failure outcomes. `INVALID_RESULT` covers low-confidence or physical-sanity rejection; no confidence scalar is part of this baseline because its numeric semantics are not yet approved.

On `DETECT_RESULT_SUCCESS`, `target_pose.header.frame_id` SHALL be `base_link`; its stamp SHALL identify the accepted RGB-D observation time. The pose is an object-centric grasp reference, never a TCP pose, and has no tool correction.

## 3. Freshness Boundary

The request timeout bounds Vision's result wait. RGB-D freshness, calibration validity, and temporal ordering remain Vision ingress concerns based on sensor/simulation timestamps and receipt-time safeguards. The service does not transfer simulation-session epoch ownership to Vision.

## 4. Verification Requirements

### VR-ICD-VIS-ORCH-01 — Object-centric pose
Orchestration SHALL treat Vision pose as object-centric, not final TCP.

### VR-ICD-VIS-ORCH-02 — Depletion distinction
Object-not-found SHALL remain distinguishable from Vision failure.

### VR-ICD-VIS-ORCH-03 — Bounded request
The request timeout SHALL bound service wait independent of paused simulation clock.
