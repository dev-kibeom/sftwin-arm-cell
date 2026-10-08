# [SF-Twin] ARM Cell Vision ↔ Orchestration Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-VISION-ORCHESTRATION_v1.4.0`
- **Document Type:** `ICD`
- **Scope:** `Vision ↔ Orchestration`
- **Version:** `1.4.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Shared Interface`

## 1. Endpoint and Request

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/vision/detect_target` | `arm_cell_interfaces/srv/DetectTarget` | Vision | Orchestration |

The executable realization is [`DetectTarget.srv`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/srv/DetectTarget.srv).

| Request field | Type | Meaning |
|---|---|---|
| `target_id` | `string` | caller-visible logical work/target selection key and Vision detector-profile selector |
| `timeout` | `builtin_interfaces/Duration` | relative maximum wait for one valid frame set/result |

On request receipt, Vision starts a local steady monotonic timer for the request's `timeout`; this is the sole deadline for acquisition and processing. The deadline covers fresh observation acquisition and processing through the terminal response. Orchestration waits for that duration plus a small bounded response-delivery margin so it can receive Vision's terminal timeout response. This is neither a ROS-time absolute deadline nor a cross-process wall/steady-clock timestamp. Simulation `/clock` pause or rollback SHALL NOT suspend this timer. If non-interruptible processing finishes after the deadline, Vision SHALL return `DETECT_RESULT_TIMEOUT` and SHALL NOT publish the late perception result as success.

## 2. Response and Observation Pose

| Response field | Type | Meaning |
|---|---|---|
| `result_code` | `DetectTargetResultCode` | canonical outcome |
| `target_pose` / `has_target_pose` | `geometry_msgs/PoseStamped` / `bool` | valid only for a successful target result |
| `estimated_height_m` / `has_estimated_height` | `float32` / `bool` | optional successful-object estimate |
| `diagnostic_detail` | `string` | non-normative diagnostic text |

The service response includes the explicit `has_target_yaw` presence semantic
without changing the meaning of the existing `target_pose.pose.orientation`
field. `has_target_yaw=true` SHALL mean that the pose orientation contains a
valid object yaw; `false` SHALL mean that Orchestration MUST ignore the Vision
orientation for yaw resolution and use the recipe fallback. A valid zero yaw
is therefore distinct from yaw absence.

Exact `DetectTargetResultCode.value` constants are `DETECT_RESULT_SUCCESS`, `DETECT_RESULT_OBJECT_NOT_FOUND`, `DETECT_RESULT_TIMEOUT`, `DETECT_RESULT_SENSOR_ERROR`, `DETECT_RESULT_TF_ERROR`, `DETECT_RESULT_GEOMETRY_ERROR`, `DETECT_RESULT_INVALID_RESULT`, and the additive `DETECT_RESULT_TEMPORARY_INVALID_TARGET`.

`DETECT_RESULT_OBJECT_NOT_FOUND` is the only normal depletion outcome. It
means a valid configured profile processed a valid observation and found no
matching target. An unknown or unconfigured `target_id` SHALL return
`DETECT_RESULT_INVALID_RESULT`, not `OBJECT_NOT_FOUND`; this is a
configuration/contract error and is non-retryable. A configured profile that
cannot produce a currently usable target because of temporary observation
validity rejection SHALL return `DETECT_RESULT_TEMPORARY_INVALID_TARGET`; this
is distinct from depletion and may consume one bounded Orchestration attempt
slot. The value is realized as `DETECT_RESULT_TEMPORARY_INVALID_TARGET=7` in
[`DetectTargetResultCode.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/DetectTargetResultCode.msg).
`TIMEOUT` remains a bounded request failure and may be retried only under
the Orchestration retry policy. `SENSOR_ERROR`, `TF_ERROR`, and
`GEOMETRY_ERROR` are failures and are retryable only if the existing
Orchestration policy classifies the specific occurrence as transient. No
confidence scalar is added because its numeric semantics are not approved.

On `DETECT_RESULT_SUCCESS`, `target_pose.header.frame_id` SHALL be `base_link`; its stamp SHALL identify the accepted RGB-D observation time. `target_pose` is the external representation of the Observation Pose: a 3D geometric reference pose whose meaning is defined by the selected detector profile or its governing contract. It is not an object-center or TCP pose unless that profile explicitly defines it so, and it contains no robot/tool correction.

`target_pose.position` SHALL be the geometric reference established by the
selected Vision profile, expressed in `base_link`. For the production RawPart
profile, this is the visible top-surface centroid; its profile-specific meaning
is defined by the RawPart Vision FDS. The cross-cutting environment invariants
for production observation and independence from simulator ground truth are
owned by [ARM Cell Simulation Environment
Semantics](../../components/arm-cell/simulation-environment-semantics.md).
Vision SHALL NOT apply robot/tool/TCP correction. Motion owns the configured
Observation-Pose-to-Object-Pose and Object-Pose-to-Target-Pose transforms.

`target_id` SHALL select a deployment-configured Vision-owned detector profile.
The mapping and its profile parameters are deployment configuration, not
fields in this public service contract. Sensor/environment eligibility and
simulator-ground-truth independence are defined by [ARM Cell Simulation
Environment Semantics](../../components/arm-cell/simulation-environment-semantics.md).

### 2.1 Validation-only Fixed Adapter Overlay

A separately launched deterministic PnP validation profile MAY provide the
same `DetectTarget` ROS shape and downstream object-reference semantics from
a fixture registration instead of an RGB-D observation. This is an explicit
validation-only provenance overlay, not production Vision behavior.

Under this overlay:

- `target_pose.header.frame_id` remains `base_link`;
- position remains the fixture registration's declared observation reference
  and contains no TCP/tool correction;
- the stamp identifies the accepted fixture registration event, not an RGB-D
  observation;
- result-code, target-ID/profile validation, one-shot depletion, yaw-presence,
  and estimated-height non-authority remain applicable;
- fixture-supplied object yaw sets `has_target_yaw=true`; absent yaw sets it
  false and leaves recipe fallback to Orchestration;
- the adapter SHALL be a separate executable/profile that the production
  operational profile cannot select.

Evidence from this overlay SHALL identify fixture provenance and SHALL NOT
claim detector-profile, sensor-observation, timestamp, segmentation, or
`VR-VIS-DETECT-11` validation. Orchestration consumes the successful
object-reference pose without learning simulator identity or attachment
details.

For a successful PICK source result, Vision is authoritative for the
Observation Pose position `x`, `y`, and `z`. Yaw is a rotation about `base_link` +Z and is
object-centric, not tool/TCP orientation. `has_target_yaw=true` SHALL take precedence; `false` SHALL select
the target-specific recipe yaw fallback. Vision owns whether yaw is valid
enough to publish, and downstream components MUST NOT reinterpret raw Vision
confidence thresholds. Vision SHALL perform validity filtering for optional
perception geometry before publication and SHALL NOT apply robot/tool/TCP
compensation. The observed Vision `z` remains the authoritative
observation-reference component. `estimated_height_m` is currently non-authoritative
and unused for PICK geometry; it SHALL NOT modify, repair, offset, replace, or
otherwise participate in Observation-Pose-to-Object-Pose or target-pose
generation. In particular it SHALL NOT modify returned target xyz/z. Any future use requires
a separately approved semantic purpose.

## 3. Freshness Boundary

The request timeout bounds Vision's result wait. RGB-D freshness, calibration validity, and temporal ordering remain Vision ingress concerns based on sensor/simulation timestamps and receipt-time safeguards. The service does not transfer simulation-session epoch ownership to Vision.

## 4. Verification Requirements

### VR-ICD-VIS-ORCH-01 — Detector-profile observation pose
Orchestration SHALL forward the Vision Observation Pose to Motion without
reinterpreting its detector-profile geometric reference or applying TCP/tool
correction.

### VR-ICD-VIS-ORCH-04 — Explicit yaw presence
For the implemented interface, a Vision result with
`has_target_yaw=true` SHALL override recipe yaw, while `has_target_yaw=false`
SHALL cause Orchestration to ignore the Vision orientation and use recipe yaw.
A valid zero yaw SHALL remain distinguishable from yaw absence.

### VR-ICD-VIS-ORCH-05 — Observation position and estimated-height authority
Orchestration SHALL forward the Vision Observation Pose position unchanged in
the PICK Motion goal. `estimated_height_m` SHALL NOT alter that value or PICK
geometry; Motion resolves Object Pose from the selected profile's reference and
object geometry.

### VR-ICD-VIS-ORCH-02 — Depletion distinction
Object-not-found SHALL remain distinguishable from Vision failure.

### VR-ICD-VIS-ORCH-03 — Bounded request
The request timeout SHALL bound service wait independent of paused simulation clock.

### VR-ICD-VIS-ORCH-06 — Target profile selection
The requested `target_id` SHALL select the Vision-owned detector profile; it
SHALL NOT be ignored by the DetectTarget service/processing seam.

### VR-ICD-VIS-ORCH-07 — Unknown target ID
An unknown or unconfigured `target_id` SHALL return
`DETECT_RESULT_INVALID_RESULT`.

### VR-ICD-VIS-ORCH-08 — Target absent under valid profile
A valid configured profile with no matching observation SHALL return
`DETECT_RESULT_OBJECT_NOT_FOUND`, distinct from an unknown/unconfigured ID.

### VR-ICD-VIS-ORCH-09 — Reference-point semantics
Successful `target_pose.position` SHALL represent the selected detector
profile's declared observation reference in `base_link`, without TCP/tool
correction. The RawPart profile's visible top-surface centroid is specified by
the RawPart Vision FDS.

### VR-ICD-VIS-ORCH-10 — Estimated-height non-authority
`estimated_height_m` SHALL NOT modify returned target xyz/z.

### VR-ICD-VIS-ORCH-11 — Validation provenance isolation
A validation-only Fixed Vision result SHALL preserve the public ROS shape,
result/frame/object-reference semantics, and fixture-registration stamp while
remaining impossible to select from the production operational profile. Its
evidence SHALL NOT be represented as RGB-D perception validation.

### VR-ICD-VIS-ORCH-12 — Profile-isolated target selection
Distinct configured `target_id` values SHALL select their distinct
Vision-owned profiles. Unknown/unconfigured IDs SHALL return `INVALID_RESULT`,
never generic foreground aggregation. Competing foreground geometry SHALL NOT
shift the selected profile's pose. A configured RawPart profile SHALL select
RawPart only when its required observation is valid; absent or invalid RawPart
shall be rejected or reported absent according to the profile contract.

### VR-ICD-VIS-ORCH-13 — Temporary invalid target distinction
Given a configured valid target profile with a temporarily invalid
observation, Vision SHALL report `DETECT_RESULT_TEMPORARY_INVALID_TARGET`.
Given an unknown/unconfigured target profile, it SHALL report
`DETECT_RESULT_INVALID_RESULT`. Neither outcome SHALL be mapped to depletion;
the distinction allows Orchestration to retry only the temporary case.
