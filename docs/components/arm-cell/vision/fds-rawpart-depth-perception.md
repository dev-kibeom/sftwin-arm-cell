# [SF-Twin] ARM Cell Production RawPart Depth Perception

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VISION-RAWPART-DEPTH_v0.2.0`
- **Document Type:** `FDS`
- **Scope:** `Vision / Production RawPart profile`
- **Version:** `0.2.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Vision`
- **Parent behavior:** [Vision DetectTarget FDS](fds-detect-target.md)

## 1. Purpose and authority

This FDS defines the nominal production depth/geometry pipeline for a
configured RawPart profile in the Final Demo. The parent DetectTarget FDS
remains the owner of request behavior, result semantics, freshness, and the
successful object-reference contract. This document refines the production
RawPart profile algorithm over sensor observations; it does not define a new
service or change that contract.

The caller's logical `target_id` selects a preconfigured Vision-owned profile.
That profile supplies the expected workpiece shape and dimensions and the
workspace and geometry constraints needed for selection. Profile values,
numeric thresholds, calibration values, marker IDs/poses, ROI dimensions, and
fiducial physical size/geometry belong to deployment configuration authority
and are intentionally not fixed here. An unknown or unconfigured profile is
an invalid configuration/request, not evidence of depletion.

`target_id` is the only perception-selection identity Orchestration supplies.
Vision resolves it to its own deployment profile; Orchestration does not store
or forward perception thresholds or geometry. A profile owns the material's
nominal geometry, admissible tolerances, workspace and support-relative
constraints, and the parameters used for candidate isolation and geometry
matching. Supporting another material is a profile/configuration change when
its perception behavior is expressible by this profile contract.

The pipeline observes synchronized RGB-D and validated CameraInfo, plus
fiducial observations and TF. It SHALL NOT use simulator prim identity, exact
world coordinates, simulator metadata, or simulator APIs as production
perception input. RGB appearance and learned detectors are optional and are
not required by this nominal profile. Environment preconditions remain owned
by [Simulation Environment Semantics](../simulation-environment-semantics.md).

## 2. Processing pipeline

```text
DetectTarget(target_id)
→ resolve configured RawPart profile
→ acquire fresh synchronized RGB-D and compatible CameraInfo
→ observe configured fiducial(s), establish support frame/plane
→ deproject depth, express samples relative to support
→ restrict to configured workspace and support-relative geometry band
→ form target candidates and validate against known workpiece profile
→ reject unrelated/ambiguous foreground; select requested target only
→ estimate the visible top surface and this profile's Observation Pose reference
→ estimate yaw only when geometry makes it observable
→ sanity/freshness checks
→ transform optical-frame reference to base_link
→ return parent DetectTarget result
```

Stages that require a valid configured profile, fresh sensor set, and valid
support reference fail closed. They SHALL NOT be interpreted as target
absence. The processing result follows the parent's result-code contract:
only a valid observation in which the configured target is absent may return
`DETECT_RESULT_OBJECT_NOT_FOUND`.

### 2.1 Input set and freshness

Vision accepts a depth image and its temporally synchronized RGB image as one
observation set, together with CameraInfo validated for the active image
geometry and optical frame. CameraInfo may be cached according to the Vision
CDS calibration rules; it is not required to have the image timestamp. The
image set must pass the ingress synchronization, ordering, and freshness
checks before geometry processing. The accepted observation timestamp is
carried into the successful `PoseStamped` result.

The service's steady-clock deadline bounds the wait for a usable set. Timeout,
missing/stale images, incompatible calibration, invalid depth, or incomplete
ingress are failures (`TIMEOUT`, `SENSOR_ERROR`, or the parent's temporary
invalid-observation outcome as applicable), never depletion.

### 2.2 Support reference from fiducial observation

The detector observes the deployment-configured ArUco/fiducial family and
marker set in RGB. It uses calibrated intrinsics and the observed marker
corners to estimate marker pose in the camera optical frame. Where the
fiducial is on the support/work-surface, its configured relation establishes
the support frame; synchronized depth samples around the observed support
region may refine the support plane. A robust plane fit (RANSAC or an
equivalent outlier-resistant estimator) is appropriate because marker edges,
depth discontinuities, and foreground points can contaminate the local
support samples. The plane normal and configured marker-to-support relation
define the support-relative origin and axes.

ArUco establishes support/work-surface reference only. Marker ID or marker
appearance SHALL NOT identify the RawPart. The reference must be supported by
a geometrically consistent marker observation and a valid support estimate.
Missing, ambiguous, stale, or geometrically inconsistent fiducials, and a
support plane that cannot be estimated within configured quality limits,
produce a perception failure (`GEOMETRY_ERROR` or temporary invalid target as
classified by the parent contract), never `OBJECT_NOT_FOUND`.

### 2.3 Support-relative depth geometry

For each valid depth pixel `(u,v)` with depth `z`, Vision deprojects using the
validated camera intrinsics, for example
`p_camera = z * K^-1 [u, v, 1]^T`. It applies the observed camera-to-support
transform to obtain `p_support`. This produces support-relative height and
planar position without consulting a simulator/world coordinate or prim.
Invalid, out-of-range, or non-finite samples are discarded under configured
sensor-validity limits. The configured workspace region and support-relative
height/geometry envelope bound candidate generation.

### 2.4 Target candidate isolation and geometry validation

Candidate generation SHALL be depth/geometry based for the nominal RawPart
profile. Within the configured workspace and above the support plane,
connected components over depth/3D-neighbor continuity are a suitable
replaceable candidate-forming method: they split spatially disconnected
foreground regions without requiring semantic RGB classification. A
profile may replace this method with another deterministic geometric
partitioner that preserves the same selection and validation behavior.

Each candidate is evaluated against the configured workpiece model: expected
shape family, dimensions/extents, support contact/height relation, and
configured admissible geometric variation. This known geometry is allowed to
isolate and validate the requested logical target. Candidate validation may
use robust planar/edge fitting, dimensions from the fitted geometry, and
support-relative pose constraints. A candidate that fails required model or
physical constraints is rejected as unrelated or invalid foreground; it is
not merged into the requested target.

If multiple candidates satisfy the requested profile, selection uses the
configured workspace/geometry constraints and target-selection policy. A
uniquely matching candidate is selected. If competing candidates remain
indistinguishable, Vision SHALL not choose an arbitrary one: it reports the
parent contract's temporary-invalid/geometry failure outcome. Other
foreground components SHALL NOT change the selected candidate's points,
centroid, or pose. If no candidate matches after a valid support reference and
valid observation have been processed, return `OBJECT_NOT_FOUND`.

Connected components are an implementation choice, not a semantic
requirement. Their replaceable boundary is the candidate set supplied to
profile matching; replacement must preserve competing-foreground rejection,
requested-profile isolation, and the same failure/depletion distinctions.

### 2.5 RawPart Observation Pose reference

Within the selected candidate, Vision identifies the observed top-facing
surface using support-relative height, surface normal, and consistency with
the configured workpiece profile. A robust plane fit may reject depth noise
and boundary outliers. The selected surface must contain sufficient valid
observations and coverage under profile-owned quality limits. Insufficient,
occluded, or geometrically inconsistent evidence for a stable position is a
geometry/temporary-observation failure, not success and not depletion.

For this RawPart profile only, the Observation Pose position is the centroid
of the selected target's observed visible top-surface points, expressed in
`base_link` after TF. This observed reference is not the workpiece's
volumetric center or a simulator-authored origin. For the declared 70 × 70 ×
80 mm RawPart geometry, Motion resolves Object Pose as the geometric center,
40 mm below this profile's Observation Pose along the local object axis. Vision
does not offset its Observation Pose for gripper opening/travel, finger
placement, TCP, or tool geometry. Motion then applies the configured
object-center-to-Target transform.

### 2.6 Optional object-centric yaw

Yaw is estimated from the selected candidate's support-plane projection and
fitted top-surface/outline geometry only when the configured profile provides
enough geometric evidence. For an elongated or otherwise directionally
observable profile, PCA over the planar inliers can provide an initial major
axis; fitted edges or the known asymmetric model resolve sign/axis ambiguity.
PCA is a replaceable estimator, not canonical behavior. A symmetry-aware
model fit or line/edge fit may replace it if it preserves observability
classification.

The estimator checks geometric conditioning, support, point coverage, model
fit, and symmetry/axis ambiguity against configured yaw-observability
constraints. If these conditions do not establish a stable object-centric
yaw, Vision SHALL preserve valid position, set `has_target_yaw=false`, and
leave orientation unusable for yaw resolution. It SHALL NOT publish arbitrary
PCA signs or noise-sensitive orientation as valid. If yaw is stable,
`has_target_yaw=true` and yaw is about `base_link` +Z. Recipe fallback remains
Orchestration-owned.

## 3. Validation, transformation, and result classification

Before success, Vision validates finite position/orientation values, positive
and profile-consistent object dimensions, support-relative height and contact
plausibility, sufficient top-surface evidence, configured workspace
membership, and observation freshness. Exact limits belong to profile
configuration. Invalid required position geometry cannot succeed. Invalid or
unobservable optional yaw does not invalidate otherwise valid position; it
sets `has_target_yaw=false`.

Vision then transforms the accepted optical-frame object reference to
`base_link` using the current valid TF for the observation time. Missing or
invalid TF returns `DETECT_RESULT_TF_ERROR`, not depletion. Success carries
the accepted observation stamp and the parent contract's object-reference
meaning.

| Condition | Outcome |
|---|---|
| Unknown/unconfigured `target_id` or profile | `DETECT_RESULT_INVALID_RESULT` |
| Valid profile, valid support and observation, no matching target | `DETECT_RESULT_OBJECT_NOT_FOUND` |
| Sensor synchronization, calibration, depth, or freshness failure | Parent sensor/timeout/temporary-invalid failure; never depletion |
| Fiducial/support estimation failure | `DETECT_RESULT_GEOMETRY_ERROR` or parent temporary-invalid failure; never depletion |
| Candidate ambiguity or temporary insufficient target evidence | Parent temporary-invalid/geometry failure; never depletion |
| Required selected-target position geometry invalid | `DETECT_RESULT_GEOMETRY_ERROR` |
| Optical-to-`base_link` transform invalid/unavailable | `DETECT_RESULT_TF_ERROR` |
| Valid position, stable yaw | `SUCCESS`, `has_target_yaw=true` |
| Valid position, yaw unobservable/unstable | `SUCCESS`, `has_target_yaw=false` |

Failure and depletion classification follow the parent
[DetectTarget FDS](fds-detect-target.md),
[Vision–Orchestration ICD](../../../interfaces/arm-cell/icd-vision-orchestration.md),
and the realized [DetectTarget result-code IDL](../../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/DetectTargetResultCode.msg).
Temporary observation-validity rejection uses the existing
`DETECT_RESULT_TEMPORARY_INVALID_TARGET=7` result. This RawPart FDS consumes
those current contract and interface definitions; it does not add or modify
result codes.

## 4. Ownership and replaceable boundaries

| Concern | Owner / replaceable boundary |
|---|---|
| Request, result contract, depletion, freshness boundary | Parent DetectTarget FDS and ICD |
| Support/work-surface invariants | Simulation Environment Semantics |
| Fiducial observation, support frame, depth geometry, RawPart candidate isolation, top-surface reference, optional yaw | This production RawPart profile FDS; implementation algorithms replaceable under the stated behavior |
| Target profile mapping, workpiece dimensions, thresholds, fiducial IDs/poses/physical geometry, ROI and quality limits | Deployment/configuration authority |
| Optical-to-`base_link` transform | Vision consumes TF; TF contract owner publishes it |
| Object-reference to executable grasp TCP and tool/gripper offsets | Motion |
| Batch retry/depletion progression | Orchestration |

## 5. Verification Requirements

Verification uses sensor observations/fixtures and declared configuration as
inputs and oracles. Numerical tolerances are declared by the applicable
configuration/fixture authority; this FDS does not set their values. The
profile-specific requirements below supplement the shared
DetectTarget behavior requirements. Simulator-ground-truth independence is
already owned by `VR-VIS-DETECT-13` and `VR-ARM-SIM-ENV-10`; profile selection
and unknown-profile outcomes by `VR-VIS-DETECT-06` and `VR-VIS-DETECT-07`;
depletion distinction by `VR-VIS-DETECT-01`, `VR-VIS-DETECT-08`, and
`VR-ICD-VIS-ORCH-02`; and object-reference/TCP separation by
`VR-VIS-DETECT-04`, `VR-VIS-DETECT-14`, and `VR-VIS-OWN-01`.

### VR-VIS-RAWPART-01 — Support reference success

Given a fresh valid RGB-D observation and a valid configured fiducial/support
observation, Vision SHALL establish a support-relative frame/plane consistent
with the declared sensor fixture within its declared tolerance.

### VR-VIS-RAWPART-02 — Support reference failure

When the required fiducial is missing, ambiguous, stale, or inconsistent, or
the support plane cannot be established, Vision SHALL fail without returning
`OBJECT_NOT_FOUND`.

### VR-VIS-RAWPART-03 — Depth-only target isolation

Given a configured RawPart profile and a valid support reference, target
candidate isolation and profile matching SHALL succeed from depth and known
geometry without requiring RGB appearance or a learned detector.

### VR-VIS-RAWPART-04 — Competing foreground rejection

Given the requested RawPart and one or more unrelated foreground objects,
Vision SHALL select only the requested profile match. Competing foreground
shall not shift, merge into, or redefine its selected geometry or reference
pose. If configured-profile candidates are geometrically ambiguous, Vision
shall report invalid/temporary perception rather than choose arbitrarily.

### VR-VIS-RAWPART-05 — Known workpiece geometry validation

Candidates inconsistent with the configured workpiece shape/dimensions or
support/workspace constraints SHALL be rejected and SHALL NOT contaminate the
selected RawPart geometry. Unknown/unconfigured profile outcome remains owned
by `VR-VIS-DETECT-07`.

### VR-VIS-RAWPART-06 — Observation-reference accuracy

For a selected target with a declared sensor observation fixture, the
successful position SHALL match the centroid of its selected observed visible
visible top-surface geometry within the fixture's declared tolerance, after
optical-to-`base_link` transformation.

### VR-VIS-RAWPART-07 — Yaw observable and unobservable cases

Stable, sufficiently constrained asymmetric geometry SHALL produce valid
object-centric yaw and `has_target_yaw=true`. Symmetric, occluded, insufficient,
or otherwise ambiguous geometry SHALL preserve valid xyz when available, set
`has_target_yaw=false`, and SHALL NOT expose arbitrary yaw as valid.
