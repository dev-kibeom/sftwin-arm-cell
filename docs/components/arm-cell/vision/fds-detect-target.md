# [SF-Twin] ARM Cell Vision DetectTarget Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VISION-DETECT-TARGET_v1.1.0`
- **Document Type:** `FDS`
- **Scope:** `Vision / DetectTarget Processing`
- **Version:** `1.1.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Vision`

## 1. Processing Flow

```text
DetectTarget request
→ fresh synchronized RGB-D + valid CameraInfo
→ resolve Vision detector profile by target_id
→ invalid/unconfigured profile? → INVALID_RESULT
→ interpret the observation under the selected profile
→ no match in a valid observation? → OBJECT_NOT_FOUND
→ derive and validate the profile-defined object reference
→ determine whether optional target yaw is observable
→ physical and freshness validation
→ produce observation-linked diagnostic semantics
→ TF optical → base_link
→ success
```

These stages define profile-independent DetectTarget behavior and result
semantics. They do not prescribe a profile's internal algorithm or ordering
for support estimation, depth transformation, candidate generation, target
isolation, or geometry fitting. Profile-specific stages and their ordering
are owned by the subordinate profile FDS, including the
[RawPart depth-perception FDS](fds-rawpart-depth-perception.md). A profile
refines this behavior without changing the service result, freshness, frame,
or Observation Pose contract below.

The caller supplies the relative `builtin_interfaces/Duration` timeout defined by [Vision ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-vision-orchestration.md). Vision starts its steady monotonic deadline when it receives that request; the timeout is not an absolute ROS-time or wall-clock deadline.

Production detection remains request-driven. A continuously published camera
stream only makes observations available; it does not keep the selected
detector profile processing every frame while no request is active. For an
active request, Vision selects a recent synchronized input under
[`fds-frame-ingress.md`](fds-frame-ingress.md), re-runs the selected detector
profile on that source RGB-D pair, returns the existing canonical result, and
becomes idle until the next request. Reusing a recent input frame does not
reuse a prior detection result or change the source identity reported by the
new detection.

## 1.1 Observation-linked diagnostic semantics

Vision owns diagnostic perception meaning used by operator annotation. The
overlay source identity is the RGB image actually used for the detection.
Diagnostic metadata SHALL preserve or reference that RGB image's original
`header.stamp` and `frame_id`. For an accepted RGB-D observation, Vision SHALL
retain which RGB frame was used; the associated depth timestamp participates
in bounded RGB-D association but SHALL NOT replace the RGB overlay matching
key. Where available from the selected detector profile and current result,
metadata may include detection active/idle, `target_id`, support/reference region, segmented object region or
extent, centroid, selected target pixel, validity/freshness/result, and yaw
detected or unavailable. These are semantic examples, not a requirement to
manufacture geometry that a detector did not produce. A bounding box,
polygon, or mask may be exposed only when the detector actually provides that
representation; Hub presentation must not imply unavailable perception.

CameraInfo compatibility and RGB/depth temporal association follow the
frame-ingress contract. Operator overlay matching uses the original RGB
identity, not the paired depth timestamp. The Hub's directly displayed raw
RGB frame retains its own capture `header.stamp` and `frame_id`. For operator
visualization, the Hub selects the nearest same-camera frame within its
bounded temporal display tolerance; that image and annotation are not
perception, control, safety, pose, or success inputs. This presentation rule
does not change Vision's exact source-observation identity or RGB-D
association. Matching only by timestamp across different cameras is
insufficient. A consumer must suppress incompatible/stale annotation or
identify it as stale/unavailable.

Vision processing state is `IDLE` or `DETECTING`. Each DetectTarget request
transitions from `IDLE` through `DETECTING` and publishes its terminal result
with the processing state returned to `IDLE`. A successful terminal diagnostic
is an observation event containing the actual source RGB identity and detector
metadata; it is not a persistent `TARGET_LOCKED` Vision state. The Hub owns
whether and how long that successful observation remains presented. Missing
or stale diagnostics are unavailable/stale and must not be inferred as `IDLE`
or a successful result.

These diagnostic semantics do not alter the public DetectTarget result
contract. The production implementation exposes them on the internal,
read-only `/vision/diagnostics` topic using `VisionDiagnostic`. This boundary
is separate from the public service so source-camera identity and detector
regions do not enlarge DetectTarget. The diagnostic's `source_rgb_header` is
copied from the synchronized observation's RGB image; `result_code`, validity,
target ID, yaw availability, selected top-surface centroid pixel, the actual
connected depth-candidate extent, and observed support-fiducial corners are
published only when those values exist. Consumers evaluate diagnostic receipt
freshness against their local monotonic clock. Publication failure on this
diagnostic path must not change the canonical DetectTarget result. The current
public result's `target_pose.header.frame_id` is `base_link`; its observation timestamp alone
does not identify the source camera frame. Therefore, the existing result
fields alone remain insufficient to prove source-camera compatibility when
the Hub matches annotation to operator video.

On a terminal timeout, `/vision/diagnostics.diagnostic_detail` identifies the
most specific observed acquisition or processing stage. The diagnostic also
reports latest RGB, depth, and CameraInfo receipt ages and the latest RGB-depth
timestamp delta when available. These values are diagnostic evidence only;
they do not change freshness acceptance or source identity. CameraInfo remains
cached calibration state and need not share an image timestamp.

## 2. Target Profile Selection

`DetectTarget.target_id` SHALL select a deployment-configured, Vision-owned
detector profile. The service boundary SHALL pass the request identity through
to profile resolution and processing. An unknown or unconfigured ID SHALL
return `DETECT_RESULT_INVALID_RESULT` without falling back to generic
foreground segmentation. A valid configured profile that finds no matching
target SHALL return `DETECT_RESULT_OBJECT_NOT_FOUND`.

Deployment configuration owns the target-ID-to-profile mapping and
profile-specific parameter values. In the supported operational profile, the
configuration schema is a `vision.detector_profiles` mapping keyed by logical
target ID. Each entry has this logical structure:

| Entry | Requirement | Contents |
|---|---|---|
| `selection_constraints` | Required | Deployment/profile-specific constraints over sensor observations used to isolate the requested target; may include a workspace ROI, connected depth components, observed appearance, or equivalent perception-owned evidence. |
| `geometry_constraints` | Required | Profile-specific size, extent, and other target-geometry validity limits. |
| `yaw_observability_constraints` | Optional | Profile-specific limits for deciding whether selected geometry supports valid object yaw. |

The mapping and values are deployment configuration. Nested constraint
parameters are defined by the selected Vision profile; neither this behavior
contract nor the public ICD fixes a particular algorithm or numeric
threshold.

The RawPart profile SHALL apply its perception-owned constraints to isolate
the requested observation from competing foreground geometry. The environment
preconditions that make this isolation possible, and the cross-cutting
independence from simulator ground truth, are owned by [ARM Cell Simulation
Environment Semantics](../simulation-environment-semantics.md). This FDS owns
the target-profile processing behavior over sensor observations.

## 3. Depletion Semantics

`OBJECT_NOT_FOUND` is normal depletion evidence.

It SHALL remain distinct from sensor, calibration, TF, or geometric-quality failure.

## 4. Geometry

Vision may use RANSAC/PCA or equivalent approved algorithms to derive:

- support/surface relation;
- object height;
- the geometric reference declared by the selected profile;
- grasp-reference yaw;
- quality metrics.

Exact internal algorithm implementation is not canonical unless required by a Verification Requirement.

`target_pose` SHALL report the selected profile's Observation Pose reference
transformed into `base_link`, without robot/tool/TCP correction. A profile may
define that reference from a visible surface, an object center, or another
sensor-derived geometric feature. For RawPart, the subordinate profile FDS
defines the visible top-surface centroid; that meaning does not generalize to
other profiles. Motion owns the conversion from Observation Pose to Object
Pose and grasp TCP geometry.

`estimated_height_m` is a separate non-authoritative estimate. It SHALL NOT
modify, repair, offset, replace, or otherwise participate in returned
`target_pose.position.x`, `.y`, or `.z`.

## 5. Yaw Ambiguity

When object geometry does not support stable yaw estimation, Vision SHALL NOT
report an arbitrary unstable orientation as a valid yaw. Vision MAY publish a
successful object result without a valid yaw; in that case Orchestration SHALL
apply the target-specific recipe yaw fallback when resolving a PICK input. The
`has_target_yaw=false` response semantic SHALL represent that condition.
Vision remains responsible for filtering invalid optional geometry before
publication.

## 6. TF

Successful pose output requires a valid transform from camera optical frame to `base_link`.

Vision consumes TF; it does not publish competing camera/robot TF.

## 7. Verification Requirements

### VR-VIS-DETECT-01 — Depletion distinction
Only `OBJECT_NOT_FOUND` SHALL represent normal depletion.

### VR-VIS-DETECT-02 — Geometry failure classification
Insufficient/invalid geometry for the required object position SHALL NOT be
returned as successful detection. If object position is valid but yaw is not
observable, Vision SHALL return a successful result with `has_target_yaw=false`.

### VR-VIS-DETECT-03 — TF failure classification
Unavailable/invalid optical→base transform SHALL produce a Vision failure outcome.

### VR-VIS-DETECT-04 — Detector-profile observation reference
Successful target pose SHALL represent the selected detector profile's declared
3D observation reference in `base_link`. It SHALL NOT imply an object-center or
grasp-TCP pose unless that profile explicitly defines that reference.

### VR-VIS-DETECT-05 — Yaw ambiguity
Given object geometry with unstable or unobservable yaw, a successful result
SHALL NOT contain an arbitrary orientation presented as a valid yaw. Vision
MAY omit the valid yaw while retaining a valid observed position; Orchestration
then SHALL use the target-specific recipe yaw for PICK resolution after the
approved explicit yaw-presence interface change. This fallback does not
authorize Vision to apply robot/tool/TCP compensation.

### VR-VIS-DETECT-06 — Target ID selects profile
For requests with different configured `target_id` values, the service SHALL
forward each ID to profile resolution, and processing SHALL use the matching
profile rather than ignoring the ID or using a global foreground set.

### VR-VIS-DETECT-07 — Unknown profile result
An unknown or unconfigured `target_id` SHALL return
`DETECT_RESULT_INVALID_RESULT` and SHALL NOT be mapped to
`DETECT_RESULT_OBJECT_NOT_FOUND`.

### VR-VIS-DETECT-08 — Configured profile depletion
A valid configured profile with no matching target SHALL return
`DETECT_RESULT_OBJECT_NOT_FOUND`.

### VR-VIS-DETECT-09 — Competing foreground isolation
When a valid requested target and one or more unrelated foreground objects
are present, the selected target pose SHALL be derived from the requested
target only; unrelated foreground geometry SHALL NOT shift or redefine it.

### VR-VIS-DETECT-10 — RawPart profile selection
The configured RawPart profile SHALL select the observed RawPart target in
the approved acceptance observation, reject competing foreground geometry,
and report object-not-found when the RawPart observation is absent.

### VR-VIS-DETECT-11 — Profile-defined Observation Pose
For a selected target with a known sensor-observation fixture, successful
position SHALL match the selected profile's declared sensor-derived geometric
reference within the fixture's declared tolerance, transformed into
`base_link`. RawPart profile behavior is verified by its subordinate FDS.

### VR-VIS-DETECT-12 — Yaw presence and absence
Stable selected surface geometry SHALL produce valid object-centric yaw with
`has_target_yaw=true`; ambiguous/unobservable yaw SHALL preserve valid xyz,
set `has_target_yaw=false`, and not publish arbitrary yaw as valid.

### VR-VIS-DETECT-13 — Environment semantics reference
Production Vision independence from simulator ground truth is owned and
verified by `VR-ARM-SIM-ENV-10` in [ARM Cell Simulation Environment
Semantics](../simulation-environment-semantics.md).

### VR-VIS-DETECT-14 — No TCP/tool correction
Changing or removing robot/tool geometry SHALL NOT change the Vision
object-reference result. Vision SHALL NOT apply TCP/tool correction.

### VR-VIS-DETECT-15 — Estimated-height non-authority
Changing, omitting, or varying `estimated_height_m` SHALL NOT change returned
target xyz/z. The returned position SHALL remain the selected detector
profile's Observation Pose after the optical-to-`base_link` transform.
