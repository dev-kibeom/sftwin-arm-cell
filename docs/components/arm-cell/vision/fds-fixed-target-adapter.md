# [SF-Twin] ARM Cell Fixed Vision Validation Adapter Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VISION-FIXED-ADAPTER_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Vision / Deterministic PnP Validation Adapter`
- **Version:** `1.0.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Vision`

## 1. Purpose

The Fixed Vision validation adapter isolates deterministic Orchestration and
Motion verification from production perception accuracy. It preserves the
public `/vision/detect_target` service contract while returning a
fixture-registered object reference in a validation-only profile. It preserves
the ROS shape, result codes, frame, and downstream object-reference semantics
under the validation provenance overlay defined by the Vision↔Orchestration
ICD; it does not claim full production RGB-D observation semantics.

The adapter is a test double. It does not validate RGB-D processing,
segmentation, support-surface estimation, ArUco localization, or
`VR-VIS-DETECT-11`.

## 2. Process and Profile Separation

The adapter SHALL be built as a separate executable from the production
DetectTarget node and SHALL be started only by a dedicated PnP validation
launch/profile.

The production operational profile SHALL NOT expose a parameter, mode flag,
plugin selection, or launch argument that can replace the production detector
with this adapter. The validation profile SHALL NOT be accepted as a supported
production profile.

Both executables implement the same public `DetectTarget` service shape, but
they SHALL NOT run under the same service name in one process graph.

## 3. Fixture Registration

A running adapter SHALL accept repeated fixture-pose registrations without
process restart through a validation-only profile-local registration service or
equivalent channel with an explicit acknowledgement. Its logical request
contains:

- the configured `target_id`;
- a run-scoped `run_id`;
- a finite `geometry_msgs/PoseStamped` object reference in `base_link`;
- `has_target_yaw`, true only when the fixture supplies object-centric yaw;
- a requested registration TTL.

The adapter SHALL return a receipt sequence, adapter receipt time, and
accept/reject diagnostic. A rejected request SHALL NOT overwrite an accepted
record. The service is not a public product interface and SHALL NOT carry or
expose Isaac prim paths.

The configured `target_id` is the identity binding to the one
Vision-owned validation profile. Unknown or mismatched IDs are rejected and
never fall through to a generic foreground pose. Only one run registration may
be active at a time. A same-`run_id` valid re-registration atomically
replaces its unconsumed record; a different `run_id` is accepted only after
the previous record is consumed or expires.

Registration validation SHALL reject wrong frame, non-finite values, zero-norm
orientation, invalid yaw metadata, malformed requests, and TTL outside
`(0, 60s]`. The default TTL is **30 seconds**. TTL expiry uses the adapter's
steady/monotonic clock and receipt time, while the incoming pose stamp is
preserved only as fixture provenance and SHALL NOT be described as an RGB-D
observation stamp. If the adapter is unavailable, registration fails and the
fixture SHALL abort the run.

For the first cube scenario, the registered reference is the cube's visible
top-surface centroid: the center in x/y and the top surface in z. The fixture
orientation is object-centric; therefore `has_target_yaw=true` when that yaw
is supplied. If yaw is not supplied, `has_target_yaw=false`; the adapter
does not invent or apply a recipe fallback.

## 4. DetectTarget Behavior

For the validation profile's configured `target_id`:

1. no available registration returns `DETECT_RESULT_OBJECT_NOT_FOUND`;
2. a valid available registration returns `DETECT_RESULT_SUCCESS` with the
   registered object reference;
3. the successful registration is atomically consumed so the next request
   returns `OBJECT_NOT_FOUND` until a new fixture pose is registered.

An unknown `target_id` returns `DETECT_RESULT_INVALID_RESULT`. The adapter
SHALL NOT fall back to a generic pose or reuse a consumed registration.

The one-shot consumption rule supports one deterministic PICK→PLACE→GO_HOME
mission iteration followed by normal depletion. It is not an automatic retry
policy.

## 5. Failure and Freshness Rules

The adapter SHALL fail closed when the registration is malformed, expired under the accepted registration TTL (default 30 seconds, maximum 60 seconds), or in the wrong frame. It
SHALL distinguish invalid request/profile from configured-target absence.

A registration receipt is the freshness authority for the validation overlay;
the incoming pose stamp remains provenance only. A failed or canceled Motion
task does not restore a consumed Vision result. Re-running a scenario requires
an explicit new fixture registration. An unavailable adapter or missing receipt
is a registration failure, and the fixture SHALL NOT invoke Orchestration.

The validation adapter does not produce holding state; that state belongs to
the GripperPort/Isaac adapter seam.

## 6. Verification Requirements

### VR-VIS-FIXED-01 — Production isolation
The production launch/profile cannot select or start the Fixed Vision adapter.

### VR-VIS-FIXED-02 — Validation overlay preservation
A valid registration is returned through `/vision/detect_target` with the
public ROS shape, result/frame/object-reference semantics, fixture-registration
stamp, and no TCP/tool correction. It SHALL NOT be reported as an accepted
RGB-D observation.

### VR-VIS-FIXED-03 — Registration contract
Accepted registrations return a receipt, bind the configured `target_id` and
`run_id`, and preserve fixture provenance. Wrong identity, malformed input,
invalid TTL, unavailable adapter, or missing receipt cannot produce success.

### VR-VIS-FIXED-04 — Live re-registration
A same-run valid registration replaces its unconsumed record without
restarting the adapter; a new run is accepted only after the prior record is
consumed or expired.

### VR-VIS-FIXED-05 — One-shot depletion
A successful result consumes exactly one registration; the next request
returns `OBJECT_NOT_FOUND` until re-registration.

### VR-VIS-FIXED-06 — Fail-closed input
Wrong-frame, non-finite, zero-norm-orientation, stale, and unknown-target
inputs cannot produce success.
