# [SF-Twin] ARM Cell Deterministic PnP Fixture Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-INTEGRATION-PNP-FIXTURE_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Integration / Isaac Deterministic PnP Validation`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Integration`

## 1. Purpose

This behavior defines a deterministic Isaac validation fixture for one
PICK→PLACE→GO_HOME iteration. It verifies coordinate propagation, Motion-owned
object-reference-to-TCP conversion, backend-neutral gripper behavior,
simulator attachment/detachment, and return Home while production perception
is replaced by the Fixed Vision validation adapter.

The fixture does not establish production Vision accuracy or physical
placement stability.

## 2. Runtime Roles

### 2.1 Persistent Isaac gripper adapter

The persistent Isaac gripper adapter realizes the backend-neutral
`GripperPort` contract. It owns simulator-specific actuator commands,
capture-volume evaluation, grasp confirmation, fixed-joint creation/removal,
and holding-state reporting.

It SHALL NOT own Motion task sequencing, target-ID resolution, mission policy,
or Vision pose generation.

### 2.2 Per-run test fixture

The PnP test fixture owns one scenario run:

- create one eligible gravity-disabled cube at a seeded random pose within the
  approved spawn volume on the material shelf/AMR top;
- compute the cube's visible top-surface centroid in `base_link`;
- register that pose with the already-running Fixed Vision adapter;
- preserve the registered object pose within the declared validation tolerance
  from accepted registration until capture is accepted;
- invoke and observe the supported Orchestration mission path;
- record the seed, spawned pose, registered reference, selected eligible prim,
  attach/detach observations, place observation, Home observation, and result;
- schedule prim deletion three seconds after Motion reports successful PLACE, after confirmed detachment.

Scene setup SHALL not contain the former persistent RawPart. Test material is
created and removed only by the fixture.

After Fixed Vision registration is accepted, the validation fixture SHALL
maintain the registered object pose within the declared validation tolerance
until PICK capture is accepted. Gravity and incidental/non-grasp contact SHALL
not invalidate the registered pose during this pre-capture interval. The
simulator-specific realization used to provide this stability is owned by the
validation runtime/profile and is not part of this functional contract.

When capture predicates pass, the adapter SHALL create its adapter-owned
attachment before releasing pre-capture fixture-stability ownership. Capture
is accepted only after that release succeeds and the ownership transition is
complete; only then may the adapter publish a fresh `HELD` observation. If the
release fails, the adapter SHALL remove the just-created adapter-owned
attachment, SHALL NOT publish `HELD`, and SHALL fail closed as `UNKNOWN` or a
grasp failure.

### 2.3 Fixture registration contract

The fixture SHALL use a profile-local registration service/channel with an
explicit acknowledgement. Its logical request contains:

- the configured `target_id`;
- a run-scoped `run_id`;
- a finite `geometry_msgs/PoseStamped` object reference;
- `has_target_yaw`, set true only when the fixture supplies object yaw;
- a requested registration TTL.

The adapter SHALL acknowledge accepted registrations with a receipt sequence and
adapter receipt time. A rejected request SHALL include a diagnostic reason and
MUST NOT overwrite the last accepted record. The registration channel is
validation infrastructure, not a shared product interface, and SHALL NOT carry
or expose Isaac prim paths to Vision.

Registration identity and lifecycle rules:

- the request `target_id` SHALL match the one target profile configured for
  the validation profile; an unknown or mismatched ID is rejected;
- only one run registration may be active at a time;
- a valid re-registration for the same `run_id` atomically replaces its
  unconsumed record;
- a different `run_id` is accepted only after the prior record was consumed
  or expired, so a delayed request cannot receive a later run's pose;
- malformed frame, non-finite pose, zero-norm orientation, invalid yaw flag,
  or invalid TTL is rejected without replacement;
- the default registration TTL is **30 seconds** and the maximum accepted TTL
  is **60 seconds**; expiry uses the adapter's steady/monotonic clock;
- the pose stamp is preserved as fixture provenance and SHALL NOT be described
  as an RGB-D observation stamp; TTL freshness uses adapter receipt time;
- an unavailable adapter returns a registration failure and the fixture SHALL
  abort the run rather than invoke Orchestration.

## 3. Reproducibility and Spawn Bounds

Each run SHALL accept or generate a seed and SHALL record it in the run
manifest. Replaying the seed with the same fixture/profile version SHALL
produce the same spawn pose.

Validation geometry frame provenance is explicit and authoritative:

- `fixture.spawn_bounds_m` and `fixture.excluded_volumes_m` are expressed in
  `world`;
- post-release placement observation compares the detached fixture center in
  `world` with the expected release center derived from the configured PLACE
  TCP pose and the existing `object_to_grasp_tcp` transform;
- a sampled spawn pose is passed to the simulator as a world pose without an
  additional base-to-world transform;
- the actual simulator world pose is transformed to `base_link` exactly once
  when producing the registered top-surface reference;
- excluded-volume comparisons SHALL use actual poses in the declared geometry
  frame. The post-release placement observation uses Euclidean center-to-center
  distance in `world`, bounded by `place.position_tolerance_m`; it does not test
  the full fixture footprint or orientation. Missing or non-finite pose data
  produces an unavailable observation and does not reverse PLACE motion success.

The registration handoff remains a `base_link` pose contract. Durable manifests
and evidence SHALL preserve explicit pose names and frame IDs, including
`spawn_pose_world` and `registered_top_surface_pose_base_link`.

The PlanningScene fixture representation uses a different geometric reference:
the `BOX` primitive pose SHALL be its geometric center. The validation ROS
seam SHALL derive that center from the registered top-surface centroid using
the declared fixture dimensions and orientation, translating by one half
fixture height along the local negative vertical axis. The registered
top-surface pose SHALL remain unchanged for Fixed Vision and orchestration;
this conversion is limited to the validation collision-object realization.

For the canonical 0.08 m axis-aligned fixture, a registered top z of 0.0635 m
therefore produces a PlanningScene box-center z of 0.0235 m. A top reference
MUST NOT be passed directly as a box-center pose. In the ROS MoveIt message,
the derived center belongs in `CollisionObject.pose`; the box primitive pose
remains identity relative to that object pose.

The spawn sampler SHALL use declared position and yaw bounds that are fully
inside the configured reachable PICK region. It SHALL reject samples that
intersect excluded geometry or violate the fixture's declared clearance.

The cube SHALL have gravity disabled for this phase. Gravity-disabled behavior
is a validation-profile characteristic and SHALL be recorded; it is not the
sole fixture-stability mechanism, does not imply that the fixture may remain a
free-floating dynamic body before capture, and is not evidence of physical
support or settling.

Step 4N2 live diagnosis established that a fresh `RELEASED` status can coexist
with a physically closed gripper (`left_knuckle` near its closed limit). This
is not a mimic-joint mapping failure: the authoritative parent joint and
derived mimic state remain consistent. Validation start therefore treats
physical OPEN readiness as a separate prerequisite.

Before fixture creation, the validation-only bootstrap SHALL issue an `open()`
command, wait for a newer physical joint-state sample, and require the
authoritative gripper parent joint to be in the canonical open region with a
consistent measured opening. It SHALL also require a fresh `RELEASED`
observation, `attached=false`, and no adapter-owned grasp joint. `RELEASED`
means no object attachment/holding; it does not prove that the fingers are
physically open. Failure of any precondition SHALL prevent fixture creation,
registration, and mission start. An unknown or stale physical sample SHALL
fail closed.

The OPEN readiness wall-clock budget is the validation-only profile value
`open_readiness_timeout_s` (canonical value: 2 seconds). It is independent of
`holding_freshness_ms`, which governs `HELD`/`RELEASED` observation freshness.
Waiting for a newer physics generation or for an active OPEN motion to settle
is transient. UNKNOWN, stale `RELEASED`, attachment, missing feedback,
non-finite parent position, and settled-but-closed parent position are terminal
fail-closed outcomes and are not retried until timeout.

## 4. Backend-Neutral GripperPort

Motion SHALL use only the following logical semantics:

- `close(grasp_width_mm)` requests closing;
- `open()` requests release;
- `holding()` returns a fresh observation with state `HELD`, `RELEASED`, or
  `UNKNOWN`, plus monotonic freshness metadata. `RELEASED` is an
  attachment/holding semantic and is not an OPEN-finger observation;
- `active()` and `stop()` support interruption and safe shutdown.

Motion SHALL not command simulator attachment/detachment directly and SHALL
not receive Isaac prim paths. PICK may continue to retract only after a fresh
`HELD` observation. PLACE may continue to retreat only after a fresh
`RELEASED` observation following `open()`; `UNKNOWN` blocks both transitions.

A real gripper adapter may establish the observation from hardware sensing. The
Isaac adapter establishes it from the snap policy below. Command success alone
never implies `HELD` or `RELEASED`.

## 5. Isaac Snap Policy

The fixture and adapter SHALL expose a fresh backend-neutral holding
observation with one of `HELD`, `RELEASED`, or `UNKNOWN`. A holding
observation older than the validation profile's **500 ms** freshness bound is
treated as `UNKNOWN`. This observation timeout is independent of the
30-second Vision registration TTL.

An object may be attached only while a close command is active and all of the
following are true:

- the object is registered as eligible by the active fixture;
- the object's declared grasp frame lies inside the TCP capture volume;
- TCP-to-grasp-frame position error is within the configured tolerance;
- orientation error is within the configured tolerance;
- `abs(measured_opening_mm - expected_contact_width_mm) <=
  contact_width_tolerance_mm`.

The fixture SHALL register a positive
`expected_contact_width_mm` for each eligible cube. The validation profile
SHALL provide a positive, explicit `contact_width_tolerance_mm`. Both values
use millimeters and SHALL be recorded in the run manifest. Equality at the
tolerance boundary is accepted; a larger error is rejected.

The adapter SHALL evaluate actual scene geometry around the TCP. It SHALL NOT
search for a distant prim using `target_id`, a configured prim path, or the
Vision result identity.

Exactly one eligible object SHALL satisfy the capture predicate. After
registration and before capture acceptance, the registered fixture pose SHALL
remain within the declared validation tolerance despite incidental contact.
The capture ownership sequence is:

```text
registration accepted
→ pre-capture pose stability maintained
→ capture predicates pass
→ adapter-owned attachment created
→ pre-capture stability ownership released
→ ownership transition complete / capture accepted
→ fresh HELD observation
→ retract
```

Zero
candidates means `RELEASED` only when the adapter has a fresh confirmed
non-holding observation; otherwise the state is `UNKNOWN`. Multiple
candidates are ambiguous and SHALL fail closed with `UNKNOWN` without
attachment. The adapter-owned attachment and fresh `HELD` observation are
authoritative only after the ownership transition above has completed.

Position/orientation tolerances, `contact_width_tolerance_mm`, capture-volume
dimensions, and the 500 ms holding freshness bound are validation-profile
configuration. They SHALL be positive, explicit, and recorded in evidence;
missing or invalid values SHALL prevent the adapter from becoming ready.

## 6. PLACE and Cleanup

PLACE motion success covers accepted release TCP motion and detachment.
Detached fixture placement and settling are post-release observations, not
acceptance gates:

1. Motion reaches the accepted PLACE/release TCP target under existing tracking
   and approach acceptance;
2. Motion calls `open()` and the Isaac adapter removes the fixed joint;
3. a fresh `RELEASED` observation and confirmed PlanningScene detach establish
   PLACE motion success;
4. gravity is enabled immediately after PLACE motion success; fixture
   fall/settling is post-release simulator behavior;
5. a missing, stale, disconnected, ambiguous, or unexpected joint/prim
   observation becomes `UNKNOWN` and blocks retreat/success;
6. after the configured cleanup delay, MoveIt removes the fixture and verifies
   PlanningScene `ABSENT`; Isaac marks the still-owned prim pending physical
   deletion and removes it from capture candidates;
7. the Isaac prim is physically deleted only after the mission reports
   `FINISHED` for normal depletion, which follows successful GO_HOME.

The fixture SHALL cancel pending cleanup safely on run abort and SHALL not
delete an attached, ambiguous, or `UNKNOWN` prim. Fixture center distance
within `place.position_tolerance_m` is a post-release diagnostic only. Physical
placement quality acceptance is out of scope; contact, bounce, and stable
support remain simulator observations.

## 7. GO_HOME Completion

After PLACE succeeds, the existing mission flow SHALL execute GO_HOME. Success
requires measured robot state within the validation profile's Home joint
tolerance and a fresh `RELEASED` observation. A mission result alone is
insufficient without these observations.

## 8. Calibration Boundary

The simulator validation profile may establish only the
object-reference-to-grasp-TCP transform, approach/retract values, Home
tolerance, capture volume, and snap tolerances.

These values SHALL be explicit and traceable to the run manifest. Zero/default
fallbacks are not accepted. Evidence from this fixture SHALL NOT be promoted
to production camera, robot, tool, AMR, or physical gripper calibration.

For the validation-only 70 mm recipe, the canonical `sf_grasp_tcp` reference
is derived from the midpoint of the inward-facing fingertip contact surfaces
at the nominal grasp configuration. The cube top-surface centroid remains the
object reference. In the canonical Robotiq model, nominal mimic
`q=0.176470588` produces the validation-only gripper-base-relative TCP origin
`[0, -0.0003749994, 0.1357482403] m` from the actual collision-surface
midpoint; this is geometry provenance, not production calibration. The
corrected-orientation contact-band scan selects a `-0.0025 m` object-top to
grasp-TCP recipe point for this validation fixture. This does not
change production tool-frame ownership or establish a production gripper
calibration.

The corrected validation tool convention is explicit. The physical Robotiq
body-to-contact direction is TCP local `+Z`; the validation top-down
orientation is a 180-degree rotation about TCP `X`, so TCP `+Z` resolves to
world `-Z` while TCP `+X` remains the bilateral closing axis. Accordingly,
`insertion_axis_tcp` is `[0, 0, +1]`, the approach retreat resolves to world
`+Z`, and fixture grasp yaw is the fixture yaw modulo `pi`.

MoveIt dry-run evidence measured a corrected-orientation fingertip-only
contact band of `[-0.024, +0.019] m` for the validation fixture. The selected
fixture recipe point is `-0.0025 m` from the object top to the grasp TCP, with
21.5 mm to each measured boundary. These are validation geometry and
PlanningScene observations, not universal Robotiq specifications or live
acceptance evidence.

During validation grasp planning only, the PlanningScene may temporarily
allow the current validation fixture object to contact exactly the left and
right fingertip links. Inner/outer knuckles, the gripper base, arm links,
pedestal, and self-collision remain disallowed. The policy is restored after
the grasp planning operation and is not available to production launches.

Failure to provide or validate the simulator profile values blocks acceptance
of this validation scenario; it does not authorize Motion to absorb Vision
error or the Isaac adapter to widen tolerances dynamically.

## 9. Non-Goals

This phase does not validate:

- RGB-D target detection or visible-surface centroid tolerance;
- ArUco marker detection or AMR support-depth estimation;
- production Depth Vision profiles;
- physical placement stability under gravity;
- safety stop/recovery behavior;
- hardware or production calibration.

## 10. Verification Requirements

### VR-INT-PNP-01 — Seeded fixture lifecycle
A seed determines the spawn pose, the manifest records it, and the fixture
removes only its own prim from PlanningScene after the successful cleanup
delay. Isaac keeps the detached, gravity-disabled prim retired and outside the
capture-candidate registry during RETRACT/GO_HOME, then physically deletes it
after GO_HOME succeeds and the mission reports normal depletion. Both the
semantic scene removal and final physical deletion are idempotent.

### VR-INT-PNP-02 — Capture-local attachment
Only one eligible object satisfying close, capture-volume, pose, orientation,
and the declared millimeter width comparison can attach; distant, ineligible,
ambiguous, and out-of-width-tolerance objects cannot attach. Attachment emits
fresh `HELD`; confirmed detach emits fresh `RELEASED`; anomalies emit `UNKNOWN`.

### VR-INT-PNP-03 — Backend insulation
Motion observes only `GripperPort` semantics and never an Isaac prim path,
fixed-joint identifier, or capture-volume implementation.

### VR-INT-PNP-04 — PICK ordering
Retraction and PICK success cannot occur before a fresh `HELD` observation. A
fresh `UNKNOWN` observation blocks retraction and success.

### VR-INT-PNP-05 — PLACE ordering
PLACE motion success requires accepted release TCP/motion tracking and approach
acceptance, fresh `RELEASED`, and confirmed physical detach; `UNKNOWN` blocks
retreat and success. Gravity is enabled immediately after that result. Cleanup
requires confirmed detach, unambiguous fixture ownership, lifecycle safety, and
mission completion/depletion conditions. The detached fixture center distance
from the expected release center is recorded as a separate post-release
observation against `place.position_tolerance_m`; it does not gate PLACE success
or cleanup. The expected center is derived from
`orchestration.mission_place_pose_xyz` / quaternion using the inverse existing
`motion.object_to_grasp_tcp_*` transform. This remains specific to the
validation scenario, not production placement-quality validation.

### VR-INT-PNP-06 — Home observation
The scenario passes only when GO_HOME follows PLACE and measured joints are
within the declared Home tolerance with no held object.

### VR-INT-PNP-07 — Scoped claim
The resulting evidence identifies gravity-disabled, fixed-pose Vision and
simulator-profile calibration limitations and makes no perception, physical
placement, safety, or hardware claim.

### VR-INT-PNP-08 — Pre-capture stability and ownership transition
After registration is accepted and before capture is accepted, the fixture's
registered object pose remains within the declared validation tolerance despite
incidental/non-grasp contact, and the registered pose remains valid for the
capture decision. Capture acceptance is reported only after the adapter-owned
attachment is created and pre-capture stability ownership is released. A
release failure removes the newly created attachment, emits no `HELD`, and
fails closed. After acceptance, the existing adapter-owned attachment and
fresh `HELD`/`RELEASED` semantics govern the object.
