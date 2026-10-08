# [SF-Twin] ARM Cell Motion Component Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-MOTION_v1.2.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / Motion Adapter`
- **Version:** `1.2.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Motion`

## 1. Purpose

Motion converts task-level robot/tool-independent intent into robot/tool execution behavior while independently enforcing Safety-provided motion capability.

## 2. Responsibilities

Motion SHALL own:

- ARM Cell motion planning/execution;
- Observation Pose → Object Pose → Target Pose conversion;
- Motion-owned tool/TCP/approach/retract offsets;
- PICK/PLACE/GO_HOME/RETRACT execution;
- MotionStatus publication;
- Motion-side ObjectState / Planning Scene synchronization;
- backend-neutral `GripperPort` use for logical close, open, holding,
  activity, and stop semantics;
- Safety capability enforcement at goal acceptance;
- StopMotion handling and actual stop-state reporting.

## 3. Non-Responsibilities

Motion SHALL NOT own:

- mission sequencing or retry policy;
- Safety capability authority;
- Safety interlock evaluation;
- Vision geometry estimation;
- simulator capture-volume evaluation, prim selection, or fixed-joint
  creation/removal;
- safety-rated E-Stop/STO authority.

Motion MUST NOT treat Orchestration as a trusted safety boundary.

## 3.1 GripperPort Boundary

Motion task sequencing depends on a backend-neutral `GripperPort`. Motion
commands logical close/open operations and observes a fresh
`HoldingObservation` with `HELD`, `RELEASED`, or `UNKNOWN`; it does not command
attachment or detachment directly.

Simulator adapters may realize holding through a capture-local snap policy, and
hardware adapters may use physical sensing. Backend-specific prim identity,
joint topology, contact implementation, and sensor details SHALL remain below
the port.

## 4. Observation-to-Target Pose Boundary

Vision supplies an Observation Pose in `base_link`. Its geometric reference is
defined by the selected Vision detector profile or its governing contract.
Motion does not reinterpret the Observation Pose as a universal visible
surface or object center.

Motion alone applies robot/tool-specific correction such as:

- TCP/gripper offset;
- tool geometry correction;
- approach/retract distance;
- Motion-owned planning/collision margin.

### 4.1 Resolved PICK input

For PICK, Orchestration SHALL resolve the logical input before submitting the
Motion goal:

- Observation Pose position `x`, `y`, and `z` SHALL come from the valid Vision
  result unchanged;
- yaw SHALL come from Vision only when the explicit yaw-presence semantic is
  true, otherwise from the target-specific recipe `grasp_yaw_rad`;
- logical `grasp_width_mm` SHALL come from the target recipe;
- the Motion goal SHALL carry the Observation Pose forwarded by Orchestration;
- `estimated_object_height_m` SHALL be non-authoritative and unused for PICK
  geometry. It SHALL NOT modify, repair, offset, replace, or otherwise
  participate in Observation-Pose-to-Object-Pose or Target-Pose generation.

Vision SHALL filter low-confidence or otherwise invalid optional geometry before
publication. Orchestration SHALL own this Vision/recipe merge and SHALL NOT
perform tool or TCP geometry calculations.

### 4.2 PICK Observation and grasp geometry contract

Motion SHALL resolve a concrete Object Pose from the Observation Pose using a
configured transform derived from the selected detector profile's observation
reference and object geometry:

```text
T_base_object = T_base_observation * T_observation_object
T_base_target = T_base_object * T_object_to_grasp_tcp
```

For the production RawPart detector profile, the Observation Pose is the
visible top-surface centroid. Given its 70 × 70 × 80 mm geometry, the configured
Observation-to-Object transform resolves the geometric center 40 mm below the
Observation Pose along the local object axis. This is RawPart profile behavior,
not generic Vision semantics.

`T_observation_object` and `T_object_to_grasp_tcp` are external
Motion-owned/profile-derived configuration. Their translations are expressed
in their respective source frames. The target's nominal yaw is the resolved
Vision yaw when present, otherwise the recipe fallback. Vision SHALL NOT
include TCP/tool compensation.

### 4.3 PLACE Desired Object Pose realization

For PLACE, the Motion goal carries the recipe's Desired Object Pose, as defined
by the [Mission Cycle FDS](../orchestration/fds-mission-cycle.md), directly in
`base_link`; Motion SHALL NOT apply detector-specific Observation-to-Object
conversion. While the object is freshly confirmed `HELD`, Motion SHALL use the
accepted rigid Object-to-TCP grasp relation to compose an executable TCP target.
The relation is not used after fresh `RELEASED` confirmation.

Recipes express PLACE orientation constraint mode as `fixed`, `bounded`, or
`free`. `fixed` requires the nominal Desired Object Pose orientation. `bounded`
allows candidates within the nominal orientation tolerance. `free` means
object orientation is not a process constraint; Motion creates a small,
deterministic set of representative orientations (nominal and fixed quarter
turns about object axes), including side-on poses. This is bounded candidate
search, not exhaustive search or a global optimum guarantee. Existing position
and position-tolerance semantics remain unchanged. Candidates outside
applicable constraints and collision/IK-invalid candidates SHALL be removed
before ranking. For `fixed`/`bounded`, an optional tool/TCP orientation
preference may rank feasible candidates within the object constraint. For
`free`, tool preference is ignored and feasible candidates are ranked by joint
movement, time-parameterized trajectory duration, then stable candidate order.
The existing bounded planning budget applies. These process constraints do not
alter Safety authority, execution completion tolerances, or post-release
verification. This demo path does not verify a settled post-release Object
Pose.

The recipe schema retains `orientation_tolerance_rad` for compatibility. It
has no effect when `orientation_constraint` is `free`.

Motion SHALL distinguish these poses:

- Observation Pose: the detector-profile-defined measured reference in
  `base_link`;
- Object Pose: the geometry-resolved object pose in `base_link`;
- Target Pose: the fixed xyz pose at which `sf_grasp_tcp` closes;
- Approach Pose: Target Pose displaced opposite the selected TCP-local
  insertion axis by the configured approach distance.

Position candidates retain the existing bounded position-tolerance behavior.
Orientation candidates follow the recipe mode: exactly nominal for `fixed`,
within tolerance for `bounded`, and the bounded deterministic representative
set for `free`. The `free` set includes quarter turns about object X and Y so
side-on object poses can be evaluated. MoveIt IK and collision validity SHALL
filter infeasible candidates.
Motion SHALL calculate TCP preference error from each candidate's computed TCP
orientation after composing its object pose with the accepted Object-to-TCP
relation. After selecting a concrete Target Pose, Motion
SHALL derive Approach Pose using that orientation's TCP-local insertion axis.
The approach and target orientations SHALL match.

PICK and PLACE SHALL each use a positive Motion-owned configured approach
distance; their values may differ by task. Approach distance is geometric
separation and is independent of recipe process tolerance. Retract distance
SHALL remain a positive configuration/calibration parameter. Their axis SHALL
be a Motion-owned signed unit insertion axis
expressed in the final grasp TCP frame. Positive distance denotes the direction
from approach toward grasp. For grasp pose `G`, insertion axis `a`, approach
distance `d_a > 0`, and retract distance `d_r > 0`:

```text
approach position = G - d_a * a
retract position  = G - d_r * a
```

After successful grasp/attach, the default retract therefore withdraws from
the object along the same geometric direction used to approach it. Retract
orientation remains the final grasp TCP orientation. A distinct retract axis
requires a later authority decision. No numeric offset, distance, or
calibrated axis value is established by this design.

For PICK, Motion SHALL execute the semantic sequence:

```text
approach -> grasp entry -> gripper action -> attach confirmation -> retract
```

Approach and Target poses MUST remain distinct unless an approved zero-distance
geometry configuration explicitly makes them coincident.

Current-state-to-Approach motion uses ordinary collision-free MoveIt planning.
Approach-to-Target motion is a fixed-orientation Cartesian line along the
selected TCP-local insertion axis. Motion SHALL close only after Target arrival
is confirmed. RETRACT uses the same selected TCP tool-axis semantics.

Each trajectory-producing phase MUST remain active until fresh measured
joint-state feedback confirms arrival at that phase target. Planning success or
command publication alone MUST NOT complete a phase. The bounded completion
wait MUST fail closed on stale/missing feedback, timeout, backend/transport
unavailability, or Safety stop/cancellation, and MUST remain interruptible by
the existing stop/cancel path.

## 5. Logical Gripper Boundary

Upper contracts use a logical gripper opening width rather than simulator-specific multi-joint representation.

Backend force/current/effort or multi-joint mapping is adapter responsibility.

## 6. Verification Requirements

### VR-MOT-OWN-01 — TCP correction ownership
Robot/tool-specific TCP correction SHALL be applied by Motion.

### VR-MOT-OWN-02 — Safety enforcement
Motion SHALL enforce Safety capability at the task boundary.

### VR-MOT-OWN-03 — MotionStatus authority
Motion SHALL be the canonical owner/publisher of externally meaningful motion execution state.

### VR-MOT-OWN-04 — Backend insulation
Mission and Vision contracts SHALL NOT depend on simulator-specific gripper articulation topology.

### VR-MOT-GEO-01 — Object-reference conversion
Motion SHALL be the sole owner of object-reference to TCP/tool rigid-transform
conversion.

### VR-MOT-GEO-02 — Distinct PICK geometry
PICK approach, grasp, and retract poses SHALL be derived from Motion-owned
geometry and SHALL not be collapsed by implementation convenience.

### VR-MOT-GEO-03 — Insertion and retract direction
Approach and default retract positions SHALL both be displaced opposite the
configured positive insertion direction from the final grasp pose, using the
equations in this CDS. Motion SHALL retain the final grasp TCP orientation for
approach and default retract unless a later approved contract states otherwise.

### VR-MOT-GEO-04 — Height and transform authority
Motion SHALL apply the object-reference to TCP rigid transform and SHALL NOT
use `estimated_object_height_m` as an input to PICK geometry.

### VR-MOT-GEO-05 — Measured phase completion
PICK approach, grasp entry, and retract phases SHALL each wait for fresh
measured joint-state confirmation of the submitted phase target before the
next phase begins or PICK succeeds. A command publication or planning result
alone SHALL NOT be treated as phase completion.
