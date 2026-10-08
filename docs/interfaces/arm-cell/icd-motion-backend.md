# [SF-Twin] ARM Cell Motion Backend Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-MOTION-BACKEND_v0.2.0`
- **Document Type:** `ICD`
- **Scope:** `Motion Core ↔ Interchangeable Robot/Gripper Backend`
- **Version:** `0.4.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Shared Interface`

## 1. Purpose and Ownership

This ICD is retained as the backend-neutral abstract contract between Motion Core and an interchangeable execution backend. It is not an Orchestration-, Vision-, or Safety-facing task interface.

Motion Core owns task semantics, TCP/tool correction, safety-capability enforcement, and canonical Motion results. A backend owns only execution of normalized requests and reporting of its own lifecycle, availability, and capability state.

Candidate backends include Isaac Sim, a Doosan robot backend, and other robot/manipulator backends. No backend implementation is the authority for this contract.

## 2. Required Backend-Neutral Semantics

The stable seam SHALL provide the following semantics without exposing backend-specific topology or controller APIs:

- backend availability and disconnect/fault reporting;
- submission and lifecycle reporting for a normalized execution request;
- cancel, hold, and stop dispatch;
- execution-active state and explicit backend-inactivity confirmation;
- backend-neutral `GripperPort` close/open, holding, activity, and stop
  semantics;
- logical gripper-width command and outcome reporting;
- mapping of backend-specific errors into canonical Motion result/error semantics.

`STOPPED` SHALL be reported by Motion only after the backend has confirmed that execution is inactive. A stop request being accepted or dispatched is not inactivity confirmation.

Logical gripper width remains backend-neutral. A backend SHALL translate it to its own actuator representation internally; simulator joint topology, vendor actuator layout, and controller API names SHALL NOT cross this boundary upward.

## 2.1 GripperPort Separation

The implementation SHALL separate arm-motion execution from logical gripper
semantics through a backend-neutral `GripperPort`, even when both ports share
one transport internally.

The port SHALL provide:

- logical `close(grasp_width_mm)` and `open()` operations;
- a `HoldingObservation` with explicit `HELD`, `RELEASED`, or `UNKNOWN` state,
  freshness metadata, and state distinct from command success;
- active-command observation and `stop()` for interruption.

Motion SHALL not command attach/detach primitives. A simulator adapter owns its
joint realization, while a hardware adapter owns its sensing/actuation
realization. Prim paths, capture volumes, fixed-joint identifiers, vendor joint
topology, and raw contact/sensor details SHALL NOT cross the port.

`HoldingObservation.state` is `HELD`, `RELEASED`, or `UNKNOWN`; it includes a
monotonic observation time and sequence. The validation profile uses a 500 ms
freshness bound. Stale/missing observations, adapter disconnect/unavailability,
ambiguous candidates, and unexpected joint/prim loss become `UNKNOWN`.
`close()`/`open()` command success never implies a holding state. Only a fresh
`HELD` observation authorizes PICK retract; only a fresh `RELEASED` observation
authorizes PLACE retreat and success.

## 2.2 Acceptance Holding-Observation Scenario Seam

The Final Demo Holding Uncertainty scenario SHALL be reproducible at the
backend's `GripperPort.holding()` observation boundary. An approved scenario
adapter may return a fresh `HoldingObservation` whose state is `UNKNOWN`, or
make the observation unavailable/stale so the normal port freshness handling
yields `UNKNOWN`. The stimulus must pass through the backend-neutral port and
normal Motion task path; it SHALL NOT write `MotionStatus`, Safety state, or
Orchestration mission state directly. Motion remains the publisher of its
canonical status. Clearing the scenario stops the injected uncertainty but
does not synthesize `HELD` or `RELEASED`; those require a new backend-owned
observation. This is an acceptance/test seam, not a public ROS interface or a
claim about physical force-sensing accuracy.

This section owns only the scenario stimulus location and its non-bypass
boundary. The resulting UNKNOWN semantics, task gating, and mission recovery
remain owned by `VR-ICD-MOT-BACKEND-05`, `VR-ICD-SAFE-MOT-09`,
`VR-MOT-TASK-01`, `VR-MOT-TASK-02`, and `VR-ORCH-REC-08`.

## 3. Optional Capabilities

Explicit arm hold support and other backend-dependent behavior SHALL be
capability-reported rather than assumed as mandatory. For a requested
PICK/PLACE operation, an unsupported gripper or holding-confirmation capability
SHALL fail closed with the applicable canonical grasp/backend failure; it
SHALL NOT silently complete the operation. Motion Core SHALL not expose
backend-private sensing, topology, attachment, or controller details to its
upper consumers.

This abstract seam does not require a ROS 2 transport or IDL. It defines backend-neutral semantics; a backend implementation may realize them in-process or through an approved adapter without changing the public Motion contracts. This ICD does not prescribe Isaac, Doosan, or other vendor transport details.

## 4. Non-Goals

This ICD SHALL NOT expose Isaac prim paths, Robotiq six-joint topology, Doosan-specific ROS service names, vendor-private controller APIs, or MoveIt internal class names.

## 5. Verification Requirements

### VR-ICD-MOT-BACKEND-01 — Backend insulation
The backend contract SHALL preserve upper Motion semantics across supported simulator/real backends, including logical gripper-width handling and the distinction between stop dispatch and verified inactivity.

### VR-ICD-MOT-BACKEND-02 — Holding confirmation boundary
`GripperPort.holding()` SHALL return a fresh `HoldingObservation`, not a bare
boolean. `HELD`, `RELEASED`, and `UNKNOWN` SHALL remain distinguishable from
logical close/open command success. Backend-private sensing, capture, snap, and
attachment mechanisms SHALL remain below the seam. Motion SHALL not retract
or report PICK success without fresh `HELD`.

### VR-ICD-MOT-BACKEND-03 — Fail-closed optional capabilities
Unsupported requested gripper or holding-confirmation capabilities SHALL
return a canonical failure and SHALL not complete PICK or PLACE successfully
by default.

### VR-ICD-MOT-BACKEND-04 — Release confirmation boundary
Motion SHALL not retreat or report PLACE success until `open()` succeeds and a
fresh `RELEASED` observation is received. `UNKNOWN` is not equivalent to
release. Simulator detachment details SHALL remain below the port.

### VR-ICD-MOT-BACKEND-05 — Freshness and unavailable state
A stale/missing observation, adapter disconnect/unavailability, ambiguous
candidate, or unexpected joint/prim loss SHALL yield `UNKNOWN` and block
PICK/PLACE progression. The 500 ms validation freshness bound is independent
of Vision registration TTL.
