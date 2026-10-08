# [SF-Twin] ARM Cell Vision Component Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-VISION_v1.2.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / RGB-D Perception`
- **Version:** `1.2.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Vision`

## 1. Purpose

Vision converts synchronized RGB-D measurements plus validated camera calibration into a detector-profile-defined Observation Pose and quality metrics. Camera streams may remain available continuously; expensive perception is acquired and processed on `DetectTarget` requests.

## 2. Responsibilities

Vision SHALL own:

- raw RGB, depth, and CameraInfo ingestion;
- RGB/depth subscription and bounded temporal association;
- validated CameraInfo calibration state used for deprojection;
- request-local/ephemeral ingress buffering and freshness bookkeeping;
- stale-frame rejection/flush behavior;
- fresh observation acquisition for each `DetectTarget` request;
- selection of a Vision-owned detector profile by `DetectTarget.target_id`;
- depth-based RawPart detection and requested-target selection within that profile;
- selected-target and diagnostic perception semantics, linked to the source observation;
- 3D deprojection;
- support/surface estimation;
- object height estimation;
- yaw estimation/ambiguity handling;
- physical sanity validation;
- camera optical frame → `base_link` result transformation;
- DetectTarget result classification.

## 3. Non-Responsibilities

Vision SHALL NOT own:

- operator video compression or remote UI transport;
- Hub drawing or rendering of annotations;
- robot/tool-specific TCP offsets;
- motion permission;
- collision-free trajectory guarantees;
- camera publisher ownership;
- camera TF publisher ownership;
- simulation-session lifecycle authority;
- timestamp/unit rewriting of sensor messages.

## 3.1 Request-driven perception and diagnostic boundary

Production perception SHALL remain request-driven:

```text
DetectTarget request
→ acquire fresh synchronized RGB-D and compatible CameraInfo
→ run the selected detector profile
→ produce the canonical result and observation-linked diagnostic semantics
→ return and wait idle for the next request
```

An active camera stream is sensor availability, not an instruction to run
expensive detection continuously. The stream may continue while Vision is
idle. Diagnostic semantics describe Vision-owned results and their source
observation; they do not make Vision an operator-video encoder or renderer.

The detector result remains the existing public DetectTarget contract. This
design does not change that contract. Any separately consumed diagnostic
metadata must preserve its source image timestamp and source camera frame
identity; adding a transport or interface for that metadata is left to
implementation planning if existing canonical producer data is insufficient.

## 4. Output Semantics

Detection is Vision-internal pixel, depth, and feature evidence used to
evaluate the selected target profile. It is not itself the pose exposed at the
service boundary. The successful `target_pose` is the external representation
of the Observation Pose: a base_link pose of a geometric reference selected by
the detector profile. Its meaning is profile-specific. It is not an Object
Pose or robot TCP command pose unless a governing profile contract explicitly
defines it so.

The cross-cutting environment invariants for sensor observability and
independence from simulator ground truth are owned by [ARM Cell Simulation
Environment Semantics](../simulation-environment-semantics.md). This CDS owns
the generic Observation Pose boundary and profile behavior. Each detector
profile or its governing contract SHALL define the geometric reference it
reports. The visible top surface is not a generic Vision reference.
Profiles SHALL remain usable with simulated and real RGB-D observations that
satisfy their declared sensor-facing constraints.

## 5. Deployment-Owned Detector Profiles

Deployment configuration owns the mapping from logical `target_id` values to
Vision detector profiles and the values used by each profile. In the supported
operational profile, this mapping belongs under the deployment-owned `vision`
configuration; it is not a shared ROS message/service schema.

The profile mapping is keyed by `target_id`. Each configured profile provides
sensor-observation selection constraints and target geometry-validity
constraints; it may also provide yaw-observability constraints. The profile
may express such constraints through a workspace region, depth-component
connectivity, observed appearance, size/extent, or other perception-owned
evidence. The particular algorithm and parameter representation are not
canonical here. Production environment and simulator-ground-truth invariants
are owned by [ARM Cell Simulation Environment
Semantics](../simulation-environment-semantics.md).

An unknown or unconfigured `target_id` is an invalid request/profile, not
evidence that the requested object is absent. A configured profile that finds
no matching object reports normal object absence.

## 5.1 Validation-only Fixed Adapter

A deterministic PnP validation profile MAY substitute a separate Fixed Vision
adapter for the production detector while preserving the public
`/vision/detect_target` ROS shape and downstream object-reference semantics
under the validation-only provenance overlay in the Vision↔Orchestration ICD.
The adapter SHALL run as a distinct
executable and SHALL accept only fixture-registered object references.

The production operational profile SHALL NOT select this adapter through a
mode flag, plugin option, or parameter. The adapter is not evidence for
production perception, RGB-D timestamp provenance, detector-profile behavior,
or visible-surface centroid accuracy. Its detailed behavior is defined by
[`fds-fixed-target-adapter.md`](fds-fixed-target-adapter.md).

## 6. Calibration State

CameraInfo is treated as validated calibration state, not as a third per-frame measurement that must share exact timestamp with every RGB/depth sample.

Vision SHALL invalidate/revalidate cached calibration when relevant calibration identity changes, such as frame/resolution/intrinsic inconsistency.

## 7. Time Responsibility

Vision owns frame/sample validity.

Integration owns simulation epoch lifecycle.

Vision may reject stale, out-of-order, temporally inconsistent, or otherwise invalid samples but SHALL NOT independently create a competing profile-restart authority.

## 8. Verification Requirements

### VR-VIS-OWN-01 — Detector-profile Observation Pose
Successful output SHALL use the geometric reference defined by the selected
detector profile and SHALL NOT include robot/tool TCP correction.

### VR-VIS-OWN-02 — Calibration validity
Vision SHALL use only validated calibration compatible with the active RGB/depth geometry.

### VR-VIS-OWN-03 — No motion authority
Vision SHALL NOT grant robot motion permission.

### VR-VIS-OWN-04 — No sensor rewriting
Vision SHALL NOT rewrite source pixels, depth values, timestamps, frames, or CameraInfo to mask upstream inconsistency.

### VR-VIS-OWN-05 — Profile selection ownership
Vision SHALL select the detector profile associated with the request's
`target_id`; deployment configuration SHALL own the target-ID-to-profile
mapping and profile values.

### VR-VIS-OWN-06 — Environment semantics reference
Cross-cutting requirements for production sensor observability and independence
from simulator ground truth are owned and verified by `VR-ARM-SIM-ENV-02` and
`VR-ARM-SIM-ENV-10` in [ARM Cell Simulation Environment
Semantics](../simulation-environment-semantics.md).

### VR-VIS-OWN-07 — Profile-defined reference point
Successful target position SHALL represent the selected detector profile's
declared observation reference, not a TCP/tool pose.

### VR-VIS-OWN-08 — Validation adapter isolation
A validation-only Fixed Vision adapter SHALL remain a separate executable and
profile that cannot be selected by the production operational profile.
