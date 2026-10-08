# [SF-Twin] ARM Cell Operator Camera and Overlay Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-INTEGRATION-CAMERA-OVERLAY_v1.3.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / Camera Sources, Operator Video, and Overlay Boundary`
- **Version:** `1.3.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Integration`
- **Related ADR:** `ADR-ARM-CELL-0003`, `ADR-ARM-CELL-0008`

## 1. Purpose

This document defines the camera integration boundary for perception and
operator display. It separates raw RGB-D perception from operator display and
defines the source identity that both simulated and physical cameras preserve.

Vision consumes this contract but does not own the camera publisher or TF overlay.

## 1.1 Operator Video Capability

The operator display consumes camera RGB without changing its perception
meaning. It is independent of Vision's raw RGB/depth/CameraInfo processing and
of Hub rendering. The simulator and a physical camera SHALL preserve this
same source-frame boundary. A remote camera deployment may provide a
transport realization appropriate to its network boundary.

Production ARM Cell runs the Hub on the same machine as the camera and
subscribes directly to `/camera/color/image_raw` and
`/camera/aligned_depth_to_color/image_raw`; it has no separate production video
topic or image transport node. The Hub presents a common display frame through
fixed RGB and aligned-depth display sources. RGB is selected by default. Both
subscriptions use bounded latest-frame best-effort queues; RGB additionally
keeps a small bounded recent-frame history for operator annotation matching,
while Depth keeps only its latest raw frame outside a PnP snapshot. No
perception observation is preserved or reconstructed by this presentation
buffering. Vision independently consumes raw RGB, aligned depth, and
CameraInfo. Vision publishes its internal read-only detector metadata on
`/vision/diagnostics` with the original source RGB header.

The Hub converts image pixels only for the selected display source and when
freezing a PnP snapshot. Aligned Depth uses the existing `32FC1` metric image
semantics and a stable 0.6–1.6 m teal-to-yellow RGBA heatmap range. This fixed
range separates the canonical 0.80, 1.10, and 1.40 m working-depth examples.
Non-finite and non-positive depth pixels have a distinct invalid color; finite
values outside the display range saturate to its endpoints. Depth display
consumes the latest frame independently and does not wait for RGB-D
synchronization. The heatmap is operator presentation only and is not a
sensor-data transformation. When Vision annotation is compatible with the
displayed Depth frame, a dark outline improves ROI and support-region contrast
without changing their coordinates; RGB annotation styling remains unchanged.

For annotation, the Hub selects the nearest operator RGB frame with the same
`frame_id` as the Vision source within the bounded temporal window. The
`annotation_source_tolerance_seconds` setting controls this operator-view
bound (default 150 ms, maximum 200 ms, configurable through the Hub's Isaac
setting / ROS parameter); it does not define annotation display duration.
When Depth is displayed, Vision annotation is overlaid only when the aligned
Depth frame has the same optical `frame_id`, pixel geometry, and bounded
temporal compatibility as the operator RGB frame associated with the
annotation. These checks apply to the overlay only; they do not gate or
synchronize Depth display. Frames from another `frame_id` or outside the
annotation temporal bound suppress the overlay. It SHALL NOT influence
perception, control, safety, pose calculation, or mission success decisions.

During ExecuteCycle, the Hub freezes the matched RGB frame, any fresh
available Depth frame, and the Vision overlay as one presentation snapshot.
The source selector remains available; it displays the selected frozen frame,
or an unavailable state when no fresh Depth frame was captured. At the
ExecuteCycle terminal status, the Hub releases the snapshot while retaining the
selected source and resumes its live frames. The Hub may drop camera frames
beyond its bounded queues/history. This behavior does not define drop semantics
for canonical mission, safety, fault, or operational state, whose existing
contracts remain in force.

## 1.2 Sim-to-real topology and invariant

```text
Simulation

Isaac Camera
├─ raw RGB ───────┐
├─ raw depth ─────┼→ Vision
└─ CameraInfo ────┘

raw RGB ──────┬→ Vision
              └→ Hub RGB display source
aligned depth ┬→ Vision
              └→ Hub Depth display source
```

```text
Physical Cell

Real/industrial RGB-D camera
├─ raw RGB ───────┐
├─ raw depth ─────┼→ Vision
└─ CameraInfo ────┘

raw RGB ──────┬→ Vision
              └→ deployment-selected operator display transport → Hub / supervisor workstation
aligned depth ─┬→ Vision
               └→ deployment-selected operator display transport → Hub / supervisor workstation
```

The invariant is that both camera realizations provide the same raw sensor
semantics and source identity to Vision and the operator display. The Hub
SHALL NOT depend on simulator-specific image access or Isaac camera APIs.

Vision diagnostics preserve the exact raw RGB frame header used for detection;
Vision's RGB-D observation and result semantics remain unchanged. The paired
depth timestamp is used only for Vision's bounded RGB-D association. For
operator presentation, the Hub compares the diagnostic's original RGB source
stamp and `frame_id` with its bounded recent operator-frame history, selecting
the nearest frame from the same `frame_id` within the configured tolerance.
The tolerance is 150 ms by default and capped at 200 ms. A different
`frame_id` or an out-of-bound timestamp suppresses the annotation. The
operator frame and annotation are not inputs to perception, control, safety,
pose calculation, or success decisions.

## 2. Logical Simulated Sensor Identity

The simulator shall expose one logical RGB-D sensor identity:

```text
/World/SF_Twin_Cell/Vision/Camera_Sensor
```

Upper ROS layers SHALL NOT depend on a D455-internal USD prim identity.

The D455 internal color camera may be used as a geometric source for derivation, but the logical sensor remains the stable integration abstraction.

## 3. Render Source

RGB, aligned depth, and native CameraInfo SHALL originate from one logical sensor/render-product acquisition source for the simulated profile.

This prevents independent stream interpretations from drifting.

## 4. Camera Geometry Derivation

Integration owns:

- discovering the relevant D455 internal color-camera geometry;
- validating the composed transform as rigid/near-rigid before derivation;
- deriving the logical camera pose;
- applying the explicit USD-camera → ROS optical-frame basis conversion;
- exporting/validating the camera snapshot used by ROS-side overlay composition.

A non-rigid or otherwise invalid composed transform SHALL fail derivation rather than be silently accepted.

## 5. Camera TF Overlay

ROS-side integration/bringup owns the camera overlay edges required by the supported profile, including:

```text
world → camera_link
camera_link → camera_color_optical_frame
```

Vision SHALL consume these TF edges and SHALL NOT create a competing publisher.

## 6. Build-Time vs Runtime Boundary

The supported development workflow derives camera geometry from inspected simulator state and composes the ROS overlay from the validated snapshot.

Runtime Vision processing SHALL NOT silently rewrite camera geometry, timestamps, or units to compensate for a mismatched snapshot.

## 7. Annotation Ownership and Video Compatibility

Vision owns diagnostic annotation semantics and ties them to the source
observation, including at minimum its source image timestamp and source
camera frame identity. Depending on actual detector output, semantics may
include detection state, target identity, reference/support region, object
region or extent, centroid, selected target pixel, validity/freshness/result,
and yaw availability. Exact box/polygon/mask representation is permitted only
when that representation is genuinely supplied by the detector.

The Hub consumes the video and Vision-produced metadata, checks bounded
temporal and frame compatibility before pairing them, and preserves a
successful pair as a detection snapshot while the associated ExecuteCycle is
active. The snapshot ends only at that ExecuteCycle's terminal action status;
then the Hub returns to live operator video. It SHALL suppress an unpaired,
stale, or incompatible annotation and SHALL NOT reuse a marker from a
different observation as current, analyze RGB itself, or infer detection to
create annotation. Exact timestamp equality is not required. The paired
raw RGB frame must belong to the same camera identity, or an explicitly
validated compatible identity; timestamp proximity alone does not make frames
from different cameras compatible. Snapshot presentation does not delay or
gate mission execution. Matching algorithm/library remains an implementation
detail.

## 8. Non-Responsibilities

This CDS does not own:

- RGB-D consumer synchronization policy;
- Vision frame buffers;
- segmentation or geometric perception;
- DetectTarget behavior;
- normative RGB-D QoS/encoding/unit semantics beyond the shared ICD.

## 9. Verification Requirements

### VR-INT-CAM-01 — Stable logical sensor identity
Upper ROS contracts SHALL bind to the logical camera abstraction rather than a D455-internal prim.

### VR-INT-CAM-02 — Near-rigid validation
Camera pose derivation SHALL reject non-rigid/invalid composed transforms.

### VR-INT-CAM-03 — Explicit optical conversion
USD-camera to ROS optical-frame conversion SHALL be explicit and deterministic.

### VR-INT-CAM-04 — Single TF authority
Camera overlay TF edges SHALL have one ROS-side authority for the supported profile.

### VR-INT-CAM-05 — No hidden runtime compensation
Runtime consumers SHALL NOT silently rewrite sensor timestamps/units/geometry to mask overlay mismatch.
