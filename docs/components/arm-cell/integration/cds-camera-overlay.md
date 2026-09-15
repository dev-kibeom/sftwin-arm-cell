# [SF-Twin] ARM Cell Simulated Camera Overlay Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-INTEGRATION-CAMERA-OVERLAY_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / Simulated RGB-D Source & Camera TF Overlay`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Integration`
- **Related ADR:** `ADR-ARM-CELL-0003`

## 1. Purpose

This document defines the static integration responsibility for constructing the logical simulated RGB-D source and exposing its camera geometry through canonical ROS frames.

Vision consumes this contract but does not own the camera publisher or TF overlay.

## 2. Logical Sensor Identity

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

## 7. Non-Responsibilities

This CDS does not own:

- RGB-D consumer synchronization policy;
- Vision frame buffers;
- segmentation or geometric perception;
- DetectTarget behavior;
- normative RGB-D QoS/encoding/unit semantics beyond the shared ICD.

## 8. Verification Requirements

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
