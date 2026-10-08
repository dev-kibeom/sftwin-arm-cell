# [SF-Twin] ARM Cell Simulated RGB-D Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-SIM-RGBD_v1.0.0`
- **Document Type:** `ICD`
- **Scope:** `Simulated RGB-D Producer ↔ Vision Consumer`
- **Version:** `1.0.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Shared Interface`
- **Related ADR:** `ADR-ARM-CELL-0003`

## 1. Purpose

This ICD defines the shared RGB-D sensor contract consumed by Vision.

The contract is intended to support both the Isaac simulated source and real RGB-D sources without promoting simulator-only exact-timestamp behavior into the general Vision contract.

## 2. Topics

Canonical sensor channels:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

## 3. Encoding

Current canonical semantics:

- RGB: color image compatible with the configured Vision pipeline;
- aligned depth: metric depth representation compatible with deprojection;
- CameraInfo: native ROS camera calibration semantics.

Exact encoding strings that are intended as hard requirements SHOULD be verified against producer/consumer implementation during Codex audit before final approval.

The current simulated baseline is expected to use `rgb8` for RGB and `32FC1` for aligned depth.

## 4. QoS

Vision RGB-D subscriptions SHALL request ROS SensorData-style QoS with:

```text
Reliability: Best Effort
Durability: Volatile
```

A producer MAY offer stronger compatible reliability.

For example, the current Isaac profile may offer Reliable/Volatile while remaining compatible with a Best-Effort/Volatile Vision subscriber.

QoS compatibility SHALL be validated at integration time.

## 5. RGB ↔ Depth Time Association

Exact timestamp equality is NOT the general contract.

A valid RGB-D pair satisfies:

```text
abs(t_rgb - t_depth) <= configured_sync_tolerance
```

The tolerance is a configurable Vision-ingress parameter subject to verification.

The current Isaac profile may naturally produce `Δt = 0`; this is an observed/simulator property, not a cross-backend requirement.

## 6. CameraInfo Association

CameraInfo is calibration state.

Vision may cache CameraInfo after validating compatibility with the active image geometry.

Exact per-frame timestamp equality between CameraInfo and each RGB-D pair is NOT required by this ICD.

Cached calibration SHALL be invalidated/revalidated if relevant frame/resolution/intrinsic identity changes.

## 7. Frames

The canonical optical measurement frame is:

```text
camera_color_optical_frame
```

Camera TF publication is owned by Integration/bringup, not Vision.

## 8. Depth Semantics

Depth values represent geometric range/depth according to the configured aligned-depth contract and are consumed without hidden runtime unit rewriting.

The current simulated profile is intended to expose metric depth compatible with direct geometric deprojection.

## 9. Data Preservation

Consumers SHALL NOT rewrite pixels, depth values, timestamps, frame IDs, or CameraInfo merely to make inconsistent input appear valid.

### 9.1 Acceptance Sensor-Fault Stimulus Seam

The Final Demo's invalid/stale/unavailable Vision scenario SHALL be
reproducible through an approved source-side stimulus on the production
RGB-D input path. The stimulus is applied before the Vision subscriber and
may temporarily withhold a required image/calibration input, publish a
stale/out-of-order sample, or present an invalid/incompatible sample or
calibration. The active stimulus and recovery to a coherent fresh stream are
observable at the canonical sensor boundary. Scenario values and exact
stimulus duration remain deployment/test configuration; this seam adds no ROS
control interface and does not change nominal sensor semantics.

The stimulus SHALL exercise the same RGB-D topics and production Vision
operational profile used by acceptance. It SHALL NOT inject a DetectTarget
result, modify Vision's internal buffers/profile/result, or write canonical
Vision, Orchestration, Motion, or Safety state. Clearing the stimulus restores
normal source publication; Vision ingress/result handling remains governed by
the existing [Vision Frame Ingress FDS](../../components/arm-cell/vision/fds-frame-ingress.md),
[DetectTarget FDS](../../components/arm-cell/vision/fds-detect-target.md),
and [Vision↔Orchestration ICD](icd-vision-orchestration.md). The validation-only
Fixed Vision adapter is not a sensor-fault seam or production perception
evidence (`VR-ICD-VIS-ORCH-11`).

## 10. Verification Requirements

### VR-ICD-RGBD-01 — Best-Effort consumer QoS
Vision SHALL request Best-Effort/Volatile SensorData-style QoS for RGB-D input.

### VR-ICD-RGBD-02 — Bounded temporal sync
RGB/depth pairing SHALL use bounded temporal association rather than requiring universal exact equality.

### VR-ICD-RGBD-03 — Calibration association
Vision SHALL use only validated CameraInfo compatible with active image geometry.

### VR-ICD-RGBD-04 — Optical frame
RGB-D geometry SHALL be interpreted in the canonical optical frame contract.

### VR-ICD-RGBD-05 — No hidden rewriting
The integration/consumer path SHALL not silently rewrite data semantics to conceal incompatibility.

### VR-ICD-RGBD-06 — Production sensor-fault scenario seam

The approved acceptance seam SHALL make unavailable, stale/out-of-order, and
invalid/incompatible sensor conditions observable at the production RGB-D
source boundary and SHALL restore coherent fresh publication when cleared.
The oracle is the sensor input path/stream condition; Vision rejection and
result classification remain verified by `VR-VIS-INGRESS-01`,
`VR-VIS-INGRESS-03` through `VR-VIS-INGRESS-05`, `VR-VIS-DETECT-01`, and
`VR-ICD-VIS-ORCH-02` in the Vision↔Orchestration ICD.
