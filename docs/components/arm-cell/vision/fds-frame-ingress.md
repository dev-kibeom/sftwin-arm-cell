# [SF-Twin] ARM Cell Vision Frame Ingress Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VISION-FRAME-INGRESS_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Vision / RGB-D Association, Freshness, Deadline`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Vision`

## 1. Purpose

This document defines Vision ingress behavior for temporally associating RGB and depth, validating calibration, selecting fresh samples, and bounding DetectTarget wait time.

## 2. RGB ↔ Depth Synchronization

Real sensors are not required to produce identical timestamps.

Vision SHALL accept an RGB/depth pair only when:

```text
abs(t_rgb - t_depth) <= configured_sync_tolerance
```

A ROS `message_filters` ApproximateTime-style synchronizer or semantically equivalent bounded association mechanism MAY implement this behavior.

The exact algorithm/library is implementation detail; the bounded association semantic is normative.

The current Isaac profile may naturally produce equal timestamps, but equality is not the general contract.

The supported ARM Cell Vision profile default is:

```text
sync_slop_ms = 10.0 ms
```

This is a profile configuration, not a general RGB-D sensor contract. The parameter remains
configurable; `0 ms` is permitted only as an explicit diagnostic or test configuration and is
not the supported default.

## 3. CameraInfo Association

CameraInfo is validated and cached as calibration state.

Before using cached calibration, Vision SHALL verify that it remains compatible with the active image geometry, including as applicable:

- calibration frame identity;
- image width/height;
- intrinsic matrix;
- distortion/projection semantics required by the active model.

If calibration becomes incompatible or invalid, Vision SHALL reject processing until valid calibration is available.

## 4. Request Start / Stale Flush

On a new DetectTarget request:

1. stale/pre-request frame candidates are excluded according to the configured freshness boundary;
2. configured settling behavior is applied when required by the observing-pose workflow;
3. Vision waits for a fresh synchronized RGB-D pair;
4. valid cached CameraInfo is associated;
5. the resulting frame set is passed to perception.

## 5. Freshness vs Request Deadline

Two time questions are intentionally separated.

### 5.1 Sensor freshness

Sensor/sample freshness is evaluated using sensor/simulation timestamp semantics plus local receipt information where required.

### 5.2 Request deadline

The DetectTarget wait/deadline is bounded by a monotonic steady-clock deadline.

A paused `/clock` SHALL NOT cause an otherwise bounded API request to wait indefinitely.

This is the intended architecture. Codex repository audit SHALL confirm current implementation alignment.

## 6. Backward / Invalid Sensor Time

Vision SHALL reject out-of-order, backward, stale, or otherwise invalid frame candidates.

Simulation-session epoch authority remains Integration responsibility.

Vision SHALL NOT create an independent permanent restart/lifecycle authority unless a later architecture decision explicitly changes ownership.

## 7. Failure Outcomes

Ingress may fail due to:

- no synchronized pair before deadline;
- stale/out-of-order data;
- malformed messages;
- invalid calibration;
- incompatible frame/resolution/calibration;
- bounded queue/association failure.

Exact public result mapping belongs to `fds-detect-target.md` and the Vision↔Orchestration ICD.

## 8. Verification Requirements

### VR-VIS-INGRESS-01 — Bounded RGB-D association
RGB and depth SHALL be associated only within configured temporal tolerance.

### VR-VIS-INGRESS-02 — No exact-time requirement
General Vision behavior SHALL NOT require exact integer-nanosecond equality between real-sensor RGB and depth.

### VR-VIS-INGRESS-03 — Calibration cache validity
Cached CameraInfo SHALL be used only while compatible with the active image geometry.

### VR-VIS-INGRESS-04 — Steady request deadline
DetectTarget wait timeout SHALL remain bounded when simulation time is paused.

### VR-VIS-INGRESS-05 — Sensor-time freshness
Frame freshness SHALL be based on sensor/simulation time semantics rather than only wall time.

### VR-VIS-INGRESS-06 — Epoch ownership separation
Vision SHALL reject invalid temporal samples without taking over Integration's session-lifecycle authority.
