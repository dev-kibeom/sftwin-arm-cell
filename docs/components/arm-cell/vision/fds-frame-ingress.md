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

The camera callback maintains only the newest structurally valid synchronized
RGB-D pair as an input candidate. It is bounded to one pair and does not retain
detector output. Each DetectTarget request first checks whether that pair is
within the configured local-receipt freshness window (`reusable_pair_max_age_ms`,
200 ms in the supported ARM Cell profile and capped at 200 ms). When a current
simulation-clock sample is available, its RGB source stamp must also be no more
than this window behind the clock. If so, that exact RGB-D pair and its original
RGB source identity are passed to a new detector execution. Prior
detection results are never reused. If no fresh candidate is available, Vision
waits for a post-request synchronized pair whose image stamps advance past the
request watermark and whose image receipts occur after request start. Cached
CameraInfo is associated only after compatibility validation.

A DetectTarget request does not itself indicate a camera, observing-pose,
scene, or material transition. A recent synchronized input can therefore be
processed again when its receipt age and available sensor-time age are within
the configured window; every request still executes the detector anew against
the preserved source frames.
Vision SHALL NOT impose an additional request-relative source-time settling
window. If camera startup, lifecycle, observing-pose, or scene/material
transition behavior requires stabilization, the contract and authority that
own that transition SHALL explicitly define the requirement and its trigger.
This FDS defines no such transition-specific settling mechanism.

## 5. Freshness vs Request Deadline

Two time questions are intentionally separated.

### 5.1 Sensor freshness

Sensor/sample freshness is evaluated using sensor/simulation timestamp semantics plus local receipt information where required.

### 5.2 Request deadline

The DetectTarget request timeout is the sole deadline source for both
acquisition of a fresh usable observation and processing of that observation,
and is bounded by a monotonic steady-clock deadline. Adapter-local defaults
MUST NOT expire an active request before its supplied timeout. Vision SHALL
return `DETECT_RESULT_TIMEOUT` when the deadline expires before processing
completes. A processing operation that
cannot be interrupted may finish after the deadline, but its result SHALL NOT
be reported as success or another terminal perception result.

A paused `/clock` SHALL NOT cause an otherwise bounded API request to wait indefinitely.

On timeout, the existing `diagnostic_detail` SHALL identify the most specific
observed acquisition stage/rejection, or identify that processing started and
exceeded the deadline. Where available it includes latest RGB, depth, and
CameraInfo receipt ages, the latest RGB-depth stamp delta, and the last ingress
status. Missing stream evidence remains explicitly unavailable.

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
