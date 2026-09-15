# [SF-Twin] ARM Cell Vision DetectTarget Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VISION-DETECT-TARGET_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Vision / DetectTarget Processing`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Vision`

## 1. Processing Flow

```text
DetectTarget request
→ fresh synchronized RGB-D + valid CameraInfo
→ segmentation
→ no target? → OBJECT_NOT_FOUND
→ deprojection
→ support/surface estimation
→ object height
→ yaw estimation / observability
→ physical sanity
→ TF optical → base_link
→ success
```

The caller supplies the relative `builtin_interfaces/Duration` timeout defined by [Vision ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-vision-orchestration.md). Vision starts its steady monotonic deadline when it receives that request; the timeout is not an absolute ROS-time or wall-clock deadline.

## 2. Depletion Semantics

`OBJECT_NOT_FOUND` is normal depletion evidence.

It SHALL remain distinct from sensor, calibration, TF, or geometric-quality failure.

## 3. Geometry

Vision may use RANSAC/PCA or equivalent approved algorithms to derive:

- support/surface relation;
- object height;
- object-centric position;
- grasp-reference yaw;
- quality metrics.

Exact internal algorithm implementation is not canonical unless required by a Verification Requirement.

## 4. Yaw Ambiguity

When object geometry does not support stable yaw estimation, Vision SHALL follow the configured approved ambiguity policy:

- use an approved default convention; or
- reject as low confidence.

It SHALL NOT report arbitrary unstable orientation as high-confidence success.

## 5. TF

Successful pose output requires a valid transform from camera optical frame to `base_link`.

Vision consumes TF; it does not publish competing camera/robot TF.

## 6. Verification Requirements

### VR-VIS-DETECT-01 — Depletion distinction
Only `OBJECT_NOT_FOUND` SHALL represent normal depletion.

### VR-VIS-DETECT-02 — Geometry failure classification
Insufficient/invalid geometry SHALL NOT be returned as successful detection.

### VR-VIS-DETECT-03 — TF failure classification
Unavailable/invalid optical→base transform SHALL produce a Vision failure outcome.

### VR-VIS-DETECT-04 — Object-centric success
Successful target pose SHALL remain an object-centric grasp reference.
