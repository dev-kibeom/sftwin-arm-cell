# [SF-Twin] ARM Cell Vision Component Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-VISION_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / RGB-D Perception`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Vision`

## 1. Purpose

Vision converts synchronized RGB-D measurements plus validated camera calibration into an object-centric grasp reference and quality metrics.

## 2. Responsibilities

Vision SHALL own:

- RGB/depth subscription and bounded temporal association;
- validated CameraInfo calibration state used for deprojection;
- request-local/ephemeral ingress buffering and freshness bookkeeping;
- stale-frame rejection/flush behavior;
- segmentation;
- 3D deprojection;
- support/surface estimation;
- object height estimation;
- yaw estimation/ambiguity handling;
- physical sanity validation;
- camera optical frame → `base_link` result transformation;
- DetectTarget result classification.

## 3. Non-Responsibilities

Vision SHALL NOT own:

- robot/tool-specific TCP offsets;
- motion permission;
- collision-free trajectory guarantees;
- camera publisher ownership;
- camera TF publisher ownership;
- simulation-session lifecycle authority;
- timestamp/unit rewriting of sensor messages.

## 4. Output Semantics

The successful `target_pose` is:

> an object-centric grasp reference pose in `base_link`.

It is not the robot TCP command pose.

## 5. Calibration State

CameraInfo is treated as validated calibration state, not as a third per-frame measurement that must share exact timestamp with every RGB/depth sample.

Vision SHALL invalidate/revalidate cached calibration when relevant calibration identity changes, such as frame/resolution/intrinsic inconsistency.

## 6. Time Responsibility

Vision owns frame/sample validity.

Integration owns simulation epoch lifecycle.

Vision may reject stale, out-of-order, temporally inconsistent, or otherwise invalid samples but SHALL NOT independently create a competing profile-restart authority.

## 7. Verification Requirements

### VR-VIS-OWN-01 — Object-centric output
Successful output SHALL be object-centric and SHALL NOT include robot/tool TCP correction.

### VR-VIS-OWN-02 — Calibration validity
Vision SHALL use only validated calibration compatible with the active RGB/depth geometry.

### VR-VIS-OWN-03 — No motion authority
Vision SHALL NOT grant robot motion permission.

### VR-VIS-OWN-04 — No sensor rewriting
Vision SHALL NOT rewrite source pixels, depth values, timestamps, frames, or CameraInfo to mask upstream inconsistency.
