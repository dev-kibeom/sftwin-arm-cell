# [SF-Twin] ARM Cell Simulation Session Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-INTEGRATION-SIM-SESSION_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `ARM Cell / Isaac Development Session Lifecycle`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Integration`
- **Related ADR:** `ADR-ARM-CELL-0001`

## 1. Purpose

This document defines dynamic session behavior across pause/resume and simulation-time epoch changes.

It is a development-session lifecycle contract, not a production safety mechanism.

## 2. Normal Session

```text
Isaac running
→ /clock monotonic
→ raw measured state received
→ canonical state published
→ TF/current-state consumers operate
```

Pause/resume is supported when simulation time remains monotonic.

## 3. Backward-Time / Epoch Change

If the integration boundary observes backward movement in the applicable simulation clock/state timestamp domain:

```text
valid session
→ backward epoch detected
→ canonical publication refused/latched
→ complete ROS profile restart required
→ clean session established
```

Transparent timestamp rebasing is prohibited.

Cached state SHALL NOT be replayed as if it belongs to the new epoch.

## 4. Component Interaction

Integration owns session-epoch authority.

Downstream components such as Vision may reject invalid, stale, out-of-order, or temporally inconsistent samples, but SHALL NOT create a competing profile-lifecycle authority.

## 5. Verification Requirements

### VR-INT-TIME-01 — Monotonic pause/resume
Pause/resume without backward time SHALL preserve normal session participation.

### VR-INT-TIME-02 — Backward epoch refusal
Backward epoch movement SHALL prevent continued canonical publication for that session.

### VR-INT-TIME-03 — No transparent rebasing
The system SHALL NOT conceal an epoch change by rewriting timestamps or replaying stale cached state.

### VR-INT-TIME-04 — Restart recovery
A complete supported ROS-profile restart SHALL establish a clean session when the simulator is valid again.
