# [SF-Twin] ARM Cell Motion Stop and Recovery Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-MOTION-STOP-RECOVERY_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Safety Stop / Preemption / Recovery Gate`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Motion`

## 1. Goal Permission

```text
MOTION_NONE
→ no trajectory task permitted

MOTION_RECOVERY_ONLY
→ RETRACT only

MOTION_NORMAL
→ normal tasks permitted
```

## 2. Stop Acceptance

StopMotion acceptance means the request was accepted for processing, not that stop is complete.

```text
active/executing
→ STOPPING
→ backend stop/cancel/hold
→ confirm trajectory inactive
→ STOPPED
```

## 3. Stop Modes

- STOP_MODE_CONTROLLED: configured controlled/decelerated stop.
- STOP_MODE_IMMEDIATE: active trajectory cancel + hold/current-position behavior.
- STOP_MODE_EMERGENCY: best-effort software cancellation/rejection synchronized with independent hardware E-Stop/STO.

These are `StopMode` request values defined by [Safety ↔ Motion ICD](../../../interfaces/arm-cell/icd-safety-motion.md). They are distinct from Motion execution-state values such as `MOTION_STATE_STOPPING` and `MOTION_STATE_STOPPED`.

## 4. Direct Safety Path

Safety→Motion direct stop is the primary software stop path.

Orchestration cancellation is secondary coordination and SHALL NOT be required for stop initiation.

## 5. Concurrency Invariant

Long-running normal planning/execution SHALL NOT indefinitely block the software stop dispatch path.

This document does not mandate a specific mutex/callback-group implementation.

## 6. Gripper During Stop

An active gripper command SHALL NOT continue uncontrolled after stop.

Attached/securely grasped object handling should preserve grasp during recovery evaluation unless a higher-authority safety condition requires otherwise.

## 7. Verification Requirements

### VR-MOT-STOP-01 — Permission gate
Normal trajectory tasks SHALL require `MOTION_NORMAL`.

### VR-MOT-STOP-02 — Recovery gate
RETRACT SHALL require recovery-capable Safety authority.

### VR-MOT-STOP-03 — Acceptance vs completion
StopMotion acceptance SHALL NOT equal stop completion.

### VR-MOT-STOP-04 — Stop completion
`STOPPED` SHALL require execution inactivity confirmation.

### VR-MOT-STOP-05 — Direct path
Safety direct stop SHALL function without waiting for Orchestration cancel.

### VR-MOT-STOP-06 — Stop-path availability
Given normal planning/execution work is deliberately blocked or long-running,
when `StopMotion` arrives, then stop handling / backend stop dispatch can make
progress before the normal work is released. This VR is behavioral and does
not prescribe a mutex, executor thread count, or callback-group type.

### VR-MOT-STOP-07 — Gripper bounded behavior
Active gripper behavior SHALL be bounded by stop policy.
