# [SF-Twin] ARM Cell External Mock Fault Injection Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VDA-FAULT-INJECTION_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Integration Fault Injection`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell External Integration`

## 1. E-Stop Injection

Activating E-Stop fault causes mock AMR/safety-HW state to reflect emergency condition and publishes the changed state immediately.

Clearing the mock fault does not bypass Safety latch/reset behavior.

## 2. Premature Undock

Activating premature undock causes the AMR report to publish `AMR_DOCKING_UNDOCKED` with `driving=true` immediately.

Clearing the fault does not imply automatic redocking.

## 3. PackML Abort

Activation publishes `PACKML_STATE_ABORTED`, the supported PackML subset value defined by [External State ↔ Safety ICD](../../../interfaces/arm-cell/icd-external-safety.md).

Clearing follows the approved mock reset behavior and publishes fresh state.

## 4. Communication Loss

Activation suppresses the required periodic publisher group used to validate Safety watchdog behavior.

Deactivation:

```text
communication restored
→ publisher group resumes
→ fresh state published immediately
```

## 5. Verification Requirements

### VR-VDA-FAULT-01 — Actual communication suppression
Communication-loss injection SHALL actually suppress the relevant heartbeat publishers.

### VR-VDA-FAULT-02 — Fresh restore
Communication restoration SHALL produce fresh state without waiting for an unrelated long delay.

### VR-VDA-FAULT-03 — E-Stop clear does not reset Safety
Clearing mock E-Stop SHALL NOT directly clear Safety's independent latch/reset policy.

### VR-VDA-FAULT-04 — Undock clear does not redock
Clearing premature-undock injection SHALL NOT silently synthesize a docked state.
