# [SF-Twin] ARM Cell External Mock Fault Injection Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VDA-FAULT-INJECTION_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Integration Fault Injection`
- **Version:** `1.0.0`
- **Status:** `Review`
- **Owner:** `ARM Cell External Integration`

The scene-level non-bypass invariant is owned by [ARM Cell Simulation
Environment Semantics](../simulation-environment-semantics.md). This FDS owns
the external mock stimulus behavior and its owner-boundary effects.

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

## 5. Transient Communication Degradation

The Final Demo scenario seam SHALL also support a bounded delay/jitter
stimulus that leaves required inputs valid and inside Safety's configured
freshness limit while placing them in its configured degraded-freshness band.
Safety then publishes the configured reduced velocity/acceleration envelope.
The stimulus is distinct from Communication Loss: if a required input crosses
the stale/invalid limit, Safety denies motion and uses its approved stop
policy instead of continuing at reduced speed. Clearing the transient
stimulus restores prompt fresh publication; Safety restores its normal
envelope only after fresh state is re-established.

Delay duration and degraded-band limits are profile configuration/calibration
values, not demo-defined constants. This seam changes publisher timing/freshness
only; it does not write Safety capability or motion-envelope state.

## 5. Verification Requirements

### VR-VDA-FAULT-01 — Actual communication suppression
Communication-loss injection SHALL actually suppress the relevant heartbeat publishers.

### VR-VDA-FAULT-02 — Fresh restore
Communication restoration SHALL produce fresh state without waiting for an unrelated long delay.

### VR-VDA-FAULT-03 — E-Stop clear does not reset Safety
Clearing mock E-Stop SHALL NOT directly clear Safety's independent latch/reset policy.

### VR-VDA-FAULT-04 — Undock clear does not redock
Given premature-undock injection has caused `UNDOCKED`, when the injection is
cleared, then clearing SHALL NOT itself synthesize `DOCKED`. The normal initial
state remains `DOCKED` when no undock event has occurred.

### VR-VDA-FAULT-05 — Degradation distinct from loss
The bounded transient delay stimulus SHALL preserve valid/fresh required
inputs within the configured degraded band, while Communication Loss SHALL
cross the required-input freshness limit. The oracle is source sample age,
validity, and Safety's resulting envelope/capability.
