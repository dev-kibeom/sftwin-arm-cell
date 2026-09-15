# [SF-Twin] ARM Cell Controlled Recovery Coordination

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-ORCHESTRATION-CONTROLLED-RECOVERY_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Safety Preemption Coordination`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Orchestration`

## 1. Entry

When Safety removes normal motion capability during a running mission:

1. Orchestration stops advancing the normal mission branch;
2. running Motion leaves are halted/canceled for coordination;
3. Orchestration waits for Safety's recovery-capability outcome.

## 2. Waiting for Recovery Authorization

Recovery SHALL NOT be requested merely because the mission was preempted.

Orchestration waits until Safety explicitly exposes a recovery-capable state/capability.

Non-recovery conditions such as active E-Stop, unresolved communication loss, or other non-recovery faults SHALL not trigger a RETRACT request.

## 3. Recovery Request

When authorized:

```text
Safety: MOTION_RECOVERY_ONLY
→ Orchestration requests RETRACT
→ Motion independently rechecks capability
```

## 4. Mission Result After Recovery

Successful recovery does not convert the original interrupted mission to success.

The original failure/preemption/cancel classification is preserved.

## 5. Verification Requirements

### VR-ORCH-REC-01 — No premature recovery
RETRACT SHALL not be requested before recovery authorization.

### VR-ORCH-REC-02 — Preserve original mission outcome
Successful recovery SHALL not transform the interrupted mission into normal success.

### VR-ORCH-REC-03 — Safety-stop independence
Orchestration cancellation SHALL not be required for Safety direct stop initiation.
