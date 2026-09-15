# [SF-Twin] ARM Cell Safety Supervision Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-SAFETY-SUPERVISION_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Watchdog / Context-Aware Interlock / Safety FSM`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Safety`

## 1. Startup

Safety begins fail-closed.

Until all required input classes have produced valid fresh samples:

```text
state = UNKNOWN
motion_capability = MOTION_NONE
```

Historical transient samples SHALL NOT alone establish a fresh safe state.

## 2. Context-Aware Evaluation

Safety SHALL evaluate state combinations rather than independent predicates only.

Examples:

- `COMPLETE + UNDOCKED + robot idle` may be normal process behavior;
- `EXECUTE + UNDOCKED` is an interlock condition;
- `IDLE/STARTING/COMPLETE + robot idle` is process gating, not automatically an interlock;
- communication loss denies motion.

## 3. State Progression

Conceptually:

```text
UNKNOWN
→ SAFE
→ INTERLOCKED
→ STOPPING
→ STOPPED
→ RECOVERY_REQUIRED
```

E-Stop/STO may force emergency-stop state from relevant states.

Exact message constants belong to the shared ICD.

## 4. Watchdog

Required input freshness is evaluated using local receipt timing plus message validity semantics appropriate to the source.

Watchdog timeout values are configuration/contract values only where explicitly declared normative in the relevant ICD.

## 5. Verification Requirements

### VR-SAFE-SUP-01 — Fresh startup
Safety SHALL remain fail-closed until required fresh inputs are observed.

### VR-SAFE-SUP-02 — Context-aware interlock
Normal process states SHALL not be misclassified solely because one predicate is false outside its relevant context.

### VR-SAFE-SUP-03 — Communication loss
Required-input communication loss SHALL deny motion capability.

### VR-SAFE-SUP-04 — E-Stop synchronization
Software state SHALL reflect E-Stop/STO while preserving independent hardware authority.
