# [SF-Twin] ARM Cell Safety Stop and Recovery Policy

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-SAFETY-STOP-RECOVERY_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `Stop Selection / Stop Confirmation / Recovery / Reset`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Safety`

## 1. Stop Selection

Safety selects stop severity based on current context and cause.

Representative policy:

- E-Stop/STO → emergency synchronization, no recovery motion while active;
- communication loss → immediate stop, no recovery until communication validity restored and policy allows reset;
- motion collision → immediate stop, no automatic recovery;
- premature AMR undock during execution → controlled stop, recovery may later be eligible;
- PackML abort while executing → immediate stop, conditional recovery;
- gripper failure → controlled/appropriate stop, conditional recovery.

Exact approved matrices may be refined without changing ownership.

The public `SafetyState.active_causes` and `latched_causes` representation is the known-cause bitset defined by [Safety ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-safety-orchestration.md). External condition clear and Safety latch/reset remain separate lifecycles.

## 2. Direct Stop

Safety requests Motion stop directly.

StopMotion service acceptance does not establish stop completion.

## 3. Stop Completion

Safety waits for MotionStatus evidence that execution is stopped before granting recovery capability.

## 4. Recovery Eligibility

Recovery may be granted only when all required conditions are satisfied, including as applicable:

- Motion stopped;
- no active trajectory;
- E-Stop/STO inactive;
- required watchdog inputs fresh;
- collision interlock inactive or explicitly cleared;
- original cause classified recovery-eligible.

Safety grants capability; it does not generate the recovery trajectory.

## 5. Reset / Latch

Stop-level interlocks are latched unless explicitly defined otherwise.

Reset SHALL fail while required cause-clear, freshness, motion-stop, E-Stop/STO, or operator-clear conditions are not satisfied.

## 6. Escalation

If a higher-severity stop cause arrives while a lower-severity stop is in progress, Safety SHALL escalate rather than suppress the more severe request.

## 7. Verification Requirements

### VR-SAFE-REC-01 — Stop completion evidence
Safety SHALL NOT infer stop completion from StopMotion response alone.

### VR-SAFE-REC-02 — Recovery after stop
Recovery capability SHALL NOT be granted before Motion is confirmed stopped.

### VR-SAFE-REC-03 — Non-recovery faults
Non-recovery conditions SHALL not grant recovery motion capability.

### VR-SAFE-REC-04 — Reset preconditions
Reset SHALL fail while required recovery/reset conditions remain invalid.

### VR-SAFE-REC-05 — Severity escalation
Higher-severity stop causes SHALL escalate active lower-severity stop handling.
