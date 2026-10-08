# [SF-Twin] ARM Cell Safety Stop and Recovery Policy

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-SAFETY-STOP-RECOVERY_v1.2.0`
- **Document Type:** `FDS`
- **Scope:** `Stop Selection / Stop Confirmation / Recovery / Reset`
- **Version:** `1.2.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Safety`

## 1. Stop Selection

Safety selects stop severity based on current context and cause.

Safety publishes its selected severity in `SafetyState.selected_stop_mode`.
Safety is the sole writer and canonical authority for this field. Consumers
may compare the shared `StopMode` ordering, but SHALL NOT recalculate severity
from `active_causes`, mission state, or local policy. With no higher-severity
stop selected, `STOP_MODE_CONTROLLED` is the lowest shared-domain baseline.

The normative stop severity ordering is:

```text
STOP_MODE_CONTROLLED < STOP_MODE_IMMEDIATE < STOP_MODE_EMERGENCY
```

While stop handling is active, a higher-severity cause SHALL escalate handling
and a lower-severity request SHALL NOT downgrade an already higher-severity
stop. Clearing the external triggering condition SHALL NOT automatically
downgrade stop handling. Active-cause clearing and Safety latch/reset/recovery
remain separate lifecycles.

Stop source does not override this ordering. Physical E-Stop or STO maps to
`STOP_MODE_EMERGENCY` and is the highest Safety authority. External/control-room
or predictive-maintenance stop evidence is evaluated by Safety and defaults to
`STOP_MODE_CONTROLLED` when no higher-severity hazard applies. Vision reports
hazard evidence only; Safety assigns its stop severity and retains StopMotion
authority. A higher-severity Vision-derived hazard therefore supersedes an
external controlled-stop request.

Current normative baseline stop policy:

| Cause/context | Stop mode | Recovery note |
|---|---|---|
| active E-Stop | `STOP_MODE_EMERGENCY` | no recovery motion while active |
| active STO | `STOP_MODE_EMERGENCY` | no recovery motion while active |
| motion collision during active execution | `STOP_MODE_IMMEDIATE` | no automatic recovery |
| required external communication loss during active execution | `STOP_MODE_IMMEDIATE` | validity and reset policy required |
| invalid required Safety input during active execution | `STOP_MODE_IMMEDIATE` | invalid input remains distinct from communication loss |
| PackML abort during active execution | `STOP_MODE_IMMEDIATE` | conditional recovery |
| premature AMR undock during active robot execution | `STOP_MODE_CONTROLLED` | recovery may later be eligible |
| gripper failure during manipulation | `STOP_MODE_CONTROLLED` | a higher-severity active cause dominates |

If Motion is inactive, an unsafe, invalid, or stale external condition does not
automatically require a redundant `StopMotion` request. Safety SHALL still deny
capability as required. Stop dispatch is required when active execution must be
stopped, or when the approved stop policy otherwise requires explicit
synchronization. Higher-severity concurrent causes dominate by the ordering
above.

Changing this matrix requires an approved FDS/authority change. Safety ownership
of stop severity remains unchanged by future approved matrix revisions.

`STOP_MODE_CONTROLLED` uses configured ramp-down / controlled-deceleration
semantics. This contract does not specify trajectory deceleration limits,
timing, or backend ramp generation; those require separate approved Motion and
backend authority.

The public `SafetyState.active_causes` and `latched_causes` representation is the known-cause bitset defined by [Safety ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-safety-orchestration.md). External condition clear and Safety latch/reset remain separate lifecycles.

## 2. Direct Stop

Safety requests Motion stop directly.

StopMotion service acceptance does not establish stop completion.

## 3. Stop Completion

Safety waits for confirmed Motion inactivity before granting recovery
capability. When Safety has stopped an active execution, only
`MOTION_STATE_STOPPED` with backend inactivity confirmation completes that
stop; StopMotion acceptance, `MOTION_STATE_STOPPING`, or a still-active backend
is insufficient. When no stop was required because Motion was already
inactive, the existing inactive-state evidence may satisfy this precondition.

## 4. Recovery Eligibility

Recovery may be granted only when all required conditions are satisfied, including as applicable:

- Motion inactivity confirmed as defined in Section 3;
- no active trajectory;
- E-Stop/STO inactive;
- required watchdog inputs fresh;
- collision interlock inactive or explicitly cleared;
- original cause classified recovery-eligible.

Safety grants capability; it does not generate the recovery trajectory.

PackML `ABORTED` is fail-closed by default. `MOTION_RECOVERY_ONLY` may be
granted only after the conditions above are satisfied and the latched cause is
explicitly recovery-eligible. PackML `IDLE` and `COMPLETE` may grant normal
motion only through the PackML capability matrix in the Safety CDS/FDS; this
permission does not select or order PICK/PLACE tasks.

## 5. Reset / Latch

Stop-level interlocks are latched unless explicitly defined otherwise.

Reset SHALL fail while required cause-clear, freshness, Motion inactivity, or
E-Stop/STO conditions are not satisfied. A cleared physical/external cause
does not clear a latched Safety condition by itself. Final Demo reset also
requires an explicit operator acknowledgement/reset request through the
proposed [Safety Operator ICD](../../../interfaces/arm-cell/icd-safety-operator.md).
Only Safety applies the request and publishes the resulting canonical
capability; the request does not grant capability.

## 6. Escalation

If a higher-severity stop cause arrives while a lower-severity stop is in progress, Safety SHALL escalate rather than suppress the more severe request.

### SEQ-SAFE-STOP-01 — Concurrent stop escalation

**Primary owner:** this Safety Stop and Recovery FDS. The
[Safety↔Motion ICD](../../../interfaces/arm-cell/icd-safety-motion.md)
defines the stop request/response and backend-inactivity contract; the
[Orchestration Controlled Recovery FDS](../orchestration/fds-controlled-recovery.md)
defines secondary mission coordination.

```mermaid
sequenceDiagram
  autonumber
  participant CauseA as Lower-severity cause
  participant CauseB as Concurrent higher-severity cause
  participant Safety
  participant Motion
  participant Orch as Orchestration

  CauseA-->>Safety: cause becomes active
  Safety->>Safety: select current severity (e.g. CONTROLLED)
  Safety->>Motion: StopMotion(CONTROLLED)
  Motion-->>Safety: accepted
  Note over Safety,Motion: Acceptance means processing accepted; stop completion is not established.
  Note over Safety,Motion: Safety owns stop initiation; this direct Safety→Motion request is the primary stop path.
  par stop and secondary coordination proceed independently
    Safety-->>CauseA: selected_stop_mode = CONTROLLED
  and
    Orch->>Motion: secondary cancellation for mission coordination
  end
  CauseB-->>Safety: higher-severity cause becomes active during stop
  Safety->>Safety: select higher severity by CONTROLLED < IMMEDIATE < EMERGENCY
  Safety->>Motion: StopMotion(higher selected severity)
  Motion-->>Safety: accepted
  CauseA-->>Safety: lower-severity cause/request clears
  Safety->>Safety: retain highest selected severity; no downgrade from lower clear/request
  Motion-->>Safety: MotionStatus confirms backend inactive / stopped
  Note over Safety,Motion: Actual completion requires backend inactivity confirmation, not StopMotion acceptance.
  opt recovery otherwise eligible
    Safety->>Safety: evaluate recovery only after confirmed backend inactivity and all recovery conditions
  end
```

Multiple concurrent causes are resolved by Safety's observable
`SafetyState.selected_stop_mode`; consumers do not derive severity from cause
sets. Stop dispatch for an active execution, severity selection, stop
completion, and Orchestration's secondary cancellation remain independently
observable. A lower-severity cause/request clear does not downgrade a selected
higher severity. Recovery eligibility is evaluated only after backend
inactivity is confirmed and the remaining recovery conditions hold.

#### Sequence verification linkage

| Sequence boundary | Existing VR(s) clarified | Observable oracle |
|---|---|---|
| concurrent cause escalation and no lower-severity downgrade | `VR-SAFE-REC-05` | ordered cause/request changes and monotonic `SafetyState.selected_stop_mode` (`CONTROLLED < IMMEDIATE < EMERGENCY`) |
| StopMotion accepted versus actual completion | `VR-SAFE-REC-01`, `VR-ICD-SAFE-MOT-03`, `VR-ICD-SAFE-MOT-04` | StopMotion response followed separately by MotionStatus/backend inactivity confirmation |
| direct Safety→Motion stop independent of Orchestration cancellation | `VR-ICD-SAFE-MOT-05` | direct stop request and secondary cancellation observable independently; stop occurs when active execution requires it even if cancellation is absent/delayed |
| context-aware dispatch when Motion is inactive versus active | `VR-SAFE-REC-06` | inactive unsafe/stale condition denies capability without redundant stop unless synchronization is required; active execution requiring stop produces StopMotion |
| recovery capability follows confirmed inactivity | `VR-SAFE-REC-02` | Motion backend inactivity confirmation precedes recovery-capability grant/evaluation |

The ordering above specifies externally observable outcomes and does not
prescribe locks, callbacks, executors, or internal arbitration mechanisms.

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
Given active lower-severity stop handling, when a higher-severity cause
arrives, Safety SHALL escalate to the higher stop mode. Given an already higher
severity stop, a lower-severity request or external-cause clear SHALL NOT
downgrade it without the separate approved latch/reset/recovery lifecycle.

The selected severity SHALL be observable as `SafetyState.selected_stop_mode`
for deterministic downstream coordination and evidence.

### VR-SAFE-REC-06 — Context-aware stop dispatch
Given Motion is inactive, when an unsafe, invalid, or stale external condition
is observed, Safety SHALL deny capability but SHALL NOT issue a redundant
`StopMotion` request unless the approved policy requires explicit
synchronization. Given active execution that must be stopped, the applicable
matrix row SHALL result in a `StopMotion` request.
