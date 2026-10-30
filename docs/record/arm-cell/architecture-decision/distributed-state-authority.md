# Architecture Record 0005: ARM Cell distributed state authority across Safety, Motion, and Orchestration

- **Original ADR ID:** `ADR-ARM-CELL-0005`
- **Document Type:** Architecture Record
- **Scope:** `ARM Cell / Safety, Motion, and Orchestration ownership`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-08`
- **Related Design:** Safety, Motion, and Orchestration CDS/FDS/ICDs
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

The cell has mission progression, motion execution, and safety permission state that interact
but answer different questions. Folding them into one global state machine would centralize
convenient observations while obscuring which component can make and change each decision.

## Decision

State authority remains distributed by responsibility:

- **Safety** owns motion capability, stop severity, permitted envelope, safety cause/latch,
  and reset authority.
- **Motion** owns execution activity, backend inactivity confirmation, motion terminal
  outcome, holding observation, and applied execution state.
- **Orchestration** owns mission progression, task sequencing, retry/recovery coordination,
  and mission-level outcome.

Cross-component progression uses explicit contracts and owner-published observations. A
component does not recreate or infer another component's authoritative state for convenience.
Safety-to-Motion direct `StopMotion` is the primary stop path; Orchestration cancellation is
secondary mission coordination and is not a prerequisite to initiate the Safety stop.

### Considered alternatives

- **Orchestration owns all global state:** simplifies mission-level inspection, but makes the
  coordinator an authority for execution and Safety facts it cannot establish.
- **Safety owns Motion execution state too:** centralizes the permission and stop view, but
  couples Safety authority to backend execution facts and lifecycle it does not own.
- **Motion decides Safety stop policy:** keeps execution decisions local, but lets an executor
  interpret policy and severity that belong to Safety.
- **Local authority with explicit contracts (selected):** requires explicit observations and
  coordination seams, while preserving an identifiable owner for each fact.

## Consequences and trade-offs

Consumers must combine published observations and tolerate the boundaries between authorities.
That coordination cost is intentional: a reported mission phase, Safety permission, and
confirmed backend inactivity remain distinguishable facts.

## Authority boundary

This Architecture Record preserves why state ownership is split. Normative state transitions, field meanings,
stop sequencing, retry, and reset behavior remain owned by the current Safety/Motion/
Orchestration FDS and ICD documents.

## References

- [`docs/design/arm-cell/components/safety/cds.md`](../../../design/arm-cell/components/safety/cds.md)
- [`docs/design/arm-cell/components/motion/cds.md`](../../../design/arm-cell/components/motion/cds.md)
- [`docs/design/arm-cell/components/orchestration/cds.md`](../../../design/arm-cell/components/orchestration/cds.md)
- [`docs/design/arm-cell/interfaces/icd-safety-motion.md`](../../../design/arm-cell/interfaces/icd-safety-motion.md)
- [`docs/design/arm-cell/interfaces/icd-safety-orchestration.md`](../../../design/arm-cell/interfaces/icd-safety-orchestration.md)
- [`docs/design/arm-cell/interfaces/icd-orchestration-motion.md`](../../../design/arm-cell/interfaces/icd-orchestration-motion.md)
- [`docs/design/arm-cell/components/motion/fds-stop-recovery.md`](../../../design/arm-cell/components/motion/fds-stop-recovery.md)
