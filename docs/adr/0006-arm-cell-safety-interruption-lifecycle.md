# ADR 0006: Separate ARM Cell Safety interruption from operational soft failure

- **Document ID:** `ADR-ARM-CELL-0006`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Failure and recovery architecture`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-08`
- **Related Design:** Safety, Motion, and Orchestration stop/recovery contracts
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

Temporary target unavailability or a retry-safe PICK failure without a held object can be
operational outcomes. E-Stop, STO, or another safety-relevant interruption changes whether
motion is permitted and may require a latched cause, direct stop, and deliberate reset.
Treating both as one retry/error lifecycle would hide this difference.

## Decision

Keep domain-local soft failures under domain and Orchestration retry policy. Keep
safety-relevant interruption under Safety authority for capability, severity, cause/latch,
and reset; Safety may directly stop Motion. Clearing an external cause, clearing a Safety
latch, and authorizing motion are distinct facts. Removing E-Stop/STO input alone does not
automatically resume work.

After a Safety interruption, recovery does not implicitly reuse the previous task, observation,
or trajectory. A new recovery decision requires current Safety authorization, confirmed Motion
inactivity, and fresh physical/system state. Software E-Stop semantics do not substitute for
safety-rated hardware functions.

### Considered alternatives

- **Route every failure through Orchestration retry:** gives one retry mechanism, but treats a
  safety permission loss like an operationally retryable result.
- **Promote every abnormal condition to a Safety fault:** simplifies fault classification,
  but unnecessarily latches temporary operational conditions.
- **Resume automatically when the external cause clears:** reduces operator steps, but cause
  removal does not establish latch reset, current state, or permission to resume.
- **Domain-local soft failure plus Safety-owned interruption (selected):** retains two recovery
  lifecycles and requires explicit coordination at their boundary.

## Consequences and trade-offs

Some interruptions need operator acknowledgement and fresh task setup even when the physical
cause has disappeared. Operational failures can remain recoverable without inflating the Safety
fault model. The distinction favors explicit recovery over automatic continuation.

## Authority boundary

This ADR records the lifecycle rationale. Current cause classification, stop escalation,
retry bounds, latch/reset gates, and recovery operations remain specified by Safety, Motion,
Orchestration FDS/ICDs and product requirements.

## References

- [`docs/components/arm-cell/safety/fds-stop-recovery.md`](../components/arm-cell/safety/fds-stop-recovery.md)
- [`docs/components/arm-cell/motion/fds-stop-recovery.md`](../components/arm-cell/motion/fds-stop-recovery.md)
- [`docs/components/arm-cell/orchestration/fds-controlled-recovery.md`](../components/arm-cell/orchestration/fds-controlled-recovery.md)
- [`docs/interfaces/arm-cell/icd-safety-motion.md`](../interfaces/arm-cell/icd-safety-motion.md)
