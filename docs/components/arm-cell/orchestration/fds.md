# [SF-Twin] ARM Cell Orchestration Dynamic Design

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-ORCHESTRATION_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `ARM Cell / Mission Orchestration`
- **Concern:** `Component-wide dynamic-design entry point`
- **Version:** `1.0.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Orchestration`
- **Related CDS:** [Orchestration CDS](cds.md)

## 1. Purpose and Scope

This document is the dynamic-design entry point for ARM Cell Orchestration. It
summarizes the component-level runtime concerns and routes each detailed
scenario to its primary owning FDS. It does not duplicate the normative
scenario sequences, terminal mappings, or recovery rules owned by those
detailed documents.

Static responsibility and dependency ownership remain in the
[Orchestration CDS](cds.md). Shared endpoint, payload, identity, and local
interaction semantics remain in the referenced ICDs.

## 2. Major Runtime Actors

| Actor | Dynamic role | Authority reference |
|---|---|---|
| External mission caller | starts or cancels ExecuteCycle and observes public progress/result | [ExecuteCycle ICD](../../../interfaces/arm-cell/icd-orchestration-execute-cycle.md) |
| Orchestration | coordinates mission progression, terminal classification, and Safety-authorized recovery coordination | [Orchestration CDS](cds.md) |
| Vision | supplies one DetectTarget outcome and successful object reference | [Vision ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-vision-orchestration.md) |
| Motion | executes logical mission tasks and reports task completion | [Orchestration ↔ Motion ICD](../../../interfaces/arm-cell/icd-orchestration-motion.md) |
| Safety | owns motion capability and independently initiates stopping through Motion | [Safety ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-safety-orchestration.md), [Safety ↔ Motion ICD](../../../interfaces/arm-cell/icd-safety-motion.md) |

## 3. Dynamic Concepts and Detailed Owners

| Dynamic concern / concepts | Primary owning FDS | Scope of this overview |
|---|---|---|
| One material-ready delivery admitted as a batch; repeated Detect/PICK/holding/PLACE/release iterations, bounded per-target retry, depletion, one terminal GO_HOME, cancellation, and recipe boundary | [Mission Cycle FDS](fds-mission-cycle.md) | identifies the scenario owner and public actors; the detailed FDS owns ordering and exits |
| Safety preemption, stop coordination, operator reset, interrupted-task termination, and permitted batch continuation | [Controlled Recovery FDS](fds-controlled-recovery.md) | identifies the scenario owner and recovery boundary; the detailed FDS owns ordering and recovery outcome rules |

The Mission Cycle FDS is the primary owner of
`SEQ-ORCH-MISSION-01 — ExecuteCycle mission flow`. The Controlled Recovery FDS
is the primary owner of `SEQ-ORCH-RECOVERY-01 — Controlled recovery`. Other
participant documents reference their applicable FDS and do not become
alternate owners of either workflow.

## 4. Component-Level Temporal and Concurrency Summary

- Normal mission advancement requires Safety-owned `MOTION_NORMAL` capability.
- Safety→Motion direct StopMotion is the primary stop path; Orchestration
  cancellation is secondary coordination and does not initiate stop authority.
- A non-Safety mission terminal decision re-observes current Safety before its
  public mission result is committed; the detailed precedence and observable
  ordering are owned by the Mission Cycle FDS.
- Recovery coordination is distinct from normal mission progression and does
  not create an implicit retry of the interrupted mission.

## 5. Verification Routing

Dynamic Verification Requirements remain with their scenario owners:

| Scenario owner | Dynamic VRs | Public observable/oracle family |
|---|---|---|
| [Mission Cycle FDS](fds-mission-cycle.md) | `VR-ORCH-MISSION-01` through `VR-ORCH-MISSION-10` | batch and iteration progress, `MissionExitReason`, retry count, selected target/freshness, processed count, submitted logical Motion goals, and GO_HOME count/result |
| [Controlled Recovery FDS](fds-controlled-recovery.md) | `VR-ORCH-REC-01` through `VR-ORCH-REC-08` | Safety capability, Motion stop state, RETRACT request count/result/termination, holding disposition, subsequent task sequence, and preserved failure provenance |

The diagrams in the detailed FDS documents clarify ordering and ownership; they
are not verification evidence by themselves.

## 6. Interface References

- [ExecuteCycle ICD](../../../interfaces/arm-cell/icd-orchestration-execute-cycle.md)
- [Orchestration ↔ Motion ICD](../../../interfaces/arm-cell/icd-orchestration-motion.md)
- [Safety ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-safety-orchestration.md)
- [Safety ↔ Motion ICD](../../../interfaces/arm-cell/icd-safety-motion.md)
- [Vision ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-vision-orchestration.md)
