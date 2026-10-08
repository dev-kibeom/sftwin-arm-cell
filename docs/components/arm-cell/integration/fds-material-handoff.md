# [SF-Twin] ARM Cell Material Handoff Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-INTEGRATION-MATERIAL-HANDOFF_v0.1.0`
- **Document Type:** FDS
- **Scope / Component:** ARM Cell Integration / material transfer
- **Concern:** Delivery visualization and transfer readiness
- **Version:** `0.1.0`
- **Status:** Approved
- **Owner:** ARM Cell Integration
- **Related CDS / HLD:** [Integration CDS](cds.md), [VDA CDS](../vda/cds.md)

## 1. Purpose

Define the observable short-motion material handoff and the point at which
Integration may publish `MATERIAL_READY`. The cross-cutting environment
continuity invariant is owned by [ARM Cell Simulation Environment
Semantics](../simulation-environment-semantics.md). This FDS owns the handoff
sequence and readiness behavior. The AMR mock remains a visualization and
external-state substitute; it does not implement navigation or admit a mission.

## 2. Participants and Authority

| Participant | Authority |
|---|---|
| AMR/VDA adapter | arrival, docked, and unload facts |
| Integration | transfer completion and `MATERIAL_READY` |
| Orchestration | batch admission after readiness and Safety checks |
| Safety | motion capability and stop authority |
| Simulation UI Hub | operator intent to request material; no delivery or readiness authority |

The cross-boundary readiness state is defined by the approved [Material
Readiness ICD](../../../interfaces/arm-cell/icd-integration-orchestration-material.md).
The operator supply request is defined by the
[Hub↔Integration Material Request ICD](../../../interfaces/arm-cell/icd-hub-integration-material-request.md).
The downstream supply request and VDA-owned delivery facts are defined by the
[Integration↔VDA Material ICD](../../../interfaces/arm-cell/icd-integration-vda-material.md).

## 3. Normal Flow

1. Hub sends an operator request to Integration using the
   `/integration/request_material` contract. Integration assigns one
   `delivery_id` and requests that delivery from VDA using the unchanged
   `/vda/request_material` contract. Integration returns `accepted=true` to the
   Hub only when VDA accepts that request for the same `delivery_id`. A VDA
   rejection or unavailable VDA produces `accepted=false`; that response's
   `delivery_id` has no meaning and creates no delivery episode, readiness, or
   admission success. A positive response confirms VDA request acceptance
   only; it does not establish arrival, transfer completion, readiness, or
   admission. An accepted request starts the predefined short-motion visualization:
   arrive → dock → unload → depart.
2. VDA publishes arrival/docked/unload/depart facts at their actual transition
   points. Those facts do not independently create readiness.
3. Once VDA accepts a delivery, Integration publishes its valid not-ready
   state with that delivery identity. Integration completes the transfer
   realization only after successful unload/transfer completion and updates
   the current state to `MATERIAL_READY` for that delivery.
4. The `MATERIAL_READY` state update triggers Orchestration to evaluate current
   readiness and Safety permission; it automatically admits no more than one
   batch for that delivery identity when all conditions are satisfied.
5. VDA depart visualization is independent of batch completion. A subsequent
   delivery during an active batch is outside this demo scope.

## 4. Failure Flow

If arrival, docking, unload, or transfer realization fails or becomes
invalid, Integration does not publish `MATERIAL_READY` and Orchestration does
not admit a batch for that delivery. Recovery or a new supply request is a separate
delivery episode; an incomplete delivery is never represented as a successful
batch.

## 5. Verification Requirements

### VR-INT-MAT-01 — Readiness after transfer
`MATERIAL_READY` is observable only after successful transfer completion.
This Integration-owned behavior refines the readiness ownership and observable
contract in `VR-ICD-MATERIAL-01`. The oracle is the ordered transfer outcome
and Integration readiness state updates.

### VR-INT-MAT-02 — Failed transfer
Given delivery/transfer failure, Integration emits no positive
`MATERIAL_READY` for that delivery. The cross-boundary consequence that this
blocks admission is owned by `VR-ICD-MATERIAL-02`.

### VR-INT-MAT-03 — Handoff is not admission
Integration transfer completion produces readiness according to the
Material Readiness ICD; it does not own batch admission. The cross-boundary
`UNLOADED` / `MATERIAL_READY` / admission distinction and admission oracle are
owned by `VR-ICD-INT-VDA-MAT-03` and `VR-ICD-MATERIAL-01` / `VR-ICD-MATERIAL-03`.
This local VR checks only that Integration does not claim or perform
Orchestration admission.

## 6. Open Decisions

AMR motion path, duration, and visual styling remain configuration/realization
details, not semantic acceptance values. Current readiness state, freshness,
and delivery replacement semantics are defined by the linked ICD.

## 7. References

- [VDA fault injection FDS](../vda/fds-fault-injection.md)
- [Material Readiness ICD](../../../interfaces/arm-cell/icd-integration-orchestration-material.md)
- [Hub↔Integration Material Request ICD](../../../interfaces/arm-cell/icd-hub-integration-material-request.md)
- [Batch Mission FDS](../orchestration/fds-mission-cycle.md)
