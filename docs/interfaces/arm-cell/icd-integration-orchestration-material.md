# [SF-Twin] ARM Cell Material Readiness ↔ Orchestration Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-MATERIAL-READY_v0.1.0`
- **Document Type:** ICD
- **Interface / Boundary ID:** `MATERIAL-READY`
- **Version:** `0.1.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Shared Interface`
- **Participants:** `Integration ↔ Orchestration`
- **Related Design:** [Material Handoff FDS](../../components/arm-cell/integration/fds-material-handoff.md), [Batch Mission FDS](../../components/arm-cell/orchestration/fds-mission-cycle.md)

## 1. Purpose

The existing AMR docking and PackML interfaces do not express that a material
transfer completed or authorize batch admission. This ICD defines the minimum
shared boundary needed to communicate the Integration-owned `MATERIAL_READY`
fact to Orchestration.

## 2. Scope

### 2.1 In Scope

- one material delivery/transfer completion identity;
- current delivery readiness state and invalidation/failure semantics;
- duplicate delivery handling at the admission boundary.

### 2.2 Out of Scope

- AMR navigation or AMR arrival/docking semantics;
- batch admission decision, which belongs to Orchestration;
- conveyor choreography or simulator realization;
- mission progress/result, owned by ExecuteCycle and Orchestration.

## 3. Participants

| Participant | Role |
|---|---|
| Integration | publishes transfer-complete / `MATERIAL_READY` fact after handoff succeeds |
| Orchestration | consumes the fact and independently checks readiness and Safety before batch admission |

Endpoint: `/integration/material_readiness` carrying
`arm_cell_interfaces/msg/MaterialReadiness`. The minimum message fields are
`std_msgs/Header header`, `unique_identifier_msgs/UUID delivery_id`,
`bool material_ready`, and `bool valid`. `delivery_id` is assigned once per
delivery episode. `material_ready=true` means the transfer is complete;
`false` means it is not complete or failed. `valid=false` cannot establish
readiness. No new enum is needed. Each accepted delivery begins with a valid,
not-ready state carrying its `delivery_id`; a successful transfer updates that
state to ready. These messages describe the latest delivery state, rather than
only recording that a READY transition once occurred.

## 4. Interface Ownership

Integration is the sole writer of transfer completion and material readiness.
Orchestration is the sole authority that admits a batch. A successful transfer
produces a current `MATERIAL_READY` state; that update triggers Orchestration to automatically
evaluate current readiness and Safety conditions. No separate operator or Hub
admission request is part of this boundary. A `MATERIAL_READY`
state is necessary but not sufficient for admission.

## 5. Minimum Data Semantics

The message header and local receipt-time freshness evaluation follow the
existing external-state convention. Orchestration checks both the source
`header.stamp` age and local receipt age against its existing freshness bound.
A retained sample's new local receipt time does not make its source timestamp
fresh. The exact watchdog threshold is profile configuration.

- A positive readiness state means the transfer for the identified delivery
  completed. Integration publishes it as the current state until invalidation
  or the start of a newer accepted delivery. Orchestration independently
  tracks whether that identity has already been admitted and applies freshness.
- When a newer accepted delivery starts, its not-ready state replaces the
  previous delivery as current. Orchestration considers readiness only for the
  latest observed delivery identity; a previous delivery's READY cannot be
  admitted after the newer state is observed.
- AMR arrival, docked, or unload facts alone do not mean `MATERIAL_READY`.
- Failed or incomplete transfer SHALL NOT publish positive readiness.
- Duplicate readiness updates for one delivery SHALL NOT admit a second batch.
- If readiness is valid but general readiness or Safety currently blocks
  admission, Orchestration SHALL retain the pending delivery and re-evaluate
  automatically as those conditions change; this wait does not require a new
  material request.
- Readiness invalidated or expired before admission SHALL block admission. A
  fresh positive confirmation may re-establish readiness for the same delivery
  identity; it does not create a new delivery.
- Within a running Orchestration instance, one delivery identity is admitted
  at most once.
- A late subscriber using compatible transient-local QoS SHALL observe the
  latest delivery state. Receiving a retained sample does not renew its source
  freshness for admission.

## 6. Verification Requirements

### VR-ICD-MATERIAL-01 — Readiness ownership
Only Integration's successful transfer-complete fact may establish
`MATERIAL_READY`; a valid fact remains pending while Orchestration waits for
current readiness and Safety permission, and admission occurs automatically
when those conditions are satisfied.

### VR-ICD-MATERIAL-02 — Failed transfer blocks admission
Given an incomplete or failed transfer, no positive `MATERIAL_READY` state
is observable and Orchestration admits no batch for that delivery.

### VR-ICD-MATERIAL-03 — Duplicate identity
Given duplicate readiness updates for one delivery identity within one running
Orchestration instance, Orchestration admits at most one batch.

### VR-ICD-MATERIAL-04 — Readiness invalidation and revalidation
If positive readiness is invalidated or expires before admission, that
readiness SHALL NOT authorize admission. A fresh positive confirmation for the
same `delivery_id` may re-establish readiness. Reconfirmation SHALL NOT create
a new delivery, and within one running Orchestration instance admission SHALL
occur at most once for that delivery identity. The observable oracle is the
ordered readiness validity/freshness and admission identity/count;
duplicate-update handling remains covered by `VR-ICD-MATERIAL-03`.

## 7. Realization Notes

OD-MATERIAL-01 is resolved for the IDL boundary: the endpoint and message
shape above are realized in `MaterialReadiness.msg`. Integration publishes
reliable transient-local state with one retained sample; Orchestration and the
Hub subscribe with compatible durability/reliability. A CLI observer can use
`ros2 topic echo /integration/material_readiness --qos-durability transient_local`.
The watchdog threshold remains profile configuration as described in §5.

## 8. References

- [External State ↔ Safety ICD](icd-external-safety.md)
- [Integration CDS](../../components/arm-cell/integration/cds.md)
- [VDA CDS](../../components/arm-cell/vda/cds.md)
