# [SF-Twin] ARM Cell Integration ↔ VDA Material Delivery Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-INTEGRATION-VDA-MATERIAL_v0.1.0`
- **Document Type:** ICD
- **Interface / Boundary ID:** `INTEGRATION-VDA-MATERIAL`
- **Version:** `0.1.0`
- **Status:** Approved
- **Owner:** ARM Cell Shared Interface
- **Participants:** ARM Cell Integration ↔ VDA mock
- **Related Design:** [Material Handoff FDS](../../components/arm-cell/integration/fds-material-handoff.md), [VDA Material Delivery FDS](../../components/arm-cell/vda/fds-material-delivery.md)

## 1. Purpose

The current external Safety ICD does not express AMR arrival, unload, or
depart facts and has no inbound supply request. This ICD defines the minimum
boundary for the Final Demo's predefined delivery visualization. It does not
implement AMR navigation.

## 2. Endpoints

| Endpoint | ROS type | Producer | Consumer |
|---|---|---|---|
| `/vda/request_material` | `arm_cell_interfaces/srv/RequestMaterial` | Integration | VDA |
| `/vda/material_handoff_state` | `arm_cell_interfaces/msg/MaterialHandoffState` | VDA | Integration |

`RequestMaterial` request carries `unique_identifier_msgs/UUID delivery_id`.
The response carries `bool accepted` and non-normative `string diagnostic_detail`.
Integration assigns a unique delivery ID and reuses it for
all updates in that delivery. A rejected request starts no delivery episode.

`MaterialHandoffState` minimally carries `std_msgs/Header header`,
`unique_identifier_msgs/UUID delivery_id`, `uint8 phase`, and `bool valid`.
The phase domain is `HANDOFF_UNKNOWN`, `HANDOFF_ARRIVED`, `HANDOFF_DOCKED`,
`HANDOFF_UNLOADED`, `HANDOFF_DEPARTED`, and `HANDOFF_FAILED`. No additional
enum is introduced beyond this distinct delivery lifecycle that the existing
AMR docking contract cannot represent.

## 3. Semantic Contract

- VDA owns and publishes the arrival, docked, unload, depart, or failed fact
  only when observed in its predefined mock sequence.
- A phase update retains the request's `delivery_id`; unknown or stale state
  cannot be treated as a successful phase.
- A valid sequence is `ARRIVED → DOCKED → UNLOADED → DEPARTED`. A failed or
  invalid transition does not imply any later success fact.
- `HANDOFF_UNLOADED` does not mean Integration transfer completed and does
  not mean `MATERIAL_READY`.
- VDA state is evidence for Integration's transfer decision; it does not
  admit an Orchestration batch.
- Duplicate `delivery_id` requests are rejected while active or after
  completion. A new request requires a new delivery identity.

## 4. Verification Requirements

### VR-ICD-INT-VDA-MAT-01 — Delivery identity
Request and state updates for one delivery retain the same UUID. Updates for
another delivery cannot be attributed to the active request.

### VR-ICD-INT-VDA-MAT-02 — Ordered handoff facts
Successful state progression is observable in order. Failed/invalid earlier
phases do not produce later success phases.

### VR-ICD-INT-VDA-MAT-03 — Handoff is not readiness/admission
`UNLOADED` is not `MATERIAL_READY`; only Integration's successful transfer
completion creates readiness, and only Orchestration admits a batch.

### VR-ICD-INT-VDA-MAT-04 — Request acceptance and episode uniqueness
An unaccepted `RequestMaterial` SHALL start no delivery episode. A duplicate
request for a `delivery_id` that is active or completed SHALL create no new
delivery episode. Rejected or duplicate requests SHALL NOT cause successful
handoff facts to be produced for a new episode or attributed to that rejected
request. Facts may continue for an already accepted active episode and remain
attributed to its original request. The observable oracle is the request
response, delivery identity, and subsequent VDA handoff-state facts; this
contract does not prescribe request-processing implementation.

## 5. Realization Notes

OD-INT-VDA-MAT-01 is resolved for the IDL boundary: the endpoint names and
message shapes above are realized in the linked `arm_cell_interfaces` types.
Topic QoS follows the existing reliable, volatile depth-10 ROS state convention;
freshness uses the producer timestamp and `valid` field with consumer-local
receipt time. No freshness threshold is introduced by the IDL.

## 6. References

- [External State ↔ Safety ICD](icd-external-safety.md)
- [Material Readiness ICD](icd-integration-orchestration-material.md)
