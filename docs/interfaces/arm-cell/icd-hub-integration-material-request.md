# [SF-Twin] ARM Cell Hub ↔ Integration Material Request Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-HUB-INTEGRATION-MATERIAL-REQUEST_v0.1.0`
- **Document Type:** ICD
- **Interface / Boundary ID:** `HUB-INTEGRATION-MATERIAL-REQUEST`
- **Version:** `0.1.0`
- **Status:** Approved
- **Owner:** ARM Cell Shared Interface
- **Participants:** Isaac Simulation UI Hub ↔ ARM Cell Integration
- **Related Design:** [Simulation UI Hub FDS](../../components/arm-cell/integration/fds-operator-ui-hub.md), [Material Handoff FDS](../../components/arm-cell/integration/fds-material-handoff.md)

## 1. Purpose and Authority

This ICD defines the operator-originated material-supply request boundary
between the Simulation UI Hub and Integration. The Hub expresses operator
intent. Integration owns request handling, assigns the delivery identity, and
is the only owner that forwards a material request to VDA. This contract does
not change the existing [Integration↔VDA Material ICD](icd-integration-vda-material.md).

## 2. Service Interaction

The service is `/integration/request_material` with type
`arm_cell_interfaces/srv/RequestMaterialSupply`:

| Direction | Field | Type | Meaning |
|---|---|---|---|
| Request | `request_id` | `unique_identifier_msgs/UUID` | Hub-generated correlation identity for this operator request |
| Response | `accepted` | `bool` | Integration accepted the request for a downstream VDA delivery episode |
| Response | `delivery_id` | `unique_identifier_msgs/UUID` | Integration-assigned identity; meaningful only when `accepted` is true |
| Response | `diagnostic_detail` | `string` | Non-normative explanation; not an outcome oracle |

The Hub creates a fresh `request_id` for each new operator request. It is
distinct from `delivery_id`; the Hub does not choose, allocate, or send a
`delivery_id`. This request identity is for correlation and does not define
deduplication or retry behavior.

## 3. Semantic Contract

- Integration allocates one `delivery_id` and sends that ID to the existing
  `/vda/request_material` service. The request's `delivery_id` and VDA response
  follow the existing Integration↔VDA Material ICD.
- Hub-facing `accepted=true` SHALL be returned only when Integration has
  accepted the Hub request and VDA has accepted the corresponding
  `/vda/request_material` request for that `delivery_id`. It confirms VDA
  acceptance of the delivery request; it does not mean the AMR has arrived,
  docked, unloaded, departed, or completed transfer. It does not establish
  `MATERIAL_READY` or admit a batch.
- If VDA rejects the request or is unavailable, Integration SHALL respond
  `accepted=false` to the Hub. When `accepted=false`, `delivery_id` has no
  meaning and SHALL NOT create a delivery episode, readiness, or batch-admission
  success. `diagnostic_detail` is informational.
- Integration owns the VDA response and relays its acceptance result to the
  Hub. The Hub SHALL NOT call `/vda/request_material` directly or own its
  response or delivery state. It reports request acceptance separately from
  the later canonical VDA handoff facts and Integration readiness publication.
- VDA handoff state remains the oracle for arrival, dock, unload, depart, and
  failure facts. Integration's `MaterialReadiness` remains the oracle for
  transfer completion and `MATERIAL_READY`. Orchestration remains the batch
  admission authority.

## 4. Verification Requirements

### VR-ICD-HUB-INT-MAT-01 — Request and delivery identity ownership

The Hub request carries a correlation `request_id`; Integration assigns the
separate `delivery_id` used for the VDA request. The observable oracle is the
Hub request contract and the unchanged Integration-to-VDA request identity.

### VR-ICD-HUB-INT-MAT-02 — Request acceptance is not completion

An accepted service response alone SHALL NOT be presented as arrival,
transfer completion, `MATERIAL_READY`, or batch admission. The oracle is the
owner-published VDA handoff, Integration readiness, and Orchestration
admission state.

### VR-ICD-HUB-INT-MAT-03 — Hub has no material-owner authority

The Hub SHALL route material requests through Integration and SHALL NOT call
VDA directly or create canonical delivery/readiness state. The oracle is the
service endpoint ownership and the absence of a Hub-to-VDA request path.

## 5. References

- [Simulation UI Hub FDS](../../components/arm-cell/integration/fds-operator-ui-hub.md)
- [Material Handoff FDS](../../components/arm-cell/integration/fds-material-handoff.md)
- [Integration↔VDA Material ICD](icd-integration-vda-material.md)
- [Material Readiness ICD](icd-integration-orchestration-material.md)
