# [SF-Twin] ARM Cell AMR Material Delivery Mock Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-VDA-MATERIAL-DELIVERY_v0.1.0`
- **Document Type:** FDS
- **Scope / Component:** ARM Cell VDA mock
- **Concern:** AMR delivery visualization and canonical external facts
- **Version:** `0.1.0`
- **Status:** Approved
- **Owner:** ARM Cell External Integration
- **Related CDS / HLD:** [VDA CDS](cds.md)

## 1. Purpose

Define the bounded AMR mock delivery sequence used by the Final Demo. This is
a predefined short motion and state projection, not VDA 5050 navigation.

## 2. Normal Flow

On an accepted supply stimulus, the mock presents the AMR's predefined short
sequence:

```text
arrive → dock → unload → depart
```

The request and delivery-state contract is the proposed
[Integration↔VDA Material ICD](../../../interfaces/arm-cell/icd-integration-vda-material.md).

The adapter publishes arrival, docked, unload, and depart facts only when each fact
becomes true. These are VDA-owned external facts. Depart is not a signal that a batch has completed. AMR
navigation, path planning, order execution, and route recovery are not
implemented.

After successful unload, Integration separately determines transfer
completion and publishes `MATERIAL_READY` under the [Material Handoff
FDS](../integration/fds-material-handoff.md). The VDA mock does not publish
`MATERIAL_READY` and does not admit work.

## 3. Failure Flow

If the mock cannot establish an arrival, docked, or unload fact, it reports the
actual valid external subset and does not invent a later fact. Failed unload
or interrupted handoff cannot be treated as successful material delivery.
Integration consequently does not publish `MATERIAL_READY`.

One accepted supply stimulus represents one delivery episode. Additional AMR
delivery during a live batch is outside the Final Demo scope.

## 4. Verification Requirements

### VR-VDA-MAT-01 — Fact ordering
The externally observable mock facts follow arrival, docked, unload, and
depart order. A failed earlier transition prevents publication of later
success facts.

### VR-VDA-MAT-02 — No admission authority
VDA produces only its observed external handoff facts and does not publish
Integration readiness or admit a batch. The cross-boundary
`UNLOADED` / `MATERIAL_READY` / admission distinction is owned by
`VR-ICD-INT-VDA-MAT-03`; this local oracle checks VDA fact ownership and
absence of VDA-published readiness/admission.

### VR-VDA-MAT-03 — No navigation claim
The mock behavior remains the predefined short-motion visualization and does
not claim VDA 5050 navigation or route execution.

## 5. References

- [VDA CDS](cds.md)
- [Material Handoff FDS](../integration/fds-material-handoff.md)
- [External State ↔ Safety ICD](../../../interfaces/arm-cell/icd-external-safety.md)
