# ADR 0004: ARM Cell simulator realization isolation

- **Document ID:** `ADR-ARM-CELL-0004`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Sim-to-real architecture`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-08`
- **Related Design:** ARM Cell component CDSs and ROS ICDs
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

Isaac Sim is the current ARM Cell realization and an important development and validation
environment. Treating its APIs, scene state, coordinate conventions, or lifecycle as
architectural authority would make application meaning difficult to carry to physical
hardware. At the ARM Cell sim-to-real boundary, high-level behavior and shared contracts
must not depend on backend-specific realizations. This decision does not replace the earlier
model/state or RGB-D decisions in ADR 0001 and ADR 0003.

## Decision

ARM Cell domain/use-case behavior and shared ROS contracts remain simulator-neutral.
Isaac-specific construction, state, coordinates, lifecycle, and command semantics are
realizations at infrastructure boundaries. Adapters translate those details into canonical
ARM Cell semantics. A physical robot or another backend must be able to realize the same
semantic ports/contracts through a different adapter without changing application meaning.

The existing `arm_cell_sim_adapter`, logical RGB-D sensor boundary, Motion backend
abstraction, and simulation gripper adapter are concrete applications of this decision.
Their current behavior and interface details remain owned by their CDS/FDS/ICD documents.

### Considered alternatives

- **ROS/domain code depends directly on Isaac APIs:** fewer translation seams, but simulator
  lifecycle and representation would propagate into application behavior and impede another
  realization.
- **Promote an Isaac-specific interface to the public contract:** convenient for the current
  backend, but makes simulator concepts shared contract authority and couples other backends
  to them.
- **Maintain separate simulation and physical application stacks:** permits local
  optimization, but duplicates semantic behavior and allows the two stacks to diverge.
- **Simulator-neutral contracts with outer adapters (selected):** requires adapter work and
  explicit translation, while preserving one application contract across realizations.

## Consequences and trade-offs

Simulator upgrades may require infrastructure adapter changes without redefining application
semantics. Adapters must preserve canonical meaning rather than expose incidental simulator
state. Physical realizations still require backend-specific engineering and validation; this
decision does not claim simulator behavior proves physical behavior.

## Authority boundary

This ADR records ARM Cell architectural rationale. Current topic, state, timing, coordinate,
lifecycle, command, and safety semantics remain owned by the applicable CDS/FDS/ICD and
product documents. ADR 0001 and ADR 0003 remain in force for their narrower decisions.

## References

- [ADR 0001](0001-arm-cell-model-state-and-time.md)
- [ADR 0003](0003-arm-cell-rgbd-sensor-contract.md)
- [`docs/interfaces/arm-cell/icd-motion-backend.md`](../interfaces/arm-cell/icd-motion-backend.md)
- [`docs/components/arm-cell/integration/cds-sim-adapter.md`](../components/arm-cell/integration/cds-sim-adapter.md)
- [`docs/interfaces/arm-cell/icd-sim-rgbd.md`](../interfaces/arm-cell/icd-sim-rgbd.md)
- [`docs/components/arm-cell/simulation-environment-semantics.md`](../components/arm-cell/simulation-environment-semantics.md)
