# ARM Cell Safety-Owned Stop Severity Publication

> **Document Type:** Durable historical record
> **Authority:** Provenance only; normative meaning remains in the referenced owner.
> **Scope:** ARM Cell

### AD-001 — Safety-owned stop severity publication

- **Context/problem:** Controlled recovery needed to observe the selected stop
  severity without moving stop authority into Orchestration.
- **Decision:** Safety publishes `SafetyState.selected_stop_mode`; Safety remains
  the sole severity authority and Orchestration only compares the published
  value.
- **Rationale:** Preserve Safety ownership while making the accepted recovery
  arbitration observable and deterministic.
- **Normative owner(s):** [Safety Stop/Recovery FDS](../../components/arm-cell/safety/fds-stop-recovery.md); [Safety ↔ Orchestration ICD](../../interfaces/arm-cell/icd-safety-orchestration.md); shared [`SafetyState.msg`](../../../ros2_ws/src/interfaces/arm_cell_interfaces/msg/SafetyState.msg); [Safety ↔ Motion ICD](../../interfaces/arm-cell/icd-safety-motion.md) for the Safety-owned stop-initiation/severity boundary
- **ADR:** None recorded
- **Status:** Accepted
