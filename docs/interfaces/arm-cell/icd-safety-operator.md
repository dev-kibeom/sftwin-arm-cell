# [SF-Twin] ARM Cell Operator Acknowledge and Safety Reset Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-SAFETY-OPERATOR_v0.1.0`
- **Document Type:** ICD
- **Interface / Boundary ID:** `SAFETY-OPERATOR-RESET`
- **Version:** `0.1.0`
- **Status:** Approved
- **Owner:** ARM Cell Shared Interface
- **Participants:** UI Hub ↔ Safety
- **Related Design:** [Safety stop/recovery FDS](../../components/arm-cell/safety/fds-stop-recovery.md), [Orchestration controlled recovery FDS](../../components/arm-cell/orchestration/fds-controlled-recovery.md)

## 1. Purpose

This ICD defines the minimum operator acknowledgement/reset request boundary
required for the Final Demo. The Hub submits a request; Safety alone evaluates
and applies reset policy.

## 2. Service Interaction

The service is `/safety/reset` with type
`arm_cell_interfaces/srv/ResetSafety`:

| Direction | Field | Type | Meaning |
|---|---|---|---|
| Request | `request_id` | `unique_identifier_msgs/UUID` | correlates this operator request |
| Request | `operator_acknowledged` | `bool` | explicit acknowledgement for this reset attempt |
| Response | `applied` | `bool` | Safety applied the reset policy; false means rejected |
| Response | `diagnostic_detail` | `string` | non-normative explanation |

SafetyState remains the canonical observable outcome; `applied=true` is not
itself proof that normal motion is permitted.

## 3. Reset Preconditions

Safety applies reset only when the triggering physical/external causes are
cleared, E-stop/STO are inactive, required inputs are valid and fresh, Motion
inactivity is confirmed, reset policy permits the latched causes, and explicit
operator acknowledgement is present. An external adapter or Hub cannot clear
Safety latches by editing canonical state. Rejected requests leave the latch
and capability unchanged.

## 4. Verification Requirements

### VR-ICD-SAFE-OP-01 — Explicit acknowledgement
Without an explicit operator acknowledgement/reset request, cleared E-stop
condition alone SHALL NOT clear the Safety latch or restore normal capability.

### VR-ICD-SAFE-OP-02 — Safety reset authority
The Hub request SHALL not directly change SafetyState. Only Safety may apply
or reject reset, and its canonical SafetyState publication is the oracle for
the outcome.

## 5. Realization Notes

OD-SAFE-OP-01 is resolved: `/safety/reset` is realized as
`arm_cell_interfaces/srv/ResetSafety` using the package's existing service
conventions.

## 6. References

- [Safety↔Orchestration ICD](icd-safety-orchestration.md)
- [Safety↔Motion ICD](icd-safety-motion.md)
- [UI Hub FDS](../../components/arm-cell/integration/fds-operator-ui-hub.md)
