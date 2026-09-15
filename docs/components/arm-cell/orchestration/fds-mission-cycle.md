# [SF-Twin] ARM Cell Mission Cycle Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-ORCHESTRATION-MISSION-CYCLE_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `ExecuteCycle`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Orchestration`

The public `/orchestration/execute_cycle` Action and its caller-visible `target_id`, progress, exit, and cancellation semantics are defined by [ExecuteCycle ICD](../../../interfaces/arm-cell/icd-orchestration-execute-cycle.md).

## 1. Normal Cycle

```text
normal motion capability
→ ensure Home
→ DetectTarget
→ PICK
→ PLACE
→ GO_HOME
→ repeat
```

The loop continues until a defined exit condition occurs.

## 2. Depletion

Only Vision `OBJECT_NOT_FOUND` is mapped to normal depletion.

```text
OBJECT_NOT_FOUND
→ DEPLETED
→ successful mission completion
```

Other Vision failures SHALL NOT be converted to depletion.

## 3. Component Failure Mapping

- Vision sensor/geometry/TF error → mission Vision failure.
- Motion execution failure → mission Motion failure.
- Safety preemption → mission Safety-stop/preempted result.
- client cancel → canceled result.

The corresponding canonical public exit values are `MISSION_EXIT_DEPLETED`, `MISSION_EXIT_VISION_ERROR`, `MISSION_EXIT_MOTION_ERROR`, `MISSION_EXIT_SAFETY_PREEMPTED`, and `MISSION_EXIT_CANCELED`.

## 4. Retry Policy

Automatic mission retry is not part of the current default contract.

Recovery success SHALL NOT automatically restart the interrupted PICK/PLACE cycle.

Any future retry policy requires explicit FDS change.

## 5. Verification Requirements

### VR-ORCH-MISSION-01 — Depletion-only success
Normal loop termination SHALL require the defined depletion condition.

### VR-ORCH-MISSION-02 — Failure preservation
Vision/Motion/Safety failures SHALL not be silently converted to success.

### VR-ORCH-MISSION-03 — No implicit retry
Interrupted tasks SHALL not auto-restart after recovery without explicit policy.
