# Architecture Record 0011: ARM Cell external-event material handoff

- **Original ADR ID:** `ADR-ARM-CELL-0011`
- **Document Type:** Architecture Record
- **Scope:** `ARM Cell / external material handoff and mission admission`
- **Version:** `1.3.0`
- **Status:** Proposed
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-09`
- **Related Design:** ARM Cell Demo Definition, Gateway/VDA FDS, material ICDs
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

The current demo path lets the Isaac Hub ask Integration to request a delivery
from VDA. Integration allocates `delivery_id`, and the VDA mock runs a short
arrival/dock/unload/depart sequence. That path assigns delivery orchestration
to ARM Cell, although future IT/MES and AMR Cell own job assignment and
delivery. The current Integration mock also treats elapsed time after VDA
`UNLOADED` as transfer completion, which does not provide independent evidence
that material reached the ARM Cell handoff position.

## Decision

IT/MES or AMR Cell owns job assignment and delivery. ARM Cell consumes an
external Handoff Trigger at its material handoff boundary and processes the
identified delivery from that point. The trigger carries the externally
assigned `delivery_id`; the same trigger contract is used by an external demo
terminal and by the Isaac Simulation UI Hub during the current demo.

Gateway completes trigger consumption when it accepts and records a valid
trigger. Later VDA or material-supply failure is the outcome of that delivery;
it does not make the trigger eligible for re-consumption. A successful supply
is separately confirmed by Transfer Completion before Gateway publishes
`MATERIAL_READY`.

Gateway is the sole authority accepting the raw Handoff Trigger and
forwards only accepted identities to the VDA mock; the mock cannot consume or
independently accept the raw source event. It publishes its short
`ARRIVED → DOCKED → UNLOADED → DEPARTED` sequence. VDA phases remain external
handoff facts. `UNLOADED` does not establish transfer completion. The accepted trigger includes a positive `quantity`, configured `material_type`
(current demo: `RawPart`), and optional random seed. Gateway alone accepts
the raw event and forwards it unchanged to both VDA Mock and the Isaac
Material Supply Adapter. The mock consumes only the accepted event and retains
its `ARRIVED → DOCKED → UNLOADED → DEPARTED` sequence.

The Isaac Material Supply Adapter creates the requested RawParts with
randomized initial poses and applies Physics Drop. It reports delivery-scoped
Transfer Completion for completed creation and simulation action. Production
providers report completion using their environment-defined criteria through
the same contract. Gateway validates provider evidence and VDA context and
alone publishes
quantity-bearing `MATERIAL_READY`. Failure, partial supply, stale/invalid
evidence, or mismatch fails closed.

The shelf construction keeps its ArUco support marker visible outside the
material receiving volume. Vision's existing sensor-derived support reference
and coordinate contracts remain unchanged; sampler-side marker avoidance is
prohibited. Vision separately publishes `SCENE_STABILITY_STABLE` from
consecutive fresh RGB-D changes in the ArUco-anchored material ROI. This result
is distinct from DetectTarget success/depletion and uses no Isaac prim or
physics state. Missing frames, inadequate observation, or continued motion
remain non-stable. Shadow/brightness variation and depth noise are handled by
robust geometric comparisons and configured deadbands; optical flow or object
tracking is not required. Exact shelf, marker, camera, ROI, physics, and
tolerance values remain environment/configuration-owned.

Orchestration consumes quantity-bearing `MATERIAL_READY`, retains a valid
pending delivery while current readiness or Safety permission is absent, and
automatically admits at most one batch per `delivery_id` when its existing
readiness and Safety conditions permit. Admission consumes the delivery
identity even if ExecuteCycle later rejects its goal or execution fails; those
outcomes and their failure provenance belong to Orchestration and do not
reactivate the delivery. The first PICK also waits for fresh
Vision scene stability. At PLACE's `TASK_PHASE_GRIPPER` release start,
Orchestration begins a one-step lookahead: it awaits scene stability and runs
DetectTarget for at most the next material while PLACE finishes. The next PICK
is prohibited until prior PLACE succeeds with fresh `RELEASED` and Safety is
revalidated. Cached detection requires fresh target TTL and unchanged stable
scene generation; otherwise Orchestration re-observes and detects again. It
maps configured `material_type` to
the existing Vision `target_id`/mission recipe boundary and processes until
`processed_count == quantity`. Reaching quantity is normal batch completion;
valid target depletion before quantity is a distinct Orchestration quantity-
shortfall Soft Error, not normal completion. Vision unavailable/invalid/stale
and Motion/Safety failures retain their existing distinct retry/recovery
policies. The Soft Error does not create a Safety state or severity. The Hub does not allocate delivery IDs, request or
manage delivery, assert readiness, or admit work. It focuses on simulation
control, fault injection, and observability; acknowledgement and reset remain
requests to the existing Safety authority.

The common runtime state meanings and fail-closed rules remain unchanged.
Environment-specific timing and performance values may be tuned under their
existing configuration owners. Snap Attach, simplified AMR motion, the Vision,
Motion, and PnP paths, and their intentional simulation-to-real approximations
remain in scope. Simulation evidence does not establish live or hardware
acceptance.

## Consequences

- External producers own job/delivery allocation and `delivery_id` uniqueness;
  ARM Cell preserves identity correlation and rejects duplicates or replayed
  completed identities using its existing in-process admission/episode guards.
- `material_type` is a configured supply/Vision/mission profile key. `RawPart`
  is the current supported value; adding a material maps its adapter supply
  profile and Vision target profile into the existing batch loop rather than
  changing core batch structure. `job_id`, `recipe_id`, scheduling, databases,
  and multi-process manufacturing remain outside this decision.
- The Hub trigger control is a demo source for the same external trigger
  contract, not an ARM-owned supply request. Its result is shown only from
  owner-published state.
- Existing M09/M10 evidence remains historical evidence for its original
  claims. It does not qualify the changed trigger/completion path. M11 live
  acceptance retains its eight scenarios and requires the new path to satisfy
  their unchanged safety, readiness, and observable outcomes.
- The simulation adapter, Physics Drop, observation criteria, and quantity
  outcomes are specified at the ICD/FDS boundary; their runtime realization
  remains for M11-W03. Environment values and live sensor/controller binding
  remain implementation/configuration decisions. The former static RawPart
  pose and elapsed-time readiness are not valid completion evidence.

## Alternatives considered

1. **Keep Hub → Integration → VDA request ownership.** This preserves current
   demos with minimal change but leaves ARM Cell responsible for initiating
   external delivery.
2. **Treat VDA `UNLOADED` or a timer after it as transfer completion.** This is
   simple to simulate but conflates AMR unload with transfer into the ARM Cell
   workspace and cannot establish the required completion evidence.
3. **Consume an external trigger and use quantity-based Physics Drop supply.**
   This preserves external delivery ownership, makes the simulated supply
   result observable, and keeps Gateway readiness and Orchestration
   admission separate. This is the selected direction.

## References

- [ARM Cell Demo Definition](../../product/definition.md)
- [Material Handoff FDS](../../design/components/gateway/fds-material-handoff.md)
- [VDA Material Delivery FDS](../../design/components/vda/fds-material-delivery.md)
- [Material Readiness ICD](../../design/interfaces/icd-integration-orchestration-material.md)
- [External Handoff Trigger ICD](../../design/interfaces/icd-external-handoff-trigger.md)
