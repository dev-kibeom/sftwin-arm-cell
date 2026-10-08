# ARM Cell Demonstration Definition

> **Document Type:** Integrated demonstration authority
> **Status:** Approved
> **Authority:** ARM Cell demo scope, demo-complete meaning, and final live acceptance meaning
> **Last Updated:** 2026-09-28

## 1. Purpose and Authority

This document is the demo-level source of truth for the question:

> What final state is considered demo-complete for the ARM Cell demonstration?

It defines integrated demo intent, observable behavior, scope boundaries, and
the meaning of final live acceptance. It does not define implementation
structure, class or file names, ROS details, UI layout, simulator workarounds,
or component-level algorithms.

The [SF-Twin Product Definition](../../definition.md) retains authority over
whole-product identity, lifecycle, global constraints, and the canonical
registry. Existing capability authorities retain authority over reusable
capability-local requirements. Approved HLD, CDS, FDS, and ICD documents
retain authority over architecture, ownership, interfaces, dynamic design,
safety mechanisms, and calibration. A delivery plan decomposes this demo
meaning into milestones and Work Units; it does not redefine it.

## 2. Demo-Complete Outcome

The ARM Cell demonstration is demo-complete when one or more representative
simulator-backed live sessions collectively pass the required independent
scenario set, demonstrating an eligible batch-processing mission,
operator-visible state and perception observability, severity-aware abnormal
operation, controlled stopping, emergency interruption behavior, holding and
target validity protections, permitted recovery, and durable evidence against
the applicable live-current baseline.

Demo completion is an integrated behavior judgment. It is not dependent on a
particular implementation, UI composition, simulator object identity, or
physical deployment configuration.

## 3. Final Operating Scenario

### 3.1 Material delivery and mission admission

One completed material transfer forms `MATERIAL_READY`. Integration owns
transfer completion and that readiness fact; Orchestration owns batch
admission. A successful valid `MATERIAL_READY` remains pending for its delivery
while Orchestration waits for current general readiness and Safety permission;
Orchestration admits automatically when those conditions are satisfied.
Invalidated or expired readiness blocks admission. Waiting for these conditions
does not require another material delivery request; no separate operator
start/admission action exists. Arrival, docking, or unload alone does not admit a
batch. Failed delivery/transfer never establishes `MATERIAL_READY` and
prohibits batch admission.

The final demonstration may use a simulated AMR or an equivalent
material-delivery mechanism. AMR hardware itself is not required for ARM Cell
demo completion.

One material delivery starts exactly one batch-processing mission. A second
AMR delivery during an active batch is outside this demo scope and does not
append work to or preempt the batch. The AMR demo uses predefined short motion
to visualize arrive → dock → unload → depart; AMR navigation is not
implemented. AMR/VDA owns arrival, docked, and unload facts; Integration owns
transfer completion / `MATERIAL_READY`; Orchestration owns batch admission.
Conveyor/transfer environment meaning and its non-authority are defined by
[ARM Cell Simulation Environment Semantics](../../../components/arm-cell/simulation-environment-semantics.md).

### 3.2 Batch mission loop

While valid targets remain, the batch repeatedly performs the following
observable progression, without returning home between iterations:

1. acquire the next target;
2. perform PICK;
3. verify holding;
4. perform PLACE;
5. verify release;
6. acquire the next target.

Only valid-target depletion allows the normal batch-completion GO_HOME and
batch completion. A permitted post-interruption recovery uses a distinct
GO_HOME task before fresh perception and does not complete the batch.
Vision errors, stale/invalid targets, communication degradation, and other
abnormal outcomes remain distinct from depletion. Batch context (delivery,
admission, processed count, terminal provenance) is separate from each
iteration's selected target, PICK attempt budget, holding/release outcome, and
iteration outcome. Retry, failure, and recovery semantics are owned by the
Orchestration dynamic design.

For this demo, PLACE is complete after the placement behavior defined by
Motion/Orchestration and release verification. Inspecting the object's final
settled pose after release and judging it against process-quality tolerance
are outside this demo's completion scope. A future Process Verification
capability may own that observation and judgment; adding it does not change
the existing demo-complete meaning.

The demo definition intentionally does not prescribe the algorithm that
distinguishes depletion from an abnormal no-target condition. Those meanings
must be made explicit by the subsequent dynamic design authority and remain
distinct from sensor, geometry, communication, or other abnormal outcomes.

## 4. Simulation UI Hub

The canonical operator surface is exactly one Isaac Simulation UI Hub. A
separate Qt HMI is outside this demo scope. The Hub is an observer and
scenario/request surface, not a system authority; it cannot write or override
canonical Safety, Motion, Orchestration, or Vision state.

At minimum, the Hub provides demo-level capability to:

- observe current system, mission, safety, material, fault, and recovery state;
- observe AMR/material state, batch/mission state, processed count, selected
  target, holding state, Safety capability, active motion envelope, active
  fault/severity, retry count, recovery/reset-required state, and PackML/CNC
  process state;
- observe the Vision sensor/camera view;
- observe annotations or overlays for detected objects and the selected target;
- initiate representative material supply or simulated delivery events;
- inject predefined fault scenarios;
- request operator acknowledgement or reset;
- request material supply, fault injection, acknowledgement, or reset as
  stimuli only; the owning runtime authority evaluates and applies each
  request;
- observe active degradation, operating restriction, fault, and recovery
  provenance.

The Hub MUST NOT directly overwrite or force canonical Safety, Motion, Vision,
or other component state. Canonical state remains owned by the applicable
system authority.

Specific widgets, controls, labels, layout, and styling are not demo
authority.

## 5. Vision Observability

During the final live demonstration, an operator can establish from the Vision
perspective:

- the sensor or camera view;
- detected objects;
- the selected target;
- whether the detection is valid and sufficiently fresh;
- which target is actually being used by the mission.

The representation may use annotations, overlays, or an equivalent observable
form. This definition does not require bounding boxes, segmentation masks, or a
particular computer-vision algorithm.

## 6. Fault-Aware Operation

Representative abnormal conditions are assessed and handled according to an
operational severity model. Severity and response are related but are not a
fixed one-to-one product enum or response table:

```text
abnormal condition
    → severity assessment
    → permitted operating envelope
    → condition/component-specific response
```

The demo behavior includes these meanings:

- a minor abnormality may remain observable as an advisory condition;
- degraded operation may continue when the permitted envelope remains safe;
- degraded operation may apply defined restrictions such as reduced speed,
  limited function, or restricted mission admission;
- when reliable progression cannot continue, progression is inhibited and a
  controlled stop or abort is performed;
- in an emergency condition, Safety authority takes precedence and enforces the
  required stop behavior immediately.

Safety authority always precedes operational or degraded policy. Severity names,
enum values, exact thresholds, and component-specific responses belong to the
applicable safety and dynamic design authorities.

Safety owns motion capability and the currently permitted motion envelope.
Motion consumes the envelope without interpreting fault condition or severity.
The envelope can constrain at least velocity and acceleration. Active
degradation applies to active motion; if its backend cannot safely enforce a
new restriction mid-trajectory, the Safety-owned stop/replan path is used.
Exact scaling values belong to configuration/calibration authority.

### 6.1 Representative fault classes

The demo scope includes representative coverage of these fault classes
without prescribing an exhaustive test-case list:

- safety interruption or emergency stop;
- communication degradation or loss of freshness;
- Motion or backend abnormal condition;
- gripper or holding uncertainty;
- Vision or target unavailability/invalidity;
- mission/admission conflict or invalid material condition;
- recoverable and non-recoverable abnormal conditions.

### 6.2 Communication degradation

When communication delay or freshness degradation is injected during the final
live demonstration, the system can show that:

- the degraded condition is observable;
- where permitted, operation transitions to reduced-speed or otherwise
  restricted operation;
- when reliable operation can no longer be maintained, further progression is
  prohibited or escalates to stop/abort.

Latency thresholds, speed percentages, timeout values, and equivalent
calibration parameters are not defined here. They belong to later design and
calibration authority.

## 7. Recovery Capability

Removing a fault does not by itself resume RUN or restart a mission. E-stop
interrupts the active task and iteration; that task is never resumed.

For representative recoverable conditions, the final demo has a recovery
meaning that includes:

1. removal or clearance of the fault condition;
2. an acknowledgement or reset request where required;
3. readiness re-evaluation;
4. permitted recovery or restart;
5. return to an eligible operational state.

For an E-stop/hard fault, explicit operator clear plus acknowledgement/reset
is followed by readiness re-evaluation. If permitted, recovery performs
GO_HOME then fresh perception before batch continuation. Only fresh `RELEASED` may permit automatic continuation. After `HELD`,
operator intervention must safely handle the object and produce a fresh
`RELEASED` disposition before recovery progression. `UNKNOWN` prohibits
automatic motion/recovery and remains fail-closed until operator intervention
produces a fresh safe disposition; continuation still requires fresh
`RELEASED`. Recovery success preserves the original failure provenance. Detailed eligibility and sequence
remain in the Orchestration and Safety FDS owners.

## 8. Final Live Acceptance

Demo Complete requires every required independent scenario below to PASS
across one or more live Isaac Sim sessions against the applicable live-current
profile. No single long single-run script substitutes for the scenario set.
Each scenario record includes precondition, stimulus, expected transition,
observable oracle, PASS condition, and non-claims.

| Scenario | Preconditions | Stimulus | Expected transition | Observable oracle | PASS condition | Non-claims |
|---|---|---|---|---|---|---|
| Nominal batch | Production Vision, valid transfer, one batch with multiple valid targets | Request one material delivery | `MATERIAL_READY` → admission → repeated Detect/PICK/HELD/PLACE/RELEASED without GO_HOME → depletion → one GO_HOME → complete | Hub camera/overlay, selected target, count, task order, depletion, GO_HOME result | All targets processed; depletion distinguished; terminal GO_HOME succeeds; batch completes | No AMR navigation or physical deployment claim |
| Degraded communication / reduced speed | Active motion; configured degraded band and reduced envelope | Inject recoverable freshness degradation | Safety publishes degradation and tighter limits; active task is safely applied or stopped/replanned; permitted continuation runs under the reduced envelope | Safety envelope/severity, stop evidence if needed, a subsequent reduced-limit trajectory, and unchanged retry count in Hub | At least one permitted trajectory runs under the reduced velocity/acceleration limits; no motion exceeds active limits; degradation alone consumes no retry | No exact demo-defined scaling or network threshold |
| Retryable soft failure | One target episode | Inject temporary target unavailability or retry-safe unheld PICK failure, then restore validity | Attempt ends; fresh perception/reselection and new PICK identity; retry counter advances | Vision freshness/selected pose, retry count, PICK goal identities | Success within four total attempt slots; no stale goal replay | No retry for held/UNKNOWN, config error, persistent fault, or Safety preemption |
| Controlled stop | Active task | Inject a representative non-emergency condition requiring stop | Safety selects severity and directly stops; Motion confirms inactivity | Safety cause/severity, StopMotion, Motion inactivity and task sequence | No subsequent task advances before stop confirmation; provenance retained | No safety-rated certification |
| E-stop + operator recovery | Motion active | Assert E-stop, clear cause, explicitly acknowledge/reset | Task/iteration terminates; reset-required remains until accepted; readiness re-evaluates; interrupted task not resumed | E-stop/Safety state, operator request/result, Motion stop, later task identity | Recovery begins only after operator action and current Safety readiness | Software demo does not validate safety-rated E-stop hardware |
| Holding uncertainty | PICK path active | Return fresh `UNKNOWN` holding or allow observation freshness to expire | PLACE and automatic continuation are blocked | MotionStatus holding value/stamp and absence of PLACE request | No PLACE or batch progression occurs | No physical force-sensing accuracy claim |
| Invalid/stale/unavailable target | Production Vision running | Inject invalid/stale/unavailable input through approved fault seam | Freshness/validity observable; no unsafe PICK; never mapped to depletion | Camera/overlay, observation stamp, validity, selected target, task requests | Abnormal result follows bounded retry or terminal policy | Fixed Vision is not production perception evidence |
| Recovery then batch continuation | Batch interrupted after known processed count; holding is RELEASED | Operator clears cause and requests ack/reset | Original cause remains visible; new GO_HOME then fresh perception starts a new iteration in retained batch | Failure provenance, holding, new task and observation identities, batch count | No task/pose replay; batch context and failure provenance preserved | HELD/UNKNOWN do not auto-continue |

The eight scenarios are individually resettable and independently assessed.
Their evidence across one or more live sessions collectively closes
acceptance. Demo Complete may be claimed only when all eight pass against the
applicable live-current baseline.

### Evidence

Durable evidence is generated to support the accepted live-current claims,
including the observed baseline identity, scenario/provenance, limitations,
and invalidation conditions required by the repository verification policy.

Live evidence is claim-scoped: synthetic or historical evidence may support
non-live claims but is not silently promoted to current live acceptance.

## 9. Scope Boundary and Non-Goals

The following are not mandatory conditions of ARM Cell demo completion:

- actual AMR hardware;
- AMR navigation;
- a real factory network;
- a separate Qt HMI;
- CNC animation;
- hardware FAT/SAT;
- production-grade UI styling;
- a particular computer-vision algorithm;
- a particular ROS implementation detail;
- a particular latency threshold or speed percentage;
- a particular Isaac prim path;
- a production-hardware gripper implementation.

The demo retains a semantic boundary that permits future hardware, network,
gripper, and material-delivery integration without changing the demo meaning of
mission, target, holding, release, safety, fault, or recovery behavior.

## 10. Downstream Authority and Current Gap

This demo definition is refined by the approved design authorities, which must
preserve its demo meaning without importing implementation details into the
demo contract:

- [system and safety HLD](../../../architecture/system-overview.md) and
  [safety/control architecture](../../../architecture/safety-and-control.md);
- [Orchestration CDS/FDS](../../../components/arm-cell/orchestration/cds.md) and
  [mission-cycle FDS](../../../components/arm-cell/orchestration/fds-mission-cycle.md);
- [Orchestration controlled recovery FDS](../../../components/arm-cell/orchestration/fds-controlled-recovery.md);
- [Safety CDS/FDS](../../../components/arm-cell/safety/cds.md) and
  [safety supervision FDS](../../../components/arm-cell/safety/fds-supervision.md);
- [Motion CDS/FDS](../../../components/arm-cell/motion/cds.md),
  [Motion task execution FDS](../../../components/arm-cell/motion/fds-task-execution.md),
  and [Motion backend ICD](../../../interfaces/arm-cell/icd-motion-backend.md);
- [Vision CDS/FDS](../../../components/arm-cell/vision/cds.md) and
  [Vision↔Orchestration ICD](../../../interfaces/arm-cell/icd-vision-orchestration.md);
- [Integration CDS](../../../components/arm-cell/integration/cds.md) and the
  [simulation-session FDS](../../../components/arm-cell/integration/fds-sim-session.md),
  [material handoff FDS](../../../components/arm-cell/integration/fds-material-handoff.md),
  and [UI Hub FDS](../../../components/arm-cell/integration/fds-operator-ui-hub.md);
- [VDA material delivery FDS](../../../components/arm-cell/vda/fds-material-delivery.md);
- [Safety↔Motion ICD](../../../interfaces/arm-cell/icd-safety-motion.md) and
  [Safety↔Orchestration ICD](../../../interfaces/arm-cell/icd-safety-orchestration.md)
  and [Safety Operator ICD](../../../interfaces/arm-cell/icd-safety-operator.md);
- [Material Readiness ICD](../../../interfaces/arm-cell/icd-integration-orchestration-material.md) and
  [ExecuteCycle ICD](../../../interfaces/arm-cell/icd-orchestration-execute-cycle.md).
- [Integration↔VDA Material ICD](../../../interfaces/arm-cell/icd-integration-vda-material.md).
- [ARM Cell Simulation Environment Semantics](../../../components/arm-cell/simulation-environment-semantics.md).

The linked Approved FDS/ICD documents refine the dynamic design and identify
contract/schema changes that remain to be implemented. Key open contract
dependencies are ROS realizations for VDA material delivery, `MATERIAL_READY`
delivery identity, a temporary-invalid-target Vision outcome, additive batch
feedback, Motion holding observation and Safety motion-envelope fields, plus
an operator-to-Safety reset request. CNC indicator projection semantics are referenced in [ARM Cell Simulation
Environment Semantics](../../../components/arm-cell/simulation-environment-semantics.md);
future shared-zone coordination is a non-normative extension point. Production
Vision, live scenario evidence, and required runtime behavior remain
implementation/acceptance work. This definition does not create delivery
Work Units or implementation tasks.
