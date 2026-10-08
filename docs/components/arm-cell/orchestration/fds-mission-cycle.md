# [SF-Twin] ARM Cell Batch Mission and Iteration Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-ORCHESTRATION-MISSION-CYCLE_v1.3.0`
- **Document Type:** `FDS`
- **Scope:** material-triggered Final Demo batch mission
- **Version:** `1.3.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Orchestration`

The `/orchestration/execute_cycle` execution contract is defined by [ExecuteCycle ICD](../../../interfaces/arm-cell/icd-orchestration-execute-cycle.md). In Final Demo, Orchestration invokes it internally only after event-driven batch admission; it is not exposed as an operator start action. Cross-cutting scene preconditions are owned by [ARM Cell Simulation Environment Semantics](../simulation-environment-semantics.md).

## 1. Batch Mission

One admitted material delivery creates exactly one batch mission. The batch
context owns the delivery/admission identity, requested recipe/profile, batch
processed count, and terminal batch outcome. Each iteration has a separate
iteration context containing its selected target observation, PICK attempt
budget and count, holding/release outcomes, and iteration outcome. Iteration
context is discarded only after its outcome has been committed to the batch.

A target episode is one logical piece-selection episode within a batch. It
begins with the first DetectTarget attempt for the next piece and ends only
when that piece is successfully placed/released or the batch terminates. The
selected physical observation may change across retries; the episode budget
does not.

Additional AMR delivery during an active batch is outside the ARM Cell demo
scope and SHALL NOT implicitly create, append to, or preempt the active batch.
When it accepts a batch, Orchestration records the delivery identity as
consumed and SHALL reject any duplicate admission for that delivery while the
Orchestration instance is running. Within a running Orchestration instance,
one delivery identity is admitted at most once.

### NF-01: Process one admitted batch

A successful, valid `MATERIAL_READY` fact is retained as pending readiness
for its delivery until admission, invalidation, or expiry. If current general
readiness or Safety capability does not yet permit operation, Orchestration
retains the delivery and automatically re-evaluates it as conditions change;
condition waiting does not discard the delivery or require another material
supply request. Orchestration automatically admits exactly one batch when all
conditions are satisfied; no operator/Hub batch-admission action is required
for Final Demo. Invalidated or expired readiness blocks admission. A fresh
positive readiness confirmation from Integration for the same delivery may
re-establish readiness; it does not create a new delivery. After admission,
Orchestration starts the batch execution through its internal ExecuteCycle boundary. Within the running Orchestration instance, a delivery identity is consumed at most once. Orchestration
uses its configured Final Demo recipe; recipe/contract error terminates the
batch with `MISSION_EXIT_CONFIGURATION_ERROR` before submitting Motion. A
delivery or readiness fact alone never bypasses the current Safety gate.

```text
valid, current MATERIAL_READY pending + general readiness + Safety permits normal motion
→ automatically admit one batch
→ repeat iteration: fresh DetectTarget → PICK → confirm HELD → PLACE → confirm RELEASED
→ confirm valid-target depletion
→ GO_HOME once
→ batch complete
```

No GO_HOME occurs between successful iterations. Only Vision's
`DETECT_RESULT_OBJECT_NOT_FOUND` from a valid configured production perception
profile establishes depletion. Invalid profile, invalid/stale result,
unavailable Vision, timeout, or sensor/TF/geometry failure is never depletion.
When depletion is established, Orchestration performs GO_HOME and completes
the batch only after GO_HOME succeeds. The demo nominal path uses production
Vision; the validation-only Fixed Vision adapter is not a Final Demo nominal
perception source. Fault scenarios may use an approved simulation or
fault-injection seam while retaining explicit provenance.

### Iteration transitions

| Current phase | Event / guard | Next phase / outcome |
|---|---|---|
| Batch admitted | general readiness and normal capability remain permitted | DETECT |
| DETECT | fresh valid target selected | PICK attempt |
| DETECT | valid profile returns OBJECT_NOT_FOUND | batch depletion confirmation, then GO_HOME |
| DETECT | terminal temporary invalid/unavailable result, or terminal timeout/freshness failure followed by communication recovery | consume one failed attempt slot; retry requires fresh recovered observation |
| Active attempt | communication degrades but operation remains permitted within the Safety envelope | continue under the active envelope; no retry episode and no budget consumption |
| DETECT | `INVALID_RESULT` profile/configuration error, stale/invalid remains, or persistent communication/backend fault | terminal batch failure; never depletion |
| PICK | Motion reports task success only after fresh HELD confirmation | PLACE |
| PICK | terminal timeout/freshness failure after communication recovery, or retry-safe planning/execution failure while fresh holding is RELEASED | retry policy; fresh Detect and a new PICK identity |
| PICK / PLACE | holding UNKNOWN | terminal controlled abort; no automatic continuation |
| PLACE | Motion reports task success only after fresh RELEASED confirmation | increment processed count; begin next DETECT |
| PLACE | release not confirmed | terminal controlled abort / intervention |
| Any active task | Safety preemption / E-stop | interrupt task and iteration; recovery is separate |
| Depletion confirmed | GO_HOME succeeds | batch complete |

### SEQ-ORCH-ADMISSION-01 — Material admission arbitration

**Primary owner:** this Mission Cycle FDS. The
[Material Readiness ICD](../../../interfaces/arm-cell/icd-integration-orchestration-material.md)
owns the cross-boundary readiness and admission identity contract; this
sequence clarifies how Orchestration evaluates competing current facts.

At Orchestration's local admission decision commit boundary, it revalidates
the required conditions against its current state: positive readiness for the
delivery remains valid and unexpired under Orchestration's local
receipt-time freshness evaluation; general readiness permits operation;
Safety permits normal motion; and the delivery identity is not consumed. If
all conditions hold at that decision boundary, Orchestration admits exactly
one batch. The decision is observable as one admission/ExecuteCycle identity
for that delivery; no distributed atomic snapshot is implied.

```mermaid
sequenceDiagram
  autonumber
  participant Integration
  participant Orch as Orchestration
  participant General as General readiness
  participant Safety

  Integration-->>Orch: MATERIAL_READY(delivery_id, positive, valid)
  par readiness and permission facts can change concurrently
    General-->>Orch: general readiness change
  and
    Safety-->>Orch: Safety permission change
  and
    Integration-->>Orch: explicit readiness invalidation/update (same delivery_id)
  and
    Orch->>Orch: local receipt-time freshness evaluation finds readiness expired
  and
    Integration-->>Orch: duplicate readiness event (same delivery_id)
  end
  Orch->>Orch: revalidate required conditions against local current state at admission commit boundary
  alt readiness remains positive, valid and unexpired AND general ready AND Safety permits normal motion AND identity not consumed
    Orch->>Orch: consume delivery identity; admit exactly one batch
    Orch->>Orch: start ExecuteCycle for delivery_id
    Integration-->>Orch: later duplicate readiness for delivery_id
    Orch->>Orch: retain consumed identity in this instance; no second admission in this instance
  else readiness invalid/expired or another admission condition false at revalidation
    Orch->>Orch: do not admit; retain no authority from stale readiness
    Note over Orch: Later Safety/general readiness improvement cannot admit using invalidated/expired readiness.
    opt fresh positive reconfirmation for same delivery_id
      Integration-->>Orch: fresh positive MATERIAL_READY
      Orch->>Orch: re-evaluate all current conditions and unconsumed identity
      alt all admission conditions now hold
        Orch->>Orch: consume identity; admit exactly one batch
      else one or more conditions remain false
        Orch->>Orch: do not admit
      end
    end
  end
```

#### Sequence verification linkage

| Sequence boundary | Existing/new VR(s) clarified | Observable oracle |
|---|---|---|
| `MATERIAL_READY` is necessary; general readiness and Safety are reevaluated at admission | `VR-ICD-MATERIAL-01` | readiness event identity/validity, current general-readiness and Safety observations, and whether a batch admission occurs |
| failed/incomplete transfer cannot provide positive readiness or admission | `VR-ICD-MATERIAL-02` | transfer result, absence of positive readiness, and absence of admission for the delivery |
| duplicate events and concurrent reevaluation cannot admit twice within one running Orchestration instance | `VR-ICD-MATERIAL-03` | one delivery identity and admission/ExecuteCycle count no greater than one for that instance |
| invalidation/expiry blocks stale admission; fresh same-identity confirmation can revalidate once | `VR-ICD-MATERIAL-04` | ordered validity/freshness changes, fresh confirmation identity, and at-most-one admission within one running Orchestration instance |

The observable result is normative; this sequence does not prescribe
serialization, locking, callback, or executor mechanisms.

### SEQ-ORCH-MISSION-01 — ExecuteCycle mission flow

**Primary owner:** this Mission Cycle FDS. The [Orchestration dynamic-design
entry point](fds.md) and participating ICDs reference this sequence; they do
not own an alternate end-to-end workflow.

```mermaid
sequenceDiagram
  autonumber
  participant Integration as Integration
  participant Orch as Orchestration
  participant Hub as UI Hub (observer)
  participant Safety as Safety
  participant Vision as Vision
  participant Motion as Motion

  Integration->>Orch: successful transfer publishes MATERIAL_READY
  Orch->>Safety: evaluate current SafetyState and general readiness
  alt admission permitted
    Orch->>Orch: automatically admit one batch for delivery identity
    Orch->>Orch: ExecuteCycle(configured target_id, delivery_id)
    loop while batch remains active
      Orch->>Vision: DetectTarget(target_id)
      Vision-->>Orch: result
      alt OBJECT_NOT_FOUND
        Orch->>Safety: re-observe current Safety at terminal decision
        alt normal motion still permitted
          Orch->>Motion: ExecuteTask(GO_HOME)
          Motion-->>Orch: task result
          Orch-->>Hub: batch complete / MISSION_EXIT_DEPLETED
        else Safety preempts normal motion
          Orch-->>Hub: MISSION_EXIT_SAFETY_PREEMPTED
        end
      else Vision error
        alt TEMPORARY_INVALID_TARGET or transient TIMEOUT and budget remains
          Note over Orch,Vision: wait for validity/freshness recovery, then fresh DetectTarget within the same target episode
        else terminal Vision error or retry budget exhausted
          Orch->>Safety: re-observe current Safety at terminal decision
          alt Safety preempts normal motion
            Orch-->>Hub: MISSION_EXIT_SAFETY_PREEMPTED
          else normal motion still permitted
            Orch-->>Hub: MISSION_EXIT_VISION_ERROR
          end
        end
      else successful object result
        Orch->>Orch: load and validate mission recipe
        alt recipe missing or invalid
          Note over Orch,Motion: No incomplete PICK or PLACE goal is submitted
          Orch->>Safety: re-observe current Safety at terminal decision
          alt Safety preempts normal motion
            Orch-->>Hub: MISSION_EXIT_SAFETY_PREEMPTED
          else current Safety permits normal motion
            Orch-->>Hub: MISSION_EXIT_CONFIGURATION_ERROR
          end
        else recipe valid
          Orch->>Motion: ExecuteTask(PICK)
          Motion-->>Orch: task result
          alt PICK succeeds with fresh HELD
            Note over Orch,Motion: Motion PICK success entails fresh HELD confirmation
            Orch->>Motion: ExecuteTask(PLACE)
            Motion-->>Orch: task result
            alt PLACE succeeds with fresh RELEASED
              Note over Orch,Motion: Motion PLACE success entails fresh RELEASED confirmation
              Orch->>Orch: increment processed count; begin next target episode
            else release not confirmed
              Orch-->>Hub: MISSION_EXIT_MOTION_ERROR; terminal batch failure
            end
          else retry-safe PICK failure while RELEASED and budget remains
            Note over Orch,Safety: Before retry commit, recheck prior attempt terminal, budget available, fresh RELEASED, recovered/fresh perception prerequisites, and current normal-motion permission.
            alt Safety preempts at retry decision boundary
              Orch-->>Hub: MISSION_EXIT_SAFETY_PREEMPTED; do not commit retry
            else all retry conditions remain satisfied
              Note over Orch,Vision: commit new attempt slot; fresh DetectTarget and a new PICK goal, never replay
            end
          else non-retryable PICK failure or budget exhausted
            Orch-->>Hub: MISSION_EXIT_MOTION_ERROR; preserve originating cause
          end
        end
      end
    end
  else Safety does not permit normal motion
    Orch-->>Hub: MISSION_EXIT_SAFETY_PREEMPTED
  end

  opt caller cancellation or Motion terminal failure/cancel
    Orch->>Safety: re-observe current Safety at terminal decision
    alt Safety preempts normal motion
      Note over Orch,Hub: enter Controlled Recovery FDS; keep batch waiting if continuation remains permitted, otherwise terminal MISSION_EXIT_SAFETY_PREEMPTED
    else caller cancellation observed
      Orch-->>Hub: MISSION_EXIT_CANCELED
    else Motion failure or cancellation is not caller-attributed
      Orch-->>Hub: MISSION_EXIT_MOTION_ERROR
    end
  end
  Note over Orch,Motion: In uninterrupted processing, no GO_HOME occurs before depletion or between iterations; one terminal GO_HOME follows depletion. Recovery GO_HOME is a separate permitted continuation task. Bounded retry is fresh-perception/new-PICK only; Safety recovery never resumes the interrupted task.
```

The successful task path in this diagram is intentionally contract-level.
Motion-internal planning, tool conversion, gripper realization, and task phase
details remain owned by Motion and are not specified here.

When Safety interrupts an active task, the task/iteration becomes terminal and
the batch Action enters `MISSION_PHASE_WAITING_RECOVERY`. The
[Controlled Recovery FDS](fds-controlled-recovery.md) owns stop, operator
reset, holding disposition, and continuation ordering. A permitted recovery
returns to this batch flow at a new iteration; an interrupted task is never
resumed. A terminal Safety-preempted result is reported only when continuation
is denied or the batch is terminated.

#### Sequence verification linkage

| Sequence boundary | Existing VR(s) clarified | Observable oracle / deterministic seam |
|---|---|---|
| `OBJECT_NOT_FOUND` revalidation before `FINISHED` / `DEPLETED` | `VR-ORCH-MISSION-01`, `VR-ORCH-MISSION-05`, `VR-ORCH-MISSION-07` | public `MissionPhase` ordering and terminal `MissionExitReason`, with controllable current Safety state and Vision result |
| Vision error and Motion terminal precedence | `VR-ORCH-MISSION-02`, `VR-ORCH-MISSION-05`, `VR-ORCH-MISSION-08` | terminal `MissionExitReason`, with controllable Vision/Motion outcome and current Safety state |
| Recipe failure before PICK/PLACE submission | `VR-ORCH-MISSION-04`, `VR-ORCH-MISSION-05`, `VR-ORCH-MISSION-06` | `MISSION_EXIT_CONFIGURATION_ERROR` or Safety preemption, and absence of the corresponding submitted logical Motion goal |
| bounded retry and terminal outcome | `VR-ORCH-MISSION-03`, `VR-ORCH-MISSION-04`, `VR-ORCH-MISSION-09`, `VR-ORCH-MISSION-10` | request identity/count, fresh target observation, task order, batch result, and absence of work after terminal outcome |

The ExecuteCycle Action payload and terminal transport mapping remain owned by
the [ExecuteCycle ICD](../../../interfaces/arm-cell/icd-orchestration-execute-cycle.md).

## 2.1 Mission recipe ownership

Orchestration owns the backend-neutral mission recipe selected by `target_id`.
The target-specific recipe SHALL provide the logical `grasp_width_mm` and the
required `grasp_yaw_rad` for PICK, expressed about `base_link` +Z. The recipe
yaw is used only when the explicit Vision yaw-presence semantic is false. The
current recipe also provides the PLACE Desired Object Pose, position
tolerance, orientation constraint mode (`fixed`, `bounded`, or `free`),
orientation tolerance, optional preferred TCP orientation, and object-frame
approach/retract geometry. `fixed` preserves nominal object orientation;
`bounded` permits orientation candidates within tolerance; `free` means object
orientation is not a process constraint. In free mode Motion performs a small
deterministic candidate search including side-on poses and selects feasible
poses by motion efficiency. It is not exhaustive/global optimization. Tool
orientation preference may rank fixed/bounded candidates only and is absent
for free recipes. Position and position-tolerance meaning is unchanged.
Orchestration SHALL send the selected recipe semantics together in the PLACE
Motion goal.
Vision provides only the observed object pose and geometry for the current
target; it does not provide the PLACE destination or mission grasp-width
policy. Motion receives these logical object-centric values and owns
TCP/tool/approach/retract conversion and all backend-specific gripper
realization. PICK approach/retract geometry remains separately configured for
PICK and SHALL NOT be sourced from PLACE recipe values.

For PICK resolution, Orchestration SHALL merge the valid Vision object
position with target-recipe values before emitting the Motion goal: Vision
owns `x/y/z`, Vision yaw takes precedence when `has_target_yaw=true`, recipe
`grasp_yaw_rad` is the fallback when it is false, and recipe owns grasp width.
Orchestration SHALL NOT calculate TCP/tool geometry. Vision `z` remains the
authoritative observed height/location component; `estimated_height_m` is
non-authoritative and unused for PICK geometry. It SHALL NOT modify, repair,
offset, replace, or otherwise participate in object-reference to TCP pose
generation.

The recipe seam SHALL remain replaceable by a future perception or
grasp-planning source without changing the Motion public contract. A missing
recipe for the requested `target_id`, a missing/invalid required
`grasp_yaw_rad` for PICK, or an invalid configured grasp width, PLACE pose, or
required PLACE frame, SHALL prevent Orchestration from emitting
the corresponding incomplete Motion goal. These are Orchestration-owned
configuration failures and SHALL produce `MISSION_EXIT_CONFIGURATION_ERROR`,
not `MISSION_EXIT_MOTION_ERROR`. Invalid values include non-finite logical
fields, a PLACE pose with a zero-norm orientation, or a present tool
orientation policy whose mode is unsupported or whose quaternion is not finite
and normalized.

## 2. Depletion

Only Vision `OBJECT_NOT_FOUND` is mapped to normal depletion.

```text
OBJECT_NOT_FOUND
→ terminal Safety revalidation
→ normal capability: perform one terminal GO_HOME
→ GO_HOME succeeds: DEPLETED / successful batch completion
→ preempted capability: SAFETY_PREEMPTED
→ GO_HOME fails: MISSION_EXIT_MOTION_ERROR
```

Other Vision failures SHALL NOT be converted to depletion.

## 3. Component Failure Mapping

| Component outcome | Mission result |
|---|---|
| `OBJECT_NOT_FOUND` + successful terminal GO_HOME | `MISSION_EXIT_DEPLETED` / normal successful batch termination |
| non-retryable or exhausted-retry Vision failure | `MISSION_EXIT_VISION_ERROR`, not depletion |
| non-retryable or exhausted-retry Motion failure, including Motion cancellation without observed caller cancellation | `MISSION_EXIT_MOTION_ERROR` |
| Safety preemption with no permitted batch continuation | `MISSION_EXIT_SAFETY_PREEMPTED` |
| observed ExecuteCycle caller cancellation | `MISSION_EXIT_CANCELED` |
| missing or invalid Orchestration mission recipe | `MISSION_EXIT_CONFIGURATION_ERROR` |

The corresponding canonical public exit values are `MISSION_EXIT_DEPLETED`, `MISSION_EXIT_VISION_ERROR`, `MISSION_EXIT_MOTION_ERROR`, `MISSION_EXIT_SAFETY_PREEMPTED`, `MISSION_EXIT_CANCELED`, and `MISSION_EXIT_CONFIGURATION_ERROR`.

Any non-Safety terminal outcome, including caller cancellation, a Vision
result, a Motion result, depletion, or configuration error, SHALL NOT be
committed without a current Safety observation at the terminal decision
boundary. Orchestration SHALL re-observe current Safety at that boundary. If
normal motion is preempted, Orchestration SHALL return
`MISSION_EXIT_SAFETY_PREEMPTED`; otherwise it SHALL commit the applicable
non-Safety result. This local decision rule does not require impossible atomic
observation across the distributed system.

`TASK_RESULT_CANCELED` from Motion SHALL map to `MISSION_EXIT_CANCELED` only
when Orchestration has observed ExecuteCycle caller cancellation. Otherwise it
is a Motion terminal failure and SHALL map to `MISSION_EXIT_MOTION_ERROR`.

## 4. Bounded Iteration Retry Policy

Each target episode has one initial attempt and at most three retries, for at
most four attempts total. An attempt begins with fresh perception; if it
returns a valid target, that attempt may submit one PICK goal. Temporary
unavailable/invalid perception consumes an attempt slot even when no PICK goal
can be submitted. Therefore the bound is both four attempt slots and at most
four PICK goals. The budget resets only when a new target episode begins.
Batch-level or mission-level automatic retry is prohibited.

A retry is a new attempt, not command replay. Immediately before committing a
retry, Orchestration rechecks that the prior attempt is terminal, retry budget
is available, the object is freshly confirmed `RELEASED`, recovered/fresh
perception prerequisites hold, Safety currently permits normal motion, and
the failure is in an approved retryable class. If Safety preempts at this
decision boundary, Safety takes precedence and the retry is not committed.
After the retry is committed, Orchestration performs fresh perception,
reselects a valid target, resolves a new PICK input, and submits a new PICK
goal with a new action identity.
Stale target poses and identical PICK goal replay are prohibited.

Retryable conditions are a terminal Vision `DETECT_RESULT_TEMPORARY_INVALID_TARGET`
or `DETECT_RESULT_TIMEOUT`/freshness failure, or a terminal PICK
timeout/freshness failure, only after communication has recovered. A retry-safe
PICK planning/execution failure is also retryable while the object is freshly
confirmed `RELEASED`. Degradation observations alone are not failures
and do not consume retry budget. Non-retryable
conditions are E-stop or Safety preemption, holding UNKNOWN, configuration or
contract error, persistent backend fault, or exhausted retry budget.
Vision stale/invalid outcomes may be retried only after freshness/validity has
recovered and the fresh result is valid; they never count as depletion.

After four unsuccessful attempts, the batch terminates as a controlled abort
or failure and preserves the originating failure classification. It does not
continue to a later target or report normal depletion.

## 5. Verification Requirements

### VR-ORCH-MISSION-01 — Depletion-only success
Given Vision returns `OBJECT_NOT_FOUND`, the batch SHALL enter the terminal
GO_HOME path. Only successful GO_HOME SHALL produce normal completion with
`MISSION_EXIT_DEPLETED`. A different Vision failure SHALL NOT produce
depletion.

### VR-ORCH-MISSION-02 — Failure preservation
Given a failure or Safety interruption terminates an attempt/task, that
attempt/task outcome SHALL remain separately observable. Recovery success
SHALL NOT mark the interrupted task/iteration successful. A later batch result
may complete only through new task identities and fresh perception, while
retaining the interruption provenance.

### VR-ORCH-MISSION-03 — Bounded fresh retry
For one target episode, Orchestration SHALL perform no more than four attempt
slots, including the initial attempt. Each retry slot SHALL follow retry
safety checks and fresh perception/target reselection; when a valid target is
returned, it SHALL create a new PICK goal. No task goal SHALL be replayed and
retry budget SHALL NOT reset within the same target episode.

### VR-ORCH-MISSION-04 — Mission recipe boundary
Given a target-specific mission recipe, Orchestration SHALL forward its logical
grasp width and required fallback yaw on PICK and its configured Desired Object
Pose and process tolerance on PLACE. Vision SHALL override that fallback yaw only when its
explicit yaw-presence semantic is true. Vision SHALL remain the source of PICK
object pose/geometry only, and
Orchestration SHALL not issue RETRACT as part of WU-09 Safety-preemption
coordination.

### VR-ORCH-MISSION-05 — Terminal Safety precedence
Given any competing non-Safety terminal outcome, including caller cancellation,
a Vision result, a Motion result, depletion, or configuration error, when Safety
preempts normal motion at the terminal-decision boundary, Orchestration SHALL
return `MISSION_EXIT_SAFETY_PREEMPTED` rather than the competing non-Safety
result.

### VR-ORCH-MISSION-06 — Configuration failure boundary
Given a missing or invalid target-specific mission recipe, Orchestration SHALL
return `MISSION_EXIT_CONFIGURATION_ERROR` and SHALL not submit an incomplete
PICK or PLACE Motion goal.

### VR-ORCH-MISSION-07 — Depletion terminal ordering
Given Vision returns `OBJECT_NOT_FOUND`, Orchestration SHALL revalidate Safety
before publishing `MISSION_PHASE_FINISHED`. Given that Safety preempts normal
motion at that boundary, Orchestration SHALL return
`MISSION_EXIT_SAFETY_PREEMPTED` and SHALL not publish `MISSION_PHASE_FINISHED`.

### VR-ORCH-MISSION-08 — Motion cancellation provenance
Given Motion returns `TASK_RESULT_CANCELED`, Orchestration SHALL return
`MISSION_EXIT_CANCELED` only when it has observed ExecuteCycle caller
cancellation; otherwise it SHALL return `MISSION_EXIT_MOTION_ERROR`.

### VR-ORCH-MISSION-09 — Batch loop and home boundary
Given one admitted batch with N valid targets and no interruption,
Orchestration SHALL complete N PICK/HELD/PLACE/RELEASED iterations without
GO_HOME before depletion or between iterations, then confirm depletion,
execute exactly one terminal GO_HOME, and report batch completion only after
its success. The observable oracle is task order,
selected target and processed count, terminal depletion, GO_HOME count/result,
and batch result.

### VR-ORCH-MISSION-10 — No unsafe retry
Given a non-retryable failure, held or UNKNOWN object state, Safety
preemption, configuration/contract error, persistent backend fault, or
exhausted budget, Orchestration SHALL issue no further PICK or PLACE. It SHALL
retain the terminal cause and end the current batch in controlled abort/failure.
