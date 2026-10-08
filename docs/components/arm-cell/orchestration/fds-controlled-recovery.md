# [SF-Twin] ARM Cell Controlled Recovery Coordination

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-ORCHESTRATION-CONTROLLED-RECOVERY_v1.3.0`
- **Document Type:** `FDS`
- **Scope:** `Safety Preemption Coordination`
- **Version:** `1.3.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Orchestration`

## 1. Entry

When Safety removes normal motion capability during a running mission:

1. Orchestration stops advancing the normal mission branch;
2. running Motion leaves are halted/canceled for coordination;
3. Orchestration waits for Motion stop evidence and Safety's recovery-capability outcome.

For a Final Demo batch, the active task and iteration are terminally
interrupted. The retained batch remains in a recovery-required state
while the operator/Safety recovery gate is unresolved. Batch context
(admission identity, processed count, and interruption provenance) is retained;
the interrupted task/iteration is never resumed or replayed. If recovery is
denied or the batch is terminated by its owning orchestration policy, the
execution ends with the preserved Safety-preemption outcome.

Safety→Motion direct StopMotion remains the primary stop path. Orchestration
cancellation is secondary coordination and is not required for stop initiation.
Safety also publishes the canonical selected stop severity in
`SafetyState.selected_stop_mode`. Orchestration compares that Safety-owned
field only; it SHALL NOT derive severity from `active_causes`, mission state,
or local policy.

### Recovery Episode Lifecycle

A controlled-recovery episode is bound to one interrupted ExecuteCycle:

```text
one interrupted ExecuteCycle
→ one controlled-recovery episode
→ at most one RETRACT
```

The episode begins when an active ExecuteCycle loses `MOTION_NORMAL` and
Orchestration enters controlled-recovery coordination for that interruption.
It persists through Safety capability changes, temporary loss of
`MOTION_RECOVERY_ONLY`, higher-severity Safety hazards, interruption of an
authorized RETRACT, RETRACT failure, and later Safety capability publications
for the same interrupted mission. The episode ends when recovery coordination
for that ExecuteCycle terminates and the original ExecuteCycle proceeds to its
preserved terminal outcome.

Capability flapping SHALL NOT reset the RETRACT allowance. A return to
`MOTION_RECOVERY_ONLY` within the same interrupted ExecuteCycle does not create
a new recovery episode. RETRACT failure and higher-severity interruption do
not create a new episode. At most one RETRACT request may be issued within an
episode; no automatic recovery retry is approved. A second RETRACT requires a
genuinely new mission/recovery episode or a future explicitly approved retry
policy.

## 2. Waiting for Recovery Authorization

Recovery SHALL NOT be requested merely because the mission was preempted.

Orchestration waits until both of these conditions are currently true:

1. Motion reports `MOTION_STATE_STOPPED`; and
2. Safety explicitly exposes `MOTION_RECOVERY_ONLY`.

Neither condition authorizes RETRACT by itself.

Non-recovery conditions such as active E-Stop, unresolved communication loss, or other non-recovery faults SHALL not trigger a RETRACT request.

## 3. Recovery Request

When authorized:

```text
Safety: MOTION_RECOVERY_ONLY
→ Orchestration requests RETRACT
→ Motion independently rechecks capability
```

For one recovery episode, repeated identical recovery-capable state
publications SHALL NOT cause duplicate `RETRACT` requests. A new `RETRACT`
request requires a new recovery episode or an explicitly authorized retry
policy; no such automatic retry is part of the current contract.

When recovery authorization is removed or a higher-severity hazard appears
while recovery is waiting or RETRACT is active, Safety remains the sole
stop-initiation and stop-severity authority. Safety→Motion direct StopMotion is
the authoritative stop path. Orchestration SHALL NOT initiate a second stop
policy; it observes recovery-task termination and Motion state, ceases recovery
progression, and SHALL NOT issue another RETRACT in this episode. A later
return to recovery capability does not authorize re-entry or retry. Motion
continues to own execution and termination behavior under the Safety stop
command, and the interrupted mission is not automatically retried.

During recovery waiting, an observed `selected_stop_mode` greater than the
episode's previously observed severity permanently closes recovery progression
for that interrupted ExecuteCycle. The episode-local allowance is not reset by
capability flapping. During active RETRACT, Orchestration does not issue a stop
command or infer a replacement stop policy; it observes Motion termination and
preserves the original mission outcome.

### SEQ-ORCH-RECOVERY-01 — Controlled recovery

**Primary owner:** this Controlled Recovery FDS. The
[Orchestration dynamic-design entry point](fds.md) and shared ICDs reference
this sequence; they do not own an alternate recovery workflow.

```mermaid
sequenceDiagram
  autonumber
  participant Safety as Safety
  participant Motion as Motion
  participant Orch as Orchestration

  Safety->>Motion: StopMotion (primary stop path)
  Safety-->>Orch: capability removes MOTION_NORMAL
  Orch->>Orch: stop normal branch advancement
  Orch->>Motion: secondary cancellation for coordination

  loop wait during one recovery episode
    Motion-->>Orch: MotionStatus
    Safety-->>Orch: SafetyState
    alt new higher-severity hazard
      Safety->>Motion: StopMotion (higher severity)
      Note over Safety,Orch: Safety remains stop authority; Orchestration observes termination and ceases recovery progression.
      Note over Orch: Later MOTION_RECOVERY_ONLY does not authorize another RETRACT in this episode.
    else MotionState == STOPPED and Safety capability == MOTION_RECOVERY_ONLY
      opt first authorization for this recovery episode
        Orch->>Motion: ExecuteTask(RETRACT)
        Motion-->>Orch: RETRACT result
      end
      Note over Safety,Orch: Repeated recovery-capable publications do not duplicate RETRACT.
    else either recovery condition is absent
      Note over Orch: Continue waiting; do not request RETRACT.
    end
  end

  alt RETRACT succeeds
    Note over Orch: Preserve original mission exit reason.
  else RETRACT fails
    Note over Orch: Preserve recovery-operation failure through approved diagnostic/evidence channels; do not retry automatically.
    Note over Orch: Preserve original mission exit reason.
  end
  Note over Orch: Recovery never rewrites the original exit reason or retries the mission automatically.
```

#### Sequence verification linkage

| Sequence boundary | Existing VR(s) clarified | Observable oracle / deterministic seam |
|---|---|---|
| Safety direct stop and secondary Orchestration cancellation | `VR-ORCH-REC-03` | direct Safety→Motion stop request is observable independently of Orchestration cancellation |
| recovery request gate and one RETRACT per episode | `VR-ORCH-REC-01`, `VR-ORCH-REC-04` | current Safety capability, Motion stop state, and RETRACT request count across repeated Safety publications |
| recovery completion preserves interruption provenance | `VR-ORCH-REC-02`, `VR-ORCH-REC-07` | batch interruption reason remains observable after recovery and through terminal batch result |
| higher-severity interruption and no recovery re-entry retry | `VR-ORCH-REC-05` | RETRACT request count remains <= 1 while Safety/Motion state changes and recovery capability later returns |
| failed or interrupted RETRACT preserves outcome | `VR-ORCH-REC-06` | RETRACT result/termination evidence, original ExecuteCycle exit reason, and no subsequent normal task or RETRACT request |

The sequence remains contract-level; implementation-specific state variables,
mutexes, callback layout, and private helper calls are not normative here.

## 4. Mission Result After Recovery

Successful recovery does not convert the original interrupted task outcome to
success. For a terminally interrupted non-batch execution, the canonical
ExecuteCycle `MissionExitReason` remains unchanged. For a Final Demo batch
that is suspended in `WAITING_RECOVERY`, the task/iteration interruption is
recorded independently while the batch execution remains active; the terminal
MissionExitReason is not established until the batch completes or aborts. A
later terminal batch result SHALL NOT clear or rewrite the retained
interruption provenance. A failed or interrupted RETRACT is a separate
recovery-operation failure and SHALL NOT create a new public mission result or
be automatically retried. A new higher-severity hazard may prevent or
interrupt recovery; Safety remains the stop-severity authority. The
interrupted task itself is never automatically retried, and
`diagnostic_detail` remains non-normative rather than a hidden machine-readable
decision contract.

### Final Demo batch continuation

For E-stop and configured hard-fault recovery, Safety retains authority over
cause clearance, explicit operator clear, acknowledgement/reset, and current
capability. Orchestration SHALL wait for the explicit operator clear and
ack/reset completion, then re-evaluate readiness from current state. Removing
the cause alone is insufficient. Fresh `RELEASED` is required before motion
continuation. For `HELD` or `UNKNOWN`, Orchestration remains motion-inhibited
until operator intervention safely handles the object and fresh `RELEASED` is
observed; no automatic disposition behavior or new recovery motion is defined.

After readiness is re-established, holding is freshly `RELEASED`, and Safety
currently permits normal motion, Orchestration may issue a new GO_HOME task. It then obtains fresh perception
and starts a new iteration in the retained batch context. This is a new task
and target-selection episode; it is not a resume of the interrupted action or
an implicit PICK retry. It SHALL NOT occur while Safety permits only
`MOTION_RECOVERY_ONLY`. The batch execution remains active across
this wait and continuation; if the batch later depletes normally, its terminal
result may be `MISSION_EXIT_DEPLETED` while the interruption provenance
remains observable and retained. This does not convert the interrupted task
into success.

The interrupted outcome and continuation outcome are independent. A successful
GO_HOME and later batch continuation SHALL preserve the original E-stop or
hard-fault provenance in batch feedback and acceptance evidence; it SHALL NOT
clear or rewrite that interruption record as success.

For E-stop and hard-fault continuation, no automatic safe-disposition action
or new recovery motion is defined. After explicit operator clear and
acknowledgement/reset, `RELEASED` permits GO_HOME then fresh perception. With
`HELD` or `UNKNOWN`, no motion is permitted for continuation until operator
intervention has safely handled the object and a fresh `RELEASED` disposition
is observed. `UNKNOWN` remains fail-closed in the meantime.

Holding disposition gates automatic continuation:

| Fresh holding state after stop/reset | Continuation |
|---|---|
| `RELEASED` | GO_HOME and fresh perception may continue the retained batch when readiness and Safety permit |
| `HELD` | no automatic continuation; in Final Demo operator intervention safely handles the object and a fresh `RELEASED` disposition is required before recovery progression |
| `UNKNOWN` | no automatic motion/recovery; fail-closed until operator intervention and fresh safe disposition are confirmed, and fresh `RELEASED` is required before continuation |

This batch continuation rule does not authorize a second RETRACT in the same
controlled-recovery episode. Any separately authorized recovery motion must
remain inside Safety's current recovery envelope and the applicable recovery
contract. Safety E-stop/reset and batch continuation do not change the
retained interruption reason; the Action's eventual terminal result describes
the batch outcome, not a rewrite of the interrupted task.

## 5. Batch Continuation Ordering

### SEQ-ORCH-RECOVERY-BATCH-01 — Operator-gated batch continuation

```mermaid
sequenceDiagram
  autonumber
  participant Safety
  participant Motion
  participant Orch as Orchestration
  participant Hub as UI Hub
  participant Vision

  Safety->>Motion: StopMotion at Safety-selected severity
  Motion-->>Safety: confirmed inactivity
  Orch->>Orch: terminate active task/iteration; retain batch context and interruption provenance
  Orch-->>Hub: MISSION_PHASE_WAITING_RECOVERY; reset required
  Hub->>Safety: explicit operator acknowledge/reset request
  Safety->>Safety: re-evaluate cause clear, freshness, stopped state, reset policy
  alt reset rejected or readiness false
    Safety-->>Hub: reset not applied; capability remains restrictive
    Orch-->>Hub: remain WAITING_RECOVERY; no task request
  else reset applied; normal capability; fresh holding == RELEASED
    Safety-->>Orch: current MOTION_NORMAL and valid envelope
    Motion-->>Orch: current RELEASED observation
    Orch->>Motion: ExecuteTask(new GO_HOME identity)
    Motion-->>Orch: GO_HOME result
    Orch->>Vision: DetectTarget with fresh request
    Vision-->>Orch: fresh result
    Orch->>Orch: continue retained batch with new iteration identity
    Note over Orch,Hub: Preserve interruption provenance through later batch completion.
  else holding == HELD or UNKNOWN
    Orch-->>Hub: no automatic continuation; safe disposition/operator intervention required
  end
```

This continuation sequence is distinct from the optional one-shot RETRACT
coordination path above. It starts only after reset/readiness grants normal
motion; it never resumes or retries the interrupted task.

#### Sequence verification linkage

| Sequence boundary | Existing VR(s) clarified | Observable oracle |
|---|---|---|
| retain interrupted task/iteration provenance through batch continuation | `VR-ORCH-REC-02` | original interruption provenance remains observable through later batch completion/abort |
| continuation uses a new GO_HOME and fresh perception; interrupted task/target is not replayed | `VR-ORCH-REC-07` | task and observation identities after reset, with no resubmission of interrupted task or reuse of its target observation |
| holding gates automatic continuation | `VR-ORCH-REC-08` | fresh holding state and subsequent task request sequence; only `RELEASED` permits automatic continuation |

## 6. Verification Requirements

### VR-ORCH-REC-01 — No premature recovery
RETRACT SHALL not be requested before Motion reports `MOTION_STATE_STOPPED`
and Safety currently exposes `MOTION_RECOVERY_ONLY`.

### VR-ORCH-REC-02 — Preserve original mission outcome
Successful recovery SHALL not mark the interrupted task/iteration successful.
Its interruption provenance SHALL remain observable through terminal batch
completion or abort. Any later batch success requires new task identities and
fresh perception.

### VR-ORCH-REC-03 — Safety-stop independence
Orchestration cancellation SHALL not be required for Safety direct stop initiation.

### VR-ORCH-REC-04 — One recovery request per episode
Given one controlled-recovery episode bound to one interrupted ExecuteCycle,
when repeated or re-entered recovery-capable Safety publications arrive,
including after temporary capability loss, then Orchestration SHALL issue at
most one `RETRACT` request for that episode. A second request SHALL NOT be
emitted unless a genuinely new mission/recovery episode or explicit future
retry authorization is observed. No retry policy is approved in the current
scope.

### VR-ORCH-REC-05 — Recovery interruption / no re-entry retry
Given one controlled-recovery episode, when recovery authorization is removed
or a higher-severity Safety condition interrupts recovery before or during
RETRACT, Orchestration SHALL cease recovery progression and SHALL NOT issue
another `RETRACT` for that episode even if `MOTION_RECOVERY_ONLY` later
returns.

The observable oracle is that the RETRACT request count remains at most one for
the episode while Safety publishes a higher `selected_stop_mode`, Motion
terminates the active RETRACT when directed by Safety, and recovery capability
may later return. The test oracle records the severity transition before the
RETRACT termination/result and verifies no Orchestration-issued stop policy.

### VR-ORCH-REC-06 — Recovery failure preserves mission outcome
Given an authorized RETRACT terminates unsuccessfully or is interrupted,
Orchestration SHALL preserve the original canonical `MissionExitReason`, issue
no automatic RETRACT retry, and issue no automatic mission retry. The
observable oracle includes the original ExecuteCycle exit reason, RETRACT
request count, normal mission task request count after interruption, and
RETRACT result/termination evidence. Recovery failure remains diagnostic or
evidence-only and does not introduce a public mission-exit result.

### VR-ORCH-REC-07 — Interrupted task is never resumed
After Safety interruption and operator clear/ack/reset, Orchestration SHALL
not resubmit the interrupted task goal or reuse its target observation. Any
permitted continuation SHALL use a new GO_HOME task followed by fresh
perception, while retaining the batch context and original failure provenance.
The ExecuteCycle remains in `WAITING_RECOVERY` until continuation or terminal
abort is decided.

### VR-ORCH-REC-08 — Holding-gated continuation
Given post-stop holding `RELEASED`, `HELD`, or `UNKNOWN`, continuation SHALL
follow the table above: only RELEASED permits automatic continuation; HELD
requires operator intervention to safely handle the object and fresh
`RELEASED` confirmation before progression; UNKNOWN remains fail-closed until
operator intervention produces a fresh safe disposition. No automatic
safe-disposition behavior or new recovery motion is defined. The oracle is the holding observation, subsequent task request
sequence, and preserved original failure provenance.
