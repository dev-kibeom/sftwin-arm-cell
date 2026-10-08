# [SF-Twin] ARM Cell Motion Task Execution Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-MOTION-TASK-EXECUTION_v1.2.0`
- **Document Type:** `FDS`
- **Scope:** `PICK / PLACE / GO_HOME / RETRACT`
- **Version:** `1.2.0`
- **Status:** `Approved`
- **Owner:** `ARM Cell Motion`

## 1. Common Preconditions

Before trajectory-producing execution, Motion SHALL:

1. validate task semantics;
2. enforce Safety capability;
3. validate required target/object information;
4. produce/validate required collision-aware planning outcome.

Motion consumes the current Safety-owned permitted motion envelope described
by the [Safety↔Motion ICD](../../../interfaces/arm-cell/icd-safety-motion.md).
It constrains plan and execution velocity/acceleration to those limits and
does not interpret condition, cause, or severity. A tighter envelope applies
to active execution. Motion may continue only when the backend safely applies
the tighter limits and provides evidence that the active trajectory complies.
Otherwise Safety initiates its authorized StopMotion path; Motion executes the
stop and confirms inactivity. Motion does not select stop policy. Any later
task requires a new plan under the current envelope. No new trajectory starts
with a missing, invalid, stale, or insufficient envelope.

### VR-MOT-TASK-07 — Safety envelope consumption
Given a current valid Safety envelope, task motion SHALL remain within its
velocity and acceleration limits. When a tighter envelope arrives during active execution, the backend SHALL
apply and attest the restriction safely or Safety SHALL stop the task and
require confirmed inactivity before a new task is planned under the new limits. Motion SHALL not derive severity or
stop mode from the envelope or fault condition.

## 2. PICK

```text
goal accepted
→ object-centric reference validated
→ Motion TCP/tool correction
→ approach
→ measured approach completion
→ grasp entry
→ measured grasp completion
→ GripperPort.close(grasp_width_mm)
→ GripperPort.holding().state == HELD (fresh)
→ retract
→ measured retract completion
→ success
```

PICK Approach geometry uses its configured PICK approach distance.

Motion SHALL not command attachment directly. A logical close-command result
alone SHALL NOT authorize retraction or PICK success. The active gripper
adapter owns its holding-confirmation mechanism; an Isaac adapter may establish
fresh `HELD` only after its capture-local snap policy creates the attachment.
Stale, missing, disconnected, ambiguous, or unexpected attachment observations
are `UNKNOWN` and block progression.

If holding confirmation is unsupported, times out, or fails, Motion SHALL not
start retract and SHALL return a canonical grasp/backend failure while
preserving the observable object state.

## 3. PLACE

```text
goal accepted
→ place approach
→ place entry
→ GripperPort.open()
→ GripperPort.holding().state == RELEASED (fresh)
→ update Motion-owned world object state
→ retreat
→ success
```

PLACE Approach geometry uses its configured PLACE approach distance. The
recipe declares the object orientation constraint as fixed, bounded, or free.
Motion composes each permitted object orientation with the accepted
Object-to-TCP relation. Free creates a bounded deterministic set including
side-on poses and selects by joint travel and time-parameterized trajectory
duration after IK/collision filtering; it does not imply exhaustive/global
optimization. An optional TCP orientation preference applies only to fixed or
bounded constraints and cannot bypass process tolerances, collision validity,
or IK validity. The schema-compatible `orientation_tolerance_rad` is ignored
when orientation mode is `free`.

Motion SHALL not command simulator detachment directly. The active gripper
adapter owns release realization. PLACE cannot begin retreat or report success until open succeeds and a fresh
`RELEASED` observation is received. `UNKNOWN` is not equivalent to release.
The PLACE result does not include a post-release settled-pose observation or
comparison with the recipe process tolerance; that is a separate Process
Verification capability.

## 4. GO_HOME

GO_HOME moves to configured/named Home and by default SHALL NOT create unrelated gripper side effects.

## 5. RETRACT

RETRACT is a constrained recovery task.

- attached object → preserve grasp and retract with object;
- no attached object → retract arm;
- uncertain/unsafe object state → reject or follow explicitly approved recovery policy;
- obstructed/unreachable recovery path → do not force motion.

## 6. Result Classification

Cancellation, safety preemption, planning failure, and gripper/object failure SHALL remain distinguishable.

At the public Motion result boundary, the minimum distinctions are client
cancellation → `TASK_RESULT_CANCELED`, Safety preemption →
`TASK_RESULT_SAFETY_PREEMPTED`, planning failure → the applicable planning
failure result such as `TASK_RESULT_IK_UNREACHABLE` or
`TASK_RESULT_PATH_OBSTRUCTED`, and gripper/object failure →
`TASK_RESULT_GRASP_FAILURE` or the applicable object/gripper result. These
categories SHALL NOT be collapsed into generic success or cancellation.

### 6.1 Internal completion outcome propagation

The transport/backend boundary SHALL preserve the reason why a measured
motion phase completed or stopped. The internal outcome SHALL be mapped to
the canonical Motion result as follows:

| Lower-layer outcome | Canonical Motion result |
| --- | --- |
| `COMPLETED` | `TASK_RESULT_SUCCESS` |
| caller `CANCELED` | `TASK_RESULT_CANCELED` |
| `SAFETY_PREEMPTED` | `TASK_RESULT_SAFETY_PREEMPTED` |
| `TIMEOUT` | `TASK_RESULT_BACKEND_ERROR` |
| `STALE_FEEDBACK` | `TASK_RESULT_BACKEND_ERROR` |
| `TRANSPORT_UNAVAILABLE` | `TASK_RESULT_BACKEND_UNAVAILABLE` |

MotionCore SHALL retain caller-cancel and Safety-preemption provenance through
the synchronous task lifecycle. Safety preemption SHALL retain precedence over
a concurrent caller cancellation; neither cause SHALL be rewritten as a
generic backend failure. The execution-scoped interruption state SHALL be
cleared on every terminal path before a subsequent goal is accepted.

## 7. Verification Requirements

### VR-MOT-TASK-01 — PICK ordering
Given a PICK attempt, when failure occurs before grasp/object-success
confirmation, then the task SHALL fail and cannot produce attachment/PICK
success. Successful PICK SHALL establish a fresh `HELD` observation before success.

A close-command result without a fresh `HELD` observation SHALL NOT permit
retract or PICK success. Stale or `UNKNOWN` state blocks progression; the
backend-specific attachment mechanism SHALL remain below `GripperPort`.

### VR-MOT-TASK-02 — PLACE ownership transition
Successful PLACE SHALL observe fresh `RELEASED` after open and update
Motion-owned world-object state consistently before retreat. `UNKNOWN` blocks
retreat and success.

### VR-MOT-TASK-03 — GO_HOME side-effect boundary
GO_HOME SHALL NOT implicitly alter gripper state unless explicitly configured.

### VR-MOT-TASK-04 — Recovery obstruction
RETRACT SHALL NOT force execution through known blocked/unreachable paths.

### VR-MOT-TASK-05 — Result classification
Given equivalent task execution, when the cause is cancellation, Safety
preemption, planning failure, or gripper/object failure, then the public result
SHALL preserve the corresponding distinguishable category.

### VR-MOT-TASK-06 — Phase interruption provenance

Given caller cancellation, Safety preemption, timeout, stale measured
feedback, or transport unavailability during any PICK/PLACE motion phase, the
phase SHALL terminate without starting a later phase and the canonical result
SHALL follow the propagation table above. A later independent goal SHALL NOT
inherit interruption state from the completed execution.
