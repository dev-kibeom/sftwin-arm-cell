# [SF-Twin] ARM Cell Motion Task Execution Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-MOTION-TASK-EXECUTION_v1.0.0`
- **Document Type:** `FDS`
- **Scope:** `PICK / PLACE / GO_HOME / RETRACT`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Motion`

## 1. Common Preconditions

Before trajectory-producing execution, Motion SHALL:

1. validate task semantics;
2. enforce Safety capability;
3. validate required target/object information;
4. produce/validate required collision-aware planning outcome.

## 2. PICK

```text
goal accepted
→ object-centric reference validated
→ Motion TCP/tool correction
→ approach
→ grasp entry
→ logical gripper command
→ grasp/object result evaluation
→ attach object
→ retract
→ success
```

Successful object attachment SHALL NOT be reported before the configured grasp/object-success condition is established.

## 3. PLACE

```text
goal accepted
→ place approach
→ place entry
→ gripper release
→ detach/update world object state
→ retreat
→ success
```

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

## 7. Verification Requirements

### VR-MOT-TASK-01 — PICK ordering
Successful PICK SHALL establish grasp/object attachment before success.

### VR-MOT-TASK-02 — PLACE ownership transition
Successful PLACE SHALL update attachment/world ownership consistently.

### VR-MOT-TASK-03 — GO_HOME side-effect boundary
GO_HOME SHALL NOT implicitly alter gripper state unless explicitly configured.

### VR-MOT-TASK-04 — Recovery obstruction
RETRACT SHALL NOT force execution through known blocked/unreachable paths.

### VR-MOT-TASK-05 — Result classification
Major task-failure causes SHALL remain distinguishable at the public task result boundary.
