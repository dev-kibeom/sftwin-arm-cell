# [SF-Twin] ARM Cell Safety Supervision Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-SAFETY-SUPERVISION_v1.1.0`
- **Document Type:** `FDS`
- **Scope:** `Watchdog / Context-Aware Interlock / Safety FSM`
- **Version:** `1.1.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Safety`

## 1. Startup

Safety begins fail-closed.

Until all required input classes have produced valid fresh samples:

```text
state = UNKNOWN
motion_capability = MOTION_NONE
```

Historical transient samples SHALL NOT alone establish a fresh safe state.

## 2. Context-Aware Evaluation

Safety SHALL evaluate state combinations rather than independent predicates only.

PackML is canonical external CNC/process state, not ARM mission state. The
read-only Isaac indicator projection is governed by [ARM Cell Simulation
Environment Semantics](../simulation-environment-semantics.md). PackML does not
globally prohibit robot motion. Final Demo does not require CNC
shared-zone occupancy/access interlocking. Future shared-zone coordination is
a non-normative extension point. `ABORTED` remains fail-closed by default:

| PackML state | Motion capability |
|---|---|
| `PACKML_STATE_ABORTED` | `MOTION_NONE` by default; `MOTION_RECOVERY_ONLY` only when the approved recovery conditions are satisfied |

`SAFE` does not imply `MOTION_NORMAL`. PackML state by itself does not define
ARM motion capability; Safety uses its independent readiness and safety inputs.

Safety owns active motion envelope publication and response to a tightened
restriction. A restriction applies while a trajectory is active. A tighter envelope applies to active motion. Motion may continue only when
the backend safely applies the restriction and provides evidence that the
active trajectory is within it. Otherwise Safety SHALL stop via its authorized
StopMotion path, await confirmed inactivity, and permit motion only from a new
plan under the current envelope. Safety owns stop selection; Motion does not
select severity. Exact envelope factors and calibration remain configuration
authority.



Safety grants motion permission only; it does not decide PICK/PLACE sequencing
or mission ordering. Orchestration owns task sequencing, and Motion executes
the requested task.

Examples:

- `IDLE/STARTING/EXECUTE/COMPLETE` are external process states and do not by
  themselves define ARM motion capability;
- `ABORTED` follows the existing fail-closed Safety recovery policy;
- required-input communication loss denies motion.

The following PackML outcomes are normative examples; PackML alone does not
create an ARM mission or shared-zone interlock:

| Context | Capability / stop outcome |
|---|---|
| EXECUTE alone | does not globally deny robot motion |
| IDLE / STARTING / EXECUTE / COMPLETE | PackML alone does not impose an ARM motion restriction |
| ABORTED | `MOTION_NONE` by default |
| ABORTED with approved recovery conditions | `MOTION_RECOVERY_ONLY` only after approved recovery conditions |

## 3. State Progression

Conceptually:

```text
UNKNOWN
→ SAFE
→ INTERLOCKED
→ STOPPING
→ STOPPED
→ RECOVERY_REQUIRED
```

E-Stop/STO may force emergency-stop state from relevant states.

Exact message constants belong to the shared ICD.

## 4. Watchdog

Required input freshness uses local monotonic receipt timing plus message validity and source timestamp plausibility. An invalid required sample is a distinct active Safety condition; Safety fails closed and stops active Motion through its direct stop path.

Watchdog timeout values are configuration/contract values only where explicitly declared normative in the relevant ICD.

For inputs configured with an intermediate degraded-freshness band, a sample
that remains valid and inside the Safety freshness limit but crosses the
configured degraded band may retain normal capability with a tighter
velocity/acceleration envelope. Crossing the stale/invalid required-input
limit remains fail-closed and denies normal motion under the existing
communication-loss policy. Clearing transient delay restores the configured
normal envelope only after fresh state is observed. Band boundaries and
envelope scale factors are configuration/calibration authority.

## 5. Verification Requirements

### VR-SAFE-SUP-01 — Fresh startup
Safety SHALL remain fail-closed until required fresh inputs are observed.

### VR-SAFE-SUP-02 — Context-aware interlock
Normal process states SHALL not be misclassified solely because one predicate is false outside its relevant context.

### VR-SAFE-SUP-03 — Communication loss
Required-input communication loss SHALL deny motion capability.

### VR-SAFE-SUP-04 — E-Stop synchronization
Software state SHALL reflect E-Stop/STO while preserving independent hardware authority.

### VR-SAFE-SUP-05 — PackML external-process interpretation
Given valid and fresh required inputs, PackML SHALL be interpreted as
external-process state rather than ARM mission state. `EXECUTE` or `STARTING`
alone SHALL NOT globally force `MOTION_NONE`. Final Demo does not require a
shared-zone/access condition. `ABORTED` remains fail-closed by default and
`MOTION_RECOVERY_ONLY` is permitted only after approved recovery conditions.
The oracle is canonical PackML state and Safety capability/envelope.

### VR-SAFE-SUP-06 — PackML does not gate ARM motion
Given canonical PackML `IDLE`, `STARTING`, `EXECUTE`, or `COMPLETE`, that state
alone SHALL not deny ARM motion. Invalid or stale external state remains subject
to the existing fail-closed input policy. The oracle is canonical PackML state and the unchanged
Safety capability absent any independent Safety condition.

### VR-SAFE-SUP-07 — Active envelope restriction
Given Safety tightens a permitted motion envelope during active execution,
Motion may continue only when the backend safely applies and attests the new
limits. Otherwise Safety SHALL issue StopMotion and await confirmed inactivity
before any new task is admitted. The new task SHALL be planned under the current
envelope. The oracle is envelope update, application evidence or stop request and
completion evidence, and subsequent task limits.

### VR-SAFE-SUP-08 — Degraded versus stale communication
Given required communication remains valid and within its freshness limit but
crosses a configured degraded band, Safety SHALL retain only the configured
permitted capability/envelope. Given a required input becomes stale or
invalid, Safety SHALL fail closed and deny normal motion. Fresh restoration
returns the normal envelope only after a fresh sample. The oracle is sample
validity/age, Safety capability, and envelope publication.
