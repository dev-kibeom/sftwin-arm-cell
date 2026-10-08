# [SF-Twin] ARM Cell Safety Supervisor Component Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-SAFETY_v1.1.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / Software Safety Supervision`
- **Version:** `1.1.0`
- **Status:** `Review`
- **Owner:** `ARM Cell Safety`

## 1. Purpose

Safety integrates AMR/PackML/Motion/Safety-HW state into a context-aware software supervision decision.

It is the sole software authority for motion capability.

## 2. Responsibilities

Safety SHALL own:

- SafetyState;
- motion-capability authority;
- context-aware interlock evaluation;
- required-input freshness/watchdog evaluation;
- fail-closed startup;
- stop-mode selection;
- selected stop severity publication through `SafetyState.selected_stop_mode`;
- direct StopMotion request to Motion;
- stop escalation;
- recovery eligibility;
- interlock latch/reset policy.

## 3. Non-Responsibilities

Safety SHALL NOT:

- generate recovery trajectories;
- sequence the mission;
- perform Motion planning;
- implement or replace safety-rated E-Stop/STO hardware functions.

## 4. Capability Model

```text
MOTION_NONE
MOTION_RECOVERY_ONLY
MOTION_NORMAL
```

`SAFE` and `MOTION_NORMAL` are not equivalent.

A safe/idle process phase may still have `MOTION_NONE`.

PackML is canonical external CNC/process state, not ARM mission state. After
required inputs are valid and fresh, it may affect Safety capability or the
`PACKML_STATE_EXECUTE` alone SHALL NOT globally force `MOTION_NONE`. Final
Demo uses PackML only as canonical external-process state and exposes it via a
read-only Isaac indicator. Future shared-zone coordination is a non-normative
extension point.
the [Safety Supervision FDS](fds-supervision.md) owns the dynamic state mapping.

The capability model grants motion permission only. Safety SHALL NOT select
PICK or PLACE, sequence tasks, or order missions. Orchestration owns task and
mission ordering, while Motion executes the requested task.

## 5. Functional Safety Boundary

ROS 2 supervision is functional/system-level software supervision.

Independent safety-rated hardware/controller paths remain authoritative for legal/physical E-Stop/STO functions.

## 6. Verification Requirements

### VR-SAFE-OWN-01 — Sole capability authority
Safety SHALL be the sole canonical source of motion capability.

### VR-SAFE-OWN-02 — Fail-closed startup
Before all required inputs are valid/fresh, Safety SHALL deny motion.

### VR-SAFE-OWN-03 — Hardware boundary
Software Safety SHALL NOT claim to replace safety-rated hardware functions.
