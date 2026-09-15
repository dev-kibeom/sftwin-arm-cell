# [SF-Twin] ARM Cell Motion Component Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-MOTION_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / Motion Adapter`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Motion`

## 1. Purpose

Motion converts task-level robot/tool-independent intent into robot/tool execution behavior while independently enforcing Safety-provided motion capability.

## 2. Responsibilities

Motion SHALL own:

- ARM Cell motion planning/execution;
- object-centric grasp reference → TCP target conversion;
- Motion-owned tool/TCP/approach/retract offsets;
- PICK/PLACE/GO_HOME/RETRACT execution;
- MotionStatus publication;
- Motion-side ObjectState / Planning Scene synchronization;
- logical gripper adaptation;
- Safety capability enforcement at goal acceptance;
- StopMotion handling and actual stop-state reporting.

## 3. Non-Responsibilities

Motion SHALL NOT own:

- mission sequencing or retry policy;
- Safety capability authority;
- Safety interlock evaluation;
- Vision geometry estimation;
- safety-rated E-Stop/STO authority.

Motion MUST NOT treat Orchestration as a trusted safety boundary.

## 4. Object-Centric Pose Boundary

Vision supplies an object-centric grasp reference.

Motion alone applies robot/tool-specific correction such as:

- TCP/gripper offset;
- tool geometry correction;
- approach/retract distance;
- Motion-owned planning/collision margin.

## 5. Logical Gripper Boundary

Upper contracts use a logical gripper opening width rather than simulator-specific multi-joint representation.

Backend force/current/effort or multi-joint mapping is adapter responsibility.

## 6. Verification Requirements

### VR-MOT-OWN-01 — TCP correction ownership
Robot/tool-specific TCP correction SHALL be applied by Motion.

### VR-MOT-OWN-02 — Safety enforcement
Motion SHALL enforce Safety capability at the task boundary.

### VR-MOT-OWN-03 — MotionStatus authority
Motion SHALL be the canonical owner/publisher of externally meaningful motion execution state.

### VR-MOT-OWN-04 — Backend insulation
Mission and Vision contracts SHALL NOT depend on simulator-specific gripper articulation topology.
