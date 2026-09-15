# SF-Twin ARM Cell Overview

> Public project overview. This document introduces the exported ARM Cell
> scope; the architecture and interface documents remain the normative source
> for component and contract semantics.

## What it demonstrates

SF-Twin ARM Cell is a simulator-backed robotics workcell used to develop and
verify a small pick-and-place cell around a Doosan M0609 arm, a Robotiq 2F-85
gripper, MoveIt, ROS 2, and Isaac Sim.  The public project focuses on how a
shared robot model, measured state, time, TF, RGB-D ingress, planning, and
safety-oriented boundaries fit together before production task execution is
implemented.

## Current scope

Implemented foundations include the canonical robot-model path, Isaac-to-ROS
independent joint-state projection, simulation time and TF composition, the
bounded static planning scene, generated-USD provenance, and the RGB-D ingress
contract. Selected acceptance reports identify the historical and current
evidence available for those foundations and its traceability limits. The
public source also contains buildable shared ROS 2 interfaces for the planned
Vision, Motion, Safety, Orchestration, and external-state boundaries.

Motion task execution, perception-driven `DetectTarget`, Safety supervision,
mission orchestration, and external adapters are planned runtime work.  Their
presence in the architecture and interface documents does not mean that a
production runtime implementation is already provided.

## System boundaries

The supported public profile is simulator-backed. Isaac Sim provides simulator
state, `/clock`, and RGB-D data; ROS components compose canonical state, TF,
MoveIt planning, and Vision ingress.  Software supervision in this project is
not a safety-rated E-stop or STO implementation and must not be used as one.

## Where to read next

- System context: `docs/architecture/`
- ARM Cell component designs: `docs/components/arm-cell/`
- Shared ROS contracts: `docs/interfaces/arm-cell/`
- Architecture decisions: `docs/adr/`
- Supported Isaac Sim procedure: `docs/guides/arm-cell-isaac-sim.md`
- Historical acceptance evidence and its limitations:
  `docs/reports/acceptance/`

## Licensing

Project-owned material is MIT licensed. Third-party material remains subject
to its own terms; see `THIRD_PARTY_NOTICES.md` and the retained upstream
licenses.
