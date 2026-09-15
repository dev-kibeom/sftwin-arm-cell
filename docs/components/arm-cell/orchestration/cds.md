# [SF-Twin] ARM Cell Orchestration Component Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-ORCHESTRATION_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / Mission Orchestration`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Orchestration`

## 1. Responsibilities

Orchestration SHALL own:

- ExecuteCycle mission lifecycle;
- sequencing Vision and Motion unit capabilities;
- ensuring configured Home/observation preconditions;
- mission-loop exit-reason classification;
- mapping component outcomes to mission result;
- coordinating cancellation after Safety preemption;
- requesting bounded recovery only when Safety authorizes it.

## 2. Non-Responsibilities

Orchestration SHALL NOT own:

- Safety capability authority;
- direct software stop completion;
- Motion TCP correction;
- perception geometry;
- controller/hardware stop behavior.

## 3. Safety Trust Boundary

Safety→Motion direct stop is primary.

Orchestration observes Safety state/capability and coordinates its own leaves, but SHALL NOT be required for the stop to begin.

## 4. Verification Requirements

### VR-ORCH-OWN-01 — Mission-only authority
Orchestration SHALL not redefine Safety or Motion authority.

### VR-ORCH-OWN-02 — Explicit exit reason
Mission completion/failure classification SHALL preserve the underlying exit reason.

### VR-ORCH-OWN-03 — Recovery coordination only
Orchestration SHALL request recovery only under Safety-authorized conditions.
