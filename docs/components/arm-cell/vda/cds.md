# [SF-Twin] ARM Cell External-System Mock / VDA Adapter Design

- **Document ID:** `[SF-Twin]_CDS-ARM-CELL-VDA_v1.0.0`
- **Document Type:** `CDS`
- **Scope:** `ARM Cell / AMR, PackML, Safety-HW Mock Boundary`
- **Version:** `1.0.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell External Integration`

## 1. Purpose

The current component provides a simulation/testing substitute for external AMR, PackML/PLC, and Safety-HW state needed for system integration.

It is not a full VDA 5050 implementation.

## 2. Responsibilities

The component SHALL own:

- AMR docking/state subset simulation;
- PackML subset state simulation;
- mock SafetyHardwareState simulation;
- periodic state/heartbeat publication;
- immediate publication on relevant state transitions;
- approved fault-injection semantics used by integration tests.

## 3. Non-Responsibilities

The component SHALL NOT claim:

- full VDA 5050 Order/InstantActions/Connection/MQTT compliance;
- safety-rated hardware behavior;
- production PLC/safety-controller ownership.

## 4. Replacement Boundary

Real deployment may replace internal mock behavior with real VDA/PLC/safety adapters while preserving canonical ARM Cell shared interface semantics.

## 5. Verification Requirements

### VR-VDA-OWN-01 — Mock-only scope
The current adapter SHALL remain explicitly identified as a simulation/integration mock subset.

### VR-VDA-OWN-02 — Stable upstream semantics
Replacing the mock with real adapters SHALL not require upper Safety logic to consume simulator-specific representation.
