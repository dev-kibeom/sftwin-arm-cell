# [SF-Twin] ARM Cell Motion Backend Interface Contract

- **Document ID:** `[SF-Twin]_ICD-ARM-CELL-MOTION-BACKEND_v0.2.0`
- **Document Type:** `ICD`
- **Scope:** `Motion Core ↔ Interchangeable Robot/Gripper Backend`
- **Version:** `0.2.0`
- **Status:** `Draft`
- **Owner:** `ARM Cell Shared Interface`

## 1. Purpose and Ownership

This ICD is retained as the backend-neutral abstract contract between Motion Core and an interchangeable execution backend. It is not an Orchestration-, Vision-, or Safety-facing task interface.

Motion Core owns task semantics, TCP/tool correction, safety-capability enforcement, and canonical Motion results. A backend owns only execution of normalized requests and reporting of its own lifecycle, availability, and capability state.

Candidate backends include Isaac Sim, a Doosan robot backend, and other robot/manipulator backends. No backend implementation is the authority for this contract.

## 2. Required Backend-Neutral Semantics

The stable seam SHALL provide the following semantics without exposing backend-specific topology or controller APIs:

- backend availability and disconnect/fault reporting;
- submission and lifecycle reporting for a normalized execution request;
- cancel, hold, and stop dispatch;
- execution-active state and explicit backend-inactivity confirmation;
- logical gripper-width command and outcome reporting;
- mapping of backend-specific errors into canonical Motion result/error semantics.

`STOPPED` SHALL be reported by Motion only after the backend has confirmed that execution is inactive. A stop request being accepted or dispatched is not inactivity confirmation.

Logical gripper width remains backend-neutral. A backend SHALL translate it to its own actuator representation internally; simulator joint topology, vendor actuator layout, and controller API names SHALL NOT cross this boundary upward.

## 3. Optional Capabilities

Attachment/release feedback, explicit hold support, and other backend-dependent behavior SHALL be capability-reported rather than assumed as mandatory. Motion Core SHALL reject or otherwise map an unsupported requested capability into the canonical task result; it SHALL NOT expose backend-private details to its upper consumers.

This abstract seam does not require a ROS 2 transport or IDL. It defines backend-neutral semantics; a backend implementation may realize them in-process or through an approved adapter without changing the public Motion contracts. This ICD does not prescribe Isaac, Doosan, or other vendor transport details.

## 4. Non-Goals

This ICD SHALL NOT expose Isaac prim paths, Robotiq six-joint topology, Doosan-specific ROS service names, vendor-private controller APIs, or MoveIt internal class names.

## 5. Verification Requirement

### VR-ICD-MOT-BACKEND-01 — Backend insulation
The backend contract SHALL preserve upper Motion semantics across supported simulator/real backends, including logical gripper-width handling and the distinction between stop dispatch and verified inactivity.
