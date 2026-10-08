# PICK Geometry Authority Decision

> **Document Type:** Durable historical record
> **Authority:** Provenance only; normative meaning remains in the referenced owner.
> **Scope:** ARM Cell

### AD-003 — PICK geometry authority

- **Context/problem:** Live PICK acceptance exposed underspecified ownership,
  yaw fallback, rigid-transform, approach/retract, and object-height semantics.
- **Decision:** Vision owns valid object `x/y/z` and optional object yaw;
  Orchestration resolves Vision and recipe values; Motion owns object-reference
  to TCP/tool conversion and distinct approach/grasp/retract geometry. A future
  `has_target_yaw` response semantic distinguishes valid zero yaw from yaw
  absence, and required recipe yaw is the fail-closed fallback.
- **Rationale:** Resolve the authority boundary without changing production
  behavior.
- **Normative owner(s):** Motion CDS/MoveIt CDS, Vision FDS, Mission FDS, and
  Vision/Orchestration/Motion ICDs
- **ADR:** None recorded
- **Status:** Accepted; implementation and future interface follow-up remain
  separate from this decision.
