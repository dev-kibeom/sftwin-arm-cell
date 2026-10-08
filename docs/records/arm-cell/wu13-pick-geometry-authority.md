# WU-13 PICK Geometry Authority Decision

> **Document Type:** Durable historical record
> **Authority:** Provenance only; normative meaning remains in the referenced owner.
> **Scope:** ARM Cell

### AD-003 — WU-13 PICK geometry authority

- **Applicable Work Unit:** WU-13
- **Context/problem:** Live PICK acceptance exposed underspecified ownership,
  yaw fallback, rigid-transform, approach/retract, and object-height semantics.
- **Decision:** Vision owns valid object `x/y/z` and optional object yaw;
  Orchestration resolves Vision and recipe values; Motion owns object-reference
  to TCP/tool conversion and distinct approach/grasp/retract geometry. A future
  `has_target_yaw` response semantic distinguishes valid zero yaw from yaw
  absence, and required recipe yaw is the fail-closed fallback.
- **Rationale:** Resolve the authority boundary without changing production
  behavior or the WU-13 lifecycle state.
- **Affected Work Unit / PR:** WU-13; PR #52
- **Normative owner(s):** Motion CDS/MoveIt CDS, Vision FDS, Mission FDS, and
  Vision/Orchestration/Motion ICDs
- **ADR:** None recorded
- **Status:** Accepted; implementation and future interface follow-up remain
  within WU-13 execution
