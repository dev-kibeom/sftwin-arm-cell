# ADR 0007: Separate manipulation completion from post-release process verification

- **Document ID:** `ADR-ARM-CELL-0007`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Manipulation and process verification boundary`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-08`
- **Related Design:** Motion PICK/PLACE and future Process Verification capability
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

Manipulation establishes that the robot performed the commanded transfer and observed the
required grasp/release semantics. A released object's later settling pose or process quality
is a different observation, affected by physics and process criteria beyond the manipulation
command itself.

## Decision

Manipulation completion covers commanded motion and grasp/release semantics through the
accepted release boundary. It does not depend on judging the object's final post-release
settling pose or process quality. A future Process Verification capability may own that
judgment without redefining the existing meaning of successful PICK/PLACE manipulation.

### Considered alternatives

- **Include settled-pose/process-quality judgment in PLACE success:** gives a single result,
  but makes manipulation success depend on a distinct post-release process criterion.
- **Keep manipulation and post-release verification as separate responsibilities (selected):**
  preserves a stable manipulation result while requiring a separate capability when process
  quality must be assessed.

## Consequences and trade-offs

Mission-level process acceptance may need an additional verification step. A manipulation
result remains meaningful even when no settling sensor or process-quality rule exists.

## Authority boundary

This ADR records a responsibility boundary. Current PICK/PLACE success and release behavior
remain owned by Product, Motion, Orchestration FDS, and their ICDs.

## References

- [`docs/product/demos/arm-cell/definition.md`](../product/demos/arm-cell/definition.md)
- [`docs/components/arm-cell/motion/fds-task-execution.md`](../components/arm-cell/motion/fds-task-execution.md)
