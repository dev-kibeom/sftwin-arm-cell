# ADR 0009: Keep ARM Cell Vision detector strategy replaceable

- **Document ID:** `ADR-ARM-CELL-0009`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Vision architecture`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-08`
- **Related Design:** Vision DetectTarget contract and detector profile design
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

The ARM Cell production target and workspace geometry are constrained enough to support
profile-specific detection. Committing the public perception interface to one algorithm would
make replacing or adding a detector a contract migration rather than an internal realization
change.

## Decision

`target_id` selects a Vision-owned detector profile/strategy. The public `DetectTarget`
contract, result semantics, and detector-profile-defined Observation Pose remain independent of
the detector implementation. Each strategy converts its internal representation, confidence,
and geometry to canonical Vision result semantics. Information a strategy cannot observe is
represented as explicitly absent, not inferred to satisfy downstream convenience; yaw
availability is one example.

The current RawPart depth/geometry detector is one realization. YOLO, learned segmentation,
classical CV, geometry fitting, or another strategy may be introduced or substituted without
Orchestration learning its internal algorithm. Simulator ground truth and robot TCP/tool
geometry are not detector-strategy authorities.

## Consequences and trade-offs

Vision must define profile semantics and honest presence/absence for each result field.
Strategy-specific diagnostics can differ; consumers rely on canonical result meaning rather
than algorithm detail.

## Authority boundary

This ADR records architecture-level replaceability. Current detection thresholds, ingress,
RawPart geometry algorithm, field definitions, and runtime behavior remain with the Vision
CDS/FDS and Vision-Orchestration ICD. In particular, this ADR does not canonize an algorithm
such as RANSAC, PCA, or connected components.

## References

- [`docs/interfaces/arm-cell/icd-vision-orchestration.md`](../interfaces/arm-cell/icd-vision-orchestration.md)
- [`docs/components/arm-cell/vision/cds.md`](../components/arm-cell/vision/cds.md)
- [`docs/components/arm-cell/vision/fds-detect-target.md`](../components/arm-cell/vision/fds-detect-target.md)
- [`docs/components/arm-cell/vision/fds-rawpart-depth-perception.md`](../components/arm-cell/vision/fds-rawpart-depth-perception.md)
