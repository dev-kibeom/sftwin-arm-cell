# ADR 0008: Preserve Vision source observation identity independently of presentation

- **Document ID:** `ADR-ARM-CELL-0008`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Vision observation and operator presentation`
- **Version:** `1.1.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `2026-10-08`
- **Related Design:** Vision detection and Hub operator-video contracts
- **Supersedes:** `None`
- **Superseded By:** `None`

## Context

Vision processing and operator video can advance independently. An overlay drawn over a live
frame may appear plausible even when its detection came from a different or stale observation.
Presentation transport also changes for deployment and performance reasons.

## Decision

Detection results and diagnostics remain attributable to the exact source RGB observation used
for detection. Operator presentation may render its immutable annotation snapshot over the
nearest same-camera operator frame within a bounded 150 ms window (hard maximum 200 ms). This is
suitable for the fixed camera and effectively static workpiece in the supported ARM Cell view.
The Hub annotation is visualization only and SHALL NOT be used for perception, control, safety,
pose calculation, or mission success decisions. Video transport, compression, and UI lifecycle
do not become perception authority. Presentation degradation is isolated from Vision, Mission,
and Safety outcomes. Changing camera transport for sim-to-real does not remove the
source-observation identity contract.

### Considered alternatives

- **Require the exact detected frame in Hub:** preserves exact visual pairing but couples
  presentation success to retaining one specific RGB frame across independent ROS consumers.
- **Let presentation transport establish which observation was detected:** centralizes display
  data, but makes UI delivery an authority for perception provenance.
- **Preserve source identity and use bounded same-camera temporal presentation (selected):**
  allows small operator-view timing differences while keeping perception provenance exact and
  presentation optional.

## Consequences and trade-offs

The operator display may temporarily show a pinned older source frame while a motion task uses
its result. The UI must communicate that state; failures in that UI path remain display issues.

## Authority boundary

This ADR preserves the separation rationale. Current source identity fields, matching rules,
snapshot lifecycle, and result semantics remain owned by Vision/Hub FDS and ICD documents.

## References

- [`docs/interfaces/arm-cell/icd-vision-orchestration.md`](../interfaces/arm-cell/icd-vision-orchestration.md)
- [`docs/components/arm-cell/integration/fds-operator-ui-hub.md`](../components/arm-cell/integration/fds-operator-ui-hub.md)
- [`docs/components/arm-cell/vision/fds-detect-target.md`](../components/arm-cell/vision/fds-detect-target.md)
- [PR #231](https://github.com/dev-kibeom/sftwin/pull/231)
- [PR #232](https://github.com/dev-kibeom/sftwin/pull/232)
