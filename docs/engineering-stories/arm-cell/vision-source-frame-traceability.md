# Making Robot Vision UI Truthful: Tracing Detections to Their Source Frames

*Portfolio narrative; non-authoritative. Current detection and presentation behavior is owned by Vision/Hub FDS and ICD.*

## Problem

Vision can process a camera observation while the operator video continues advancing. An
annotation rendered over whichever frame is currently live can therefore imply that a target
was found in an image the detector never examined. That makes the display look clear while
losing the evidence trail.

## Investigation and resolution

The design preserved source RGB identity in Vision's successful detection diagnostic and
matched its frame identity against the Hub's bounded frame history under bounded timestamp
compatibility. On a matching source frame, Hub pins an
immutable detection snapshot and keeps it visible through the associated PnP action. It then
returns to live presentation at the action's terminal result.

The snapshot lifecycle belongs to presentation. It does not create a Vision lock state or
change DetectTarget, mission, or Safety outcome. A missing, stale, or incompatible display
frame degrades the operator view rather than rewriting perception truth.

## Result

An operator can inspect the source frame associated with a detection instead of seeing an
unrelated live image under its annotations. The guarantee follows the observation identity,
not a particular video transport or compression format.

## What we learned

When a UI visualizes a machine result, keep provenance with the result and make the UI join on
that identity. A convincing overlay without source matching is worse than an explicit display
degradation.

## References

- [PR #231](https://github.com/dev-kibeom/sftwin/pull/231), [#232](https://github.com/dev-kibeom/sftwin/pull/232), [#233](https://github.com/dev-kibeom/sftwin/pull/233)
- [`Hub operator UI FDS`](../../components/arm-cell/integration/fds-operator-ui-hub.md)
- [`Vision-Orchestration ICD`](../../interfaces/arm-cell/icd-vision-orchestration.md)
