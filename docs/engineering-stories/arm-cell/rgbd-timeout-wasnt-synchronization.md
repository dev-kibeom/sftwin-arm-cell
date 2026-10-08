# Debugging an RGB-D Timeout That Wasn't a Synchronization Problem

*Portfolio narrative; non-authoritative. Current acquisition and timeout semantics are owned by Vision FDS/ICD.*

## Problem

`DetectTarget` sometimes reached its 500 ms timeout while waiting for RGB-D
input, before detection began. The initial suspicion was a camera restart or
RGB/depth synchronization lifecycle defect: after Isaac Sim Stop/Play, the two
`ROS2CameraHelper`s could initialize at different phases.

## Investigation

PR [#261](https://github.com/dev-kibeom/sftwin/pull/261) added timing
acceptance measurements for the individual streams, synchronized-pair cadence,
request-to-observation wait, and processing time. Its live ROS-side evidence
could not by itself distinguish Isaac source/render cadence from ROS publish
drop. PR [#262](https://github.com/dev-kibeom/sftwin/pull/262) experimentally
sequenced the RGB and Depth helper execution in one graph lifecycle sequence
to reduce phase divergence after restart. That topology risked adding delay to
the steady-state simulation path. PR [#263](https://github.com/dev-kibeom/sftwin/pull/263)
therefore restored direct helper fan-out while retaining the diagnostics; the
Stop/Play resynchronization concern remained a separate bounded issue.

PR [#266](https://github.com/dev-kibeom/sftwin/pull/266) then made request
diagnostics distinguish RGB and Depth arrivals, source stamps and receipt
periods, actual synchronized-pair callbacks, policy evaluations, usable
observations, and detector entry. The clean-start trace changed the diagnosis:
a valid RGB-D pair arrived after the request, but was rejected until source
time advanced by another 50 ms. That request-scoped settling condition was not
required by the applicable FDS/ICD or demo contract; freshness already
protected the request from older observations.

## Resolution

PR [#268](https://github.com/dev-kibeom/sftwin/pull/268) removed the
request-relative settling condition and its related status/timeout wording.
It retained post-request RGB/Depth receipt and source-watermark freshness,
RGB-D synchronization, the 500 ms request deadline, and the 10 ms sync
tolerance. The team did not enlarge the timeout or change the camera graph to
resolve this policy defect. Stabilization, if required for a camera or scene
transition, belongs to an explicit contract owned by that transition.

## Result

The unnecessary policy gate that rejected a valid first pair was removed.
Focused Vision tests and the affected package build passed for #268; Isaac
live acceptance was still pending at that PR. This correction resolves that
specific policy defect. It does not establish that all intermittent Vision
timeouts or RGB-D acquisition failures are solved. Later observations of
missing Depth frames and absent sync callbacks are tracked separately in the
[intermittent RGB-D input investigation](rgbd-intermittent-input-instability.md)
and its [Known Issue](../../records/arm-cell/vision-rgbd-intermittent-acquisition-failure.md).

## What we learned

Measure arrival, pairing, policy acceptance, and detector entry as separate
stages before changing timing or lifecycle behavior. A timeout can come from
a local acceptance gate even when transport and pairing are healthy; removing
that gate does not rule out independent failures earlier in the input path.

## References

- [PR #261](https://github.com/dev-kibeom/sftwin/pull/261) — timing
  acceptance measurements and timeout diagnostics.
- [PR #262](https://github.com/dev-kibeom/sftwin/pull/262) — sequenced helper
  lifecycle experiment.
- [PR #263](https://github.com/dev-kibeom/sftwin/pull/263) — restored
  fan-out and retained diagnostics.
- [PR #266](https://github.com/dev-kibeom/sftwin/pull/266) — request-scoped
  RGB-D synchronization diagnostics.
- [PR #268](https://github.com/dev-kibeom/sftwin/pull/268) — removed
  per-request settling.
- [`Vision Frame Ingress FDS`](../../components/arm-cell/vision/fds-frame-ingress.md)
