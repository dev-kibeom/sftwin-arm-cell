# Narrowing Intermittent RGB-D Input Failures in the ARM Cell

*Portfolio narrative; non-authoritative. Current acquisition and timeout semantics are owned by Vision FDS/ICD.*

## Problem

After the per-request settling defect was removed, intermittent `DetectTarget`
timeouts still occurred before detector entry in Isaac Sim–ROS 2 ARM Cell PnP.
The remaining question was whether fresh-pair reuse and correlated request
logs could improve successful response time and diagnosis, while separating
those improvements from failures to deliver or synchronize a pair.

## Investigation

The Vision input path now reuses a recent structurally valid RGB-D input pair
when it meets the freshness policy.
This avoids waiting for another pair when an eligible input is already
available. It does **not** cache detector results. If no eligible pair is
available, the request continues through acquisition under the existing
deadline.

Orchestration logs now correlate delivery and `ExecuteCycle`
dispatch/acceptance with
Vision dispatch and terminal outcomes using their IDs. This improves request
traceability; it did not change the RGB-D input pipeline.

The subsequent observations included two successful response paths and two
failures before detector entry:

- **Success A — cached pair:** Orchestration response wait was about 230 ms.
- **Success B — new pair:** Orchestration response wait was about 460 ms.
- **Failure A — no Depth arrival:** Four RGB frames and zero Depth frames
  arrived; last Depth age was 769.4 ms; the detector did not start.
- **Failure B — no synchronized callback:** Three RGB and five Depth frames
  arrived, but the sync callback count was zero; the detector did not start.

## Improvement

The cache path reduces avoidable acquisition waiting when a fresh usable
pair exists. Request correlation makes a delivery's `ExecuteCycle` and Vision
events easier to follow. Neither result is a fix for the two observed
pre-detector failure shapes, and neither proves that the input path is stable.

## Validation and diagnostic boundary

The approximately 230 ms and 460 ms response waits establish that cached-pair
reuse and new-pair acquisition each completed successfully in those observed
runs. In the failure runs, one had no Depth arrivals while the other received
both streams but had no synchronized callback. Both failed before detector
entry. This narrows the failure boundary to stream arrival and/or pair
formation, but does not locate the responsible component or establish one
common cause.

Isaac publisher behavior, DDS delivery, subscriber/executor delay, timestamp
distributions, and synchronization tolerance remain possible but unconfirmed
contributors. The observations do not distinguish among them. Follow-up
measurement should capture actual timestamp distributions, effective
synchronization tolerance, cache freshness at request time, and acquisition
duration separately from detector/processing duration. Any timeout
recalculation or synchronization-policy change should be grounded in those
measurements.

The active investigation is tracked in the [Known Issue](../../records/arm-cell/vision-rgbd-intermittent-acquisition-failure.md).
The separate resolved settling-policy defect is documented in
[Debugging an RGB-D Timeout That Wasn't a Synchronization Problem](rgbd-timeout-wasnt-synchronization.md).

## References

- [`Vision Frame Ingress FDS`](../../components/arm-cell/vision/fds-frame-ingress.md)
- [`Detect Target FDS`](../../components/arm-cell/vision/fds-detect-target.md)
- [`Vision–Orchestration ICD`](../../interfaces/arm-cell/icd-vision-orchestration.md)
