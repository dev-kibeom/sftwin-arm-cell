# Resolved Issue — Intermittent ARM Cell RGB-D Acquisition Failure

> **Document Type:** Durable engineering record — resolved issue
> **Authority:** Investigation status and provenance only; current behavior and
> policy remain with the referenced design owners.
> **Scope:** ARM Cell Vision acquisition in Isaac Sim–ROS 2 PnP
> **Status:** RESOLVED in v0.3.0

## Resolution

The intermittent request-time failure to acquire a usable synchronized RGB-D pair was resolved in v0.3.0. The depth-reception gaps and missing synchronization callbacks below are retained as pre-resolution observations. The v0.3.1 Docker UDPv4 configuration addresses a separate DDS topic-delivery problem.

## Symptom

`DetectTarget` intermittently fails to obtain a usable RGB-D pair and reaches
the Vision 500 ms request timeout before detector processing begins. This can
make `DetectTarget` fail and stop an `ExecuteCycle`.

## Confirmed failure observations

- **Depth reception gap:** Four RGB frames and zero Depth frames were received;
  reported Depth age was 769.4 ms; the detector did not start.
- **Pairing failure after reception:** Three RGB frames and five Depth frames
  were received, but the synchronization callback count was zero; the detector
  did not start.

These observations establish two failure shapes before detector entry. They
do not identify the responsible component or establish a shared root cause.

## Unconfirmed causes

The original investigation did not distinguish the cause among Isaac publisher behavior, DDS delivery, subscriber/executor delay, or timestamp synchronization conditions. These hypotheses are historical and do not represent a current open defect.

## Follow-up investigation

The issue is closed. The following measurements were proposed during the original investigation and are retained for historical context:

- RGB and Depth source timestamp distributions and inter-stream deltas;
- the effective synchronization tolerance and callback outcomes;
- cache freshness at request time and whether a cached pair was eligible;
- acquisition duration separately from detector/processing duration.

No timeout or synchronization-policy change is proposed by this closed record.

## Related record

The separate Engineering Story covers the successful cache-reuse path, the
correlated request logs, and the limits of the current diagnosis:
[Narrowing Intermittent RGB-D Input Failures in the ARM Cell](rgbd-intermittent-input-instability.md).

The earlier settling-policy defect and its resolution are documented
separately in [Debugging an RGB-D Timeout That Wasn't a Synchronization
Problem](rgbd-timeout-wasnt-synchronization.md).

At the time of the original investigation, PR #278 enabled eligible fresh-pair reuse and PR #282 improved request correlation; neither change alone was treated as proof of resolution. The issue was subsequently resolved in v0.3.0.
