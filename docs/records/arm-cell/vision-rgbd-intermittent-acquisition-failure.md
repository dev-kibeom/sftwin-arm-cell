# Known Issue — Intermittent ARM Cell RGB-D Acquisition Failure

> **Document Type:** Durable engineering record — known issue
> **Authority:** Investigation status and provenance only; current behavior and
> policy remain with the referenced design owners.
> **Scope:** ARM Cell Vision acquisition in Isaac Sim–ROS 2 PnP
> **Status:** OPEN

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

The cause has not been distinguished among Isaac publisher behavior, DDS
delivery, subscriber/executor delay, or timestamp synchronization conditions.
The available observations do not show whether these factors interact.

## Follow-up investigation

Capture and compare, for correlated requests:

- RGB and Depth source timestamp distributions and inter-stream deltas;
- the effective synchronization tolerance and callback outcomes;
- cache freshness at request time and whether a cached pair was eligible;
- acquisition duration separately from detector/processing duration.

Use those observations to evaluate any timeout recalculation and only the
minimum synchronization-policy change supported by evidence. No cause or
remediation is presumed by this record.

## Related record

The separate portfolio narrative covers the successful cache-reuse path, the
correlated request logs, and the limits of the current diagnosis:
[Narrowing Intermittent RGB-D Input Failures in the ARM Cell](../../engineering-stories/arm-cell/rgbd-intermittent-input-instability.md).

The earlier settling-policy defect and its resolution are documented
separately in [Debugging an RGB-D Timeout That Wasn't a Synchronization
Problem](../../engineering-stories/arm-cell/rgbd-timeout-wasnt-synchronization.md).

The Vision input path enables eligible fresh-pair reuse, and request
correlation connects Orchestration and Vision events. Neither is evidence that
the intermittent acquisition/pairing failure is resolved.
