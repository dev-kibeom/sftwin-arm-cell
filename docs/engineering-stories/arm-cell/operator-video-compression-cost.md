# Removing an Unnecessary Compression Hop from Operator Video

*Portfolio narrative; non-authoritative. Current video composition and traceability behavior is owned by Hub and bringup design.*

## Problem

Production demo runs on one machine. The operator-video path nevertheless compressed and
decompressed camera frames through ROS image transport and a local sidecar, adding work to a
path whose consumer was on the same host. At the same time, removing transport stages could
not be allowed to weaken Vision-to-frame traceability.

## Investigation and resolution

The team kept the source-frame identity and snapshot-matching behavior established for truthful
presentation, then simplified only the transport realization. Production Hub now consumes the
raw RGB topic directly; the production `image_transport` republisher and local zlib image
compression were removed. Annotation rendering and source-frame pinning remained.

## Result

The video path has fewer transformations and the live operator display later showed successful
PnP with reduced lag. No quantitative RTF improvement was established, and the change was not
claimed to explain or eliminate the broader simulator slowdown.

## What we learned

Keep end-user meaning stable while measuring and simplifying deployment-specific plumbing.
A simpler transport is a bounded optimization; without controlled measurement it is not proof
of a system-wide performance gain.

## References

- PR #231, PR #232, PR #233
- PR #269
- [`Hub operator UI FDS`](../../components/arm-cell/integration/fds-operator-ui-hub.md)
