# Geometry-First RawPart Perception Without a Learned Detector

*Portfolio narrative; non-authoritative. Current detector behavior and contract are owned by Vision CDS/FDS and ICD.*

## Problem

The production cell needs to identify and localize a known industrial RawPart in a constrained
workspace. A generic YOLO or learned detector was not the only reasonable starting point: the
part geometry, workspace, depth source, and support surface were known, and the required
Observation Pose had a precise downstream meaning.

## Investigation and resolution

The production strategy uses RGB-D as measured sensor input. RGB ArUco/fiducial observations
establish the support/work-surface reference, not the target's identity. Vision deprojects
depth into 3D and builds support-relative geometry, then applies ROI and support-relative
height filtering. Candidate isolation is based on depth/3D continuity; the known RawPart shape
and dimensions validate the resulting target candidate. RGB appearance or a learned detector
is not a required input to the nominal RawPart profile.

The reported Observation Pose uses the visible top-surface centroid, matching the production
reference needed by Motion. The detector reports yaw only when its geometry supports an
unambiguous estimate. When it cannot, it sets `has_target_yaw=false`, leaving the explicit
fallback to the downstream recipe rather than manufacturing certainty.

No simulator ground truth, world-coordinate shortcut, USD prim identity, or robot TCP/tool
geometry is used as production perception input. RANSAC, PCA, and connected-component steps
are implementation techniques that can be replaced while preserving the Vision contract.

## Result

The constrained target and workspace can use depth geometry directly and produce a
contract-level object reference without binding perception to a learned model or simulator
internals. The result is scoped to the current RawPart profile; it does not establish that
geometry-first detection is preferable for every future target.

## What we learned

Choose the detector around the available physical evidence and the target's geometry. Define
the reference point and uncertainty honestly, then keep the algorithm behind a stable profile
contract so the next target can use a different strategy.

## References

- [`RawPart Depth Perception FDS`](../../components/arm-cell/vision/fds-rawpart-depth-perception.md)
- [`DetectTarget ICD`](../../interfaces/arm-cell/icd-vision-orchestration.md)
- PR #55, PR #70, PR #189
