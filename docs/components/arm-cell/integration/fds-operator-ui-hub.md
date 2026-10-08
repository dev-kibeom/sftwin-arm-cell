# [SF-Twin] ARM Cell Operator UI Hub Behavior

- **Document ID:** `[SF-Twin]_FDS-ARM-CELL-INTEGRATION-OPERATOR-UI-HUB_v0.4.0`
- **Document Type:** FDS
- **Scope / Component:** ARM Cell Integration / Operator UI Hub
- **Concern:** Canonical operator observation and scenario request surface
- **Version:** `0.4.0`
- **Status:** Approved
- **Owner:** ARM Cell Integration
- **Related CDS / HLD:** [Integration CDS](cds.md)

## 1. Purpose and Authority

The operator UI Hub is the single canonical operator surface for the
Final Demo. It observes canonical state and submits requests/stimuli through
owning component boundaries. It is not an authority for Safety, Motion,
Orchestration, Vision, VDA, or Integration canonical runtime state. A separate
Qt HMI is outside this demo scope.

`Operator UI Hub` is the architecture/component role. Its current Final Demo
realization is an Isaac Sim/Kit extension (plugin), whose implementation
realizes this FDS. The FDS defines the role contract without depending on
Isaac Sim or Kit; a future physical-cell supervisor desktop or web UI may
realize the same contract.

## 2. Observation Surface

The Hub SHALL make these current values observable without substituting its
own state for the canonical source:

- AMR/material delivery and `MATERIAL_READY` state;
- batch and iteration/mission state, batch processed count, and terminal
  provenance from Orchestration execution feedback;
- operator camera video/overlay, selected target, freshness/validity, and the
  target currently used by the active iteration;
- holding state (`HELD`, `RELEASED`, `UNKNOWN`);
- Safety motion capability and active permitted motion envelope;
- active fault and Safety-selected severity;
- iteration retry count and retry budget;
- recovery/reset-required state and operator acknowledgement/reset outcome;
- canonical PackML/CNC external-process state.

The PackML indicator follows the read-only projection semantics in [ARM Cell
Simulation Environment Semantics](../simulation-environment-semantics.md). CNC
animation is not part of this demo behavior.

### Responsive presentation

The Hub presentation SHALL keep its camera view and primary controls accessible
when the window is resized or docked narrowly. In a wide window, camera video
and its source/status panel are shown side by side; in a narrow window they are
stacked. The camera view preserves the source frame's aspect ratio and fits the
available width up to its normal display size. Resizing the window changes only
UI layout and display dimensions; it SHALL NOT cause camera re-subscription,
source-frame conversion, or changes to video/annotation identity.

The Hub content SHALL scroll vertically when its available height is too small
to show all observations and controls. Source selection, material request,
operator acknowledge/reset, and scenario controls remain reachable. Long
status text wraps within the available panel width. Camera topic paths and
supporting display details may be collapsed to preserve space for current state
and operator controls.

Displayed state identifies stale or unavailable source data as such. The Hub
does not infer a canonical state from a missing feed, animation, or UI-local
request result.

Material readiness is projected directly from Integration's
`/integration/material_readiness` current state, including its delivery
identity and validity. The Hub's compatible subscription can observe the
latest owner state when it starts after a readiness update. A newer delivery's
not-ready state replaces the prior delivery in the projection. Hub display
freshness does not authorize mission admission; Orchestration retains that
authority and its independent freshness checks.

### 2.1 Operator camera video and Vision annotation

The Hub displays either `/camera/color/image_raw` or
`/camera/aligned_depth_to_color/image_raw` directly as its operator camera
view, independent of Vision's RGB-D processing. RGB is selected by default;
the operator can switch sources at any time, including during ExecuteCycle.
The Hub SHALL NOT access a simulator-local Isaac image source directly; the
same camera topic boundary applies when the source is a physical industrial
camera.

Aligned Depth uses the existing metric `32FC1` image contract. The Hub
converts selected Depth frames to an RGBA heatmap over a fixed 0.6–1.6 m
display range, with a stable teal-to-yellow color map. This range keeps the
canonical 0.80, 1.10, and 1.40 m working-depth examples visually separated.
Valid values outside the range saturate to the nearest endpoint color.
Non-finite and non-positive pixels use a distinct invalid color. This mapping
is presentation-only and SHALL NOT alter sensor values or Vision input.
Live Depth display consumes the newest available frame independently and
SHALL NOT wait for an RGB-D pair. Missing, stale, or unsupported Depth is
shown as unavailable for that source and SHALL NOT disable RGB display, Vision
diagnostics, or PnP behavior.

The Hub renders annotation from Vision-produced diagnostic metadata only.
Vision remains the authority for detection/idle state, `target_id`, source
observation timestamp and camera `frame_id`, support/reference region,
segmented object region or extent, centroid, selected target pixel,
validity/freshness/result, and yaw detected/unavailable semantics. These are
limited to information actually produced by the detector and canonical
Vision result. The Hub SHALL NOT analyze the RGB image, infer a detection, or
create perception annotation itself. Bounding boxes, polygons, and masks may
be rendered only when the detector supplies that representation.

Vision diagnostic/overlay metadata SHALL preserve or reference the original
`header.stamp` and `frame_id` of the RGB frame actually used by detection.
The displayed raw RGB frame SHALL retain its own capture header values; it may
be a different frame from the Vision source frame. The associated depth
timestamp is used for Vision's bounded RGB-D association and SHALL NOT replace
the original RGB source timestamp in Vision diagnostics. The Hub selects the
nearest buffered operator RGB frame whose `frame_id` matches the Vision source
and whose timestamp is within the configured bounded temporal window (default
150 ms, maximum 200 ms). Timestamp proximity across different camera frame IDs
is insufficient.

The Hub annotation is an operator visualization of Vision-produced metadata.
It SHALL NOT be used as an input to perception, control, safety, pose
calculation, or mission success decisions. Vision's result and the original
RGB-D observation remain the sole perception and control provenance. A fixed
camera and effectively static workpiece make a nearby same-camera frame
suitable for this display purpose. Metadata is suppressed when no compatible
frame is available or the temporal bound is exceeded.
Stale, old, or incompatible metadata is suppressed or clearly shown as
stale/unavailable. A marker outside the bounded same-camera display window
SHALL NOT be shown as current. The matching algorithm/library is an
implementation detail.

Vision annotation is an overlay independent of the selected display source.
When Depth is selected, the overlay is rendered only if the aligned Depth
frame has the same optical `frame_id`, pixel geometry, and bounded
operator-view temporal compatibility as the RGB operator frame associated
with the annotation. These overlay compatibility checks do not gate or
synchronize the Depth stream itself. Depth annotation uses a dark outline
under the existing ROI and support-region colors to improve contrast against
the heatmap; RGB annotation appearance is unchanged. Annotation geometry is
not modified by this display styling.

Operator camera display prioritizes freshness and may drop old frames. This
does not impose video-like loss semantics on canonical mission, Safety, fault,
or operational state observers. Hub camera subscriptions use bounded,
latest-frame best-effort queues; other canonical state subscriptions retain
their existing QoS. The material
readiness subscriber uses reliable transient-local QoS compatible with the
Integration owner publisher so a late Hub observes the retained current state.

In the current ROS realization, the Hub consumes RGB and aligned Depth using
bounded latest-frame queues. It keeps a small bounded raw RGB history for
annotation correlation and the latest Depth frame for independent display;
Vision metadata arrives on the read-only `/vision/diagnostics` feed. Vision processing state is `IDLE` or `DETECTING`.
A successful terminal result returns processing to `IDLE` and emits a
successful observation event. While an ExecuteCycle is active, the Hub matches
that event to the nearest compatible same-camera operator RGB frame and
captures the RGB frame, any fresh available Depth frame, and annotation as an
immutable presentation snapshot. The operator may switch sources during the
snapshot; the selected frozen source is displayed, or Depth is shown
unavailable if no fresh frame was captured. It displays the snapshot as
`DETECTION SNAPSHOT · PnP ACTIVE`, retaining Vision-supplied
support-fiducial corners, candidate extent, centroid, target ID, and yaw
availability through PICK, PLACE, and GO_HOME. The Hub releases it only when
the associated ExecuteCycle reaches a terminal action status, then resumes
the current source selection on its live stream. Source selection is retained
across the transition. Snapshot presentation does not block mission work.

The operator-frame tolerance uses the configurable
`annotation_source_tolerance_seconds` setting (default 150 ms, maximum
200 ms); it is not a display timeout or a Vision freshness setting. After a
snapshot is latched, new live-frame timestamps do not invalidate it. If no
compatible operator frame is available, the Hub keeps live video and
suppresses the annotation. ExecuteCycle feedback/result and action status
define mission lifecycle; Vision diagnostics describe only the perception
result and original source frame. Existing selected-target feedback remains
separately visible as canonical mission provenance.

To find a nearby operator frame captured before its diagnostic event, the Hub
retains a small bounded set of recent raw RGB frames with their source identity
and local monotonic receipt time. Pixel conversion is performed for the
selected source or when freezing a PnP snapshot; unselected live sources are
not repeatedly converted or uploaded to Kit. Depth keeps only its latest raw
frame outside a snapshot. This presentation buffering does not preserve or
reconstruct Vision's source observation.

Outside a latched snapshot the Hub displays `LIVE` with current Vision
processing state, such as `IDLE` or `DETECTING`. Missing or stale diagnostics
are shown as unavailable/stale, never inferred as idle or a successful result.
Invalid or unavailable camera frames and annotation rendering failures
degrade only the operator observation surface and do not alter DetectTarget,
ExecuteCycle, or Safety results.

### 2.2 Existing selected-target feedback annotation

The selected-target view may also use the existing ExecuteCycle feedback
fields `target_valid`, `target_observation_stamp`, and `selected_target_pose` /
`has_selected_target_pose` for the object reference actually consumed by the
active iteration. The raw camera source follows the [RGB-D ICD](../../../interfaces/arm-cell/icd-sim-rgbd.md).
The Hub may label an observation stale from its source stamp and configured
freshness bound as display-only state; it SHALL NOT promote an expired
observation to current. The Hub SHALL NOT choose or replace the selected
target from a UI-local click, overlay, simulator identity, or stale prior
pose.

An image-space target marker MAY be drawn only from the valid, fresh
selected-target reference for the active iteration (`target_valid` and
`has_selected_target_pose`), transformed using the existing camera TF at its
observation stamp and projected with CameraInfo intrinsics/calibration
validated as compatible with that RGB image's frame and resolution. TF alone
does not define the image-space projection. If validity, freshness, compatible
calibration, or the required transform is unavailable, the Hub SHALL suppress
the marker or label it explicitly stale/unavailable; it SHALL NOT present a
prior marker as the current target.
This seam is diagnostic visualization only: it does not require bounding
boxes or segmentation masks and does not publish back into Vision, Motion, or
Safety. Observer authority and canonical operational observability are covered
by `VR-INT-UI-01` and `VR-INT-UI-02`; selected-target visualization freshness
is covered separately by `VR-INT-UI-04`.

## 3. Requests and Stimuli

Hub actions may request material supply through the Integration-owned
`/integration/request_material` endpoint defined by the
[Hub↔Integration Material Request ICD](../../../interfaces/arm-cell/icd-hub-integration-material-request.md),
activate an approved fault scenario, or request operator acknowledgement/reset
through the [Safety Operator ICD](../../../interfaces/arm-cell/icd-safety-operator.md).
The receiving owner validates and applies each request under its own authority.
For material supply, the Hub supplies a request correlation UUID; Integration
assigns the `delivery_id` and forwards it through the existing
Integration-to-VDA `/vda/request_material` endpoint. An accepted service
response is `accepted=true` only when VDA accepts that downstream request for
the same `delivery_id`; VDA rejection or unavailability produces
`accepted=false`. With `accepted=false`, `delivery_id` has no meaning and
creates no delivery episode, readiness, or admission success. Even
`accepted=true` confirms only VDA acceptance of the request, not arrival,
transfer, `MATERIAL_READY`, or batch admission. Integration owns the VDA
response and relays its acceptance result; the Hub neither calls VDA directly
nor owns the VDA response. The Hub reports those later outcomes only from
canonical owner-published state. A UI button
or optimistic local display does not establish material readiness, admit a
batch, change Safety capability, reset a latch, choose severity, select a
Vision target, advance a mission, or report a recovery as successful.

For acceptance, an invalid/stale/unavailable target scenario routes to the
source-side RGB-D stimulus seam defined by the [RGB-D ICD](../../../interfaces/arm-cell/icd-sim-rgbd.md),
and Holding Uncertainty routes to the backend-observation seam defined by the
[Motion Backend ICD](../../../interfaces/arm-cell/icd-motion-backend.md).
The Hub requests scenario activation and observes the resulting owner-published
state; it does not alter sensor messages, `MotionStatus`, Safety state, or
Mission state directly. These seams exercise the production Vision and normal
Motion backend paths; the validation-only Fixed Vision adapter is not
production perception evidence.

The operator surface groups controls under **Material Supply**, **Vision
Faults**, **Gripper Faults**, **Safety & External Faults**, and **Safety
Recovery**. Each fault offers separate Inject Fault and Clear Fault actions
with a short description of its stimulus. Vision and Gripper actions continue
to use their existing owner-local adapters. VDA mock actions use the existing
`SetBool` fault services under `/external_state_mock/fault/<scenario>`; the Hub
reports service unavailability, rejection, and call failure distinctly.

Fault request results and canonical effects remain separate. A successful
owner/mock response updates only the Hub's record of the applied stimulus
intent; the Hub identifies the relevant owner observations as the source of
the actual effect. Failed requests do not change that intent record. Clearing
a stimulus is separate from operator acknowledgement/reset: clearing E-Stop
does not reset the Safety latch, clearing Premature Undock does not redock the
AMR, and clearing Communication Degradation does not itself restore normal
motion limits. Safety recovery is reported only from current Safety and other
canonical observations.

## 4. Verification Requirements

### VR-INT-UI-01 — Observer authority
Changing a Hub control or local presentation state SHALL NOT overwrite
canonical Safety, Motion, Orchestration, Vision, VDA, or material state. The
oracle is comparison of owner-published state before and after a request.

### VR-INT-UI-02 — Required observability
Canonical operational values in Section 2 other than selected-target camera
visualization are observable from the single Hub during acceptance scenarios.
Stale/unavailable source state remains distinguishable from a current positive
value. Selected-target camera/overlay freshness is independently covered by
`VR-INT-UI-04`.

### VR-INT-UI-03 — Request semantics
A request is not shown as an accomplished runtime transition until the owning
component's canonical state reports that outcome.

### VR-INT-UI-04 — Current selected-target visualization
An overlay may be presented as current only for the fresh, valid
selected-target reference actually used by the active iteration. Required TF,
CameraInfo, and compatible calibration must be valid for the observation.
Stale, unavailable, or invalid target data SHALL NOT leave a prior marker
presented as current; the Hub may suppress it or label it explicitly stale or
unavailable. Annotation is also compatible with the currently displayed
camera frame's source timestamp and frame identity under a bounded tolerance;
exact timestamp equality is not required. Incompatible camera/metadata pairs
SHALL suppress the marker or label it stale/unavailable.
The Hub observes target selection and is not its authority. The oracle is the
active iteration's selected-target feedback and observation
stamp, source freshness/validity, required TF and CameraInfo/calibration
validity, and whether the current overlay is shown, suppressed, or marked
stale/unavailable. No particular widget, layout, or rendering method is
required.

### VR-INT-UI-05 — Independent RGB and Depth Heatmap display sources

The operator can switch between the default RGB source and aligned Depth
Heatmap before, during, and after ExecuteCycle. Live Depth uses its latest
fresh frame without waiting for an RGB-D pair. While a PnP snapshot is active,
the selected source shows its frame captured at snapshot creation; if fresh
Depth was unavailable then, Depth is shown unavailable until the snapshot
ends. Terminal ExecuteCycle status clears the snapshot and resumes the current
source selection on its live stream. Missing, stale, or unsupported Depth
SHALL NOT affect RGB, Vision diagnostic state, or PnP behavior. A Vision
annotation on Depth is shown only for matching optical frame identity, pixel
geometry, and the bounded operator-view temporal window. The oracle is the
selected source's displayed frame/status and overlay visibility before, during,
and after ExecuteCycle; no display-source state may author canonical control,
Safety, pose, or mission outcomes.

## 5. References

- [Integration CDS](cds.md)
- [Operator camera and overlay CDS](cds-camera-overlay.md)
- [Material Handoff FDS](fds-material-handoff.md)
- [Integration↔VDA Material ICD](../../../interfaces/arm-cell/icd-integration-vda-material.md)
- [Hub↔Integration Material Request ICD](../../../interfaces/arm-cell/icd-hub-integration-material-request.md)
- [Material Readiness ICD](../../../interfaces/arm-cell/icd-integration-orchestration-material.md)
- [Safety Operator ICD](../../../interfaces/arm-cell/icd-safety-operator.md)
- [Orchestration Batch Mission FDS](../orchestration/fds-mission-cycle.md)
- [Safety Supervision FDS](../safety/fds-supervision.md)
- [Vision ↔ Orchestration ICD](../../../interfaces/arm-cell/icd-vision-orchestration.md)
- [Simulated RGB-D ICD](../../../interfaces/arm-cell/icd-sim-rgbd.md)
- [Motion Backend ICD](../../../interfaces/arm-cell/icd-motion-backend.md)
