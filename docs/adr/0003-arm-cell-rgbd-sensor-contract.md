# ADR 0003: ARM Cell simulated RGB-D sensor identity and geometry derivation

- **Document ID:** `ADR-ARM-CELL-0003`
- **Document Type:** ADR
- **Scope:** `ARM Cell / Milestone 1C simulated RGB-D source`
- **Version:** `1.0.0`
- **Status:** `Accepted`
- **Decision Owner:** `Final Architecture Authority`
- **Decision Date:** `Historical Milestone 1C decision; exact date not recorded`
- **Related Design:** `docs/interfaces/arm-cell/icd-sim-rgbd.md`
- **Supersedes:** `None`
- **Superseded By:** `None`

## 1. Context

Milestone 1C needs a stable simulated RGB-D source for later perception without binding the
ROS contract to asset-internal D455 prim structure or relying on handwritten calibration.

The composed USD hierarchy can contain numerical affine drift and simulator-specific camera
conventions. Copying a raw affine transform or binding ROS directly to an internal camera prim
would make the external contract depend on asset implementation details. Hand-calculating
CameraInfo or silently rewriting timestamps/units would also create a second source of truth.

The milestone therefore needs one logical sensor identity, explicit transform derivation, and
an auditable conversion between USD camera coordinates and the ROS optical convention.

## 2. Decision

We will treat logical `Camera_Sensor` as one simulated RGB-D sensor and derive its ROS-facing
geometry from the composed camera only through explicit validation and coordinate conversion.

Specifically:

- RGB, aligned depth, and native CameraInfo are produced from the same logical sensor/render
  product rather than independently configured sensor identities.
- CameraInfo is produced by Isaac's native camera helper rather than handwritten calibration.
- `Camera_Sensor` remains the logical ROS source; ROS is not bound directly to the D455
  asset-internal camera prim.
- scene construction derives the logical sensor pose from the composed D455 color camera only
  after an explicit near-rigid validation accepts bounded numerical composition drift;
- meaningful scale, shear, non-uniform scale, or reflection rejects derivation rather than being
  silently normalized;
- the raw affine matrix is not copied as the ROS transform;
- ARM Cell bringup owns a camera TF overlay that separates installation-frame camera identity
  from the ROS optical child frame;
- USD camera coordinates are explicitly converted to the ROS optical convention rather than
  relying on implicit axis assumptions;
- acquisition timestamps and depth units are preserved by the transport contract; no hidden
  timestamp or unit rewriting is introduced;
- build-time camera-pose derivation is not runtime pose synchronization.

Exact topic names, encodings, calibration values, QoS, depth semantics, timestamp rules, and
TF contracts are owned by the current interface/design documents rather than this ADR.

### 2.1 Considered Alternatives

#### Option A — Bind ROS directly to the D455 asset-internal camera prim

- **Advantages:** less logical indirection.
- **Trade-offs:** external contract becomes coupled to asset hierarchy and internal prim naming.
- **Why not selected:** the logical sensor identity should survive asset-internal changes.

#### Option B — Hand-author CameraInfo/calibration in ROS

- **Advantages:** simple static configuration.
- **Trade-offs:** creates a second calibration source that can drift from the composed camera.
- **Why not selected:** native camera output should remain authoritative for simulated intrinsics.

#### Option C — Copy the composed affine matrix directly into ROS TF

- **Advantages:** minimal transform processing.
- **Trade-offs:** can silently carry scale/shear/reflection and ignores coordinate-convention
  differences.
- **Why not selected:** rigid-pose validity and basis conversion must be explicit.

#### Option D — Normalize timestamps/units in a downstream ROS adapter

- **Advantages:** downstream consumers receive a convenient standardized stream.
- **Trade-offs:** hides source semantics and creates derived timing/unit authority.
- **Why not selected:** Milestone 1C is intended to verify the source contract before perception
  consumes it.

## 3. Consequences and Trade-offs

### Benefits

- Perception can depend on one stable logical simulated sensor identity.
- Camera geometry is auditable from composed stage state to ROS optical TF.
- Native calibration and acquisition metadata remain source-authoritative.
- Asset-internal hierarchy can change without automatically redefining the ROS-facing identity.

### Trade-offs / Costs

- Scene construction and snapshot tooling must perform explicit rigidness and coordinate-basis
  checks.
- Camera TF requires a dedicated overlay rather than being inferred implicitly.
- The decision does not yet provide a Vision node, synchronization service, runtime pose
  tracking, command path, or planning-scene update.

## 4. Mitigation Strategy

- Reject non-rigid geometry instead of silently normalizing it.
- Preserve direct snapshot/inspection evidence so transform derivation can be independently
  checked.
- Keep current topic/QoS/calibration/depth/timestamp details in ICD-level design and keep this
  ADR focused on the rationale for sensor identity and derivation.
- Promote RGB-D acceptance results to a durable report only when the Milestone 1C baseline is
  explicitly complete and identified.

## 5. References

- `docs/interfaces/arm-cell/icd-sim-rgbd.md`
- `docs/guides/arm-cell-isaac-sim.md`
