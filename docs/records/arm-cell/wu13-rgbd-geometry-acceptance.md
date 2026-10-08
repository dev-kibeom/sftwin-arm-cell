# RGB-D Geometry Acceptance Checkpoint

> **Document Type:** Durable historical record
> **Authority:** Provenance only; normative meaning remains in the referenced owner.
> **Scope:** ARM Cell

### RGB-D geometry acceptance checkpoint

- **Status at checkpoint:** Verified
- **Evidence:** Pair stamp `43016668910`; snapshot
  `0351f179958d21c9b2cbfaf39b3784eafee91169df09a7258b6027c4c3c5aeec`
- **Scope:** The authoritative RGB-D object surface is
  `/World/SF_Twin_Cell/AMR/Mockup/RawPart`. Auxiliary `RawPartMarker` geometry
  was removed. Reference generation now uses RawPart authored dimensions to
  construct the visible top surface rather than the prim origin/center.
- **Result:** TF, pixel correspondence, metres, and
  `DistanceToImagePlane` / optical-axis-Z semantics verified. Three RawPart
  samples matched with maximum z error `5.3e-7 m` under the unchanged `0.002 m`
  tolerance.
- **Traceability:** This records RGB-D geometry acceptance only; it does not
  establish PICK/Motion calibration, live PICK/PLACE, stop/recovery, or
  integrated mission behavior.
