# ARM Cell live acquisition replay fixture

This fixture comes from the saved Vision diagnostic record for the latest
available live test session on 2026-10-24, acquisition 11 (depth stamp
`180.366676073`, frame `camera_color_optical_frame`). The matching diagnostic
JSON is retained verbatim and contains the applied RawPart profile, camera
intrinsics, Tray calibration, observation metadata, and the original detection
failure.

The replay input is the exact 161 x 241 row-major Height Grid (`float64`, little
endian, 2.5 mm spacing, origin `[-0.3, -0.2]` in Tray) and the exact 161 x 241
image-boundary mask saved with that record. The acquisition did not preserve
the original RGB or raw Depth pixel arrays. The PNG files in the live Vision
Snapshot directory are rendered diagnostic views and are not used as test
input.

This acquisition is associated with an Isaac rigid-body pose recording from
the same live-test session. RawPart 5's last `pose_changed` sample was at
simulation time `173.116666667 s` (7.250 s before the Vision observation); no
later pose event for that object appears before the acquisition. Transforming
that world pose with the camera snapshot and the acquisition's recorded
Tray-from-camera calibration gives Ground Truth Top Reference
`[0.21580078, -0.01609743, 0.07143482] m`, with the Recipe `0.08 m` top axis at
`1.05716223 rad` modulo pi. The replayed candidate is `[0.215471, -0.0138791,
0.0714564] m`, yaw `1.07077 rad`, footprint `0.08 x 0.07 m`; its XY error is
`2.24 mm`, top-height error `0.02 mm`, and yaw error `0.014 rad`. This is an
accuracy comparison against the time-adjacent recorded rigid-body pose; it is
separate from the synthetic rotation tests, which check deterministic proposal
and handoff consistency for ambiguous observations.

SHA-256:

- Height Grid: `bbb3cf02f9e66dae2f5bc9067f46d11bb6cb61714dfc3c3633391581288b2c98`
- Boundary mask: `3e58f3c1232113aae65787be3151240ed51f991db403d03464510546ccadaf3e`
- Diagnostic JSON: `08a4278f3f6d179542114e19318e2f9f44b321f2bee6bea11622cc539ee77a86`
