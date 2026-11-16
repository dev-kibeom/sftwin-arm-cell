# M0609 + Hand-E C10 — stage 1

This assembly reuses the existing M0609 and standard tool adapter without
changing M0609, RB5, HCR-14 or Robotiq 2F-85 assemblies.
The C10 housing back face mounts directly at `tool_mount`; no vendor coupling
or long custom finger is modelled. This is a simulated mount, not a fabrication
or physical fit approval.

The three prismatic joints retain upstream limits **0–0.0192 m**, velocity
**0.0535 m/s**, effort **175 N per finger**, and direct 1:1 mimic relationships.
Their axes are local `-X`, rotated by 0°, 120°, 240° around the housing axis.
Increasing position contracts for **external diameter (OD)** closing; decreasing
position expands for **internal diameter (ID)** closing. Stage 1 validates
motion only, not either grasp operation.

`sf_grasp_tcp` is an identity alias of the official `gripper_tcp_link`: on the
housing axis, **154.77 mm along +Z from its back face**, at the top of the larger
finger step in the vendor CAD. +Z remains the tool insertion direction. The
manual's flange-to-TCP distance includes a coupling which this assembly does
not model; it must not be added to the housing datum again.

Visual and split collision STL, inertia and TCP data come from
[the pinned upstream source](../../tools/hand_e_c10/third_party/robotiq/UPSTREAM.md).
The base has housing/front collision pieces and each finger has slider/tip
pieces. They remain separate convex collision parts during Isaac import.
The existing importer keeps semantic fixed frames and disables PhysX mimic;
the experiment commands all three linear drives using the preserved URDF mimic
multipliers/offsets. No 2F-85 width conversion or bilateral contact logic is used.

Run from repository root after sourcing ROS 2 Humble:

```bash
bash infra/isaac_sim/scripts/run_model_experiment.sh \
  ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10.yaml
# Bounded live verification:
bash infra/isaac_sim/scripts/run_model_experiment.sh \
  ros2_ws/src/bringup/config/model_experiments/m0609_hand_e_c10.yaml \
  --headless --cycles 1
```

The Launcher has a separate labelled C10 model experiment button. It runs a
standalone model and repeats contraction/expansion, with ROS start disabled.
The model experiment is rejected by the production runtime profile loader.
There is no Mission, GRASP/RELEASE, Vision, payload, physical grasp, new Cell or
material integration. Runtime tuning is experimental and has no calibrated
force or retention claim. Internal self-collision follows the existing importer
policy (disabled); live checks cover imported collision presence, not physical
contact accuracy or payload handling.

Each run records its resolved configuration and `result.json` under
`.run_records/model-experiment-*/`, and produces a derived USD with the existing
model provenance. Measurements cover joint limits, endpoint tracking,
synchronization, radial displacement, TCP and split collision presence. A PASS
requires a complete contraction/expansion cycle. A closed window or failed check
must not be reported as a completed validation.
