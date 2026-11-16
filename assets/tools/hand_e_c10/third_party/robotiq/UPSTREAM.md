# Robotiq Hand-E C10 upstream

- Repository: https://github.com/robotiq/robotiq-ros
- Commit: `91ca17bae8129c202eb33eb4c16b91060fee8fab` (retrieved 2026-11-16).
- Package: `grippers/robotiq_description`, package.xml version `1.3.0`.
- License: BSD-3-Clause. Both repository Robotiq copyright (`LICENSE.robotiq`)
  and grippers PickNik copyright (`LICENSE`) notices are retained here.
- Source Xacro: `grippers/robotiq_description/urdf/robotiq_hand_e_c10_macro.urdf.xacro`,
  `hand_e_c10.ros2_control.xacro`, `common.ros2_control.xacro`.
- Mesh source: `grippers/robotiq_description/meshes/{visual,collision}/hand_e_c10/`.
  All six STL files are byte-identical; `mesh_sha256.json` records their hashes.

Local adaptations: replace package mesh URIs with the `mesh_dir` macro argument;
resolve ros2_control includes locally and move the C10 control include outside
its macro so nested invocation resolves from this vendored file. The assembly
passes `include_ros2_control=false`. Geometry, inertias, axes, limits, mimic
relationships and the vendor TCP offset are unchanged. `assembly.xacro` adds the
canonical `sf_grasp_tcp` alias and mounts to existing `tool_mount`.

The upstream CAD has a 19.2 mm radial stroke per finger (38.4 mm diameter change),
not an exact 40 mm diameter stroke. Positive position contracts: external/OD
closing. Decreasing position expands: internal/ID closing. Neither direction
alone is evidence of a successful grasp.
