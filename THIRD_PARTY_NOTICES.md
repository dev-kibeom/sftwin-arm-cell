# Third-Party Notices

This notice applies to the SF-Twin ARM Cell public export.  The repository's
MIT license covers project-owned code only.  It does not replace the terms of
third-party material distributed with the project.

## Robotiq description material

The ARM Cell description includes a locally adapted copy of the Robotiq
description material.  Its original robot geometry, kinematics, inertial
parameters, and joint definitions are retained unless a local change is
explicitly documented.

- Upstream: `https://github.com/robotiq/ros`
- License: BSD 3-Clause
- Distributed notices and license:
  - `ros2_ws/src/arm_cell/arm_cell_description/third_party/robotiq/LICENSE`
  - `ros2_ws/src/arm_cell/arm_cell_description/third_party/robotiq/UPSTREAM.md`
  - `infra/isaac_sim/assets/third_party/robotiq_description/LICENSE`
  - `infra/isaac_sim/assets/third_party/robotiq_description/UPSTREAM.md`

## Doosan Robotics M0609 model material

- Upstream source/copyright holder: Doosan Robotics Inc.,
  [`DoosanRobotics/doosan-robot2`](https://github.com/DoosanRobotics/doosan-robot2)
  at reviewed revision `6c5f3ba622bfa9d6f9cffebf21fa44f57db55b48` (`humble`).
- Provenance: the 30 local M0609 DAE meshes match the corresponding upstream
  Collada content canonically; some files are byte-identical and the remaining
  differences are XML serialization whitespace only.
- Affected material: M0609 collision, white visual, and blue visual meshes;
  local M0609 Xacro/URDF descriptions; and any generated USD distributed from
  these sources.
- Relationship: the meshes preserve upstream geometry. Local robot
  descriptions are adapted derivatives, including Isaac dynamics settings;
  the combined M0609/Robotiq assembly and generated USD are derived
  compositions. SF-Twin does not claim ownership of Doosan model material.
- The upstream repository root declares BSD 3-Clause. The `dsr_description2`
  package containing the M0609 material declares Apache License 2.0 in its
  `package.xml`. The upstream root BSD text and standard Apache 2.0 license
  text are included under `third_party/licenses/doosan-robot2/`; no
  package-local Apache license text was present upstream.
- Both upstream declarations are recorded and preserved. This does not assert
  that the assets are dual-licensed or determine which declaration controls
  the model files.

The project MIT license applies only to project-owned material and does not
replace these third-party declarations or terms.
