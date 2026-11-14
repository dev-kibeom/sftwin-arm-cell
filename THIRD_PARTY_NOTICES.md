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
  - `assets/robots/m0609_robotiq_2f85/third_party/robotiq/LICENSE`
  - `assets/robots/m0609_robotiq_2f85/third_party/robotiq/UPSTREAM.md`
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

## NVIDIA D455 sensor and materials

- Source: NVIDIA Isaac Sim 5.1 content pack,
  `Isaac/Sensors/Intel/RealSense/rsd455.usd` on
  `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/`.
- Location: `infra/isaac_sim/assets/sensors/d455/`.
- Creator/provider: NVIDIA Corporation and its licensors; RealSense D455
  represents Intel hardware. No affiliation or endorsement is implied.
- The asset is incorporated into the ARM Cell simulation camera rig. It is
  third-party Content, not project-owned MIT material or a standalone asset pack.
- Governing content-pack notice and supplied Omniverse agreement are preserved
  as `LICENSE.txt` and `OMNIVERSE-LICENSE.txt` beside the asset. Use and further
  distribution remain subject to those terms, including the Content distribution
  conditions and restrictions on standalone redistribution.
- Kit USD Collector localized the USD, MDL and texture dependencies and rewrote
  their paths. `.collect.mapping.json` records source URLs and hashes. The local
  materials still use the OmniPBR implementation supplied by Isaac Sim.
- All files are checked into Git directly. No asset download or preparation step
  is required after a complete checkout.

## Rainbow Robotics RB5-850E model material

- Upstream: [RainbowRobotics/rbpodo_ros2](https://github.com/RainbowRobotics/rbpodo_ros2),
  revision `5e8294a985e7ce5e20e70564c2681130afc5f502`, `rbpodo_description` 0.0.1.
- License: Apache-2.0; upstream package license and metadata are preserved in
  `assets/robots/rb5_850/third_party/rbpodo_description/`.
- Original URDF, joint/link data and meshes are retained; the local Xacro is an
  adapted derivative. Generated RB5 USD layers are derived compositions with
  the existing Robotiq material (its BSD-3-Clause terms remain applicable).
- Local modifications, variant limitations and redistribution review are recorded
  in that directory's `UPSTREAM.md`; `SHA256SUMS.json` identifies original inputs.
  The separately restricted manufacturer kinematics PDF is not redistributed.
