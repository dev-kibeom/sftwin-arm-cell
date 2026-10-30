# Separating PICK Approach IK Failure from Geometric Reachability

*Engineering Story; non-authoritative. Current PICK planning and robot-model behavior remain owned by the applicable ARM Cell implementation and design contracts.*

## Problem

An ARM Cell Live PICK attempt failed during approach IK evaluation even though the operator's inspection considered the target pickable. A preceding attempt at another target had reached its approach and gripper-close stages. The immediate question was whether the second target was geometrically unreachable, whether its orientation or approach construction mattered, or whether the offline IK result reflected solver/model behavior.

The first attempt must be described precisely: it reached and executed the approach and issued gripper close, but its terminal result was `GRASP_FAILURE` with `gripper_held_confirmed=false`. It was an approach success, not a completed successful pick.

## Investigation and judgment change

The recorded second PICK used grasp TCP position `[0.145037, -0.735018, 0.0159729]` m, a 150 mm approach distance, and nominal orientation quaternion `[0.970031, 0.242706, 0.0104295, -0.00488916]` (xyzw). Its configured insertion axis was TCP +Z. That axis points mostly toward world −Z for the nominal orientation, so the existing insertion-axis offset places the approach near world Z `0.16593` m; the observed coordinate was not a sign-flipped height.

The live planning diagnostic reported all 20 pose candidates as `APPROACH_IK_UNREACHABLE`, with `ik_rejected=20`, `collision_rejected=0`, `planning_failed=0`, and no selected candidate. This is evidence that the configured IK solver returned no solution for those candidates; it does not establish physical or geometric impossibility. The earlier 1 mm approach-distance sweep also showed a sharp model/solver boundary: with the same nominal pose and HOME seed, offsets through 61 mm solved and 62 mm did not. A threshold in an offline IK search is not proof of a workspace boundary.

The follow-up offline comparison kept the failed grasp position and 150 mm approach distance. For each tested orientation, grasp and approach had the same TCP orientation. Condition A offset the approach along the rotated TCP insertion axis. Condition B held world X/Y fixed and placed approach exactly 150 mm above grasp in world Z. The static planning scene was loaded with eight collision objects and read back before path checks.

| World tilt applied to nominal orientation | Grasp IK | A: insertion-axis approach IK | B: world-Z approach IK | B Cartesian approach-to-grasp |
|---|---|---|---|---|
| World X −15° | pass | pass | pass | fraction 1.0; all checked states collision-free and within joint limits |
| World X −30° | pass | pass | pass | fraction 1.0, but approach start state collides (`link_2` / `amr_tray_rail_rear`) |
| World Y −30° | pass | fail (`NO_IK_SOLUTION`) | fail (`NO_IK_SOLUTION`) | not run |

For B, all three tested approach targets were `[0.145037, -0.735018, 0.1659729]` m. The corresponding tested quaternions (xyzw) were:

- World X −15°: `[0.962370414, 0.241990939, -0.021339216, 0.12176712]`
- World X −30°: `[0.938243403, 0.237135347, -0.052742812, 0.246339931]`
- World Y −30°: `[0.934278642, 0.235701401, 0.261136621, 0.058094369]`

The A approach positions were respectively `[0.142358, -0.698313, 0.161388]`, `[0.142358, -0.661928, 0.146933]`, and `[0.067737, -0.737200, 0.144502]` m. A broader world-axis tilt search found additional A-pose IK solutions at World X −45°, −60°, and −90°, and World Y −45° and −60°. A world-Z yaw sweep and the prior 20 candidate orientations did not solve the nominal approach. The targeted B comparison was limited to the three priority tilts above; it was not a full B-orientation search.

For the World X −15° B case, the 150 mm Cartesian path completed with fraction 1.0 and 14 trajectory points. Forward kinematics showed a Z change of `−0.149999947` m and maximum X/Y drift of about `9.4 µm`. Maximum orientation error along the sampled trajectory was `0.001978` rad (about `0.113°`). Every sampled state was valid in the loaded static scene and within the model's joint limits. For World X −30°, the Cartesian fraction alone was not sufficient: the approach start state already collided with the rear tray rail, so the full route was not collision-free.

## Result

The requested offline test found a B-condition candidate at World X −15° for which both grasp and approach IK solved and the sampled 150 mm vertical Cartesian path completed without reported collision or joint-limit violation in the loaded static scene. This demonstrates that, for this model and solver setup, keeping the failed target's X/Y and 150 mm vertical separation does not inherently prevent an IK solution when orientation changes.

The experiment does not show that the original pose was physically unreachable, nor that the World X −15° pose is safe or executable on the live robot. IK success, Cartesian interpolation completion, collision checking in a static scene, and physical pick success are separate observations.

## What we learned

An IK failure should be reported as a solver result for a specific target, orientation, seed/model, and planning configuration. It is not by itself evidence of geometric unreachability. Separating grasp IK, approach IK, Cartesian path completion, state validity, and physical grasp outcome prevents one stage's result from being mistaken for another's.

## References

- Live execution records: `local-artifact://arm_cell_pick_execution/aea3bf63c6c7eb1692ae301652a18fe0.json` (first attempt) and `local-artifact://arm_cell_pick_execution/eefc4bd9e48d637948e7cffe10ee1a81.json` (failed second attempt). The local artifacts are not included in the Public repository.
- Associated Vision acquisition identifiers: `0d3318f4-434c265d-b13d8b3e-acq-1` and `5b6527de-83b7-a823-250dee87-acq-2`; the latter's snapshot was inspected during diagnosis.
- ROS runtime logs for the two attempts correspond to 05:35 and 05:44 KST on 2026-10-13; the local logs are not included in the Public repository.
- Offline planning setup: `ros2_ws/src/arm_cell/arm_cell_bringup/config/isaac_planning.yaml` and `ros2_ws/src/arm_cell/arm_cell_moveit_config/config/m0609_pnp_static_scene.json`.
