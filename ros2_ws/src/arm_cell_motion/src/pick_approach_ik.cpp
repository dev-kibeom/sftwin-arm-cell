#include "arm_cell_motion/pick_approach_ik.hpp"
#include "pick_kinematics_validation_detail.hpp"

#include <cmath>

namespace arm_cell_motion
{
PickApproachIkResult solve_pick_approach_ik(
  const moveit::core::RobotState & grasp_state,
  const moveit::core::JointModelGroup * joint_group,
  const geometry_msgs::msg::PoseStamped & approach_goal,
  const Eigen::Quaterniond & expected_orientation,
  double timeout_s)
{
  PickApproachIkResult result;
  if (!joint_group || timeout_s <= 0.0 || !std::isfinite(timeout_s)) {
    result.reason = "approach_ik_failed";
    return result;
  }
  moveit::core::RobotState approach_state(grasp_state);
  if (!approach_state.setFromIK(
      joint_group, approach_goal.pose, "sf_grasp_tcp", timeout_s))
  {
    result.check = PickApproachIkCheck::APPROACH_IK_FAILED;
    result.reason = "approach_ik_failed";
    return result;
  }
  if (!approach_state.satisfiesBounds(joint_group)) {
    result.check = PickApproachIkCheck::JOINT_BOUNDS;
    result.reason = "approach_joint_bounds";
    return result;
  }
  approach_state.update();
  const auto & fk = approach_state.getGlobalLinkTransform("sf_grasp_tcp");
  if (!detail::matches_pose(fk, approach_goal, expected_orientation, true)) {
    result.check = PickApproachIkCheck::FK_TARGET_MISMATCH;
    result.reason = "approach_fk_target_mismatch";
    return result;
  }
  approach_state.copyJointGroupPositions(joint_group, result.joint_positions);
  result.check = PickApproachIkCheck::VALID;
  result.reason.clear();
  return result;
}
}  // namespace arm_cell_motion
