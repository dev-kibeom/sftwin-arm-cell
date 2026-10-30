#include "arm_cell_motion/pick_grasp_kinematics.hpp"
#include "pick_kinematics_validation_detail.hpp"

#include <algorithm>
#include <cmath>

namespace arm_cell_motion
{
namespace
{
bool finite_joints(const std::vector<double> & positions)
{
  return std::all_of(
    positions.begin(), positions.end(), [](double value) {
      return std::isfinite(value);
    });
}
}  // namespace

PickGraspKinematics validate_pick_grasp_kinematics(
  const moveit::core::RobotState & current_state,
  const moveit::core::JointModelGroup * joint_group,
  const std::vector<double> & positions,
  std::size_t expected_joint_count,
  const geometry_msgs::msg::PoseStamped & grasp_goal,
  bool validate_orientation)
{
  PickGraspKinematics result{PickGraspKinematicCheck::BACKEND_OR_INVALID,
    moveit::core::RobotState(current_state), Eigen::Quaterniond::Identity(), {}};
  if (!joint_group || positions.size() != expected_joint_count || !finite_joints(positions)) {
    result.check = PickGraspKinematicCheck::JOINT_BOUNDS;
    result.reason = "grasp_joint_bounds";
    return result;
  }
  result.state.setJointGroupPositions(joint_group, positions);
  result.state.update();
  if (!result.state.satisfiesBounds(joint_group)) {
    result.check = PickGraspKinematicCheck::JOINT_BOUNDS;
    result.reason = "joint_bounds";
    return result;
  }
  const auto & fk = result.state.getGlobalLinkTransform("sf_grasp_tcp");
  const Eigen::Quaterniond expected_orientation(
    grasp_goal.pose.orientation.w, grasp_goal.pose.orientation.x,
    grasp_goal.pose.orientation.y, grasp_goal.pose.orientation.z);
  result.actual_orientation = Eigen::Quaterniond(fk.rotation());
  if (!detail::matches_pose(fk, grasp_goal, expected_orientation, validate_orientation)) {
    result.check = PickGraspKinematicCheck::FK_TARGET_MISMATCH;
    result.reason = "grasp_fk_target_mismatch";
    return result;
  }
  result.check = PickGraspKinematicCheck::VALID;
  result.reason.clear();
  return result;
}
}  // namespace arm_cell_motion
