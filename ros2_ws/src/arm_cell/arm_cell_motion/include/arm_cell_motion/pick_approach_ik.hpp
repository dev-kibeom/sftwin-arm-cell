#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>

namespace arm_cell_motion
{
enum class PickApproachIkCheck {VALID, APPROACH_IK_FAILED, JOINT_BOUNDS, FK_TARGET_MISMATCH,
  BACKEND_OR_INVALID};

struct PickApproachIkResult
{
  PickApproachIkCheck check{PickApproachIkCheck::BACKEND_OR_INVALID};
  std::vector<double> joint_positions;
  std::string reason;
};

// Runs one Approach IK from the supplied state and validates bounds and target FK.
PickApproachIkResult solve_pick_approach_ik(
  const moveit::core::RobotState & grasp_state,
  const moveit::core::JointModelGroup * joint_group,
  const geometry_msgs::msg::PoseStamped & approach_goal,
  const Eigen::Quaterniond & expected_orientation,
  double timeout_s);
}  // namespace arm_cell_motion
