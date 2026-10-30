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
enum class PickGraspKinematicCheck {VALID, JOINT_BOUNDS, FK_TARGET_MISMATCH, BACKEND_OR_INVALID};

struct PickGraspKinematics
{
  PickGraspKinematicCheck check{PickGraspKinematicCheck::BACKEND_OR_INVALID};
  moveit::core::RobotState state;
  Eigen::Quaterniond actual_orientation{Eigen::Quaterniond::Identity()};
  std::string reason;
};

// Checks one selected Grasp IK result for bounds and target FK.
PickGraspKinematics validate_pick_grasp_kinematics(
  const moveit::core::RobotState & current_state,
  const moveit::core::JointModelGroup * joint_group,
  const std::vector<double> & positions,
  std::size_t expected_joint_count,
  const geometry_msgs::msg::PoseStamped & grasp_goal,
  bool validate_orientation);
}  // namespace arm_cell_motion
