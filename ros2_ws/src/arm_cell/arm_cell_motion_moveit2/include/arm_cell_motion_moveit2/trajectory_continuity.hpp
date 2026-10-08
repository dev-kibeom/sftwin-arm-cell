#pragma once

#include <functional>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_cell_motion_moveit2
{

struct JointContinuityLimit
{
  double lower{0.0};
  double upper{0.0};
  bool wrap_equivalent{false};
};

struct JointTrajectoryContinuityResult
{
  bool valid{false};
  bool normalized{false};
  std::string reason;
  std::vector<double> raw_endpoint_delta;
  std::vector<double> wrap_aware_endpoint_delta;
  std::vector<bool> normalized_joints;
  std::vector<double> raw_cumulative_motion;
};

struct JointGoalSelectionResult
{
  bool valid{false};
  bool normalized{false};
  bool unnecessary_full_turn{false};
  std::string reason;
  std::vector<double> raw_goal;
  std::vector<double> selected_goal;
  std::vector<double> raw_endpoint_delta;
  std::vector<double> wrap_aware_endpoint_delta;
  std::vector<double> selected_endpoint_delta;
  std::vector<double> selected_wrap_aware_endpoint_delta;
};

using JointGoalStateValidator = std::function<bool (
      const std::vector<double> &, std::string &)>;

JointGoalSelectionResult select_nearest_joint_goal(
  const std::vector<double> & raw_goal,
  const std::vector<double> & measured_start,
  const std::vector<JointContinuityLimit> & limits,
  const JointGoalStateValidator & state_validator = {});

JointTrajectoryContinuityResult normalize_joint_trajectory(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & measured_start,
  const std::vector<JointContinuityLimit> & limits,
  bool allow_representation_normalization = true);

}  // namespace arm_cell_motion_moveit2
