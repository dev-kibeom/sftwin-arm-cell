#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace arm_cell_motion
{

struct TcpPoseAccuracy
{
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

struct JointPositionError
{
  std::string name;
  double target_position_rad{0.0};
  double actual_position_rad{0.0};
  double position_error_rad{0.0};
};

struct JointPositionAccuracy
{
  std::vector<JointPositionError> joints;
  double maximum_position_error_rad{0.0};
};

inline std::optional<JointPositionAccuracy> measure_joint_position_accuracy(
  const std::vector<std::string> & joint_names,
  const std::vector<double> & target_positions,
  const std::vector<double> & actual_positions)
{
  if (joint_names.empty() || joint_names.size() != target_positions.size() ||
    joint_names.size() != actual_positions.size())
  {
    return std::nullopt;
  }
  JointPositionAccuracy result;
  result.joints.reserve(joint_names.size());
  for (std::size_t index = 0; index < joint_names.size(); ++index) {
    if (!std::isfinite(target_positions[index]) || !std::isfinite(actual_positions[index])) {
      return std::nullopt;
    }
    const double error = std::abs(target_positions[index] - actual_positions[index]);
    result.maximum_position_error_rad =
      std::max(result.maximum_position_error_rad, error);
    result.joints.push_back({
      joint_names[index], target_positions[index], actual_positions[index], error});
  }
  return result;
}

inline TcpPoseAccuracy measure_tcp_pose_accuracy(
  const Eigen::Isometry3d & target, const Eigen::Isometry3d & actual)
{
  const Eigen::Quaterniond target_orientation(target.rotation());
  const Eigen::Quaterniond actual_orientation(actual.rotation());
  const double orientation_error_rad = 2.0 * std::acos(std::clamp(
      std::abs(target_orientation.normalized().dot(actual_orientation.normalized())), 0.0, 1.0));
  return {(actual.translation() - target.translation()).norm(), orientation_error_rad};
}

}  // namespace arm_cell_motion
