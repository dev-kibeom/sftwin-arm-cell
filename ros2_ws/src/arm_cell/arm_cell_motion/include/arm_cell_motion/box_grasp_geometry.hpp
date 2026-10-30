#pragma once

#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <cmath>
#include <optional>

namespace arm_cell_motion
{

enum class BoxFaceEvidence
{
  INFERRED_SIDE_PAIR,
  OBSERVED_TOP_INFERRED_BOTTOM
};

struct BoxGraspContact
{
  std::size_t closing_axis{0};
  double width_mm{0.0};
  BoxFaceEvidence face_evidence{BoxFaceEvidence::INFERRED_SIDE_PAIR};
};

// The jaw closing axis is TCP +X. Opposing planar faces require it to be
// parallel to a box axis. This tolerance covers numerical FK/IK residual only;
// it is not a process tilt allowance. Rotation about the closing axis is free.
inline std::optional<BoxGraspContact> box_grasp_contact(
  const Eigen::Quaterniond & object_orientation,
  const Eigen::Quaterniond & tcp_orientation,
  double long_side_m, double short_side_m, double height_m,
  double policy_min_mm, double policy_max_mm, double gripper_max_mm = 85.0)
{
  if (!object_orientation.coeffs().allFinite() ||
    !tcp_orientation.coeffs().allFinite() ||
    object_orientation.norm() < 1e-9 || tcp_orientation.norm() < 1e-9 ||
    !std::isfinite(long_side_m) || !std::isfinite(short_side_m) ||
    !std::isfinite(height_m) || long_side_m < short_side_m ||
    short_side_m <= 0.0 || height_m <= 0.0 ||
    !std::isfinite(policy_min_mm) || !std::isfinite(policy_max_mm) ||
    policy_min_mm <= 0.0 || policy_max_mm < policy_min_mm ||
    !std::isfinite(gripper_max_mm) || gripper_max_mm <= 0.0)
  {
    return std::nullopt;
  }
  const auto closing_in_object = object_orientation.normalized().conjugate() *
    (tcp_orientation.normalized() * Eigen::Vector3d::UnitX());
  const double kNumericalAxisResidual = std::sin(3.14159265358979323846 / 180.0);
  const std::array<double, 3> lengths{long_side_m, short_side_m, height_m};
  for (std::size_t axis = 0; axis < lengths.size(); ++axis) {
    if ((closing_in_object.cross(Eigen::Vector3d::Unit(axis))).norm() >
      kNumericalAxisResidual)
    {
      continue;
    }
    const double width_mm = lengths[axis] * 1000.0;
    if (width_mm < policy_min_mm || width_mm > policy_max_mm || width_mm > gripper_max_mm) {
      return std::nullopt;
    }
    return BoxGraspContact{
      axis, width_mm,
      axis == 2 ? BoxFaceEvidence::OBSERVED_TOP_INFERRED_BOTTOM :
      BoxFaceEvidence::INFERRED_SIDE_PAIR};
  }
  return std::nullopt;
}

}  // namespace arm_cell_motion
