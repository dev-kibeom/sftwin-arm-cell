#pragma once

#include <Eigen/Geometry>
#include <array>
#include <vector>

#include "arm_cell_motion/box_grasp_geometry.hpp"

namespace arm_cell_motion
{
struct FingerClearanceParameters
{
  double radius_m{0.010};
  double contact_span_m{0.020};
  double depth_m{0.025};
};

void validate_finger_clearance_parameters(const FingerClearanceParameters & parameters);

// Local X: face outward normal, Y: contact span, Z: insertion direction.
// x >= 0, distance((x,y), {0} x [-span/2,span/2]) <= radius,
// -depth <= z <= 0. Thus no protection extends into the target material.
struct FingerProtectionVolume
{
  Eigen::Isometry3d frame{Eigen::Isometry3d::Identity()};
  FingerClearanceParameters dimensions;
};

// Indices 0/1 identify the TCP -X/+X finger, respectively.
std::array<FingerProtectionVolume, 2> make_finger_protection_volumes(
  const Eigen::Isometry3d & tcp, const Eigen::Vector3d & object_center,
  const Eigen::Quaterniond & object_orientation, const BoxGraspContact & contact,
  const Eigen::Vector3d & insertion_axis_tcp, const FingerClearanceParameters & parameters);

// Exact rounded cross-section versus an oriented box; boundary-only contact is allowed.
bool finger_protection_intersects_box(
  const FingerProtectionVolume & volume, const Eigen::Isometry3d & box_pose,
  const Eigen::Vector3d & box_dimensions);
bool finger_protection_intersects_convex_top(
  const FingerProtectionVolume & volume, const Eigen::Isometry3d & pose,
  const std::vector<Eigen::Vector3d> & top, double bottom);
}  // namespace arm_cell_motion
