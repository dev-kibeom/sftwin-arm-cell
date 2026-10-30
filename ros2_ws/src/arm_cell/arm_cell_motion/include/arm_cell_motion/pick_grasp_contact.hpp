#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <optional>
#include <string>
#include "arm_cell_motion/box_grasp_geometry.hpp"

namespace arm_cell_motion
{
enum class PickGraspContactCheck {NOT_REQUIRED, VALID, MISMATCH};

struct PickGraspContactValidation
{
  PickGraspContactCheck check{PickGraspContactCheck::NOT_REQUIRED};
  std::optional<BoxGraspContact> contact;
  std::string reason;
};

// Applies the selected candidate's contact policy to one already-validated orientation.
PickGraspContactValidation validate_pick_grasp_contact(
  const Eigen::Quaterniond & actual_grasp_orientation,
  const std::optional<std::size_t> & expected_closing_axis,
  const Eigen::Quaterniond & object_orientation,
  double rectangle_long_side_m, double rectangle_short_side_m, double object_height_m,
  double grasp_width_min_mm, double grasp_width_max_mm, double gripper_max_width_mm);
}  // namespace arm_cell_motion
