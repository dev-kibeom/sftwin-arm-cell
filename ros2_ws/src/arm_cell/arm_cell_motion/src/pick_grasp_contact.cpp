#include "arm_cell_motion/pick_grasp_contact.hpp"

namespace arm_cell_motion
{
PickGraspContactValidation validate_pick_grasp_contact(
  const Eigen::Quaterniond & actual_grasp_orientation,
  const std::optional<std::size_t> & expected_closing_axis,
  const Eigen::Quaterniond & object_orientation,
  double rectangle_long_side_m, double rectangle_short_side_m, double object_height_m,
  double grasp_width_min_mm, double grasp_width_max_mm, double gripper_max_width_mm)
{
  PickGraspContactValidation result;
  if (!expected_closing_axis) {
    return result;
  }
  result.contact = box_grasp_contact(
    object_orientation, actual_grasp_orientation, rectangle_long_side_m,
    rectangle_short_side_m, object_height_m, grasp_width_min_mm,
    grasp_width_max_mm, gripper_max_width_mm);
  if (!result.contact || result.contact->closing_axis != *expected_closing_axis) {
    result.check = PickGraspContactCheck::MISMATCH;
    result.reason = "grasp_contact_mismatch";
    return result;
  }
  result.check = PickGraspContactCheck::VALID;
  return result;
}
}  // namespace arm_cell_motion
