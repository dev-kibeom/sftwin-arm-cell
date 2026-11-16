#include "arm_cell_motion/box_grasp_collision_policy.hpp"
#include <cmath>
#include <geometric_shapes/shapes.h>

namespace arm_cell_motion
{
bool prepare_closed_box_grasp_collision(
  moveit::core::RobotState & grasp_state,
  planning_scene::PlanningScene & common_scene, planning_scene::PlanningScene & full_scene,
  const Eigen::Vector3d & object_center, const Eigen::Quaterniond & object_orientation,
  const std::optional<BoxGraspContact> & contact, const Eigen::Vector3d & target_dimensions)
{
  constexpr const char * joint_name = "gripper_robotiq_85_left_knuckle_joint";
  const auto * joint = grasp_state.getRobotModel()->getJointModel(joint_name);
  if (!contact || !std::isfinite(contact->width_mm) || contact->width_mm <= 0.0 ||
    contact->width_mm > 85.0 || !object_center.allFinite() ||
    !object_orientation.coeffs().allFinite() || object_orientation.norm() < 1e-9 ||
    !target_dimensions.allFinite() ||
    (target_dimensions.array() <= 0.0).any() || !joint || joint->getVariableCount() != 1U)
  {
    return false;
  }
  // Same normalized width-to-actuator mapping as the Tool runtime. Setting
  // the driving joint also updates every URDF mimic joint in this copy.
  const auto & bounds = joint->getVariableBounds().front();
  if (!bounds.position_bounded_ || !std::isfinite(bounds.min_position_) ||
    !std::isfinite(bounds.max_position_) || bounds.max_position_ <= bounds.min_position_)
  {
    return false;
  }
  const double position = bounds.min_position_ +
    (1.0 - contact->width_mm / 85.0) * (bounds.max_position_ - bounds.min_position_);
  grasp_state.setVariablePosition(joint_name, position);
  grasp_state.update();
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = object_center;
  target.linear() = object_orientation.normalized().toRotationMatrix();
  if (!target.matrix().allFinite()) {
    return false;
  }
  constexpr const char * target_id = "pick_selected_target";
  for (auto * scene : {&common_scene, &full_scene}) {
    scene->getWorldNonConst()->removeObject(target_id);
    scene->getWorldNonConst()->addToObject(
      target_id,
      shapes::ShapeConstPtr(
        new shapes::Box(
          target_dimensions.x(), target_dimensions.y(), target_dimensions.z())), target);
    auto & acm = scene->getAllowedCollisionMatrixNonConst();
    // Only the intended finger surfaces may touch the target. Palm and
    // linkage remain collision geometry, independently of occupancy mode.
    for (const auto * link : grasp_state.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
      acm.setEntry(target_id, link->getName(), false);
    }
    for (const auto & side : {std::string("left"), std::string("right")}) {
      const auto prefix = "gripper_robotiq_85_" + side;
      acm.setEntry(target_id, prefix + "_finger_link", true);
      acm.setEntry(target_id, prefix + "_finger_tip_link", true);
      collision_detection::DecideContactFn floor_contact =
        [](collision_detection::Contact & c) {
          return std::isfinite(c.depth) && c.depth <= 0.0005 &&
                 std::abs(c.normal.z()) >= 0.99;
        };
      acm.setEntry(prefix + "_finger_tip_link", "amr_tray", floor_contact);
    }
  }
  return true;
}
}  // namespace arm_cell_motion
