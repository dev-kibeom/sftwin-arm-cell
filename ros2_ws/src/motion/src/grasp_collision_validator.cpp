#include "arm_cell_motion/grasp_collision_validator.hpp"
#include <stdexcept>
#include <geometric_shapes/shapes.h>

namespace arm_cell_motion
{
GraspOccupancyMode parse_grasp_occupancy_mode(const std::string & value)
{
  if (value == "NONE") {return GraspOccupancyMode::NONE;}
  if (value == "FULL_COLLISION") {return GraspOccupancyMode::FULL_COLLISION;}
  if (value == "FINGER_CLEARANCE") {return GraspOccupancyMode::FINGER_CLEARANCE;}
  throw std::invalid_argument(
          "grasp_occupancy_mode must be NONE, FULL_COLLISION or FINGER_CLEARANCE");
}
const char * grasp_occupancy_mode_name(GraspOccupancyMode mode)
{
  switch (mode) {
    case GraspOccupancyMode::NONE: return "NONE";
    case GraspOccupancyMode::FULL_COLLISION: return "FULL_COLLISION";
    case GraspOccupancyMode::FINGER_CLEARANCE: return "FINGER_CLEARANCE";
  }
  throw std::invalid_argument("invalid grasp occupancy mode");
}

GraspCollisionValidationResult validate_grasp_collision(
  GraspOccupancyMode mode, const planning_scene::PlanningScene & common_scene,
  const planning_scene::PlanningScene & full_scene, const moveit::core::RobotState & state,
  const Eigen::Isometry3d & tcp, const Eigen::Vector3d & object_center,
  const Eigen::Quaterniond & object_orientation, const std::optional<BoxGraspContact> & contact,
  const Eigen::Vector3d & insertion_axis_tcp,
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & occupancy,
  const FingerClearanceParameters & parameters, const std::string & planning_group,
  const std::optional<Eigen::Vector3d> & target_dimensions)
{
  GraspCollisionValidationResult result;
  result.mode = mode;
  result.evaluated = true;
  moveit::core::RobotState grasp_state(state);
  const auto copy_scene = [](const planning_scene::PlanningScene & source) {
      auto copy = std::make_shared<planning_scene::PlanningScene>(source.getRobotModel());
      moveit_msgs::msg::PlanningScene snapshot;
      source.getPlanningSceneMsg(snapshot);
      copy->setPlanningSceneMsg(snapshot);
      // Message serialization cannot carry conditional contact callbacks.
      copy->getAllowedCollisionMatrixNonConst() = source.getAllowedCollisionMatrix();
      return copy;
    };
  auto common = copy_scene(common_scene);
  auto full = copy_scene(full_scene);
  // The fixed robot base is mounted on this riser. Whole-robot endpoint
  // checks include that installation contact; only this exact pair is exempt.
  // Apply to private copies so shared scene policy and other contacts survive.
  for (const auto & scene : {common, full}) {
    if (state.getRobotModel()->hasLinkModel("base_link") &&
      scene->getWorld()->hasObject("robot_riser"))
    {
      scene->getAllowedCollisionMatrixNonConst().setEntry("base_link", "robot_riser", true);
    }
  }
  std::string collision_group = planning_group;
  if (target_dimensions) {
    constexpr const char * joint_name = "gripper_robotiq_85_left_knuckle_joint";
    const auto * joint = state.getRobotModel()->getJointModel(joint_name);
    if (!contact || !std::isfinite(contact->width_mm) || contact->width_mm <= 0.0 ||
      contact->width_mm > 85.0 || !object_center.allFinite() ||
      !object_orientation.coeffs().allFinite() || object_orientation.norm() < 1e-9 ||
      !target_dimensions->allFinite() ||
      (target_dimensions->array() <= 0.0).any() || !joint || joint->getVariableCount() != 1U)
    {
      result.valid = false;
      result.reason = "CLOSED_GRASP_GEOMETRY_UNAVAILABLE";
      return result;
    }
    // Same normalized width-to-actuator mapping as the Tool runtime. Setting
    // the driving joint also updates every URDF mimic joint in this copy.
    const auto & bounds = joint->getVariableBounds().front();
    if (!bounds.position_bounded_ || !std::isfinite(bounds.min_position_) ||
      !std::isfinite(bounds.max_position_) || bounds.max_position_ <= bounds.min_position_)
    {
      result.valid = false;
      result.reason = "CLOSED_GRASP_GEOMETRY_UNAVAILABLE";
      return result;
    }
    const double position = bounds.min_position_ +
      (1.0 - contact->width_mm / 85.0) * (bounds.max_position_ - bounds.min_position_);
    grasp_state.setVariablePosition(joint_name, position);
    grasp_state.update();
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = object_center;
    target.linear() = object_orientation.normalized().toRotationMatrix();
    if (!target.matrix().allFinite()) {
      result.valid = false;
      result.reason = "CLOSED_GRASP_GEOMETRY_UNAVAILABLE";
      return result;
    }
    constexpr const char * target_id = "pick_selected_target";
    for (const auto & scene : {common, full}) {
      scene->getWorldNonConst()->removeObject(target_id);
      scene->getWorldNonConst()->addToObject(
        target_id,
        shapes::ShapeConstPtr(
          new shapes::Box(
            target_dimensions->x(), target_dimensions->y(), target_dimensions->z())), target);
      auto & acm = scene->getAllowedCollisionMatrixNonConst();
      // Only the intended finger surfaces may touch the target. Palm and
      // linkage remain collision geometry, independently of occupancy mode.
      for (const auto * link : state.getRobotModel()->getLinkModelsWithCollisionGeometry()) {
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
    // Gripper joints need not belong to the arm planning group. Include the
    // whole robot at this endpoint rather than silently omitting its fingers.
    collision_group.clear();
  }
  result.common = validate_pick_state_collision(*common, grasp_state, collision_group);
  if (mode == GraspOccupancyMode::FULL_COLLISION) {
    result.occupancy_evaluated = true;
    result.occupancy = validate_pick_state_collision(*full, grasp_state, collision_group);
    result.allowed = !result.common.collision && !result.occupancy.collision;
    result.reason = result.common.collision ? "COMMON_COLLISION" :
      result.allowed ? "NONE" : "OCCUPANCY_COLLISION";
    return result;
  }
  if (result.common.collision) {
    result.reason = "COMMON_COLLISION";
    return result;
  }
  if (mode == GraspOccupancyMode::NONE) {
    result.allowed = true;
    result.reason = "NONE";
    return result;
  }
  auto finger_result = evaluate_finger_occupancy(
    tcp, object_center, object_orientation, contact, insertion_axis_tcp, occupancy, parameters);
  finger_result.common = result.common;
  return finger_result;
}

GraspCollisionValidationResult evaluate_finger_occupancy(
  const Eigen::Isometry3d & tcp, const Eigen::Vector3d & object_center,
  const Eigen::Quaterniond & object_orientation, const std::optional<BoxGraspContact> & contact,
  const Eigen::Vector3d & insertion_axis_tcp,
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & occupancy,
  const FingerClearanceParameters & parameters)
{
  GraspCollisionValidationResult result;
  result.mode = GraspOccupancyMode::FINGER_CLEARANCE;
  result.evaluated = true;
  try {
    if (!contact) {throw std::invalid_argument("FINGER_CONTACT_UNAVAILABLE");}
    result.finger_volumes = make_finger_protection_volumes(
      tcp, object_center, object_orientation, *contact, insertion_axis_tcp, parameters);
    for (const auto & primitive : occupancy) {
      if (primitive.shape_type != arm_cell_interfaces::msg::CollisionPrimitive::CONVEX_TOP &&
        (primitive.shape_type != arm_cell_interfaces::msg::CollisionPrimitive::SOLID_PRIMITIVE ||
        primitive.shape.type != shape_msgs::msg::SolidPrimitive::BOX ||
        primitive.shape.dimensions.size() != 3U))
      {
        throw std::invalid_argument("FINGER_OCCUPANCY_REQUIRES_BOX");
      }
    }
    result.occupancy_evaluated = true;
    for (std::size_t i = 0; i < occupancy.size(); ++i) {
      const auto & primitive = occupancy[i];
      Eigen::Isometry3d box = Eigen::Isometry3d::Identity();
      const auto & p = primitive.pose.position;
      const auto & q = primitive.pose.orientation;
      box.translation() = Eigen::Vector3d(p.x, p.y, p.z);
      box.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
      std::vector<Eigen::Vector3d> top;
      if (primitive.shape_type == arm_cell_interfaces::msg::CollisionPrimitive::CONVEX_TOP) {
        for (const auto & vertex : primitive.top_vertices) {
          top.emplace_back(vertex.x, vertex.y, vertex.z);
        }
      }
      for (std::size_t side = 0; side < result.finger_volumes->size(); ++side) {
        const bool intersects = primitive.shape_type ==
          arm_cell_interfaces::msg::CollisionPrimitive::CONVEX_TOP ?
          finger_protection_intersects_convex_top(
          (*result.finger_volumes)[side], box, top, primitive.bottom_height_m) :
          finger_protection_intersects_box(
          (*result.finger_volumes)[side], box,
          Eigen::Vector3d(
            primitive.shape.dimensions[0], primitive.shape.dimensions[1],
            primitive.shape.dimensions[2]));
        if (intersects) {
          result.intrusions.push_back(
            {side, std::string(kCandidateOccupancyIdPrefix) +
              std::to_string(i)});
        }
      }
    }
    result.occupancy.collision = !result.intrusions.empty();
    result.allowed = !result.occupancy.collision;
    result.reason = result.allowed ? "NONE" : "FINGER_CLEARANCE_INTRUSION";
  } catch (const std::invalid_argument & error) {
    result.valid = false;
    result.reason = error.what();
  }
  return result;
}
}  // namespace arm_cell_motion
