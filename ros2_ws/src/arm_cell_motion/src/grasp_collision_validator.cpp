#include "arm_cell_motion/grasp_collision_validator.hpp"
#include <stdexcept>

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
  const FingerClearanceParameters & parameters)
{
  GraspCollisionValidationResult result;
  result.mode = mode;
  result.evaluated = true;
  result.common = validate_pick_state_collision(common_scene, state);
  if (mode == GraspOccupancyMode::FULL_COLLISION) {
    result.occupancy_evaluated = true;
    result.occupancy = validate_pick_state_collision(full_scene, state);
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
          finger_protection_intersects_box((*result.finger_volumes)[side], box,
          Eigen::Vector3d(primitive.shape.dimensions[0], primitive.shape.dimensions[1],
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
