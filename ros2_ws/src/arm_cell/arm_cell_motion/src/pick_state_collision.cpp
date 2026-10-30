#include "arm_cell_motion/pick_state_collision.hpp"

#include <moveit/collision_detection/collision_common.h>

namespace arm_cell_motion
{
PickCollisionValidationResult validate_pick_state_collision(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state)
{
  collision_detection::CollisionRequest request;
  request.group_name = "arm";
  request.contacts = true;
  request.max_contacts = 32;
  collision_detection::CollisionResult collision;
  scene.checkCollision(request, collision, state);
  PickCollisionValidationResult result;
  result.collision = collision.collision;
  for (const auto & contact_pair : collision.contacts) {
    result.contacts.emplace_back(contact_pair.first.first, contact_pair.first.second);
  }
  return result;
}
}  // namespace arm_cell_motion
