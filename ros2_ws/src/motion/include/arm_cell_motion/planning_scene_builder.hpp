#pragma once

#include <algorithm>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <moveit_msgs/msg/planning_scene.hpp>

#include "arm_cell_motion/candidate_occupancy.hpp"
#include "arm_cell_motion/pick_retract_collision_policy.hpp"
#include "arm_cell_motion/planning_scene_policy.hpp"

namespace arm_cell_motion
{

struct PlanningSceneDiffResult
{
  bool valid{false};
  std::string failure_reason;
  moveit_msgs::msg::PlanningScene diff;
  std::vector<std::string> candidate_ids;
};

// Builds a request-local diff from one immutable scene snapshot. Vision and
// tray exception are independent policy inputs and never mutate the monitor.
class PlanningSceneBuilder
{
public:
  PlanningSceneBuilder(
    moveit_msgs::msg::PlanningScene snapshot,
    std::vector<arm_cell_interfaces::msg::CollisionPrimitive> occupancy,
    std_msgs::msg::Header observation_header)
  : snapshot_(std::move(snapshot)), occupancy_(std::move(occupancy)),
    observation_header_(std::move(observation_header)),
    previous_candidate_ids_(candidate_occupancy_ids(snapshot_))
  {}

  PlanningSceneDiffResult build(const PlanningScenePolicy & policy) const
  {
    PlanningSceneDiffResult result;
    result.diff.is_diff = true;
    result.diff.robot_state.is_diff = true;
    if (previous_candidate_ids_.size() != std::set<std::string>(
        previous_candidate_ids_.begin(), previous_candidate_ids_.end()).size())
    {
      result.failure_reason = "VISION_OCCUPANCY_IDENTITY_AMBIGUOUS";
      return result;
    }

    result.diff = candidate_scene_diff(
      previous_candidate_ids_, occupancy_, observation_header_,
      policy.vision_occupancy, result.candidate_ids);
    if (policy.apply_pick_retract_tray_exception) {
      const bool tray_present = std::any_of(
        snapshot_.world.collision_objects.begin(), snapshot_.world.collision_objects.end(),
        [](const auto & object) {return object.id == kPickRetractTrayCollisionObject;});
      const auto allowed = make_pick_retract_allowed_collision_matrix(
        snapshot_.allowed_collision_matrix);
      if (!tray_present || !allowed) {
        result.failure_reason = tray_present ? "ACM_INVALID" : "TRAY_NOT_FOUND";
        return result;
      }
      result.diff.allowed_collision_matrix = *allowed;
    }
    result.valid = true;
    return result;
  }

  const moveit_msgs::msg::PlanningScene & snapshot() const {return snapshot_;}

private:
  const moveit_msgs::msg::PlanningScene snapshot_;
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> occupancy_;
  const std_msgs::msg::Header observation_header_;
  const std::vector<std::string> previous_candidate_ids_;
};

}  // namespace arm_cell_motion
