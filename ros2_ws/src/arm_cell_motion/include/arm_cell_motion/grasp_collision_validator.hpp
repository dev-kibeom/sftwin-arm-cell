#pragma once

#include <optional>
#include <string>
#include <vector>
#include "arm_cell_motion/candidate_occupancy.hpp"
#include "arm_cell_motion/finger_clearance.hpp"
#include "arm_cell_motion/pick_state_collision.hpp"

namespace arm_cell_motion
{
enum class GraspOccupancyMode {NONE, FULL_COLLISION, FINGER_CLEARANCE};
GraspOccupancyMode parse_grasp_occupancy_mode(const std::string & value);
const char * grasp_occupancy_mode_name(GraspOccupancyMode mode);

struct FingerOccupancyIntrusion
{
  std::size_t finger_index;
  std::string occupancy_id;
};
struct GraspCollisionValidationResult
{
  GraspOccupancyMode mode{GraspOccupancyMode::FULL_COLLISION};
  bool evaluated{false};
  bool valid{true};
  PickCollisionValidationResult common;
  bool occupancy_evaluated{false};
  PickCollisionValidationResult occupancy;
  std::optional<std::array<FingerProtectionVolume, 2>> finger_volumes;
  std::vector<FingerOccupancyIntrusion> intrusions;
  bool allowed{false};
  std::string reason{"NOT_RUN"};
};

// The leaf supplies immutable request scenes and already-validated contact
// geometry. Strategy selection/calculation stay here, independent of BT search.
GraspCollisionValidationResult validate_grasp_collision(
  GraspOccupancyMode mode, const planning_scene::PlanningScene & common_scene,
  const planning_scene::PlanningScene & full_scene, const moveit::core::RobotState & state,
  const Eigen::Isometry3d & tcp, const Eigen::Vector3d & object_center,
  const Eigen::Quaterniond & object_orientation, const std::optional<BoxGraspContact> & contact,
  const Eigen::Vector3d & insertion_axis_tcp,
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & occupancy,
  const FingerClearanceParameters & parameters);
}  // namespace arm_cell_motion
