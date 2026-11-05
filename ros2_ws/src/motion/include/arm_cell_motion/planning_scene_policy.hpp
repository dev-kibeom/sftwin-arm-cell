#pragma once

#include "arm_cell_motion/candidate_occupancy.hpp"

namespace arm_cell_motion
{

// Independent scene inputs for one validation or planning request. Consumers
// choose a policy explicitly; Scene Builder code never infers it from stage.
struct PlanningScenePolicy
{
  VisionOccupancyPolicy vision_occupancy{VisionOccupancyPolicy::EXCLUDE};
  bool apply_pick_retract_tray_exception{false};

  friend bool operator==(const PlanningScenePolicy & lhs, const PlanningScenePolicy & rhs)
  {
    return lhs.vision_occupancy == rhs.vision_occupancy &&
           lhs.apply_pick_retract_tray_exception == rhs.apply_pick_retract_tray_exception;
  }
};

}  // namespace arm_cell_motion
