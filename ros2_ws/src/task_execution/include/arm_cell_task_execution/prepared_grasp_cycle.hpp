#pragma once

#include <string>
#include "arm_cell_motion/grasp_strategy.hpp"

namespace arm_cell_task_execution
{
// An explicitly prepared, taught work operation. Targets remain in the Motion
// registry; fixture creation and geometry solving stay with their producers.
// The common Pick BT owns grasp confirmation/recovery and held transfer.
struct PreparedGraspCycle
{
  arm_cell_motion::GraspSpecification grasp;
  std::string approach_motion;
  std::string insertion_motion;
  std::string transfer_motion;
  std::string retreat_motion;

  bool valid() const
  {
    return grasp.valid() && !approach_motion.empty() && !insertion_motion.empty() &&
           !transfer_motion.empty() && !retreat_motion.empty();
  }
};
}  // namespace arm_cell_task_execution
