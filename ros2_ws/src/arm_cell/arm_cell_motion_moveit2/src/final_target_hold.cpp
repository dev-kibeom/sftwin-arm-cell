#include "arm_cell_motion_moveit2/final_target_hold.hpp"

#include <algorithm>
#include <cmath>

namespace arm_cell_motion_moveit2
{

bool FinalTargetHold::begin(
  const std::vector<double> & target_positions, double start_simulation_time)
{
  if (target_positions.empty() || !std::isfinite(start_simulation_time) ||
    std::any_of(
      target_positions.begin(), target_positions.end(),
      [](double value) {return !std::isfinite(value);}))
  {
    return false;
  }
  target_positions_ = target_positions;
  start_simulation_time_ = start_simulation_time;
  active_ = true;
  return true;
}

void FinalTargetHold::cancel()
{
  active_ = false;
}

double FinalTargetHold::elapsed(double simulation_time) const
{
  return active_ && std::isfinite(simulation_time) ?
         std::max(0.0, simulation_time - start_simulation_time_) : 0.0;
}

TimedJointSample FinalTargetHold::command(double simulation_time) const
{
  TimedJointSample result;
  if (!active_) {
    return result;
  }
  result.positions = target_positions_;
  result.velocities.assign(target_positions_.size(), 0.0);
  result.trajectory_time = elapsed(simulation_time);
  return result;
}

}  // namespace arm_cell_motion_moveit2
