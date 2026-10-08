#pragma once

#include <vector>

#include "arm_cell_motion_moveit2/trajectory_sampler.hpp"

namespace arm_cell_motion_moveit2
{

class FinalTargetHold final
{
public:
  bool begin(const std::vector<double> & target_positions, double start_simulation_time);
  void cancel();

  bool active() const {return active_;}
  double elapsed(double simulation_time) const;
  TimedJointSample command(double simulation_time) const;

private:
  std::vector<double> target_positions_;
  double start_simulation_time_{0.0};
  bool active_{false};
};

}  // namespace arm_cell_motion_moveit2
