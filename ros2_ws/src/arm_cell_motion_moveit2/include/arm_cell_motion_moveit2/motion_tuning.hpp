#pragma once

namespace arm_cell_motion_moveit2
{

// Planning and execution tolerances captured once for each Motion action.
struct MotionTuning
{
  double planning_time_s{5.0};
  double place_candidate_planning_time_s{1.0};
  double place_total_planning_time_s{5.0};
  double planning_velocity_scaling_factor{0.1};
  double planning_acceleration_scaling_factor{0.1};
  double motion_progress_stall_timeout_s{60.0};
  double approach_entry_position_tolerance_m{0.005};
  double approach_entry_orientation_tolerance_rad{0.08726646259971647};
  double place_approach_entry_position_tolerance_m{0.005};
  double place_tracking_tolerance_rad{0.01};
};

}  // namespace arm_cell_motion_moveit2
