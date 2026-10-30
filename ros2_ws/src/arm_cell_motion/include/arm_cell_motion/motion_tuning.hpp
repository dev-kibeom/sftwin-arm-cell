#pragma once

namespace arm_cell_motion
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
  double cartesian_tcp_orientation_tolerance_rad{0.08726646259971647};
  double trajectory_goal_tolerance_rad{0.03};
  double disturbance_push_distance_m{0.02};
  double disturbance_approach_clearance_m{0.01};
  double disturbance_speed_fraction{0.2};
};

}  // namespace arm_cell_motion
