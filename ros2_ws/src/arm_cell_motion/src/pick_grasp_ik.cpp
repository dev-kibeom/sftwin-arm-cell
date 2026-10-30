#include "arm_cell_motion/pick_grasp_ik.hpp"

namespace arm_cell_motion
{
PickGraspIkResult solve_pick_grasp_ik(
  const kinematics::KinematicsBasePtr & solver,
  const geometry_msgs::msg::Pose & goal,
  const std::vector<double> & seed,
  double timeout_s,
  const kinematics::KinematicsBase::IKCallbackFn & validate_solution,
  int initial_error_code)
{
  PickGraspIkResult result;
  result.solver_available = static_cast<bool>(solver);
  result.error.val = initial_error_code;
  if (solver) {
    result.solver_returned = solver->searchPositionIK(
      goal, seed, timeout_s, result.joint_positions, validate_solution, result.error);
  }
  return result;
}
}  // namespace arm_cell_motion
