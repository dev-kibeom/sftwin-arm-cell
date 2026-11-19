#include "arm_cell_motion/trajectory_generation.hpp"
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <cmath>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <unordered_map>

namespace arm_cell_motion
{
namespace
{
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
bool valid_target(const Eigen::Isometry3d & target)
{
  return target.matrix().allFinite() &&
         (target.linear().transpose() * target.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(target.linear().determinant() - 1.0) < 1e-6;
}
}

moveit::core::RobotModelPtr make_kinematic_model(
  const moveit::core::RobotModelConstPtr & source, const std::string & group,
  const kinematics::KinematicsBasePtr & solver)
{
  // MoveIt's allocator/timeout setters also modify the solver instance. The
  // caller may share it with another consumer, so retain that consumer's default.
  struct RestoreSolverTimeout
  {
    kinematics::KinematicsBasePtr solver;
    double timeout;
    ~RestoreSolverTimeout() {if (solver) {solver->setDefaultTimeout(timeout);}}
  } restore{solver, solver ? solver->getDefaultTimeout() : 0.0};
  auto model = std::make_shared<moveit::core::RobotModel>(source->getURDF(), source->getSRDF());
  for (const auto & name : source->getVariableNames()) {
    const auto * joint = source->getJointOfVariable(name);
    model->getJointModel(joint->getName())->setVariableBounds(name, source->getVariableBounds(name));
  }
  if (solver) {model->setKinematicsAllocators({{group, [solver](const auto *) {return solver;}}});}
  if (const auto * original_group = source->getJointModelGroup(group)) {
    model->getJointModelGroup(group)->setDefaultIKTimeout(original_group->getDefaultIKTimeout());
  }
  return model;
}

std::uint8_t resolve_pose_joint_target(
  const moveit::core::RobotState & start, const std::string & group,
  const std::string & tcp_link, const Eigen::Isometry3d & target,
  std::vector<double> & joints)
{
  joints.clear();
  const auto * jmg = start.getRobotModel()->getJointModelGroup(group);
  if (!jmg || !start.getRobotModel()->hasLinkModel(tcp_link) || !valid_target(target)) {
    return Code::TASK_RESULT_INVALID_GOAL;
  }
  if (!jmg->getSolverInstance()) {return Code::TASK_RESULT_IK_UNREACHABLE;}
  auto solution = start;
  const moveit::core::GroupStateValidityCallbackFn bounds =
    [](moveit::core::RobotState * state, const moveit::core::JointModelGroup * joints,
    const double * values) {
      state->setJointGroupPositions(joints, values);
      return state->satisfiesBounds(joints);
    };
  if (!solution.setFromIK(jmg, target, tcp_link, 0.2, bounds) || !solution.satisfiesBounds(jmg)) {
    return Code::TASK_RESULT_IK_UNREACHABLE;
  }
  for (const auto * joint : jmg->getActiveJointModels()) {
    if (joint->getType() != moveit::core::JointModel::REVOLUTE || joint->getVariableCount() != 1U) {continue;}
    const auto & name = joint->getVariableNames().front();
    solution.setVariablePosition(name, nearest_revolute_equivalent(
      solution.getVariablePosition(name), start.getVariablePosition(name),
      solution.getRobotModel()->getVariableBounds(name)));
  }
  solution.copyJointGroupPositions(jmg, joints);
  return Code::TASK_RESULT_SUCCESS;
}

std::uint8_t generate_linear_joint_trajectory(
  const moveit::core::RobotState & start, const std::string & group,
  const std::string & tcp_link, const Eigen::Isometry3d & target,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::function<std::uint8_t()> & interruption)
{
  trajectory = moveit_msgs::msg::RobotTrajectory();
  const auto interrupted = [&] {return interruption ? interruption() : Code::TASK_RESULT_SUCCESS;};
  if (const auto cause = interrupted(); cause != Code::TASK_RESULT_SUCCESS) {return cause;}
  const auto * jmg = start.getRobotModel()->getJointModelGroup(group);
  const auto * link = start.getRobotModel()->getLinkModel(tcp_link);
  if (!jmg || !link || !valid_target(target)) {return Code::TASK_RESULT_INVALID_GOAL;}
  if (!jmg->getSolverInstance()) {return Code::TASK_RESULT_IK_UNREACHABLE;}
  auto current = start;
  std::vector<moveit::core::RobotStatePtr> states;
  const moveit::core::GroupStateValidityCallbackFn bounds =
    [&](moveit::core::RobotState * state, const moveit::core::JointModelGroup * joints,
    const double * values) {
      state->setJointGroupPositions(joints, values);
      return interrupted() == Code::TASK_RESULT_SUCCESS && state->satisfiesBounds(joints);
    };
  const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
    &current, jmg, states, link, target, true, moveit::core::MaxEEFStep(0.005),
    moveit::core::JumpThreshold(1.5), bounds);
  if (const auto cause = interrupted(); cause != Code::TASK_RESULT_SUCCESS) {return cause;}
  if (!std::isfinite(fraction) || fraction < 1.0 - 1e-9 || states.size() < 2U) {
    return Code::TASK_RESULT_IK_UNREACHABLE;
  }
  robot_trajectory::RobotTrajectory path(start.getRobotModel(), group);
  for (const auto & state : states) {path.addSuffixWayPoint(*state, 0.0);}
  path.getRobotTrajectoryMsg(trajectory);
  return Code::TASK_RESULT_SUCCESS;
}
bool retime_joint_trajectory(
  const moveit::core::RobotModelConstPtr & model, const std::string & group,
  const moveit::core::RobotState & start, moveit_msgs::msg::RobotTrajectory & trajectory,
  double velocity_scale, double acceleration_scale,
  const std::vector<JointMotionLimit> * limits)
{
  const auto * joints = model->getJointModelGroup(group);
  if (!joints) {return false;}
  const auto & names = joints->getVariableNames();
  if (limits && limits->size() != names.size()) {return false;}
  std::unordered_map<std::string, double> velocities, accelerations;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto & bounds = model->getVariableBounds(names[i]);
    const double model_velocity = bounds.velocity_bounded_ ?
      std::min(std::abs(bounds.min_velocity_), std::abs(bounds.max_velocity_)) : 1.0;
    const double model_acceleration = bounds.acceleration_bounded_ ?
      std::min(std::abs(bounds.min_acceleration_), std::abs(bounds.max_acceleration_)) : 1.0;
    double velocity = limits ? (*limits)[i].max_velocity : model_velocity;
    double acceleration = limits ? (*limits)[i].max_acceleration : model_acceleration;
    if (bounds.velocity_bounded_) {velocity = std::min(velocity, model_velocity);}
    if (bounds.acceleration_bounded_) {
      acceleration = std::min(acceleration, model_acceleration);
    }
    if (!std::isfinite(velocity) || velocity <= 0 ||
      !std::isfinite(acceleration) || acceleration <= 0) {return false;}
    velocities[names[i]] = velocity * velocity_scale * 0.95;
    accelerations[names[i]] = acceleration * acceleration_scale * 0.95;
  }
  robot_trajectory::RobotTrajectory timed(model, group);
  timed.setRobotTrajectoryMsg(start, trajectory);
  // Preserve the generated joint path while retiming. Scene validation is
  // a separate operation and may not be selected for this movement.
  trajectory_processing::TimeOptimalTrajectoryGeneration parameterization(0.0, 0.02);
  // Leave room for numerical differentiation in the execution sampler; the
  // requested scales remain upper bounds, never relaxed validation limits.
  // Humble's scalar overload does not scale its fallback acceleration when
  // the model omits acceleration bounds. Supply absolute limits explicitly.
  if (!parameterization.computeTimeStamps(timed, velocities, accelerations))
  {return false;}
  timed.getRobotTrajectoryMsg(trajectory);
  std::vector<double> velocity_limits, acceleration_limits;
  for (const auto & name : trajectory.joint_trajectory.joint_names) {
    const auto velocity = velocities.find(name), acceleration = accelerations.find(name);
    if (velocity == velocities.end() || acceleration == accelerations.end()) {return false;}
    velocity_limits.push_back(velocity->second);
    acceleration_limits.push_back(acceleration->second);
  }
  // TOTG resampling can overshoot at sharp joint-path direction changes.
  // Bound the resulting timed fields without changing the prepared path.
  return stretch_taught_trajectory_to_limits(
    trajectory.joint_trajectory, velocity_limits, acceleration_limits);
}

}  // namespace arm_cell_motion
