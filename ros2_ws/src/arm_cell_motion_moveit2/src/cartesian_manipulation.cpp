#include "arm_cell_motion_moveit2/cartesian_manipulation.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace arm_cell_motion_moveit2
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kLimitTolerance = 1e-9;

bool finite_vector(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

bool in_bounds(double value, const JointContinuityLimit & limit)
{
  return value >= limit.lower - kLimitTolerance && value <= limit.upper + kLimitTolerance;
}

geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & target,
  double progress)
{
  geometry_msgs::msg::Pose sample;
  sample.position.x = start.position.x + progress * (target.position.x - start.position.x);
  sample.position.y = start.position.y + progress * (target.position.y - start.position.y);
  sample.position.z = start.position.z + progress * (target.position.z - start.position.z);

  const Eigen::Quaterniond start_orientation(
    start.orientation.w, start.orientation.x, start.orientation.y, start.orientation.z);
  const Eigen::Quaterniond target_orientation(
    target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
  const Eigen::Quaterniond orientation =
    start_orientation.normalized().slerp(progress, target_orientation.normalized());
  sample.orientation.x = orientation.x();
  sample.orientation.y = orientation.y();
  sample.orientation.z = orientation.z();
  sample.orientation.w = orientation.w();
  return sample;
}

void fail(
  CartesianLinearGenerationResult & result,
  std::size_t sample_index,
  const geometry_msgs::msg::Pose & tcp,
  const std::string & reason,
  std::size_t segment_count)
{
  result.valid = false;
  result.failure.sample_index = sample_index;
  result.failure.tcp = tcp;
  result.failure.reason = reason;
  result.reason = reason;
  result.path_fraction = segment_count == 0U ? 0.0 :
    static_cast<double>(sample_index) / static_cast<double>(segment_count);
}
}

CartesianLinearGenerationResult generate_cartesian_linear_trajectory(
  const geometry_msgs::msg::Pose & start_tcp,
  const geometry_msgs::msg::Pose & target_tcp,
  const std::vector<double> & start_positions,
  const std::vector<std::string> & joint_names,
  const std::vector<JointContinuityLimit> & limits,
  double spatial_step_m,
  const CartesianIkSolver & solve_ik,
  const CartesianCollisionChecker & check_collision)
{
  CartesianLinearGenerationResult result;
  result.trajectory.joint_names = joint_names;
  if (joint_names.empty() || start_positions.size() != joint_names.size() ||
    limits.size() != joint_names.size() || !finite_vector(start_positions) ||
    !std::isfinite(spatial_step_m) || spatial_step_m <= 0.0 || !solve_ik || !check_collision)
  {
    result.reason = "invalid Cartesian generator input";
    return result;
  }

  const double dx = target_tcp.position.x - start_tcp.position.x;
  const double dy = target_tcp.position.y - start_tcp.position.y;
  const double dz = target_tcp.position.z - start_tcp.position.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const auto segment_count = std::max(
    1U, static_cast<unsigned int>(std::ceil(distance / spatial_step_m)));
  result.sample_count = static_cast<std::size_t>(segment_count) + 1U;
  result.trajectory.points.reserve(result.sample_count);
  result.trajectory.points.emplace_back();
  result.trajectory.points.front().positions = start_positions;

  std::string failure_reason;
  if (!check_collision(0U, start_tcp, start_positions, failure_reason)) {
    fail(
      result, 0U, start_tcp,
      failure_reason.empty() ? "collision/contact" : failure_reason, segment_count);
    return result;
  }

  std::vector<double> previous_positions = start_positions;
  for (std::size_t index = 1U; index <= segment_count; ++index) {
    const double progress = static_cast<double>(index) / static_cast<double>(segment_count);
    const auto sample_tcp = interpolate_pose(start_tcp, target_tcp, progress);
    std::vector<double> solution;
    failure_reason.clear();
    if (!solve_ik(sample_tcp, previous_positions, solution, failure_reason)) {
      fail(
        result, index, sample_tcp,
        failure_reason.empty() ? "IK" : failure_reason, segment_count);
      return result;
    }
    if (solution.size() != joint_names.size() || !finite_vector(solution)) {
      fail(result, index, sample_tcp, "IK", segment_count);
      return result;
    }
    for (std::size_t joint_index = 0; joint_index < solution.size(); ++joint_index) {
      if (!in_bounds(solution[joint_index], limits[joint_index])) {
        fail(result, index, sample_tcp, "bounds", segment_count);
        return result;
      }
      if (std::abs(solution[joint_index] - previous_positions[joint_index]) > kPi) {
        fail(result, index, sample_tcp, "continuity", segment_count);
        return result;
      }
    }
    failure_reason.clear();
    if (!check_collision(index, sample_tcp, solution, failure_reason)) {
      fail(
        result, index, sample_tcp,
        failure_reason.empty() ? "collision/contact" : failure_reason, segment_count);
      return result;
    }
    result.trajectory.points.emplace_back();
    result.trajectory.points.back().positions = solution;
    previous_positions = std::move(solution);
  }

  result.continuity = normalize_joint_trajectory(
    result.trajectory, start_positions, limits, false);
  if (!result.continuity.valid) {
    fail(
      result, result.sample_count - 1U,
      interpolate_pose(start_tcp, target_tcp, 1.0), "continuity", segment_count);
    return result;
  }
  result.path_fraction = 1.0;
  result.valid = true;
  result.reason.clear();
  return result;
}

CartesianLinearGenerationResult validate_cartesian_joint_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & measured_start,
  const std::vector<JointContinuityLimit> & limits,
  const CartesianExecutableStateValidator & validate_state)
{
  CartesianLinearGenerationResult result;
  result.trajectory = trajectory;
  result.sample_count = trajectory.points.size();
  if (trajectory.points.empty() || measured_start.empty() ||
    measured_start.size() != limits.size() || !validate_state)
  {
    result.reason = "invalid executable trajectory validation input";
    return result;
  }

  std::vector<double> previous = measured_start;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & positions = trajectory.points[index].positions;
    if (positions.size() != measured_start.size() || !finite_vector(positions)) {
      result.failure.sample_index = index;
      result.failure.reason = "continuity";
      result.reason = result.failure.reason;
      return result;
    }
    for (std::size_t joint_index = 0; joint_index < positions.size(); ++joint_index) {
      if (std::abs(positions[joint_index] - previous[joint_index]) > kPi) {
        result.failure.sample_index = index;
        result.failure.reason = "continuity";
        result.reason = result.failure.reason;
        return result;
      }
    }
    previous = positions;
  }

  auto continuity_trajectory = trajectory;
  result.continuity = normalize_joint_trajectory(
    continuity_trajectory, measured_start, limits, false);
  if (!result.continuity.valid) {
    result.failure.sample_index = trajectory.points.size() - 1U;
    result.failure.reason = "continuity";
    result.reason = result.failure.reason;
    return result;
  }

  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    std::string reason;
    std::vector<std::string> contact_bodies;
    if (!validate_state(index, trajectory.points[index].positions, reason, contact_bodies)) {
      result.failure.sample_index = index;
      result.failure.reason = reason.empty() ? "collision/contact" : reason;
      result.failure.contact_bodies = std::move(contact_bodies);
      result.reason = result.failure.reason;
      return result;
    }
  }

  result.path_fraction = 1.0;
  result.valid = true;
  return result;
}

}  // namespace arm_cell_motion_moveit2
