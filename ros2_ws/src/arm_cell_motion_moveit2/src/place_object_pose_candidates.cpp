#include "arm_cell_motion_moveit2/place_approach_ik_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace arm_cell_motion_moveit2
{

namespace
{
constexpr double kDuplicateEpsilon = 1e-12;

double orientation_distance(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  const auto dot = std::abs(
    lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z + lhs.w * rhs.w);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

void append_candidate(
  std::vector<PlaceObjectPoseCandidate> & candidates,
  const geometry_msgs::msg::PoseStamped & desired,
  const std::array<double, 3> & translation,
  const tf2::Quaternion & rotation_delta,
  double position_tolerance_m, double orientation_tolerance_rad)
{
  auto candidate = desired;
  candidate.pose.position.x += translation[0];
  candidate.pose.position.y += translation[1];
  candidate.pose.position.z += translation[2];

  tf2::Quaternion object_rotation(
    desired.pose.orientation.x, desired.pose.orientation.y,
    desired.pose.orientation.z, desired.pose.orientation.w);
  object_rotation.normalize();
  auto candidate_rotation = object_rotation * rotation_delta;
  candidate_rotation.normalize();
  candidate.pose.orientation.x = candidate_rotation.x();
  candidate.pose.orientation.y = candidate_rotation.y();
  candidate.pose.orientation.z = candidate_rotation.z();
  candidate.pose.orientation.w = candidate_rotation.w();

  const double position_deviation = std::sqrt(
    translation[0] * translation[0] + translation[1] * translation[1] +
    translation[2] * translation[2]);
  const double angular_deviation = orientation_distance(
    desired.pose.orientation, candidate.pose.orientation);
  const double normalized_position = position_tolerance_m > 0.0 ?
    position_deviation / position_tolerance_m : 0.0;
  const double normalized_orientation = orientation_tolerance_rad > 0.0 ?
    angular_deviation / orientation_tolerance_rad : 0.0;
  const double deviation = std::sqrt(
    normalized_position * normalized_position +
    normalized_orientation * normalized_orientation);

  const bool duplicate = std::any_of(
    candidates.begin(), candidates.end(), [&](const auto & existing) {
      const auto & lhs = existing.object_pose.pose;
      const auto & rhs = candidate.pose;
      const double dx = lhs.position.x - rhs.position.x;
      const double dy = lhs.position.y - rhs.position.y;
      const double dz = lhs.position.z - rhs.position.z;
      return dx * dx + dy * dy + dz * dz <= kDuplicateEpsilon &&
             orientation_distance(lhs.orientation, rhs.orientation) <= kDuplicateEpsilon;
    });
  if (!duplicate) {
    candidates.push_back({std::move(candidate), deviation});
  }
}

}  // namespace

std::vector<PlaceObjectPoseCandidate> generate_place_object_pose_candidates(
  const geometry_msgs::msg::PoseStamped & desired_object_pose,
  double position_tolerance_m, double orientation_tolerance_rad,
  PlaceOrientationConstraint orientation_constraint)
{
  std::vector<PlaceObjectPoseCandidate> candidates;
  if (!std::isfinite(position_tolerance_m) || position_tolerance_m < 0.0 ||
    !std::isfinite(orientation_tolerance_rad) || orientation_tolerance_rad < 0.0)
  {
    return candidates;
  }

  const tf2::Quaternion identity(0.0, 0.0, 0.0, 1.0);
  append_candidate(
    candidates, desired_object_pose, {0.0, 0.0, 0.0}, identity,
    position_tolerance_m, orientation_tolerance_rad);

  const std::array<tf2::Vector3, 6> axes = {
    tf2::Vector3(1.0, 0.0, 0.0), tf2::Vector3(-1.0, 0.0, 0.0),
    tf2::Vector3(0.0, 1.0, 0.0), tf2::Vector3(0.0, -1.0, 0.0),
    tf2::Vector3(0.0, 0.0, 1.0), tf2::Vector3(0.0, 0.0, -1.0)};

  if (position_tolerance_m > 0.0) {
    for (const auto & axis : axes) {
      append_candidate(
        candidates, desired_object_pose,
        {position_tolerance_m * axis.x(), position_tolerance_m * axis.y(),
          position_tolerance_m * axis.z()}, identity,
        position_tolerance_m, orientation_tolerance_rad);
    }
  }

  if (orientation_constraint == PlaceOrientationConstraint::BOUNDED &&
    orientation_tolerance_rad > 0.0)
  {
    for (const auto & axis : axes) {
      append_candidate(
        candidates, desired_object_pose, {0.0, 0.0, 0.0},
        tf2::Quaternion(axis, orientation_tolerance_rad),
        position_tolerance_m, orientation_tolerance_rad);
    }
  }

  if (orientation_constraint == PlaceOrientationConstraint::FREE) {
    constexpr double kQuarterTurn = 1.5707963267948966;
    const std::array<tf2::Quaternion, 6> side_on = {
      tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), kQuarterTurn),
      tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), -kQuarterTurn),
      tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), kQuarterTurn),
      tf2::Quaternion(tf2::Vector3(0.0, 1.0, 0.0), -kQuarterTurn),
      tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), kQuarterTurn),
      tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), -kQuarterTurn)};
    for (const auto & rotation : side_on) {
      append_candidate(
        candidates, desired_object_pose, {0.0, 0.0, 0.0}, rotation,
        position_tolerance_m, 0.0);
    }
  }

  std::stable_sort(
    candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.nominal_deviation < rhs.nominal_deviation;
    });
  return candidates;
}

BoundedPlacePoseCandidateGroups group_bounded_place_pose_candidates(
  const std::vector<PlaceObjectPoseCandidate> & candidates,
  const geometry_msgs::msg::PoseStamped & nominal_object_pose)
{
  BoundedPlacePoseCandidateGroups groups;
  const auto & nominal = nominal_object_pose.pose.position;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto & position = candidates[index].object_pose.pose.position;
    const bool at_nominal_position =
      std::abs(position.x - nominal.x) <= 1e-9 &&
      std::abs(position.y - nominal.y) <= 1e-9 &&
      std::abs(position.z - nominal.z) <= 1e-9;
    (at_nominal_position ? groups.primary_orientation_indices :
      groups.position_fallback_indices).push_back(index);
  }
  return groups;
}

}  // namespace arm_cell_motion_moveit2
