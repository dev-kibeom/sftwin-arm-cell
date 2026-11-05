#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "arm_cell_motion/box_grasp_geometry.hpp"

namespace arm_cell_motion
{

enum class PickGraspPoseOrderPolicy
{
  DISTANCE_THEN_TILT,
  DISTANCE_THEN_DIVERSITY
};

inline PickGraspPoseOrderPolicy parse_pick_grasp_pose_order_policy(const std::string & value)
{
  if (value == "distance_then_tilt") {
    return PickGraspPoseOrderPolicy::DISTANCE_THEN_TILT;
  }
  if (value == "distance_then_diversity") {
    return PickGraspPoseOrderPolicy::DISTANCE_THEN_DIVERSITY;
  }
  throw std::invalid_argument(
          "pick_grasp_pose_order_policy must be 'distance_then_tilt' or 'distance_then_diversity'");
}

struct PickGraspPoseCandidate
{
  Eigen::Quaterniond tcp_orientation{Eigen::Quaterniond::Identity()};
  std::optional<BoxGraspContact> contact;
  Eigen::Vector3d grasp_position_offset_object{Eigen::Vector3d::Zero()};
  std::size_t orientation_candidate_index{0U};
  std::size_t axis_orientation_sample_index{0U};
  std::size_t generation_index{0U};
  double withdrawal_tilt_rad{0.0};
};

struct PickGraspSearchSampling
{
  double maximum_withdrawal_tilt_deg{90.0};
  double position_range_half_length_fraction{0.5};
  std::size_t position_offset_samples_per_tangent_axis{3U};
  std::size_t orientation_samples_per_closing_axis{8U};
  std::size_t candidate_limit{48U};
  double maximum_downward_position_offset_m{0.005};
};

// Enumerate the box's recipe-width-feasible opposing face pairs in stable
// priority order. The reference grasp pose comes first when it is already a
// valid grasp; the existing box-face alignment is then reused for other pairs.
inline std::vector<PickGraspPoseCandidate> generate_pick_grasp_pose_candidates(
  const Eigen::Quaterniond & object_orientation,
  const Eigen::Quaterniond & reference_grasp_tcp_orientation,
  double long_side_m, double short_side_m, double height_m,
  double policy_min_mm, double policy_max_mm, double gripper_max_mm,
  std::size_t candidate_limit)
{
  std::vector<PickGraspPoseCandidate> candidates;
  if (candidate_limit == 0U || !object_orientation.coeffs().allFinite() ||
    !reference_grasp_tcp_orientation.coeffs().allFinite() ||
    object_orientation.norm() == 0.0 || reference_grasp_tcp_orientation.norm() == 0.0)
  {
    return candidates;
  }

  const auto object_rotation = object_orientation.normalized();
  const auto reference_grasp_tcp_rotation = reference_grasp_tcp_orientation.normalized();
  const auto reference_grasp_contact = box_grasp_contact(
    object_rotation, reference_grasp_tcp_rotation, long_side_m, short_side_m, height_m,
    policy_min_mm, policy_max_mm, gripper_max_mm);
  if (reference_grasp_contact) {
    candidates.push_back({reference_grasp_tcp_rotation, *reference_grasp_contact});
  }

  const Eigen::Vector3d reference_grasp_closing_direction =
    reference_grasp_tcp_rotation * Eigen::Vector3d::UnitX();
  const std::array<double, 3> box_lengths{long_side_m, short_side_m, height_m};
  for (std::size_t axis = 0U; axis < box_lengths.size() &&
    candidates.size() < candidate_limit; ++axis)
  {
    if (!std::isfinite(box_lengths[axis]) || box_lengths[axis] <= 0.0) {continue;}
    const double width_mm = box_lengths[axis] * 1000.0;
    if (!std::isfinite(policy_min_mm) || !std::isfinite(policy_max_mm) ||
      !std::isfinite(gripper_max_mm) || width_mm < policy_min_mm ||
      width_mm > policy_max_mm || width_mm > gripper_max_mm)
    {
      continue;
    }
    if (reference_grasp_contact && reference_grasp_contact->closing_axis == axis) {continue;}

    auto face_normal = object_rotation * Eigen::Vector3d::Unit(axis);
    if (reference_grasp_closing_direction.dot(face_normal) < 0.0) {
      face_normal = -face_normal;
    }
    const auto alignment = Eigen::Quaterniond::FromTwoVectors(
      reference_grasp_closing_direction, face_normal);
    const auto candidate_rotation = (alignment * reference_grasp_tcp_rotation).normalized();
    const auto contact = box_grasp_contact(
      object_rotation, candidate_rotation, long_side_m, short_side_m, height_m,
      policy_min_mm, policy_max_mm, gripper_max_mm);
    if (contact && contact->closing_axis == axis) {
      candidates.push_back({candidate_rotation, *contact});
    }
  }
  return candidates;
}

// Selection is independent of evaluation: give each face/orientation a turn
// across the finite offset lattice, then order only the selected poses.
inline std::vector<PickGraspPoseCandidate> generate_pick_grasp_search_candidates(
  const Eigen::Quaterniond & object_orientation,
  const Eigen::Quaterniond & reference_grasp_tcp_orientation,
  const Eigen::Vector3d & reference_grasp_position_object,
  const Eigen::Vector3d & object_position_base,
  const Eigen::Vector3d & insertion_axis_tcp,
  double long_side_m, double short_side_m, double height_m,
  double policy_min_mm, double policy_max_mm, double gripper_max_mm,
  const PickGraspSearchSampling & sampling)
{
  auto candidates = generate_pick_grasp_pose_candidates(
    object_orientation, reference_grasp_tcp_orientation, long_side_m, short_side_m,
    height_m, policy_min_mm, policy_max_mm, gripper_max_mm, 3U);
  if (candidates.empty() || !reference_grasp_position_object.allFinite() ||
    !object_position_base.allFinite() || !insertion_axis_tcp.allFinite() ||
    insertion_axis_tcp.norm() < 1e-9 || sampling.candidate_limit == 0U)
  {
    return {};
  }
  const auto object_rotation = object_orientation.normalized();
  // The supported production domain uses a horizontal Tray and RawPart poses
  // whose local +Z is the top normal, so this is also the outward support
  // normal. A separate support-plane interface is not part of this domain.
  const Eigen::Vector3d support_normal = object_rotation * Eigen::Vector3d::UnitZ();
  const auto insertion = insertion_axis_tcp.normalized();
  const std::array<double, 3> lengths{long_side_m, short_side_m, height_m};
  const auto seeds = candidates;
  candidates.clear();
  const double maximum_withdrawal_tilt = sampling.maximum_withdrawal_tilt_deg *
    3.14159265358979323846 / 180.0;
  std::array<std::vector<PickGraspPoseCandidate>, 3> by_axis;
  for (const auto & seed : seeds) {
    const Eigen::Vector3d closing = seed.tcp_orientation * Eigen::Vector3d::UnitX();
    const auto count = std::max<std::size_t>(1U, sampling.orientation_samples_per_closing_axis);
    for (std::size_t step = 0U; step < count; ++step) {
      auto pose = seed;
      const double angle = 2.0 * 3.14159265358979323846 * static_cast<double>(step) /
        static_cast<double>(count);
      pose.tcp_orientation = (Eigen::Quaterniond(Eigen::AngleAxisd(angle, closing)) *
        seed.tcp_orientation).normalized();
      pose.contact = box_grasp_contact(
        object_rotation, pose.tcp_orientation, long_side_m, short_side_m, height_m,
        policy_min_mm, policy_max_mm, gripper_max_mm);
      if (!pose.contact || pose.contact->closing_axis != seed.contact->closing_axis) {continue;}
      const Eigen::Vector3d withdrawal = -(pose.tcp_orientation * insertion);
      pose.withdrawal_tilt_rad = std::acos(std::clamp(withdrawal.dot(support_normal), -1.0, 1.0));
      if (pose.withdrawal_tilt_rad > maximum_withdrawal_tilt + 1e-9) {continue;}
      auto & axis_poses = by_axis[pose.contact->closing_axis];
      if (std::any_of(
          axis_poses.begin(), axis_poses.end(), [&](const auto & existing) {
            return std::abs(existing.tcp_orientation.dot(pose.tcp_orientation)) > 1.0 - 1e-9;
          })) {continue;}
      pose.axis_orientation_sample_index = axis_poses.size();
      axis_poses.push_back(pose);
    }
  }
  // Interleave eligible faces at each axis-local orientation sample index. The
  // reference face pose is sample zero when it passes the unchanged rules.
  std::vector<PickGraspPoseCandidate> orientations;
  for (std::size_t axis_orientation_sample_index = 0U;;
    ++axis_orientation_sample_index)
  {
    bool found = false;
    for (const auto & axis_poses : by_axis) {
      if (axis_orientation_sample_index >= axis_poses.size()) {continue;}
      auto pose = axis_poses[axis_orientation_sample_index];
      pose.orientation_candidate_index = orientations.size();
      orientations.push_back(pose);
      found = true;
    }
    if (!found) {break;}
  }
  const auto add = [&](const PickGraspPoseCandidate & pose) {
      if (candidates.size() >= sampling.candidate_limit) {return;}
      for (const auto & existing : candidates) {
        if ((existing.grasp_position_offset_object - pose.grasp_position_offset_object).norm() <
          1e-9 &&
          std::abs(existing.tcp_orientation.dot(pose.tcp_orientation)) > 1.0 - 1e-9)
        {return;}
      }
      auto selected_pose = pose;
      selected_pose.generation_index = candidates.size();
      candidates.push_back(selected_pose);
    };
  // A zero offset is one point of the same pose space. Reserve it for every
  // eligible orientation before distributing remaining slots; no runtime group.
  for (const auto & pose : orientations) {
    add(pose);
  }
  const std::size_t radii = sampling.position_offset_samples_per_tangent_axis >= 2U &&
    sampling.position_range_half_length_fraction >
    0.0 ? sampling.position_offset_samples_per_tangent_axis - 1U : 0U;
  const std::size_t offset_count = radii * 8U;
  std::vector<std::size_t> cursors(orientations.size(), 0U);
  const std::array<std::array<int, 2>, 8> directions{{
    {{-1, 0}}, {{1, 0}}, {{0, -1}}, {{0, 1}},
    {{-1, -1}}, {{-1, 1}}, {{1, -1}}, {{1, 1}}}};
  while (candidates.size() < sampling.candidate_limit) {
    const auto before = candidates.size();
    for (std::size_t i = 0U; i < orientations.size() &&
      candidates.size() < sampling.candidate_limit; ++i)
    {
      // Enumerate lazily. Spread first offset samples across directions AND radii,
      // rather than exhausting the inner ring for every orientation.
      while (cursors[i] < offset_count) {
        const auto sample = (cursors[i]++ + i) % offset_count;
        const auto radius = sample % radii + 1U;
        const auto & signs = directions[sample / radii];
        auto pose = orientations[i];
        const auto axis = pose.contact->closing_axis;
        const std::array<std::size_t, 2> tangents{(axis + 1U) % 3U, (axis + 2U) % 3U};
        for (std::size_t j = 0U; j < tangents.size(); ++j) {
          const auto tangent = tangents[j];
          const double value = signs[j] * sampling.position_range_half_length_fraction *
            lengths[tangent] *
            0.5 * static_cast<double>(radius) / static_cast<double>(radii);
          pose.grasp_position_offset_object[tangent] = tangent == 2U && value < 0.0 ?
            -std::min(-value, sampling.maximum_downward_position_offset_m) : value;
        }
        const auto point = reference_grasp_position_object + pose.grasp_position_offset_object;
        if (std::abs(point[tangents[0]]) >= lengths[tangents[0]] * 0.5 ||
          std::abs(point[tangents[1]]) >= lengths[tangents[1]] * 0.5) {continue;}
        const auto size = candidates.size();
        add(pose);
        if (candidates.size() != size) {break;}
      }
    }
    if (before == candidates.size()) {break;}
  }
  return candidates;
}

// Compare rounded squared distances only to suppress floating-point noise in
// equal lattice radii. Policies share that distance key and stable tie handling;
// they differ only in the order of the tie-break keys.
inline void order_pick_grasp_search_candidates(
  std::vector<PickGraspPoseCandidate> & candidates, PickGraspPoseOrderPolicy policy)
{
  const auto offset_distance_key = [](const auto & pose) {
      return std::llround(pose.grasp_position_offset_object.squaredNorm() * 1e15);
    };
  std::vector<std::size_t> same_orientation_distance_revisit_counts(candidates.size(), 0U);
  for (std::size_t i = 0U; i < candidates.size(); ++i) {
    for (std::size_t j = 0U; j < i; ++j) {
      if (candidates[i].orientation_candidate_index == candidates[j].orientation_candidate_index &&
        offset_distance_key(candidates[i]) == offset_distance_key(candidates[j]))
      {
        ++same_orientation_distance_revisit_counts[i];
      }
    }
  }
  std::vector<std::size_t> order;
  for (std::size_t i = 0U; i < candidates.size(); ++i) {
    order.push_back(i);
  }
  std::stable_sort(
    order.begin(), order.end(), [&](auto a, auto b) {
      const auto key = [&](auto i) -> std::tuple<long long, long long, long long, long long> {
        const auto distance = offset_distance_key(candidates[i]);
        const auto tilt = std::llround(candidates[i].withdrawal_tilt_rad * 1e12);
        const auto diversity = same_orientation_distance_revisit_counts[i];
        const auto sample = candidates[i].axis_orientation_sample_index;
        if (policy == PickGraspPoseOrderPolicy::DISTANCE_THEN_TILT) {
          return std::make_tuple(
            distance, tilt, static_cast<long long>(diversity), static_cast<long long>(sample));
        }
        return std::make_tuple(
          distance, static_cast<long long>(diversity), static_cast<long long>(sample), tilt);
      };
      return key(a) < key(b);
    });
  const auto selected = candidates;
  for (std::size_t i = 0U; i < order.size(); ++i) {
    candidates[i] = selected[order[i]];
  }
}

}  // namespace arm_cell_motion
