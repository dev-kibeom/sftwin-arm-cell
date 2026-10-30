#pragma once

#include <Eigen/Geometry>

#include <arm_cell_interfaces/action/execute_task.hpp>

#include <cstdint>
#include <vector>

namespace arm_cell_motion
{

inline Eigen::Quaterniond pick_orientation_seed(
  const Eigen::Quaterniond & object_orientation,
  const Eigen::Quaterniond & tcp_orientation,
  uint8_t symmetry_order,
  bool has_observed_rectangle)
{
  if (has_observed_rectangle ||
    symmetry_order == arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
    symmetry_order == arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_FOURFOLD)
  {
    return object_orientation.normalized();
  }
  return tcp_orientation.normalized();
}

inline Eigen::Quaterniond compose_object_tcp_orientation(
  const Eigen::Quaterniond & object_orientation,
  const Eigen::Quaterniond & object_to_tcp_rotation)
{
  return (object_orientation.normalized() * object_to_tcp_rotation.normalized()).normalized();
}

inline bool pick_candidate_requires_object_tcp_composition(
  uint8_t symmetry_order, bool has_observed_rectangle)
{
  return has_observed_rectangle ||
         symmetry_order == arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
         symmetry_order == arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_FOURFOLD;
}

inline std::vector<Eigen::Quaterniond> twofold_pick_orientations(
  const Eigen::Quaterniond & measured_object_orientation)
{
  const auto nominal = measured_object_orientation.normalized();
  const Eigen::Quaterniond half_turn(Eigen::AngleAxisd(
      3.14159265358979323846, Eigen::Vector3d::UnitZ()));
  return {nominal, (half_turn * nominal).normalized()};
}

inline std::vector<Eigen::Quaterniond> fourfold_pick_orientations(
  const Eigen::Quaterniond & measured_object_orientation)
{
  std::vector<Eigen::Quaterniond> candidates;
  candidates.reserve(4U);
  for (std::size_t index = 0; index < 4U; ++index) {
    const Eigen::Quaterniond quarter_turn(Eigen::AngleAxisd(
        static_cast<double>(index) * 1.57079632679489661923,
        Eigen::Vector3d::UnitZ()));
    candidates.push_back((quarter_turn * measured_object_orientation.normalized()).normalized());
  }
  return candidates;
}

}  // namespace arm_cell_motion
