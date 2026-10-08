#pragma once

#include <chrono>
#include <cstdint>

namespace arm_cell_safety_fsm
{

inline bool source_timestamp_is_plausible(
  int64_t source_time_ns, int64_t current_ros_time_ns,
  std::chrono::milliseconds maximum_age)
{
  const auto age_ns = current_ros_time_ns - source_time_ns;
  return age_ns >= 0 &&
         age_ns <= std::chrono::duration_cast<std::chrono::nanoseconds>(maximum_age).count();
}

}  // namespace arm_cell_safety_fsm
