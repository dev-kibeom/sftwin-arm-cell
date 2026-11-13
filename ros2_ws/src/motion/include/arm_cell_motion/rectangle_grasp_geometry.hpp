#pragma once

#include <cmath>
#include <optional>

namespace arm_cell_motion
{

inline std::optional<double> rectangle_expected_contact_width_mm(
  double long_side_m, double short_side_m, bool closing_axis_parallel_to_long)
{
  if (!std::isfinite(long_side_m) || !std::isfinite(short_side_m) ||
    short_side_m <= 0.0 || long_side_m < short_side_m)
  {
    return std::nullopt;
  }
  return (closing_axis_parallel_to_long ? long_side_m : short_side_m) * 1000.0;
}

inline bool rectangle_contact_width_allowed(
  double expected_width_mm, double physical_min_mm, double physical_max_mm)
{
  return std::isfinite(expected_width_mm) && std::isfinite(physical_min_mm) &&
         std::isfinite(physical_max_mm) && physical_min_mm >= 0.0 &&
         physical_min_mm <= physical_max_mm && expected_width_mm > 0.0 &&
         expected_width_mm >= physical_min_mm && expected_width_mm <= physical_max_mm;
}

}  // namespace arm_cell_motion
