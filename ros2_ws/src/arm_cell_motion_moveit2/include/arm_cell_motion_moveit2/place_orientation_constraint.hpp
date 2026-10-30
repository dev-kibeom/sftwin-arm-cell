#pragma once

#include <cstdint>

namespace arm_cell_motion_moveit2
{

enum class PlaceOrientationConstraint : std::uint8_t
{
  FIXED,
  BOUNDED,
  FREE
};

}  // namespace arm_cell_motion_moveit2
