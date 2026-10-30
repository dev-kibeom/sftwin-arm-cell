#pragma once

#include <cstdint>

namespace arm_cell_motion
{

enum class PickApproachAxis : std::uint8_t
{
  TCP_INSERTION_AXIS,
  PLANNING_FRAME_VERTICAL
};

}  // namespace arm_cell_motion
