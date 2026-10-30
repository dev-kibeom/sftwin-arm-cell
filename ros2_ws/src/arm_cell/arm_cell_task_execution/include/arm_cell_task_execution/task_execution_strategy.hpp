#pragma once

#include <stdexcept>
#include <string>

namespace arm_cell_task_execution
{
enum class PickPlaceStrategy
{
  PLANNED,
  WAYPOINT
};

struct PickPlaceStrategies
{
  PickPlaceStrategy pick{PickPlaceStrategy::PLANNED};
  PickPlaceStrategy place{PickPlaceStrategy::PLANNED};
};

inline PickPlaceStrategy parse_pick_place_strategy(
  const std::string & value, const std::string & parameter_name)
{
  if (value == "PLANNED") {return PickPlaceStrategy::PLANNED;}
  if (value == "WAYPOINT") {return PickPlaceStrategy::WAYPOINT;}
  throw std::invalid_argument(
          parameter_name + " must be PLANNED or WAYPOINT; got '" + value + "'");
}

inline const char * pick_place_strategy_name(PickPlaceStrategy strategy)
{
  switch (strategy) {
    case PickPlaceStrategy::PLANNED: return "PLANNED";
    case PickPlaceStrategy::WAYPOINT: return "WAYPOINT";
  }
  return "UNKNOWN";
}
}  // namespace arm_cell_task_execution
