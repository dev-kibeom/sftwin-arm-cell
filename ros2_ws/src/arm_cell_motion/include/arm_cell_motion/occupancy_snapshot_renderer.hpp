#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <arm_cell_interfaces/msg/collision_primitive.hpp>

namespace arm_cell_motion
{

struct OccupancySnapshot
{
  std::vector<std::uint8_t> png;
  std::string status;
};

// Renders the same request-local occupancy primitive list passed to planning.
OccupancySnapshot render_occupancy_snapshot(
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & primitives,
  const std::string & acquisition_id, const std::string & selected_candidate_id);

void save_occupancy_snapshot_png(
  const std::filesystem::path & destination, const OccupancySnapshot & snapshot);

}  // namespace arm_cell_motion
