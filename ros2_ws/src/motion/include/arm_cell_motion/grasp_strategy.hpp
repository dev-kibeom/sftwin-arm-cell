#pragma once

#include <cmath>
#include <string>

namespace arm_cell_motion
{
// Work requirement, independent of the robot and finger length. Tool adapters
// advertise supported strategies and own actuator/capture realization.
enum class GraspStrategy { BOX_EXTERIOR, INTERNAL_RADIAL, SURFACE_SUCTION };

struct GraspExecutionIdentity
{
  std::string execution_id, material_id, supply_id, slot_id, supply_epoch, reservation_id;
  bool valid() const {return execution_id.size() == 32 &&
    execution_id.find_first_not_of("0123456789abcdefABCDEF") == std::string::npos &&
    execution_id.find_first_not_of('0') != std::string::npos && !material_id.empty() &&
    !supply_id.empty() && !slot_id.empty() && !supply_epoch.empty() && !reservation_id.empty();}
};

struct GraspSpecification
{
  GraspStrategy strategy{GraspStrategy::BOX_EXTERIOR};
  // Internal radial solution from selected Tool collision geometry. This is
  // not a Material/Vision schema or a permission to disable scene collision.
  double insertion_depth_m{0.0};
  double thickness_m{0.0};
  double bore_radius_m{0.0};
  double release_position_m{0.0};
  double grasp_position_m{0.0};

  bool valid() const
  {
    if (strategy == GraspStrategy::BOX_EXTERIOR || strategy == GraspStrategy::SURFACE_SUCTION) {return true;}
    if (strategy != GraspStrategy::INTERNAL_RADIAL) {return false;}
    return std::isfinite(insertion_depth_m) && std::isfinite(thickness_m) &&
           std::isfinite(bore_radius_m) && std::isfinite(release_position_m) &&
           std::isfinite(grasp_position_m) && thickness_m > 0.0 &&
           insertion_depth_m > thickness_m / 2.0 && bore_radius_m > 0.0 &&
           release_position_m > grasp_position_m;
  }
};
}  // namespace arm_cell_motion
