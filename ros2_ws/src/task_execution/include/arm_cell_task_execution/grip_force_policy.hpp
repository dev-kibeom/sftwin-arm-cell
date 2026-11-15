#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

namespace arm_cell_task_execution
{
// All forces are the sum of the two opposing finger normal forces, in N.
struct MaterialGripProperties
{
  double mass_kg;
  double friction_coefficient_lower_bound;
  double damage_max_force_n;
};

struct GripForcePolicy
{
  double acceleration_bound_m_s2{1.0};
  double safety_factor{2.0};
  double tool_min_force_n{40.0};
  double tool_max_force_n{470.0};
  std::map<std::string, MaterialGripProperties> materials;

  double required_force(const std::string & material_id) const
  {
    const auto positive = [](double v) {return std::isfinite(v) && v > 0.0;};
    const auto found = materials.find(material_id);
    if (found == materials.end()) {throw std::invalid_argument("unknown grip material");}
    const auto & m = found->second;
    if (!positive(m.mass_kg) || !positive(m.friction_coefficient_lower_bound) ||
      !positive(m.damage_max_force_n) ||
      !std::isfinite(acceleration_bound_m_s2) || acceleration_bound_m_s2 < 0.0 ||
      !std::isfinite(safety_factor) || safety_factor <= 1.0 ||
      !positive(tool_min_force_n) || !positive(tool_max_force_n) ||
      tool_min_force_n > tool_max_force_n)
    {
      throw std::invalid_argument("invalid grip force policy");
    }
    const auto required = safety_factor * m.mass_kg *
      (9.80665 + acceleration_bound_m_s2) / m.friction_coefficient_lower_bound;
    const auto target = std::max(required, tool_min_force_n);
    if (!std::isfinite(target) || target > std::min(tool_max_force_n, m.damage_max_force_n)) {
      throw std::invalid_argument("required grip force exceeds allowable force");
    }
    return target;
  }
};
}  // namespace arm_cell_task_execution
