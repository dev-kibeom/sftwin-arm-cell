#pragma once

#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "arm_cell_task_execution/prepared_grasp_cycle.hpp"

namespace arm_cell_task_execution
{
struct ConfiguredGraspCatalog
{
  std::map<std::string, PreparedGraspCycle> by_slot;
  std::optional<PreparedGraspCycle> fixed;

  std::set<std::string> material_types;

  const PreparedGraspCycle * find(const std::string & slot, const std::string & type = {}) const
  {
    if (fixed) {return fixed->identity.slot_id == slot && material_types.count(type) ? &*fixed : nullptr;}
    const auto entry = by_slot.find(slot);
    return entry == by_slot.end() ? nullptr : &entry->second;
  }
};

inline ConfiguredGraspCatalog parse_configured_grasp_cycles(
  const std::string & source)
{
  const auto json = nlohmann::json::parse(source);
  if (!json.is_object()) {
    throw std::invalid_argument("Configured grasp catalog must be a mapping");
  }
  ConfiguredGraspCatalog result;
  for (const auto & entry : json.items()) {
    try {
      const auto & value = entry.value();
      const auto & grasp = value.at("grasp");
      PreparedGraspCycle cycle;
      if (entry.key().empty()) {throw std::invalid_argument("Empty configured slot");}
      if (grasp.at("strategy") == "surface_suction") {
        cycle.grasp.strategy = arm_cell_motion::GraspStrategy::SURFACE_SUCTION;
      } else if (grasp.at("strategy") == "internal_radial") {
        cycle.grasp.strategy = arm_cell_motion::GraspStrategy::INTERNAL_RADIAL;
        cycle.grasp.insertion_depth_m = grasp.at("insertion_depth_m");
        cycle.grasp.thickness_m = grasp.at("thickness_m");
        cycle.grasp.bore_radius_m = grasp.at("inner_radius_m");
        cycle.grasp.release_position_m = grasp.at("release_position_m");
        cycle.grasp.grasp_position_m = grasp.at("grasp_position_m");
      } else {
        throw std::invalid_argument("Unsupported configured grasp strategy");
      }
      cycle.approach_motion = value.at("approach_motion");
      cycle.insertion_motion = value.at("insertion_motion");
      cycle.transfer_motion = value.at("transfer_motion");
      cycle.retreat_motion = value.at("retreat_motion");
      cycle.identity.supply_id = value.at("supply_id");
      cycle.identity.material_id = value.at("material_id");
      if (!cycle.valid() || cycle.identity.supply_id.empty() ||
        cycle.identity.material_id.empty())
      {
        throw std::invalid_argument("Invalid configured grasp operation");
      }
      if (value.contains("pick_slot_id")) {
        if (json.size() != 1) {
          throw std::invalid_argument("A fixed teaching must be the only catalog entry");
        }
        cycle.identity.slot_id = value.at("pick_slot_id");
        result.material_types = value.at("material_type_ids").get<std::set<std::string>>();
        if (cycle.identity.slot_id.empty() || result.material_types.empty() || result.material_types.count("")) {
          throw std::invalid_argument("Fixed teaching requires a pick location and compatible types");
        }
        result.fixed = std::move(cycle);
      } else {
        if (value.value("use_reserved_slot", false)) {
          throw std::invalid_argument("Re-export the fixed teaching with a physical pick location");
        }
        cycle.identity.slot_id = entry.key();
        result.by_slot.emplace(entry.key(), std::move(cycle));
      }
    } catch (const std::exception & error) {
      throw std::invalid_argument("Configured grasp slot " + entry.key() + ": " + error.what());
    }
  }
  return result;
}
}  // namespace arm_cell_task_execution
