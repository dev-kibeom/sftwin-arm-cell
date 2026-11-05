#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "arm_cell_orchestration/safety_observation.hpp"

namespace arm_cell_orchestration
{

using RecoveryEntryMap =
  std::unordered_map<std::string, std::optional<SafetyObservation>>;

inline void begin_recovery_entry(const std::string & key, RecoveryEntryMap & entries)
{
  entries[key] = std::nullopt;
}

inline void observe_recovery_entry(
  const std::string & key,
  const SafetyObservation & observation,
  RecoveryEntryMap & entries)
{
  const auto entry = entries.find(key);
  if (entry == entries.end() || entry->second || !observation.delivery_valid ||
    observation.state.motion_capability.value ==
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL)
  {
    return;
  }
  entry->second = observation;
}

inline std::optional<SafetyObservation> read_recovery_entry(
  const std::string & key, const RecoveryEntryMap & entries)
{
  const auto entry = entries.find(key);
  return entry == entries.end() ? std::nullopt : entry->second;
}

inline void erase_recovery_entry(const std::string & key, RecoveryEntryMap & entries)
{
  entries.erase(key);
}

}  // namespace arm_cell_orchestration
