#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include <arm_cell_interfaces/msg/motion_capability.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>

namespace arm_cell_orchestration_bt
{

using RecoveryEntryMap =
  std::unordered_map<std::string, std::optional<arm_cell_interfaces::msg::SafetyState>>;

inline void begin_recovery_entry(const std::string & key, RecoveryEntryMap & entries)
{
  entries[key] = std::nullopt;
}

inline void observe_recovery_entry(
  const std::string & key,
  const arm_cell_interfaces::msg::SafetyState & state,
  RecoveryEntryMap & entries)
{
  const auto entry = entries.find(key);
  if (entry == entries.end() || entry->second || !state.valid || !state.required_inputs_fresh ||
    state.motion_capability.value == arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL)
  {
    return;
  }
  entry->second = state;
}

inline std::optional<arm_cell_interfaces::msg::SafetyState> read_recovery_entry(
  const std::string & key, const RecoveryEntryMap & entries)
{
  const auto entry = entries.find(key);
  return entry == entries.end() ? std::nullopt : entry->second;
}

inline void erase_recovery_entry(const std::string & key, RecoveryEntryMap & entries)
{
  entries.erase(key);
}

}  // namespace arm_cell_orchestration_bt
