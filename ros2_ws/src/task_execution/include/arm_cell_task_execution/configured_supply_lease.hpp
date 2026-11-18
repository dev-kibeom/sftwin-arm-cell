#pragma once
#include <chrono>
#include <set>
#include <arm_cell_interfaces/msg/configured_supply_state.hpp>
#include "arm_cell_motion/grasp_strategy.hpp"

namespace arm_cell_task_execution
{
class ConfiguredSupplyLease
{
public:
  using Clock = std::chrono::steady_clock;
  void update(const arm_cell_interfaces::msg::ConfiguredSupplyState & state, Clock::time_point now)
  {state_ = state; received_ = now; observed_ = true;}
  bool matches(
    const arm_cell_motion::GraspExecutionIdentity & identity,
    Clock::time_point now) const
  {
    if (!observed_ || now < received_ || now - received_ > std::chrono::milliseconds(500) ||
      !state_.valid || state_.supply_id != identity.supply_id ||
      state_.material_id != identity.material_id ||
      state_.supply_epoch != identity.supply_epoch ||
      state_.reservation_id != identity.reservation_id ||
      state_.selected_slot_id != identity.slot_id ||
      state_.slot_ids.size() != state_.occupied.size() ||
      std::set<std::string>(
        state_.slot_ids.begin(),
        state_.slot_ids.end()).size() != state_.slot_ids.size())
    {return false;}
    for (std::size_t i = 0; i < state_.slot_ids.size(); ++i) {
      if (state_.slot_ids[i] == identity.slot_id) {return state_.occupied[i];}
    }
    return false;
  }

private:
  arm_cell_interfaces::msg::ConfiguredSupplyState state_;
  Clock::time_point received_{};
  bool observed_{false};
};
}
