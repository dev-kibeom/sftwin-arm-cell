#pragma once

#include <chrono>
#include <vector>

#include <arm_cell_interfaces/msg/handoff_trigger.hpp>
#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace arm_cell_vda_adapter
{

class MaterialDeliveryMock final
{
public:
  using Clock = std::chrono::steady_clock;
  using AcceptedTrigger = arm_cell_interfaces::msg::HandoffTrigger;
  using HandoffState = arm_cell_interfaces::msg::MaterialHandoffState;
  using DeliveryId = unique_identifier_msgs::msg::UUID;

  explicit MaterialDeliveryMock(std::chrono::milliseconds phase_period);

  bool consume_accepted_trigger(
    const AcceptedTrigger & trigger, Clock::time_point receipt_time);
  void fail_active();
  [[nodiscard]] std::vector<HandoffState> snapshot_all(
    Clock::time_point now, const builtin_interfaces::msg::Time & stamp);

private:
  static bool valid_id(const DeliveryId & delivery_id);
  static bool same_id(const DeliveryId & left, const DeliveryId & right);

  std::chrono::milliseconds phase_period_;
  struct DeliveryState
  {
    DeliveryId delivery_id;
    uint8_t phase{HandoffState::HANDOFF_ARRIVED};
    Clock::time_point phase_started{};
  };
  std::vector<DeliveryState> active_deliveries_;
  std::vector<DeliveryId> seen_deliveries_;
  uint8_t phase_{HandoffState::HANDOFF_UNKNOWN};
  Clock::time_point phase_started_{};
};

}  // namespace arm_cell_vda_adapter
