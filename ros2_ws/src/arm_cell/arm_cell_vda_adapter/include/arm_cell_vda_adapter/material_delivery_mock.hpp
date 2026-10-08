#pragma once

#include <chrono>
#include <optional>
#include <vector>

#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace arm_cell_vda_adapter
{

class MaterialDeliveryMock final
{
public:
  using Clock = std::chrono::steady_clock;
  using HandoffState = arm_cell_interfaces::msg::MaterialHandoffState;
  using DeliveryId = unique_identifier_msgs::msg::UUID;

  explicit MaterialDeliveryMock(std::chrono::milliseconds phase_period);

  bool request(const DeliveryId & delivery_id, Clock::time_point receipt_time);
  void fail_active();
  [[nodiscard]] std::optional<HandoffState> snapshot(
    Clock::time_point now, const builtin_interfaces::msg::Time & stamp);

private:
  static bool valid_id(const DeliveryId & delivery_id);
  static bool same_id(const DeliveryId & left, const DeliveryId & right);

  std::chrono::milliseconds phase_period_;
  std::optional<DeliveryId> active_delivery_;
  std::vector<DeliveryId> seen_deliveries_;
  uint8_t phase_{HandoffState::HANDOFF_UNKNOWN};
  Clock::time_point phase_started_{};
};

}  // namespace arm_cell_vda_adapter
