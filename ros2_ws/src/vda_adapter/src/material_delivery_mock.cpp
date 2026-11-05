#include "arm_cell_vda_adapter/material_delivery_mock.hpp"

#include <algorithm>
#include <stdexcept>

namespace arm_cell_vda_adapter
{

MaterialDeliveryMock::MaterialDeliveryMock(std::chrono::milliseconds phase_period)
: phase_period_(phase_period)
{
  if (phase_period_.count() <= 0) {
    throw std::invalid_argument("phase_period_ms must be greater than zero");
  }
}

bool MaterialDeliveryMock::valid_id(const DeliveryId & delivery_id)
{
  return std::any_of(
    delivery_id.uuid.begin(), delivery_id.uuid.end(), [](uint8_t byte) {return byte != 0;});
}

bool MaterialDeliveryMock::same_id(const DeliveryId & left, const DeliveryId & right)
{
  return left.uuid == right.uuid;
}

bool MaterialDeliveryMock::consume_accepted_trigger(
  const AcceptedTrigger & trigger, Clock::time_point receipt_time)
{
  const auto & delivery_id = trigger.delivery_id;
  if (!trigger.valid || !valid_id(delivery_id) || trigger.quantity == 0 ||
    trigger.material_type.empty() ||
    std::any_of(
      seen_deliveries_.begin(), seen_deliveries_.end(),
      [&delivery_id](const auto & seen) {return same_id(seen, delivery_id);}))
  {
    return false;
  }
  active_deliveries_.push_back({delivery_id, HandoffState::HANDOFF_ARRIVED, receipt_time});
  seen_deliveries_.push_back(delivery_id);
  return true;
}

void MaterialDeliveryMock::fail_active()
{
  for (auto & delivery : active_deliveries_) {
    if (delivery.phase != HandoffState::HANDOFF_DEPARTED) {
      delivery.phase = HandoffState::HANDOFF_FAILED;
    }
  }
}

std::vector<MaterialDeliveryMock::HandoffState> MaterialDeliveryMock::snapshot_all(
  Clock::time_point now, const builtin_interfaces::msg::Time & stamp)
{
  std::vector<HandoffState> output;
  output.reserve(active_deliveries_.size());
  for (auto & delivery : active_deliveries_) {
    if (delivery.phase != HandoffState::HANDOFF_FAILED &&
      delivery.phase != HandoffState::HANDOFF_DEPARTED &&
      now - delivery.phase_started >= phase_period_)
    {
      ++delivery.phase;
      delivery.phase_started = now;
    }

    HandoffState message;
    message.header.stamp = stamp;
    message.delivery_id = delivery.delivery_id;
    message.phase = delivery.phase;
    message.valid = delivery.phase != HandoffState::HANDOFF_FAILED;
    output.push_back(std::move(message));
  }
  active_deliveries_.erase(
    std::remove_if(
      active_deliveries_.begin(), active_deliveries_.end(), [](const auto & delivery) {
        return delivery.phase == HandoffState::HANDOFF_FAILED ||
               delivery.phase == HandoffState::HANDOFF_DEPARTED;
      }),
    active_deliveries_.end());
  return output;
}

}  // namespace arm_cell_vda_adapter
