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

bool MaterialDeliveryMock::request(
  const DeliveryId & delivery_id, Clock::time_point receipt_time)
{
  if (!valid_id(delivery_id) || active_delivery_ ||
    std::any_of(
      seen_deliveries_.begin(), seen_deliveries_.end(),
      [&delivery_id](const auto & seen) {return same_id(seen, delivery_id);}))
  {
    return false;
  }
  active_delivery_ = delivery_id;
  seen_deliveries_.push_back(delivery_id);
  phase_ = HandoffState::HANDOFF_ARRIVED;
  phase_started_ = receipt_time;
  return true;
}

void MaterialDeliveryMock::fail_active()
{
  if (active_delivery_ && phase_ != HandoffState::HANDOFF_DEPARTED) {
    phase_ = HandoffState::HANDOFF_FAILED;
  }
}

std::optional<MaterialDeliveryMock::HandoffState> MaterialDeliveryMock::snapshot(
  Clock::time_point now, const builtin_interfaces::msg::Time & stamp)
{
  if (!active_delivery_) {
    return std::nullopt;
  }
  if (phase_ != HandoffState::HANDOFF_FAILED &&
    phase_ != HandoffState::HANDOFF_DEPARTED && now - phase_started_ >= phase_period_)
  {
    ++phase_;
    phase_started_ = now;
  }

  HandoffState message;
  message.header.stamp = stamp;
  message.delivery_id = *active_delivery_;
  message.phase = phase_;
  message.valid = phase_ != HandoffState::HANDOFF_FAILED;
  if (phase_ == HandoffState::HANDOFF_FAILED ||
    phase_ == HandoffState::HANDOFF_DEPARTED)
  {
    active_delivery_.reset();
  }
  return message;
}

}  // namespace arm_cell_vda_adapter
