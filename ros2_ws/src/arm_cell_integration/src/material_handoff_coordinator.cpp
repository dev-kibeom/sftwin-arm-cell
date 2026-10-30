#include "arm_cell_integration/material_handoff_coordinator.hpp"

#include <algorithm>
#include <stdexcept>

namespace arm_cell_integration
{

MaterialHandoffCoordinator::MaterialHandoffCoordinator(
  std::chrono::milliseconds transfer_duration,
  std::chrono::milliseconds source_freshness_timeout)
: transfer_duration_(transfer_duration), source_freshness_timeout_(source_freshness_timeout)
{
  if (transfer_duration_.count() <= 0 || source_freshness_timeout_.count() <= 0) {
    throw std::invalid_argument("material transfer and freshness bounds must be positive");
  }
}

bool MaterialHandoffCoordinator::valid_id(const DeliveryId & delivery_id)
{
  return std::any_of(
    delivery_id.uuid.begin(), delivery_id.uuid.end(), [](uint8_t byte) {return byte != 0;});
}

bool MaterialHandoffCoordinator::can_begin(const DeliveryId & delivery_id) const
{
  return valid_id(delivery_id) && seen_deliveries_.count(delivery_id.uuid) == 0 &&
         (!active_delivery_ || terminal_failure_);
}

bool MaterialHandoffCoordinator::begin(const DeliveryId & delivery_id, Clock::time_point)
{
  if (!can_begin(delivery_id)) {
    return false;
  }
  pending_readiness_.clear();
  seen_deliveries_.insert(delivery_id.uuid);
  active_delivery_ = delivery_id;
  last_phase_ = HandoffState::HANDOFF_UNKNOWN;
  has_source_ = false;
  transfer_pending_ = false;
  readiness_published_ = false;
  terminal_failure_ = false;
  stale_reported_ = false;
  return true;
}

void MaterialHandoffCoordinator::reject_unaccepted(const DeliveryId & delivery_id)
{
  if (active_delivery_ && active_delivery_->uuid == delivery_id.uuid && !has_source_) {
    seen_deliveries_.erase(delivery_id.uuid);
    active_delivery_.reset();
    last_phase_ = HandoffState::HANDOFF_UNKNOWN;
    has_source_ = false;
    last_source_receipt_ = Clock::time_point{};
    transfer_started_ = Clock::time_point{};
    transfer_pending_ = false;
    readiness_published_ = false;
    terminal_failure_ = false;
    stale_reported_ = false;
  }
}

std::optional<MaterialHandoffCoordinator::Readiness> MaterialHandoffCoordinator::make_readiness(
  const DeliveryId & delivery_id, bool material_ready, bool valid,
  const builtin_interfaces::msg::Time & stamp) const
{
  Readiness message;
  message.header.stamp = stamp;
  message.delivery_id = delivery_id;
  message.material_ready = material_ready;
  message.valid = valid;
  return message;
}

std::optional<MaterialHandoffCoordinator::Readiness> MaterialHandoffCoordinator::fail(
  bool valid, const builtin_interfaces::msg::Time & stamp)
{
  if (terminal_failure_) {
    return std::nullopt;
  }
  terminal_failure_ = true;
  transfer_pending_ = false;
  return active_delivery_ ?
         make_readiness(*active_delivery_, false, valid, stamp) : std::nullopt;
}

std::optional<MaterialHandoffCoordinator::Readiness> MaterialHandoffCoordinator::observe(
  const HandoffState & state, Clock::time_point receipt_time,
  const builtin_interfaces::msg::Time & publication_time)
{
  if (!active_delivery_ || state.delivery_id.uuid != active_delivery_->uuid || terminal_failure_) {
    return std::nullopt;
  }
  last_source_receipt_ = receipt_time;
  has_source_ = true;
  if (!state.valid) {
    return fail(false, publication_time);
  }
  if (state.phase == HandoffState::HANDOFF_FAILED ||
    state.phase == HandoffState::HANDOFF_UNKNOWN)
  {
    return fail(true, publication_time);
  }
  if (state.phase == last_phase_) {
    return std::nullopt;
  }

  uint8_t expected_phase = HandoffState::HANDOFF_ARRIVED;
  switch (last_phase_) {
    case HandoffState::HANDOFF_ARRIVED:
      expected_phase = HandoffState::HANDOFF_DOCKED;
      break;
    case HandoffState::HANDOFF_DOCKED:
      expected_phase = HandoffState::HANDOFF_UNLOADED;
      break;
    case HandoffState::HANDOFF_UNLOADED:
      expected_phase = HandoffState::HANDOFF_DEPARTED;
      break;
    case HandoffState::HANDOFF_UNKNOWN:
      break;
    default:
      return fail(false, publication_time);
  }
  if (state.phase != expected_phase) {
    return fail(false, publication_time);
  }
  last_phase_ = state.phase;
  if (state.phase == HandoffState::HANDOFF_UNLOADED) {
    transfer_started_ = receipt_time;
    transfer_pending_ = true;
  } else if (state.phase == HandoffState::HANDOFF_DEPARTED) {
    if (transfer_pending_ && !readiness_published_) {
      pending_readiness_.push_back(
        PendingReadiness{*active_delivery_, transfer_started_ + transfer_duration_});
    }
    active_delivery_.reset();
    has_source_ = false;
    transfer_pending_ = false;
  }
  return std::nullopt;
}

std::optional<MaterialHandoffCoordinator::Readiness> MaterialHandoffCoordinator::tick(
  Clock::time_point now, const builtin_interfaces::msg::Time & publication_time)
{
  if (active_delivery_ && !terminal_failure_ && has_source_ && !stale_reported_ &&
    now - last_source_receipt_ > source_freshness_timeout_)
  {
    stale_reported_ = true;
    readiness_published_ = false;
    return fail(false, publication_time);
  }
  if (!pending_readiness_.empty() && now >= pending_readiness_.front().ready_at) {
    const auto delivery_id = pending_readiness_.front().delivery_id;
    pending_readiness_.pop_front();
    return make_readiness(delivery_id, true, true, publication_time);
  }
  if (!active_delivery_ || terminal_failure_ || !has_source_) {
    return std::nullopt;
  }
  if (transfer_pending_ && !readiness_published_ &&
    now - transfer_started_ >= transfer_duration_)
  {
    readiness_published_ = true;
    transfer_pending_ = false;
    return make_readiness(*active_delivery_, true, true, publication_time);
  }
  return std::nullopt;
}

}  // namespace arm_cell_integration
