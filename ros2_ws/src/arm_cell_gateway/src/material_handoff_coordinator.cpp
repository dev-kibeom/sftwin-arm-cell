#include "material_handoff_coordinator.hpp"

#include <algorithm>
#include <stdexcept>

namespace arm_cell_gateway
{

MaterialHandoffCoordinator::MaterialHandoffCoordinator(
  std::chrono::milliseconds delivery_deadline)
: delivery_deadline_(delivery_deadline)
{
  if (delivery_deadline_.count() <= 0) {
    throw std::invalid_argument("material delivery deadline must be positive");
  }
}

bool MaterialHandoffCoordinator::valid_id(const DeliveryId & delivery_id)
{
  return std::any_of(
    delivery_id.uuid.begin(), delivery_id.uuid.end(), [](uint8_t byte) {return byte != 0;});
}

MaterialHandoffCoordinator::DeliveryKey MaterialHandoffCoordinator::key(
  const DeliveryId & delivery_id)
{
  return delivery_id.uuid;
}

bool MaterialHandoffCoordinator::can_begin(const Trigger & trigger) const
{
  return trigger.valid && valid_id(trigger.delivery_id) && trigger.quantity > 0 &&
         trigger.material_type == "RawPart" && can_begin(trigger.delivery_id);
}

bool MaterialHandoffCoordinator::can_begin(const DeliveryId & delivery_id) const
{
  return valid_id(delivery_id) && deliveries_.count(key(delivery_id)) == 0;
}

std::string MaterialHandoffCoordinator::rejection_reason(const Trigger & trigger) const
{
  if (!trigger.valid) {
    return "invalid_trigger";
  }
  if (!valid_id(trigger.delivery_id)) {
    return "invalid_delivery_id";
  }
  if (trigger.quantity == 0) {
    return "invalid_quantity";
  }
  if (trigger.material_type != "RawPart") {
    return "unsupported_material_type";
  }
  if (deliveries_.count(key(trigger.delivery_id)) != 0) {
    return "duplicate_delivery_id";
  }
  return {};
}

bool MaterialHandoffCoordinator::begin(
  const Trigger & trigger, Clock::time_point receipt_time)
{
  if (!can_begin(trigger)) {
    return false;
  }
  DeliveryState delivery;
  delivery.trigger = trigger;
  delivery.deadline_at = receipt_time + delivery_deadline_;
  deliveries_.emplace(key(trigger.delivery_id), std::move(delivery));
  return true;
}

std::optional<MaterialHandoffCoordinator::Readiness>
MaterialHandoffCoordinator::make_readiness(
  const DeliveryState & delivery, bool material_ready, bool valid,
  const builtin_interfaces::msg::Time & stamp) const
{
  Readiness message;
  message.header.stamp = stamp;
  message.delivery_id = delivery.trigger.delivery_id;
  message.material_type = delivery.trigger.material_type;
  message.quantity = delivery.trigger.quantity;
  message.material_ready = material_ready;
  message.valid = valid;
  return message;
}

std::optional<MaterialHandoffCoordinator::Readiness>
MaterialHandoffCoordinator::fail(
  DeliveryState & delivery, bool valid, const builtin_interfaces::msg::Time & stamp)
{
  if (delivery.terminal_failure || delivery.readiness_published) {
    return std::nullopt;
  }
  delivery.terminal_failure = true;
  delivery.readiness_published = true;
  return make_readiness(delivery, false, valid, stamp);
}

bool MaterialHandoffCoordinator::completion_matches(
  const Completion & completion, const Trigger & trigger)
{
  return completion.delivery_id.uuid == trigger.delivery_id.uuid &&
         completion.material_type == trigger.material_type &&
         completion.requested_quantity == trigger.quantity;
}

std::optional<MaterialHandoffCoordinator::Readiness>
MaterialHandoffCoordinator::ready_if_complete(
  DeliveryState & delivery, const builtin_interfaces::msg::Time & stamp)
{
  if (delivery.terminal_failure || delivery.readiness_published ||
    !delivery.has_vda || !delivery.has_completion ||
    delivery.last_phase < HandoffState::HANDOFF_UNLOADED)
  {
    return std::nullopt;
  }
  delivery.readiness_published = true;
  return make_readiness(delivery, true, true, stamp);
}

std::optional<MaterialHandoffCoordinator::Readiness>
MaterialHandoffCoordinator::observe_vda(
  const HandoffState & state, Clock::time_point receipt_time,
  const builtin_interfaces::msg::Time & publication_time)
{
  const auto found = deliveries_.find(key(state.delivery_id));
  if (found == deliveries_.end()) {
    return std::nullopt;
  }
  auto & delivery = found->second;
  if (delivery.terminal_failure || delivery.readiness_published) {
    return std::nullopt;
  }
  if (receipt_time >= delivery.deadline_at) {
    return fail(delivery, false, publication_time);
  }
  delivery.has_vda = true;
  if (!state.valid) {
    return fail(delivery, false, publication_time);
  }
  if (state.phase == HandoffState::HANDOFF_FAILED ||
    state.phase == HandoffState::HANDOFF_UNKNOWN)
  {
    return fail(delivery, true, publication_time);
  }
  if (state.phase == delivery.last_phase) {
    return std::nullopt;
  }
  const auto expected = static_cast<uint8_t>(delivery.last_phase + 1U);
  if (state.phase != expected || state.phase > HandoffState::HANDOFF_DEPARTED) {
    return fail(delivery, false, publication_time);
  }
  delivery.last_phase = state.phase;
  return ready_if_complete(delivery, publication_time);
}

MaterialHandoffCoordinator::CompletionObservation
MaterialHandoffCoordinator::observe_completion(
  const Completion & completion, Clock::time_point receipt_time,
  const builtin_interfaces::msg::Time & publication_time)
{
  CompletionObservation observation;
  const auto found = deliveries_.find(key(completion.delivery_id));
  if (found == deliveries_.end()) {
    observation.disposition = deliveries_.empty() ?
      CompletionDisposition::kNoActiveDelivery : CompletionDisposition::kDeliveryMismatch;
    return observation;
  }
  auto & delivery = found->second;
  observation.expected_delivery_id = delivery.trigger.delivery_id;
  if (delivery.terminal_failure || delivery.readiness_published) {
    observation.disposition = CompletionDisposition::kPreviousTerminalFailure;
    return observation;
  }
  if (receipt_time >= delivery.deadline_at) {
    observation.disposition = CompletionDisposition::kDeadlineExpired;
    observation.readiness = fail(delivery, false, publication_time);
    return observation;
  }
  if (!completion_matches(completion, delivery.trigger)) {
    observation.disposition = CompletionDisposition::kMetadataMismatch;
    observation.readiness = fail(delivery, false, publication_time);
    return observation;
  }
  if (!completion.valid) {
    observation.disposition = CompletionDisposition::kInvalidObservation;
    observation.readiness = fail(delivery, false, publication_time);
    return observation;
  }
  if (!completion.completed || completion.failure_reason != Completion::TRANSFER_FAILURE_NONE) {
    observation.disposition = CompletionDisposition::kTransferFailure;
    observation.failure_reason = completion.failure_reason;
    observation.readiness = fail(delivery, true, publication_time);
    return observation;
  }
  if (completion.supplied_quantity != delivery.trigger.quantity)
  {
    observation.disposition = CompletionDisposition::kQuantityMismatch;
    observation.readiness = fail(delivery, true, publication_time);
    return observation;
  }
  delivery.has_completion = true;
  observation.disposition = CompletionDisposition::kCompleted;
  observation.readiness = ready_if_complete(delivery, publication_time);
  return observation;
}

std::optional<MaterialHandoffCoordinator::Readiness>
MaterialHandoffCoordinator::tick(
  Clock::time_point now, const builtin_interfaces::msg::Time & publication_time)
{
  for (auto & [delivery_id, delivery] : deliveries_) {
    (void)delivery_id;
    if (delivery.terminal_failure || delivery.readiness_published) {
      continue;
    }
    if (now >= delivery.deadline_at) {
      auto readiness = fail(delivery, false, publication_time);
      if (readiness) {
        return readiness;
      }
    } else {
      auto readiness = ready_if_complete(delivery, publication_time);
      if (readiness) {
        return readiness;
      }
    }
  }
  return std::nullopt;
}

}  // namespace arm_cell_gateway
