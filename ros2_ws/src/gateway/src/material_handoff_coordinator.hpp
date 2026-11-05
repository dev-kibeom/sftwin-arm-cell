#pragma once

#include <array>
#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <arm_cell_interfaces/msg/handoff_trigger.hpp>
#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/transfer_completion.hpp>

namespace arm_cell_gateway
{

class MaterialHandoffCoordinator final
{
public:
  using Clock = std::chrono::steady_clock;
  using Trigger = arm_cell_interfaces::msg::HandoffTrigger;
  using HandoffState = arm_cell_interfaces::msg::MaterialHandoffState;
  using Completion = arm_cell_interfaces::msg::TransferCompletion;
  using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
  using DeliveryId = unique_identifier_msgs::msg::UUID;
  using DeliveryKey = std::array<uint8_t, 16>;

  enum class CompletionDisposition
  {
    kNoActiveDelivery,
    kDeliveryMismatch,
    kDeadlineExpired,
    kPreviousTerminalFailure,
    kMetadataMismatch,
    kInvalidObservation,
    kTransferFailure,
    kQuantityMismatch,
    kCompleted
  };

  struct CompletionObservation
  {
    CompletionDisposition disposition{CompletionDisposition::kNoActiveDelivery};
    std::optional<DeliveryId> expected_delivery_id;
    uint8_t failure_reason{Completion::TRANSFER_FAILURE_NONE};
    std::optional<Readiness> readiness;
  };

  explicit MaterialHandoffCoordinator(std::chrono::milliseconds delivery_deadline);

  bool can_begin(const Trigger & trigger) const;
  bool can_begin(const DeliveryId & delivery_id) const;
  std::string rejection_reason(const Trigger & trigger) const;
  bool begin(const Trigger & trigger, Clock::time_point receipt_time);
  std::optional<Readiness> observe_vda(
    const HandoffState & state, Clock::time_point receipt_time,
    const builtin_interfaces::msg::Time & publication_time);
  CompletionObservation observe_completion(
    const Completion & completion, Clock::time_point receipt_time,
    const builtin_interfaces::msg::Time & publication_time);
  std::optional<Readiness> tick(
    Clock::time_point now, const builtin_interfaces::msg::Time & publication_time);

private:
  struct DeliveryState
  {
    Trigger trigger;
    uint8_t last_phase{HandoffState::HANDOFF_UNKNOWN};
    Clock::time_point deadline_at{};
    bool has_vda{false};
    bool has_completion{false};
    bool readiness_published{false};
    bool terminal_failure{false};
  };

  static bool valid_id(const DeliveryId & delivery_id);
  static DeliveryKey key(const DeliveryId & delivery_id);
  static bool completion_matches(const Completion & completion, const Trigger & trigger);
  std::optional<Readiness> make_readiness(
    const DeliveryState & delivery, bool material_ready, bool valid,
    const builtin_interfaces::msg::Time & stamp) const;
  std::optional<Readiness> fail(
    DeliveryState & delivery, bool valid, const builtin_interfaces::msg::Time & stamp);
  std::optional<Readiness> ready_if_complete(
    DeliveryState & delivery, const builtin_interfaces::msg::Time & stamp);

  std::chrono::milliseconds delivery_deadline_;
  std::map<DeliveryKey, DeliveryState> deliveries_;
};

}  // namespace arm_cell_gateway
