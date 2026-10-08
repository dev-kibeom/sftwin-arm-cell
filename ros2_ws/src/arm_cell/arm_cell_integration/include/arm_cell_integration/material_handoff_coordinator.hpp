#pragma once

#include <array>
#include <chrono>
#include <deque>
#include <optional>
#include <set>

#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>

namespace arm_cell_integration
{

class MaterialHandoffCoordinator final
{
public:
  using Clock = std::chrono::steady_clock;
  using HandoffState = arm_cell_interfaces::msg::MaterialHandoffState;
  using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
  using DeliveryId = unique_identifier_msgs::msg::UUID;

  MaterialHandoffCoordinator(
    std::chrono::milliseconds transfer_duration,
    std::chrono::milliseconds source_freshness_timeout);

  bool begin(const DeliveryId & delivery_id, Clock::time_point now);
  bool can_begin(const DeliveryId & delivery_id) const;
  // Roll back pre-arm only if no VDA state was observed for this identity.
  void reject_unaccepted(const DeliveryId & delivery_id);
  std::optional<Readiness> observe(
    const HandoffState & state, Clock::time_point receipt_time,
    const builtin_interfaces::msg::Time & publication_time);
  std::optional<Readiness> tick(
    Clock::time_point now, const builtin_interfaces::msg::Time & publication_time);

private:
  struct PendingReadiness
  {
    DeliveryId delivery_id;
    Clock::time_point ready_at;
  };

  static bool valid_id(const DeliveryId & delivery_id);
  std::optional<Readiness> make_readiness(
    const DeliveryId & delivery_id, bool material_ready, bool valid,
    const builtin_interfaces::msg::Time & stamp) const;
  std::optional<Readiness> fail(
    bool valid, const builtin_interfaces::msg::Time & stamp);

  std::chrono::milliseconds transfer_duration_;
  std::chrono::milliseconds source_freshness_timeout_;
  std::optional<DeliveryId> active_delivery_;
  std::deque<PendingReadiness> pending_readiness_;
  std::set<std::array<uint8_t, 16>> seen_deliveries_;
  uint8_t last_phase_{HandoffState::HANDOFF_UNKNOWN};
  Clock::time_point last_source_receipt_{};
  Clock::time_point transfer_started_{};
  bool has_source_{false};
  bool transfer_pending_{false};
  bool readiness_published_{false};
  bool terminal_failure_{false};
  bool stale_reported_{false};
};

}  // namespace arm_cell_integration
