#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include "arm_cell_orchestration/safety_observation.hpp"

namespace arm_cell_orchestration
{

class MaterialAdmissionGate
{
public:
  using Clock = std::chrono::steady_clock;
  using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
  using Safety = SafetyObservation;
  using DeliveryId = unique_identifier_msgs::msg::UUID;

  struct Admission
  {
    DeliveryId delivery_id;
    std::string material_type;
    uint32_t quantity{0};
  };

  enum class PlaceCommitResult
  {
    COMMITTED,
    DUPLICATE,
    ACCOUNTING_INCONSISTENT
  };

  bool observe(const Readiness & state)
  {
    const auto key = identity_key(state.delivery_id);
    if (key.empty()) {
      return false;
    }
    current_delivery_key_ = key;
    auto [entry, inserted] = records_.try_emplace(key);
    if (inserted) {
      delivery_order_.push_back(key);
    }
    auto & record = entry->second;
    if (is_completion(record.state)) {
      return true;
    }
    record.state = state;
    if (is_completion(state) && !record.supply_counted) {
      total_supplied_ += state.quantity;
      record.supplied_quantity = state.quantity;
      record.supply_counted = true;
    }
    return true;
  }

  bool has_admitted_delivery(
    const DeliveryId & delivery_id, const std::string & material_type) const
  {
    const auto key = identity_key(delivery_id);
    const auto record = records_.find(key);
    return !key.empty() && record != records_.end() && record->second.admitted &&
           record->second.state.material_type == material_type &&
           is_completion(record->second.state);
  }

  // Commit a monotonic cycle-local count only after Motion confirms PLACE/RELEASED.
  PlaceCommitResult record_successful_place(const std::string & cycle_key, uint64_t count)
  {
    if (cycle_key.empty() || count == 0U) {
      accounting_consistent_ = false;
      return PlaceCommitResult::ACCOUNTING_INCONSISTENT;
    }
    const auto previous = cycle_processed_counts_.find(cycle_key);
    const auto prior_count = previous == cycle_processed_counts_.end() ? 0U : previous->second;
    if (count == prior_count && prior_count != 0U) {
      return PlaceCommitResult::DUPLICATE;
    }
    if (count < prior_count || total_placed_ > total_supplied_) {
      accounting_consistent_ = false;
      return PlaceCommitResult::ACCOUNTING_INCONSISTENT;
    }
    const auto delta = count - prior_count;
    if (delta > total_supplied_ - total_placed_) {
      accounting_consistent_ = false;
      return PlaceCommitResult::ACCOUNTING_INCONSISTENT;
    }

    auto remaining = delta;
    for (const auto & key : delivery_order_) {
      auto & record = records_.at(key);
      const auto available = static_cast<uint64_t>(record.supplied_quantity - record.placed_quantity);
      const auto assigned = std::min(remaining, available);
      record.placed_quantity += static_cast<uint32_t>(assigned);
      remaining -= assigned;
      if (record.supply_counted && record.placed_quantity == record.supplied_quantity) {
        record.admitted = true;
      }
      if (remaining == 0U) {
        break;
      }
    }
    if (remaining != 0U) {
      accounting_consistent_ = false;
      return PlaceCommitResult::ACCOUNTING_INCONSISTENT;
    }
    total_placed_ += delta;
    cycle_processed_counts_[cycle_key] = count;
    return PlaceCommitResult::COMMITTED;
  }

  std::optional<Admission> try_admit(
    const Safety & safety,
    const std::string & supported_material_type,
    std::string * block_reason = nullptr,
    DeliveryId * current_delivery_id = nullptr,
    const std::string & prerequisite_block_reason = {})
  {
    if (block_reason) {
      block_reason->clear();
    }
    if (current_delivery_id) {
      *current_delivery_id = DeliveryId{};
      const auto candidate = pending_delivery_key(supported_material_type);
      const auto key = candidate.empty() ? current_delivery_key_ : candidate;
      if (!key.empty()) {
        *current_delivery_id = records_.at(key).state.delivery_id;
      }
    }
    const auto block = [block_reason](const char * reason) {
        if (block_reason) {
          *block_reason = reason;
        }
        return std::optional<Admission>{};
      };
    if (!accounting_consistent_) {
      return block("material_accounting_inconsistent");
    }
    if (!safety.received || !safety.delivery_valid) {
      return block("safety_permission_unavailable");
    }
    if (!safety.normal_motion_allowed()) {
      return block("safety_motion_limited");
    }

    const auto key = pending_delivery_key(supported_material_type);
    if (key.empty()) {
      if (current_delivery_key_.empty()) {
        return block("readiness_unavailable");
      }
      const auto & current = records_.at(current_delivery_key_);
      if (current.admitted || is_completion(current.state)) {
        if (is_completion(current.state) && !current.admitted &&
          current.state.material_type != supported_material_type)
        {
          return block("readiness_material_type_unsupported");
        }
        return block("readiness_already_admitted");
      }
      if (!current.state.valid) {
        return block("readiness_invalid");
      }
      if (!current.state.material_ready) {
        return block("readiness_not_ready");
      }
      return block("readiness_fields_invalid");
    }
    auto & record = records_.at(key);
    const auto outstanding = total_supplied_ - total_placed_;
    if (outstanding == 0U || outstanding > std::numeric_limits<uint32_t>::max()) {
      return block("readiness_fields_invalid");
    }
    if (!prerequisite_block_reason.empty()) {
      return block(prerequisite_block_reason.c_str());
    }
    record.admitted = true;
    return Admission{record.state.delivery_id, record.state.material_type,
      static_cast<uint32_t>(outstanding)};
  }

private:
  struct Record
  {
    Readiness state;
    bool admitted{false};
    bool supply_counted{false};
    uint32_t supplied_quantity{0U};
    uint32_t placed_quantity{0U};
  };

  static bool is_completion(const Readiness & state)
  {
    return state.valid && state.material_ready && !state.material_type.empty() &&
           state.quantity > 0U;
  }

  std::string pending_delivery_key(const std::string & supported_material_type) const
  {
    for (const auto & key : delivery_order_) {
      const auto & record = records_.at(key);
      if (is_completion(record.state) && !record.admitted &&
        record.state.material_type == supported_material_type)
      {
        return key;
      }
    }
    return {};
  }

  static std::string identity_key(const DeliveryId & id)
  {
    bool all_zero = true;
    std::string key;
    key.reserve(id.uuid.size());
    for (const auto byte : id.uuid) {
      all_zero = all_zero && byte == 0;
      key.push_back(static_cast<char>(byte));
    }
    return all_zero ? std::string{} : key;
  }

  std::unordered_map<std::string, Record> records_;
  std::vector<std::string> delivery_order_;
  std::unordered_map<std::string, uint64_t> cycle_processed_counts_;
  uint64_t total_supplied_{0U};
  uint64_t total_placed_{0U};
  bool accounting_consistent_{true};
  std::string current_delivery_key_;
};

}  // namespace arm_cell_orchestration
