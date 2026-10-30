#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/motion_capability.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>

namespace arm_cell_orchestration_bt
{

class MaterialAdmissionGate
{
public:
  using Clock = std::chrono::steady_clock;
  using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
  using SafetyState = arm_cell_interfaces::msg::SafetyState;
  using DeliveryId = unique_identifier_msgs::msg::UUID;

  bool observe(
    const Readiness & state, Clock::time_point receipt_time,
    Clock::duration source_age)
  {
    const auto key = identity_key(state.delivery_id);
    if (key.empty()) {
      return false;
    }
    current_delivery_key_ = key;
    auto & record = records_[key];
    record.state = state;
    record.receipt_time = receipt_time;
    record.source_age = source_age;
    record.observed = true;
    return true;
  }

  std::optional<DeliveryId> try_admit(
    Clock::time_point now,
    const SafetyState & safety,
    Clock::time_point safety_receipt,
    Clock::duration freshness_timeout)
  {
    if (now < safety_receipt || now - safety_receipt >= freshness_timeout ||
      !safety.valid || !safety.required_inputs_fresh ||
      safety.safety_state != SafetyState::SAFETY_STATE_SAFE ||
      safety.motion_capability.value !=
      arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL)
    {
      return std::nullopt;
    }

    if (current_delivery_key_.empty()) {
      return std::nullopt;
    }
    auto & record = records_.at(current_delivery_key_);
    const auto local_age = now - record.receipt_time;
    if (!record.observed || record.admitted || !record.state.valid ||
      !record.state.material_ready || now < record.receipt_time ||
      record.source_age < Clock::duration::zero() ||
      record.source_age + local_age >= freshness_timeout)
    {
      return std::nullopt;
    }
    record.admitted = true;
    return record.state.delivery_id;
  }

private:
  struct Record
  {
    Readiness state;
    Clock::time_point receipt_time{};
    Clock::duration source_age{};
    bool observed{false};
    bool admitted{false};
  };

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
  std::string current_delivery_key_;
};

}  // namespace arm_cell_orchestration_bt
