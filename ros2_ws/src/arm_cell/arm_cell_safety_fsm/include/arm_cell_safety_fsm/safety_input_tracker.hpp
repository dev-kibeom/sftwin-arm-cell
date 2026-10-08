#pragma once

#include <array>
#include <chrono>
#include <vector>

namespace arm_cell_safety_fsm
{

enum class SafetyInput : std::size_t
{
  AMR = 0,
  PACKML,
  HARDWARE,
  MOTION,
  COUNT
};

class SafetyInputTracker
{
public:
  using Clock = std::chrono::steady_clock;

  explicit SafetyInputTracker(std::chrono::milliseconds timeout)
  : timeout_(timeout) {}

  void record(
    SafetyInput input,
    bool message_valid,
    bool source_timestamp_plausible,
    Clock::time_point receipt_time)
  {
    records_[index(input)] = Record{true, message_valid, source_timestamp_plausible, receipt_time};
  }

  bool received(SafetyInput input) const {return records_[index(input)].received;}

  bool all_received_valid() const
  {
    for (std::size_t i = 0; i < index(SafetyInput::COUNT); ++i) {
      const auto & record = records_[i];
      if (!record.received || !record.message_valid || !record.source_timestamp_plausible) {
        return false;
      }
    }
    return true;
  }

  bool any_received_invalid() const
  {
    for (std::size_t i = 0; i < index(SafetyInput::COUNT); ++i) {
      const auto & record = records_[i];
      if (record.received && (!record.message_valid || !record.source_timestamp_plausible)) {
        return true;
      }
    }
    return false;
  }

  bool is_fresh(SafetyInput input, Clock::time_point now) const
  {
    const auto & record = records_[index(input)];
    if (!record.received || !record.message_valid || !record.source_timestamp_plausible ||
      now < record.receipt_time)
    {
      return false;
    }
    return now - record.receipt_time <= timeout_;
  }

  bool all_required_fresh(Clock::time_point now) const
  {
    for (std::size_t i = 0; i < index(SafetyInput::COUNT); ++i) {
      if (!is_fresh(static_cast<SafetyInput>(i), now)) {
        return false;
      }
    }
    return true;
  }

  bool any_in_degraded_band(
    Clock::time_point now, std::chrono::milliseconds degraded_threshold,
    const std::vector<SafetyInput> & configured_inputs) const
  {
    if (degraded_threshold.count() <= 0 || configured_inputs.empty()) {
      return false;
    }
    for (const auto input : configured_inputs) {
      if (input == SafetyInput::COUNT) {
        continue;
      }
      const auto i = index(input);
      const auto & record = records_[i];
      if (
        is_fresh(input, now) &&
        now - record.receipt_time > degraded_threshold)
      {
        return true;
      }
    }
    return false;
  }

private:
  struct Record
  {
    bool received{false};
    bool message_valid{false};
    bool source_timestamp_plausible{false};
    Clock::time_point receipt_time{};
  };

  static constexpr std::size_t index(SafetyInput input)
  {
    return static_cast<std::size_t>(input);
  }

  std::chrono::milliseconds timeout_;
  std::array<Record, static_cast<std::size_t>(SafetyInput::COUNT)> records_{};
};

}  // namespace arm_cell_safety_fsm
