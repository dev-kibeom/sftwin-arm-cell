#pragma once

#include <array>
#include <chrono>
#include <cstddef>

namespace arm_cell_safety_fsm
{

enum class SafetyInput : std::size_t
{
  HARDWARE = 0,
  MOTION,
  COUNT
};

enum class SafetyInputReason
{
  NOT_RECEIVED,
  INVALID,
  STALE,
  RECEIPT_TIME_INVALID,
  OK
};

inline const char * safety_input_name(SafetyInput input)
{
  switch (input) {
    case SafetyInput::HARDWARE: return "HARDWARE";
    case SafetyInput::MOTION: return "MOTION";
    case SafetyInput::COUNT: return "UNKNOWN";
  }
  return "UNKNOWN";
}

inline const char * safety_input_reason_name(SafetyInputReason reason)
{
  switch (reason) {
    case SafetyInputReason::NOT_RECEIVED: return "NOT_RECEIVED";
    case SafetyInputReason::INVALID: return "INVALID";
    case SafetyInputReason::STALE: return "STALE";
    case SafetyInputReason::RECEIPT_TIME_INVALID: return "RECEIPT_TIME_INVALID";
    case SafetyInputReason::OK: return "OK";
  }
  return "UNKNOWN";
}

struct SafetyInputDiagnostic
{
  bool received{false};
  bool message_valid{false};
  bool source_timestamp_plausible{false};
  bool age_available{false};
  std::chrono::milliseconds receipt_age{0};
  std::int64_t receipt_timeout_ns{0};
  std::int64_t source_stamp_ns{0};
  std::int64_t source_evaluation_ros_time_ns{0};
  std::int64_t source_age_ns{0};
  std::int64_t source_maximum_age_ns{0};
  bool source_time_available{false};
  SafetyInputReason reason{SafetyInputReason::NOT_RECEIVED};
};

template<std::size_t N>
bool safety_input_decision_changed(
  const std::array<SafetyInputDiagnostic, N> & current,
  const std::array<SafetyInputDiagnostic, N> & previous)
{
  for (std::size_t index = 0; index < N; ++index) {
    const auto & lhs = current[index];
    const auto & rhs = previous[index];
    if (lhs.received != rhs.received || lhs.message_valid != rhs.message_valid ||
      lhs.source_timestamp_plausible != rhs.source_timestamp_plausible ||
      lhs.age_available != rhs.age_available || lhs.reason != rhs.reason)
    {
      return true;
    }
  }
  return false;
}

inline bool safety_diagnostic_record_required(
  bool diagnostics_enabled, bool safety_state_changed, bool input_decision_changed)
{
  return diagnostics_enabled && (safety_state_changed || input_decision_changed);
}

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
    Clock::time_point receipt_time,
    std::int64_t source_stamp_ns = 0,
    std::int64_t source_evaluation_ros_time_ns = 0,
    std::int64_t source_maximum_age_ns = 0)
  {
    records_[index(input)] = Record{
      true, message_valid, source_timestamp_plausible, receipt_time,
      source_stamp_ns, source_evaluation_ros_time_ns, source_maximum_age_ns};
  }

  bool received(SafetyInput input) const {return records_[index(input)].received;}

  SafetyInputDiagnostic diagnostic(SafetyInput input, Clock::time_point now) const
  {
    const auto & record = records_[index(input)];
    SafetyInputDiagnostic result;
    result.received = record.received;
    result.message_valid = record.message_valid;
    result.source_timestamp_plausible = record.source_timestamp_plausible;
    result.receipt_timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      timeout_).count();
    result.source_stamp_ns = record.source_stamp_ns;
    result.source_evaluation_ros_time_ns = record.source_evaluation_ros_time_ns;
    result.source_maximum_age_ns = record.source_maximum_age_ns;
    result.source_time_available = record.received;
    if (record.received) {
      result.source_age_ns = record.source_evaluation_ros_time_ns - record.source_stamp_ns;
    }
    if (record.received && now >= record.receipt_time) {
      result.age_available = true;
      result.receipt_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - record.receipt_time);
    }
    if (!record.received) {
      result.reason = SafetyInputReason::NOT_RECEIVED;
    } else if (!record.message_valid) {
      result.reason = SafetyInputReason::INVALID;
    } else if (now < record.receipt_time) {
      result.reason = SafetyInputReason::RECEIPT_TIME_INVALID;
    } else {
      if (result.receipt_age > timeout_) {
        result.reason = SafetyInputReason::STALE;
      } else {
        result.reason = SafetyInputReason::OK;
      }
    }
    return result;
  }

  std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)>
  diagnostics(Clock::time_point now) const
  {
    std::array<SafetyInputDiagnostic, static_cast<std::size_t>(SafetyInput::COUNT)> result{};
    for (std::size_t i = 0; i < result.size(); ++i) {
      result[i] = diagnostic(static_cast<SafetyInput>(i), now);
    }
    return result;
  }

  bool all_received_valid() const
  {
    for (std::size_t i = 0; i < index(SafetyInput::COUNT); ++i) {
      const auto & record = records_[i];
      if (!record.received || !record.message_valid) {
        return false;
      }
    }
    return true;
  }

  bool any_received_invalid() const
  {
    for (std::size_t i = 0; i < index(SafetyInput::COUNT); ++i) {
      const auto & record = records_[i];
      if (record.received && !record.message_valid) {
        return true;
      }
    }
    return false;
  }

  bool is_fresh(SafetyInput input, Clock::time_point now) const
  {
    const auto & record = records_[index(input)];
    if (!record.received || !record.message_valid || now < record.receipt_time) {
      return false;
    }
    return now - record.receipt_time <= timeout_;
  }

  bool all_required_fresh(Clock::time_point now) const
  {
    for (const auto input : {SafetyInput::HARDWARE, SafetyInput::MOTION}) {
      if (!is_fresh(input, now)) {
        return false;
      }
    }
    return true;
  }

private:
  struct Record
  {
    bool received{false};
    bool message_valid{false};
    bool source_timestamp_plausible{false};
    Clock::time_point receipt_time{};
    std::int64_t source_stamp_ns{0};
    std::int64_t source_evaluation_ros_time_ns{0};
    std::int64_t source_maximum_age_ns{0};
  };

  static constexpr std::size_t index(SafetyInput input)
  {
    return static_cast<std::size_t>(input);
  }

  std::chrono::milliseconds timeout_;
  std::array<Record, static_cast<std::size_t>(SafetyInput::COUNT)> records_{};
};

}  // namespace arm_cell_safety_fsm
