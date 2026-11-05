#pragma once

#include <chrono>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

namespace arm_cell_safety_fsm
{

enum class SourceTimestampQualityEvent {NONE, ANOMALY_ENTERED, RECOVERED};

class SourceTimestampAnomalyTracker
{
public:
  SourceTimestampQualityEvent observe(bool plausible) noexcept
  {
    if (!plausible && !anomaly_active_) {
      anomaly_active_ = true;
      return SourceTimestampQualityEvent::ANOMALY_ENTERED;
    }
    if (plausible && anomaly_active_) {
      anomaly_active_ = false;
      return SourceTimestampQualityEvent::RECOVERED;
    }
    return SourceTimestampQualityEvent::NONE;
  }

private:
  bool anomaly_active_{false};
};

inline void log_source_timestamp_quality_event(
  const rclcpp::Logger & logger, SourceTimestampQualityEvent event, const char * source)
{
  switch (event) {
    case SourceTimestampQualityEvent::ANOMALY_ENTERED:
      RCLCPP_WARN(
        logger,
        "%s source timestamp became implausible; timestamp quality remains diagnostic-only",
        source);
      break;
    case SourceTimestampQualityEvent::RECOVERED:
      RCLCPP_INFO(logger, "%s source timestamp quality recovered", source);
      break;
    case SourceTimestampQualityEvent::NONE:
      break;
  }
}

inline bool source_timestamp_is_plausible(
  int64_t source_time_ns, int64_t current_ros_time_ns,
  std::chrono::milliseconds maximum_age)
{
  const auto age_ns = current_ros_time_ns - source_time_ns;
  return age_ns >= 0 &&
         age_ns <= std::chrono::duration_cast<std::chrono::nanoseconds>(maximum_age).count();
}

}  // namespace arm_cell_safety_fsm
