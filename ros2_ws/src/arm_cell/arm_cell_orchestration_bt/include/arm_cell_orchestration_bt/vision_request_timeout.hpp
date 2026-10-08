#ifndef ARM_CELL_ORCHESTRATION_BT__VISION_REQUEST_TIMEOUT_HPP_
#define ARM_CELL_ORCHESTRATION_BT__VISION_REQUEST_TIMEOUT_HPP_

#include <chrono>
#include <optional>

#include <builtin_interfaces/msg/duration.hpp>

namespace arm_cell_orchestration_bt
{

inline std::chrono::nanoseconds vision_response_wait_timeout(
  std::chrono::nanoseconds request_timeout)
{
  return request_timeout + std::chrono::milliseconds(100);
}

inline std::optional<std::chrono::nanoseconds> vision_request_timeout(
  const builtin_interfaces::msg::Duration & timeout)
{
  if (timeout.sec < 0 || timeout.nanosec >= 1'000'000'000U) {
    return std::nullopt;
  }
  return std::chrono::seconds(timeout.sec) + std::chrono::nanoseconds(timeout.nanosec);
}

}  // namespace arm_cell_orchestration_bt

#endif  // ARM_CELL_ORCHESTRATION_BT__VISION_REQUEST_TIMEOUT_HPP_
