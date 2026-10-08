#pragma once

#include <chrono>
#include <cstdint>

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

namespace arm_cell_motion_moveit2
{

enum class HoldingState : uint8_t
{
  HELD,
  RELEASED,
  UNKNOWN
};

struct HoldingObservation
{
  using Clock = std::chrono::steady_clock;
  HoldingState state{HoldingState::UNKNOWN};
  Clock::time_point observed_at{};
  std::uint64_t sequence{0};

  bool fresh(
    Clock::time_point now = Clock::now(),
    std::chrono::milliseconds max_age = std::chrono::milliseconds(500)) const
  {
    if (state == HoldingState::UNKNOWN || sequence == 0 || now < observed_at) {
      return false;
    }
    return now - observed_at <= max_age;
  }
};

class GripperPort
{
public:
  using ResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;

  virtual ~GripperPort() = default;

  virtual uint8_t close(float width_mm) = 0;
  virtual uint8_t open() = 0;
  virtual HoldingObservation holding() const = 0;
  virtual bool active() const = 0;
  virtual void stop() = 0;
};

}  // namespace arm_cell_motion_moveit2
