#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include "arm_cell_motion/grasp_strategy.hpp"

namespace arm_cell_motion
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
  std::optional<double> width_mm;

  // Recency is independent of whether the device could establish possession.
  bool recent(
    Clock::time_point now = Clock::now(),
    std::chrono::milliseconds max_age = std::chrono::milliseconds(500)) const
  {
    if (sequence == 0 || now < observed_at) {
      return false;
    }
    return now - observed_at <= max_age;
  }

  bool fresh(
    Clock::time_point now = Clock::now(),
    std::chrono::milliseconds max_age = std::chrono::milliseconds(500)) const
  {return state != HoldingState::UNKNOWN && recent(now, max_age);}

};

class GripperPort
{
public:
  using ResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;

  virtual ~GripperPort() = default;

  virtual uint8_t grasp() {return close();}
  virtual uint8_t grasp(double grip_force_n) {return close(grip_force_n);}
  virtual uint8_t release() {return open();}

  virtual bool supportsGraspStrategy(GraspStrategy strategy) const
  {return strategy == GraspStrategy::BOX_EXTERIOR;}
  // Device adapter must validate the solution against its selected Tool before
  // accepting it. Legacy parallel adapters retain their unchanged Box path.
  virtual uint8_t prepareGrasp(const GraspSpecification & specification)
  {
    return specification.valid() && specification.strategy == GraspStrategy::BOX_EXTERIOR ?
           ResultCode::TASK_RESULT_SUCCESS : ResultCode::TASK_RESULT_INVALID_GOAL;
  }

  virtual uint8_t close() = 0;
  // Sum of opposing finger contact normal forces in N. Native adapters must
  // explicitly implement their manufacturer-specific conversion; never ignore it.
  virtual uint8_t close(double /* grip_force_n */)
  {return ResultCode::TASK_RESULT_GRASP_FAILURE;}
  virtual uint8_t open() = 0;
  virtual HoldingObservation holding() const = 0;
  virtual bool active() const = 0;
  virtual void stop() = 0;
};

}  // namespace arm_cell_motion
