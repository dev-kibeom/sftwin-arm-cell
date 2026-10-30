#ifndef ARM_CELL_VISION__CLOCK_EPOCH_OBSERVER_HPP_
#define ARM_CELL_VISION__CLOCK_EPOCH_OBSERVER_HPP_

#include <cstdint>
#include <optional>

#include "arm_cell_vision/vision_sensor_ingress.hpp"

namespace arm_cell_vision
{

class ClockEpochObserver
{
public:
  explicit ClockEpochObserver(SensorIngressPolicy & ingress);

  IngressResult observe(int64_t clock_stamp_ns);

private:
  SensorIngressPolicy & ingress_;
  std::optional<int64_t> last_clock_stamp_ns_;
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__CLOCK_EPOCH_OBSERVER_HPP_
