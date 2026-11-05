#include "arm_cell_vision/clock_epoch_observer.hpp"

namespace arm_cell_vision
{

ClockEpochObserver::ClockEpochObserver(SensorIngressPolicy & ingress)
: ingress_(ingress)
{
}

IngressResult ClockEpochObserver::observe(int64_t clock_stamp_ns)
{
  if (clock_stamp_ns < 0) {
    return {IngressStatus::kMalformedObservation, std::nullopt};
  }
  if (last_clock_stamp_ns_ && clock_stamp_ns < *last_clock_stamp_ns_) {
    return ingress_.invalidate_epoch_from_clock();
  }
  last_clock_stamp_ns_ = clock_stamp_ns;
  return {IngressStatus::kWaiting, std::nullopt};
}

}  // namespace arm_cell_vision
