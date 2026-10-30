#ifndef ARM_CELL_VISION_ACCEPTANCE__VISION_ROS_INGRESS_ACCEPTANCE_LOGIC_HPP_
#define ARM_CELL_VISION_ACCEPTANCE__VISION_ROS_INGRESS_ACCEPTANCE_LOGIC_HPP_

#include <optional>

#include "arm_cell_vision/vision_sensor_ingress.hpp"

namespace arm_cell_vision::acceptance
{

inline bool is_terminal_deadline_status(IngressStatus status)
{
  return status == IngressStatus::kRequestTimeout ||
         status == IngressStatus::kTimeRollback ||
         status == IngressStatus::kEpochInvalidated;
}

class DeadlineStatusLatch
{
public:
  void observe(IngressStatus status)
  {
    if (!latched_status_ && is_terminal_deadline_status(status)) {
      latched_status_ = status;
    }
  }

  bool latched() const {return latched_status_.has_value();}
  std::optional<IngressStatus> status() const {return latched_status_;}

private:
  std::optional<IngressStatus> latched_status_;
};

inline bool normal_mode_pass(
  IngressStatus begin_status,
  const std::optional<IngressStatus> & deadline_status,
  uint64_t observation_ready_count,
  bool epoch_invalidated)
{
  return begin_status == IngressStatus::kWaiting &&
         observation_ready_count > 0 && !epoch_invalidated &&
         (!deadline_status || *deadline_status != IngressStatus::kTimeRollback);
}

inline bool expect_timeout_mode_pass(
  IngressStatus begin_status,
  const std::optional<IngressStatus> & deadline_status,
  uint64_t observation_ready_count,
  bool epoch_invalidated,
  bool clock_rollback_observed,
  bool observation_rollback_observed)
{
  return begin_status == IngressStatus::kWaiting && deadline_status &&
         *deadline_status == IngressStatus::kRequestTimeout &&
         observation_ready_count == 0 && !epoch_invalidated &&
         !clock_rollback_observed && !observation_rollback_observed;
}

}  // namespace arm_cell_vision::acceptance

#endif  // ARM_CELL_VISION_ACCEPTANCE__VISION_ROS_INGRESS_ACCEPTANCE_LOGIC_HPP_
