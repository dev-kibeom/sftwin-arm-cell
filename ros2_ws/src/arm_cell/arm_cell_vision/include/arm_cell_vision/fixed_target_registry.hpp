#ifndef ARM_CELL_VISION__FIXED_TARGET_REGISTRY_HPP_
#define ARM_CELL_VISION__FIXED_TARGET_REGISTRY_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace arm_cell_vision
{

struct FixedTargetRegistration
{
  std::string target_id;
  std::string run_id;
  geometry_msgs::msg::PoseStamped pose;
  bool has_target_yaw = false;
  int64_t ttl_ns = 30'000'000'000LL;
};

struct FixedTargetReceipt
{
  bool accepted = false;
  uint64_t receipt_sequence = 0;
  int64_t receipt_time_ns = 0;
  std::string diagnostic;
};

struct FixedTargetRecord : FixedTargetRegistration
{
  uint64_t receipt_sequence = 0;
  int64_t receipt_time_ns = 0;
};

class FixedTargetRegistry
{
public:
  using SteadyNow = std::function<int64_t()>;

  FixedTargetRegistry(std::string configured_target_id, SteadyNow steady_now);

  FixedTargetReceipt register_target(const FixedTargetRegistration & registration);
  std::optional<FixedTargetRecord> consume(
    const std::string & target_id, int64_t now_ns);

private:
  bool expired(int64_t now_ns) const;
  static bool finite_pose(const geometry_msgs::msg::PoseStamped & pose);

  std::string configured_target_id_;
  SteadyNow steady_now_;
  std::optional<FixedTargetRecord> record_;
  uint64_t next_receipt_sequence_ = 1;
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__FIXED_TARGET_REGISTRY_HPP_
