#include "arm_cell_vision/fixed_target_registry.hpp"

#include <cmath>
#include <utility>

namespace arm_cell_vision
{
namespace
{

constexpr int64_t kMaxTtlNs = 60'000'000'000LL;

void normalize_orientation(geometry_msgs::msg::PoseStamped & pose)
{
  const auto & q = pose.pose.orientation;
  const auto norm = std::sqrt(
    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  pose.pose.orientation.x /= norm;
  pose.pose.orientation.y /= norm;
  pose.pose.orientation.z /= norm;
  pose.pose.orientation.w /= norm;
}

}  // namespace

FixedTargetRegistry::FixedTargetRegistry(
  std::string configured_target_id, SteadyNow steady_now)
: configured_target_id_(std::move(configured_target_id)), steady_now_(std::move(steady_now))
{
}

FixedTargetReceipt FixedTargetRegistry::register_target(
  const FixedTargetRegistration & registration)
{
  const auto receipt_time_ns = steady_now_();

  if (registration.target_id != configured_target_id_) {
    return {false, 0, receipt_time_ns, "target_id does not match validation profile"};
  }
  if (registration.run_id.empty()) {
    return {false, 0, receipt_time_ns, "run_id must not be empty"};
  }
  if (registration.pose.header.frame_id != "base_link") {
    return {false, 0, receipt_time_ns, "pose must be expressed in base_link"};
  }
  if (!finite_pose(registration.pose)) {
    return {false, 0, receipt_time_ns, "pose contains non-finite values"};
  }
  if (registration.ttl_ns <= 0 || registration.ttl_ns > kMaxTtlNs) {
    return {false, 0, receipt_time_ns, "ttl must be in (0, 60s]"};
  }

  if (record_ && !expired(receipt_time_ns) && record_->run_id != registration.run_id) {
    return {false, 0, receipt_time_ns, "another run registration is still active"};
  }

  FixedTargetRecord next;
  static_cast<FixedTargetRegistration &>(next) = registration;
  normalize_orientation(next.pose);
  next.receipt_time_ns = receipt_time_ns;
  next.receipt_sequence = next_receipt_sequence_++;
  record_ = std::move(next);
  return {true, record_->receipt_sequence, receipt_time_ns, {}};
}

std::optional<FixedTargetRecord> FixedTargetRegistry::consume(
  const std::string & target_id, int64_t now_ns)
{
  if (target_id != configured_target_id_ || !record_ || expired(now_ns)) {
    return std::nullopt;
  }

  auto result = record_;
  record_.reset();
  return result;
}

bool FixedTargetRegistry::expired(int64_t now_ns) const
{
  return !record_ || now_ns < record_->receipt_time_ns ||
         now_ns - record_->receipt_time_ns >= record_->ttl_ns;
}

bool FixedTargetRegistry::finite_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const auto finite = [](double value) {return std::isfinite(value);};
  const auto norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return finite(p.x) && finite(p.y) && finite(p.z) &&
         finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
         std::isfinite(norm) && norm > 1e-12;
}

}  // namespace arm_cell_vision
