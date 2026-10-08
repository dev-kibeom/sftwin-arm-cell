#include "arm_cell_vision/vision_sensor_ingress.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace arm_cell_vision
{
namespace
{

bool valid_sample(const SensorSample & sample)
{
  return sample.stamp_ns && sample.receipt_monotonic_ns && *sample.stamp_ns >= 0 &&
         *sample.receipt_monotonic_ns >= 0 && sample.width > 0 && sample.height > 0 &&
         !sample.frame_id.empty();
}

bool compatible_candidate(const SynchronizedObservationCandidate & candidate)
{
  if (!candidate.common_stamp_ns || *candidate.common_stamp_ns < 0 ||
    !valid_sample(candidate.rgb) || !valid_sample(candidate.depth) ||
    !valid_sample(candidate.camera_info))
  {
    return false;
  }
  const auto stamp = *candidate.common_stamp_ns;
  return *candidate.rgb.stamp_ns == stamp &&
         candidate.rgb.width == candidate.depth.width &&
         candidate.depth.width == candidate.camera_info.width &&
         candidate.rgb.height == candidate.depth.height &&
         candidate.depth.height == candidate.camera_info.height &&
         candidate.rgb.frame_id == candidate.depth.frame_id &&
         candidate.depth.frame_id == candidate.camera_info.frame_id;
}

bool exact_stamps_present(const SynchronizedObservationCandidate & candidate)
{
  return candidate.common_stamp_ns && candidate.rgb.stamp_ns && candidate.depth.stamp_ns &&
         candidate.camera_info.stamp_ns;
}

bool receipt_is_fresh(
  const SensorSample & sample, int64_t now, std::optional<int64_t> maximum_age)
{
  return !maximum_age ||
         (now >= *sample.receipt_monotonic_ns &&
         now - *sample.receipt_monotonic_ns <= *maximum_age);
}

IngressResult result(IngressStatus status)
{
  return {status, std::nullopt};
}

}  // namespace

SynchronizedObservation::SynchronizedObservation(SynchronizedObservationCandidate candidate)
: rgb_(std::move(candidate.rgb)), depth_(std::move(candidate.depth)),
  camera_info_(std::move(candidate.camera_info)), stamp_ns_(*candidate.common_stamp_ns)
{
}

int64_t SynchronizedObservation::stamp_ns() const {return stamp_ns_;}
const SensorSample & SynchronizedObservation::rgb() const {return rgb_;}
const SensorSample & SynchronizedObservation::depth() const {return depth_;}
const SensorSample & SynchronizedObservation::camera_info() const {return camera_info_;}

SensorIngressPolicy::SensorIngressPolicy(SensorIngressConfig config)
: config_(std::move(config))
{
  if (config_.request_timeout_ns <= 0 ||
    (config_.max_receipt_age_ns && *config_.max_receipt_age_ns < 0))
  {
    throw std::invalid_argument("sensor ingress configuration is invalid");
  }
}

bool SensorIngressPolicy::is_compatible_candidate(
  const SynchronizedObservationCandidate & candidate)
{
  return exact_stamps_present(candidate) && valid_sample(candidate.rgb) &&
         valid_sample(candidate.depth) && valid_sample(candidate.camera_info) &&
         compatible_candidate(candidate);
}

IngressResult SensorIngressPolicy::begin_acquisition(
  int64_t watermark, int64_t now, std::optional<int64_t> request_timeout_ns)
{
  if (epoch_invalidated_) {
    return result(IngressStatus::kEpochInvalidated);
  }
  const auto timeout_ns = request_timeout_ns.value_or(config_.request_timeout_ns);
  if (watermark < 0 || now < 0 || timeout_ns < 0 ||
    timeout_ns > std::numeric_limits<int64_t>::max() - now)
  {
    return result(IngressStatus::kMalformedObservation);
  }
  clear_request_state();
  flush_watermark_stamp_ns_ = watermark;
  request_start_monotonic_ns_ = now;
  deadline_monotonic_ns_ = now + timeout_ns;
  active_ = true;
  return result(IngressStatus::kWaiting);
}

IngressResult SensorIngressPolicy::evaluate(
  const SynchronizedObservationCandidate & candidate, int64_t now)
{
  if (epoch_invalidated_) {
    return result(IngressStatus::kEpochInvalidated);
  }
  if (!active_) {
    return result(IngressStatus::kNoActiveAcquisition);
  }
  if (now < 0) {
    return result(IngressStatus::kMalformedObservation);
  }
  if (now >= *deadline_monotonic_ns_) {
    clear_request_state();
    return result(IngressStatus::kRequestTimeout);
  }
  if (!exact_stamps_present(candidate) || !valid_sample(candidate.rgb) ||
    !valid_sample(candidate.depth) || !valid_sample(candidate.camera_info))
  {
    return result(IngressStatus::kMalformedObservation);
  }
  if (!compatible_candidate(candidate)) {
    return result(IngressStatus::kIncompatibleMetadata);
  }
  if (*candidate.rgb.receipt_monotonic_ns <= *request_start_monotonic_ns_ ||
    *candidate.depth.receipt_monotonic_ns <= *request_start_monotonic_ns_)
  {
    return result(IngressStatus::kStaleObservation);
  }

  const auto stamp = *candidate.common_stamp_ns;
  if (last_synchronized_epoch_stamp_ns_) {
    if (stamp < *last_synchronized_epoch_stamp_ns_) {
      return invalidate_epoch();
    }
    if (stamp == *last_synchronized_epoch_stamp_ns_) {
      return result(IngressStatus::kDuplicateObservation);
    }
  }
  last_synchronized_epoch_stamp_ns_ = stamp;

  if (stamp <= *flush_watermark_stamp_ns_) {
    return result(IngressStatus::kStaleObservation);
  }
  if (!receipt_is_fresh(candidate.rgb, now, config_.max_receipt_age_ns) ||
    !receipt_is_fresh(candidate.depth, now, config_.max_receipt_age_ns))
  {
    return result(IngressStatus::kReceiptStaleObservation);
  }
  active_ = false;
  return {IngressStatus::kObservationReady, SynchronizedObservation(candidate)};
}

IngressResult SensorIngressPolicy::evaluate_cached(
  const SynchronizedObservationCandidate & candidate, int64_t now,
  int64_t maximum_receipt_age_ns, std::optional<int64_t> current_sensor_stamp_ns)
{
  if (epoch_invalidated_) {
    return result(IngressStatus::kEpochInvalidated);
  }
  if (!active_) {
    return result(IngressStatus::kNoActiveAcquisition);
  }
  if (now < 0 || maximum_receipt_age_ns < 0) {
    return result(IngressStatus::kMalformedObservation);
  }
  if (now >= *deadline_monotonic_ns_) {
    clear_request_state();
    return result(IngressStatus::kRequestTimeout);
  }
  if (!exact_stamps_present(candidate) || !valid_sample(candidate.rgb) ||
    !valid_sample(candidate.depth) || !valid_sample(candidate.camera_info))
  {
    return result(IngressStatus::kMalformedObservation);
  }
  if (!compatible_candidate(candidate)) {
    return result(IngressStatus::kIncompatibleMetadata);
  }
  if (!receipt_is_fresh(candidate.rgb, now, maximum_receipt_age_ns) ||
    !receipt_is_fresh(candidate.depth, now, maximum_receipt_age_ns))
  {
    return result(IngressStatus::kReceiptStaleObservation);
  }

  const auto stamp = *candidate.common_stamp_ns;
  if (current_sensor_stamp_ns && *current_sensor_stamp_ns >= stamp &&
    *current_sensor_stamp_ns - stamp > maximum_receipt_age_ns)
  {
    return result(IngressStatus::kReceiptStaleObservation);
  }
  if (last_synchronized_epoch_stamp_ns_ && stamp < *last_synchronized_epoch_stamp_ns_) {
    return invalidate_epoch();
  }
  if (!last_synchronized_epoch_stamp_ns_ || stamp > *last_synchronized_epoch_stamp_ns_) {
    last_synchronized_epoch_stamp_ns_ = stamp;
  }
  active_ = false;
  return {IngressStatus::kObservationReady, SynchronizedObservation(candidate)};
}

IngressResult SensorIngressPolicy::check_deadline(int64_t now)
{
  if (epoch_invalidated_) {
    return result(IngressStatus::kEpochInvalidated);
  }
  if (!active_) {
    return result(IngressStatus::kNoActiveAcquisition);
  }
  if (now < 0) {
    return result(IngressStatus::kMalformedObservation);
  }
  if (now >= *deadline_monotonic_ns_) {
    clear_request_state();
    return result(IngressStatus::kRequestTimeout);
  }
  return result(IngressStatus::kWaiting);
}

IngressResult SensorIngressPolicy::reset_after_lifecycle()
{
  clear_request_state();
  epoch_invalidated_ = false;
  last_synchronized_epoch_stamp_ns_.reset();
  return result(IngressStatus::kWaiting);
}

IngressResult SensorIngressPolicy::invalidate_epoch_from_clock()
{
  if (epoch_invalidated_) {
    return result(IngressStatus::kEpochInvalidated);
  }
  return invalidate_epoch();
}

bool SensorIngressPolicy::epoch_invalidated() const {return epoch_invalidated_;}

void SensorIngressPolicy::clear_request_state()
{
  active_ = false;
  flush_watermark_stamp_ns_.reset();
  request_start_monotonic_ns_.reset();
  deadline_monotonic_ns_.reset();
}

IngressResult SensorIngressPolicy::invalidate_epoch()
{
  clear_request_state();
  epoch_invalidated_ = true;
  return result(IngressStatus::kTimeRollback);
}

}  // namespace arm_cell_vision
