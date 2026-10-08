#ifndef ARM_CELL_VISION__VISION_SENSOR_INGRESS_HPP_
#define ARM_CELL_VISION__VISION_SENSOR_INGRESS_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace arm_cell_vision
{

enum class IngressStatus
{
  kWaiting,
  kNoActiveAcquisition,
  kObservationReady,
  kDuplicateObservation,
  kSynchronizationSlopExceeded,
  kStaleObservation,
  kReceiptStaleObservation,
  kCalibrationUnavailable,
  kIncompatibleMetadata,
  kMalformedObservation,
  kRequestTimeout,
  kTimeRollback,
  kEpochInvalidated,
};

struct SensorSample
{
  std::optional<int64_t> stamp_ns;
  std::optional<int64_t> receipt_monotonic_ns;
  int32_t width;
  int32_t height;
  std::string frame_id;
};

// This is an internal adapter-to-policy value. The ROS adapter owns bounded
// RGB/depth association and supplies independently validated cached CameraInfo.
struct SynchronizedObservationCandidate
{
  SensorSample rgb;
  SensorSample depth;
  SensorSample camera_info;
  std::optional<int64_t> common_stamp_ns;
};

struct SensorIngressConfig
{
  int64_t request_timeout_ns;
  std::optional<int64_t> max_receipt_age_ns;
};

class SynchronizedObservation
{
public:
  explicit SynchronizedObservation(SynchronizedObservationCandidate candidate);

  int64_t stamp_ns() const;
  const SensorSample & rgb() const;
  const SensorSample & depth() const;
  const SensorSample & camera_info() const;

private:
  const SensorSample rgb_;
  const SensorSample depth_;
  const SensorSample camera_info_;
  const int64_t stamp_ns_;
};

struct IngressResult
{
  IngressStatus status;
  std::optional<SynchronizedObservation> observation;
};

class SensorIngressPolicy
{
public:
  explicit SensorIngressPolicy(SensorIngressConfig config);

  IngressResult begin_acquisition(
    int64_t flush_watermark_stamp_ns, int64_t now_monotonic_ns,
    std::optional<int64_t> request_timeout_ns = std::nullopt);
  IngressResult evaluate(
    const SynchronizedObservationCandidate & candidate,
    int64_t now_monotonic_ns);
  IngressResult evaluate_cached(
    const SynchronizedObservationCandidate & candidate,
    int64_t now_monotonic_ns, int64_t maximum_receipt_age_ns,
    std::optional<int64_t> current_sensor_stamp_ns = std::nullopt);
  static bool is_compatible_candidate(const SynchronizedObservationCandidate & candidate);
  IngressResult check_deadline(int64_t now_monotonic_ns);
  IngressResult reset_after_lifecycle();
  IngressResult invalidate_epoch_from_clock();

  bool epoch_invalidated() const;

private:
  bool active_ = false;
  bool epoch_invalidated_ = false;
  std::optional<int64_t> last_synchronized_epoch_stamp_ns_;
  std::optional<int64_t> flush_watermark_stamp_ns_;
  std::optional<int64_t> request_start_monotonic_ns_;
  std::optional<int64_t> deadline_monotonic_ns_;
  SensorIngressConfig config_;

  void clear_request_state();
  IngressResult invalidate_epoch();
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__VISION_SENSOR_INGRESS_HPP_
