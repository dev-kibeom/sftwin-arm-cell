#ifndef ARM_CELL_VISION__VISION_ROS_INGRESS_HPP_
#define ARM_CELL_VISION__VISION_ROS_INGRESS_HPP_

#include <cstdint>
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "arm_cell_vision/clock_epoch_observer.hpp"
#include "arm_cell_vision/vision_ros_adapter.hpp"

namespace arm_cell_vision
{

enum class VisionClockSampleRelation
{
  kNotReceived,
  kFirstSample,
  kAdvanced,
  kUnchanged,
  kBackward
};

struct VisionIngressAcceptanceSnapshot
{
  // Explicit Calibration RGB-D association; not the DetectTarget Depth gate.
  uint64_t synchronized_callback_count = 0;
  uint64_t policy_delivery_count = 0;
  uint64_t observation_ready_count = 0;
  bool calibration_cache_valid = false;
  // Legacy pair spelling: this is the bounded Depth cache, independent of RGB.
  bool cached_pair_reused = false;
  uint64_t calibration_cache_generation = 0;
  std::optional<int64_t> latest_rgb_stamp_ns;
  std::optional<int64_t> latest_depth_stamp_ns;
  std::optional<int64_t> latest_camera_info_stamp_ns;
  std::optional<int64_t> latest_canonical_stamp_ns;
  std::optional<int64_t> latest_rgb_receipt_monotonic_ns;
  std::optional<int64_t> latest_depth_receipt_monotonic_ns;
  std::optional<int64_t> latest_camera_info_receipt_monotonic_ns;
  uint64_t rgb_message_count = 0;
  uint64_t depth_message_count = 0;
  uint64_t camera_info_message_count = 0;
  std::optional<int64_t> latest_rgb_received_stamp_ns;
  std::optional<int64_t> latest_depth_received_stamp_ns;
  std::optional<int64_t> latest_rgb_received_monotonic_ns;
  std::optional<int64_t> latest_depth_received_monotonic_ns;
  std::optional<int64_t> latest_camera_info_received_monotonic_ns;
  std::optional<int64_t> last_synchronized_receipt_monotonic_ns;
  std::optional<int64_t> last_depth_observation_monotonic_ns;
  std::optional<int64_t> latest_oldest_receipt_age_ns;
  std::optional<int64_t> latest_receipt_spread_ns;
  std::optional<IngressStatus> last_status;
  std::optional<int64_t> latest_clock_stamp_ns;
  std::optional<int64_t> latest_clock_received_monotonic_ns;
  std::optional<IngressStatus> last_clock_observation_status;
  VisionClockSampleRelation last_clock_sample_relation{VisionClockSampleRelation::kNotReceived};
  uint64_t clock_sample_count = 0;
  bool clock_rollback_observed = false;
  bool observation_rollback_observed = false;
  bool epoch_invalidated = false;
  uint64_t clock_epoch = 0;
};

struct VisionIngressSampleDiagnostic
{
  std::string stream;
  std::string frame_id;
  int64_t stamp_ns;
  int64_t receipt_monotonic_ns;
};

struct VisionIngressPairFreshnessDiagnostic
{
  int64_t evaluation_steady_ns = 0;
  std::optional<int64_t> evaluation_clock_ns;
  int64_t rgb_source_stamp_ns = 0;
  int64_t depth_source_stamp_ns = 0;
  std::optional<int64_t> rgb_source_minus_clock_ns;
  std::optional<int64_t> depth_source_minus_clock_ns;
  std::optional<int64_t> clock_receipt_age_ns;
  std::optional<int64_t> clock_received_steady_ns;
  int64_t maximum_absolute_delta_ns = 0;
  bool accepted = false;
  std::string reason;
  std::optional<IngressStatus> clock_observation_status;
  VisionClockSampleRelation clock_sample_relation{VisionClockSampleRelation::kNotReceived};
  uint64_t clock_sample_count = 0;
  uint64_t clock_epoch = 0;
  bool clock_rollback_observed = false;
  bool epoch_invalidated = false;
};

struct VisionIngressPairDiagnostic
{
  uint64_t sequence = 0;
  std::string rgb_frame_id;
  std::string depth_frame_id;
  int64_t rgb_stamp_ns = 0;
  int64_t depth_stamp_ns = 0;
  int64_t rgb_receipt_monotonic_ns = -1;
  int64_t depth_receipt_monotonic_ns = -1;
  int64_t delta_ns = 0;
  bool pre_request_sample = false;
  bool watermark_passed = false;
  std::optional<bool> policy_evaluated;
  std::optional<bool> observation_ready;
  std::optional<IngressStatus> status;
  std::optional<VisionIngressPairFreshnessDiagnostic> freshness_evaluation;
};

struct VisionIngressRequestDiagnostics
{
  uint64_t request_id = 0;
  int64_t request_start_monotonic_ns = 0;
  int64_t request_watermark_stamp_ns = 0;
  bool active = false;
  bool detailed_capture_enabled = false;
  uint64_t clock_epoch_start = 0;
  bool processing_started = false;
  // Legacy pair spelling: this is the bounded Depth cache, independent of RGB.
  bool cached_pair_reused = false;
  bool cached_pair_available = false;
  std::optional<IngressStatus> cached_pair_status;
  uint64_t rgb_received = 0;
  uint64_t depth_received = 0;
  uint64_t sync_callbacks = 0;
  uint64_t policy_evaluations = 0;
  uint64_t observation_ready = 0;
  uint64_t unknown_pair_receipt_count = 0;
  uint64_t watermark_rejected_pairs = 0;
  uint64_t sample_detail_dropped = 0;
  uint64_t pair_detail_dropped = 0;
  uint64_t source_stamp_freshness_evaluated = 0;
  uint64_t source_stamp_freshness_accepted = 0;
  uint64_t source_stamp_freshness_rejected = 0;
  uint64_t source_stamp_freshness_clock_unavailable = 0;
  uint64_t source_stamp_freshness_clock_delta_rejected = 0;
  uint64_t depth_source_stamp_freshness_evaluated = 0;
  uint64_t depth_source_stamp_freshness_accepted = 0;
  uint64_t depth_source_stamp_freshness_rejected = 0;
  std::optional<int64_t> latest_rgb_stamp_ns;
  std::optional<int64_t> latest_depth_stamp_ns;
  std::optional<int64_t> latest_rgb_receipt_monotonic_ns;
  std::optional<int64_t> latest_depth_receipt_monotonic_ns;
  std::optional<int64_t> latest_stamp_delta_ns;
  std::optional<int64_t> minimum_cross_stream_delta_ns;
  std::optional<int64_t> minimum_cross_rgb_stamp_ns;
  std::optional<int64_t> minimum_cross_depth_stamp_ns;
  std::optional<int64_t> rgb_stamp_period_mean_ns;
  std::optional<int64_t> rgb_stamp_period_latest_ns;
  std::optional<int64_t> rgb_receipt_period_mean_ns;
  std::optional<int64_t> rgb_receipt_period_latest_ns;
  std::optional<int64_t> depth_stamp_period_mean_ns;
  std::optional<int64_t> depth_stamp_period_latest_ns;
  std::optional<int64_t> depth_receipt_period_mean_ns;
  std::optional<int64_t> depth_receipt_period_latest_ns;
  std::optional<int64_t> latest_pair_rgb_stamp_ns;
  std::optional<int64_t> latest_pair_depth_stamp_ns;
  std::optional<int64_t> latest_pair_delta_ns;
  std::optional<IngressStatus> last_status;
  std::vector<VisionIngressSampleDiagnostic> sample_details;
  std::vector<VisionIngressPairDiagnostic> pair_details;
  std::vector<VisionIngressPairFreshnessDiagnostic> depth_source_freshness_details;
};

struct VisionRequestDiagnosticRecord
{
  VisionIngressRequestDiagnostics ingress;
  VisionIngressAcceptanceSnapshot clock;
  int64_t deadline_monotonic_ns = 0;
  int64_t decision_monotonic_ns = 0;
  int64_t timeout_ns = 0;
  int64_t sync_slop_ns = 0;
  uint8_t result_code = 0;
};

struct VisionObservation
{
  sensor_msgs::msg::Image rgb;
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo camera_info;
  int64_t rgb_receipt_monotonic_ns = -1;
  int64_t depth_receipt_monotonic_ns = -1;
  std::string rgb_presentation_status = "not_received";
};

struct SnapshotRgbSelection
{
  std::optional<sensor_msgs::msg::Image> image;
  std::string status = "not_received";
};

class VisionRosIngress : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  explicit VisionRosIngress(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  IngressResult begin_acquisition(
    int64_t flush_watermark_stamp_ns,
    std::optional<int64_t> request_start_monotonic_ns = std::nullopt,
    std::optional<int64_t> request_timeout_ns = std::nullopt);
  IngressResult check_deadline();
  void end_acquisition();
  IngressResult reset_after_lifecycle();
  std::optional<IngressResult> last_result() const;
  const rclcpp::QoS & subscription_qos() const;
  int64_t sync_slop_ns() const;
  SnapshotRgbSelection latest_snapshot_rgb(
    const Image & depth, const CameraInfo & camera_info) const;
  bool has_cached_calibration() const;
  uint64_t calibration_cache_generation() const;
  const std::vector<int64_t> & synchronized_callback_rgb_stamps() const;
  std::array<size_t, 2> receipt_metadata_sizes() const;
  VisionIngressAcceptanceSnapshot acceptance_snapshot() const;
  VisionIngressPairFreshnessDiagnostic evaluate_pair_source_stamp_freshness(
    const VisionObservation & observation, int64_t maximum_absolute_delta_ns);
  VisionIngressPairFreshnessDiagnostic evaluate_depth_source_stamp_freshness(
    const VisionObservation & observation, int64_t maximum_absolute_delta_ns);
  VisionIngressRequestDiagnostics finish_request_diagnostics();
  bool detailed_capture_enabled() const;
  void mark_detector_processing_started();
  std::optional<VisionObservation> latest_observation() const;
  void set_observation_observer(std::function<void(const VisionObservation &)> observer);
  void set_synchronized_observer(
    std::function<void(const VisionObservation &)> observer);
  void set_observation_invalidator(std::function<void()> invalidator);

private:
  using ReceiptMap = std::map<int64_t, int64_t>;

  rclcpp::QoS subscription_qos_;
  int64_t sync_slop_ns_;
  bool detailed_request_capture_enabled_ = false;
  int64_t reusable_pair_max_age_ns_;
  size_t receipt_metadata_capacity_ = 20;
  size_t calibration_callback_history_capacity_ = 20;
  SensorIngressPolicy ingress_;
  mutable std::mutex sensor_policy_mutex_;
  ClockEpochObserver clock_observer_;
  message_filters::Subscriber<Image> rgb_subscriber_;
  message_filters::Subscriber<Image> depth_subscriber_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  ReceiptMap rgb_receipts_;
  ReceiptMap depth_receipts_;
  mutable std::mutex receipt_mutex_;
  struct CachedCalibration
  {
    CameraInfo message;
    int64_t receipt_monotonic_ns;
    uint64_t generation;
  };
  mutable std::mutex latest_rgb_mutex_;
  Image::ConstSharedPtr latest_rgb_;
  mutable std::mutex calibration_mutex_;
  std::optional<CachedCalibration> cached_calibration_;
  uint64_t calibration_cache_generation_ = 0;
  std::optional<IngressResult> last_result_;
  mutable std::mutex result_mutex_;
  std::optional<VisionObservation> latest_observation_;
  std::function<void(const VisionObservation &)> observation_observer_;
  std::function<void(const VisionObservation &)> synchronized_observer_;
  std::function<void()> observation_invalidator_;
  mutable std::mutex observer_mutex_;
  mutable std::mutex observation_mutex_;
  struct CachedObservation
  {
    VisionObservation observation;
    SynchronizedObservationCandidate candidate;
  };
  std::optional<CachedObservation> cached_observation_;
  std::optional<int64_t> cached_observation_stamp_ns_;
  std::vector<int64_t> synchronized_callback_rgb_stamps_;
  mutable std::mutex acceptance_mutex_;
  VisionIngressAcceptanceSnapshot acceptance_snapshot_;
  mutable std::mutex request_diagnostics_mutex_;
  VisionIngressRequestDiagnostics request_diagnostics_;
  std::optional<int64_t> last_depth_freshness_stamp_ns_;
  std::optional<int64_t> last_depth_freshness_receipt_ns_;
  std::optional<int64_t> last_depth_freshness_max_delta_ns_;
  std::optional<VisionIngressPairFreshnessDiagnostic> last_depth_freshness_;
  uint64_t next_request_diagnostic_id_ = 1;
  struct StreamPeriodState
  {
    std::optional<int64_t> last_stamp_ns;
    std::optional<int64_t> last_receipt_ns;
    int64_t stamp_period_sum_ns = 0;
    uint64_t stamp_period_count = 0;
    int64_t receipt_period_sum_ns = 0;
    uint64_t receipt_period_count = 0;
  };
  StreamPeriodState request_rgb_period_;
  StreamPeriodState request_depth_period_;
  std::vector<int64_t> request_rgb_stamps_;
  std::vector<int64_t> request_depth_stamps_;
  uint64_t next_pair_diagnostic_sequence_ = 1;

  static int64_t monotonic_now_ns();
  static int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp);
  static rclcpp::QoS camera_subscription_qos();
  void record_receipt(ReceiptMap & receipts, int64_t stamp, int64_t receipt_monotonic_ns);
  int64_t take_receipt(ReceiptMap & receipts, int64_t stamp);
  uint64_t record_request_sample(
    const char * stream, const std::string & frame_id, int64_t stamp_ns, int64_t receipt_ns);
  std::optional<uint64_t> record_request_pair(
    int64_t rgb_stamp_ns, int64_t depth_stamp_ns,
    int64_t rgb_receipt_ns, int64_t depth_receipt_ns,
    const std::string & rgb_frame_id, const std::string & depth_frame_id);
  void complete_request_pair(
    std::optional<uint64_t> sequence, bool policy_evaluated, IngressStatus status);
  void record_pair_source_stamp_freshness(
    const VisionObservation & observation,
    const VisionIngressPairFreshnessDiagnostic & evaluation);
  void on_camera_info(const CameraInfo::ConstSharedPtr & camera_info);
  void on_synchronized(
    const Image::ConstSharedPtr & rgb,
    const Image::ConstSharedPtr & depth);
  void on_depth(const Image::ConstSharedPtr & depth, int64_t receipt_monotonic_ns);
  void on_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & clock);
  void set_last_result(const IngressResult & result);
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__VISION_ROS_INGRESS_HPP_
