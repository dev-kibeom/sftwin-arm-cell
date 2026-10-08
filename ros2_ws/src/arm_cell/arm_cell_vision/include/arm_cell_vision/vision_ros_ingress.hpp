#ifndef ARM_CELL_VISION__VISION_ROS_INGRESS_HPP_
#define ARM_CELL_VISION__VISION_ROS_INGRESS_HPP_

#include <cstdint>
#include <array>
#include <map>
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

struct VisionIngressAcceptanceSnapshot
{
  uint64_t synchronized_callback_count = 0;
  uint64_t policy_delivery_count = 0;
  uint64_t observation_ready_count = 0;
  bool calibration_cache_valid = false;
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
  std::optional<int64_t> latest_rgb_depth_delta_ns;
  std::optional<int64_t> last_synchronized_receipt_monotonic_ns;
  std::optional<int64_t> last_pair_observation_monotonic_ns;
  std::optional<int64_t> latest_oldest_receipt_age_ns;
  std::optional<int64_t> latest_receipt_spread_ns;
  std::optional<IngressStatus> last_status;
  std::optional<int64_t> latest_clock_stamp_ns;
  uint64_t clock_sample_count = 0;
  bool clock_rollback_observed = false;
  bool observation_rollback_observed = false;
  bool epoch_invalidated = false;
};

struct VisionIngressSampleDiagnostic
{
  std::string stream;
  std::string frame_id;
  int64_t stamp_ns;
  int64_t receipt_monotonic_ns;
};

struct VisionIngressPairDiagnostic
{
  uint64_t sequence = 0;
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
};

struct VisionIngressRequestDiagnostics
{
  uint64_t request_id = 0;
  int64_t request_start_monotonic_ns = 0;
  int64_t request_watermark_stamp_ns = 0;
  bool active = false;
  bool processing_started = false;
  bool cached_pair_reused = false;
  bool cached_pair_available = false;
  std::optional<IngressStatus> cached_pair_status;
  uint64_t rgb_received = 0;
  uint64_t depth_received = 0;
  uint64_t sync_callbacks = 0;
  uint64_t policy_evaluations = 0;
  uint64_t observation_ready = 0;
  uint64_t pre_request_pair_candidates = 0;
  uint64_t unknown_pair_receipt_count = 0;
  uint64_t watermark_rejected_pairs = 0;
  uint64_t sample_detail_dropped = 0;
  uint64_t pair_detail_dropped = 0;
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
};

struct VisionObservation
{
  sensor_msgs::msg::Image rgb;
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo camera_info;
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
  IngressResult reset_after_lifecycle();
  std::optional<IngressResult> last_result() const;
  const rclcpp::QoS & subscription_qos() const;
  int64_t sync_slop_ns() const;
  bool has_cached_calibration() const;
  uint64_t calibration_cache_generation() const;
  const std::vector<int64_t> & synchronized_callback_rgb_stamps() const;
  std::array<size_t, 2> receipt_metadata_sizes() const;
  VisionIngressAcceptanceSnapshot acceptance_snapshot() const;
  VisionIngressRequestDiagnostics finish_request_diagnostics();
  void mark_detector_processing_started();
  std::optional<VisionObservation> latest_observation() const;

private:
  using ReceiptMap = std::map<int64_t, int64_t>;

  rclcpp::QoS subscription_qos_;
  int64_t sync_slop_ns_;
  int64_t reusable_pair_max_age_ns_;
  size_t receipt_metadata_capacity_ = 20;
  size_t callback_history_capacity_ = 20;
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
  mutable std::mutex calibration_mutex_;
  std::optional<CachedCalibration> cached_calibration_;
  uint64_t calibration_cache_generation_ = 0;
  std::optional<IngressResult> last_result_;
  mutable std::mutex result_mutex_;
  std::optional<VisionObservation> latest_observation_;
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
  void record_request_sample(
    const char * stream, const std::string & frame_id, int64_t stamp_ns, int64_t receipt_ns);
  std::optional<uint64_t> record_request_pair(
    int64_t rgb_stamp_ns, int64_t depth_stamp_ns,
    int64_t rgb_receipt_ns, int64_t depth_receipt_ns);
  void complete_request_pair(
    std::optional<uint64_t> sequence, bool policy_evaluated, IngressStatus status);
  void on_camera_info(const CameraInfo::ConstSharedPtr & camera_info);
  void on_synchronized(
    const Image::ConstSharedPtr & rgb,
    const Image::ConstSharedPtr & depth);
  void on_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & clock);
  void set_last_result(const IngressResult & result);
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__VISION_ROS_INGRESS_HPP_
