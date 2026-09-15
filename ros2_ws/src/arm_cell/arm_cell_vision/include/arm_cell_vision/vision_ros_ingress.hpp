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
  uint64_t calibration_cache_generation = 0;
  std::optional<int64_t> latest_rgb_stamp_ns;
  std::optional<int64_t> latest_depth_stamp_ns;
  std::optional<int64_t> latest_camera_info_stamp_ns;
  std::optional<int64_t> latest_canonical_stamp_ns;
  std::optional<int64_t> latest_rgb_receipt_monotonic_ns;
  std::optional<int64_t> latest_depth_receipt_monotonic_ns;
  std::optional<int64_t> latest_camera_info_receipt_monotonic_ns;
  std::optional<int64_t> latest_oldest_receipt_age_ns;
  std::optional<int64_t> latest_receipt_spread_ns;
  std::optional<IngressStatus> last_status;
  std::optional<int64_t> latest_clock_stamp_ns;
  uint64_t clock_sample_count = 0;
  bool clock_rollback_observed = false;
  bool observation_rollback_observed = false;
  bool epoch_invalidated = false;
};

class VisionRosIngress : public rclcpp::Node
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

  explicit VisionRosIngress(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  IngressResult begin_acquisition(int64_t flush_watermark_stamp_ns);
  IngressResult check_deadline();
  IngressResult reset_after_lifecycle();
  const std::optional<IngressResult> & last_result() const;
  const rclcpp::QoS & subscription_qos() const;
  int64_t sync_slop_ns() const;
  bool has_cached_calibration() const;
  uint64_t calibration_cache_generation() const;
  const std::vector<int64_t> & synchronized_callback_rgb_stamps() const;
  std::array<size_t, 2> receipt_metadata_sizes() const;
  VisionIngressAcceptanceSnapshot acceptance_snapshot() const;

private:
  using ReceiptMap = std::map<int64_t, int64_t>;

  rclcpp::QoS subscription_qos_;
  int64_t sync_slop_ns_;
  size_t receipt_metadata_capacity_ = 20;
  size_t callback_history_capacity_ = 20;
  SensorIngressPolicy ingress_;
  ClockEpochObserver clock_observer_;
  message_filters::Subscriber<Image> rgb_subscriber_;
  message_filters::Subscriber<Image> depth_subscriber_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  ReceiptMap rgb_receipts_;
  ReceiptMap depth_receipts_;
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
  std::vector<int64_t> synchronized_callback_rgb_stamps_;
  mutable std::mutex acceptance_mutex_;
  VisionIngressAcceptanceSnapshot acceptance_snapshot_;

  static int64_t monotonic_now_ns();
  static int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp);
  static rclcpp::QoS camera_subscription_qos();
  void record_receipt(ReceiptMap & receipts, int64_t stamp);
  int64_t take_receipt(ReceiptMap & receipts, int64_t stamp);
  void on_camera_info(const CameraInfo::ConstSharedPtr & camera_info);
  void on_synchronized(
    const Image::ConstSharedPtr & rgb,
    const Image::ConstSharedPtr & depth);
  void on_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & clock);
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__VISION_ROS_INGRESS_HPP_
