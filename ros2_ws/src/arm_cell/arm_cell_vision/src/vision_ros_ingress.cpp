#include "arm_cell_vision/vision_ros_ingress.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace arm_cell_vision
{
namespace
{

constexpr char kRgbTopic[] = "/camera/color/image_raw";
constexpr char kDepthTopic[] = "/camera/aligned_depth_to_color/image_raw";
constexpr char kCameraInfoTopic[] = "/camera/color/camera_info";

std::optional<int64_t> receipt_age_limit_ns(rclcpp::Node & node)
{
  const auto milliseconds = node.declare_parameter<double>("receipt_max_age_ms", -1.0);
  if (!std::isfinite(milliseconds) || milliseconds < -1.0) {
    throw std::invalid_argument("receipt_max_age_ms must be -1 or a non-negative finite value");
  }
  if (milliseconds < 0.0) {
    return std::nullopt;
  }
  return static_cast<int64_t>(milliseconds * 1'000'000.0);
}

int64_t milliseconds_to_ns(double milliseconds, const char * parameter_name)
{
  if (!std::isfinite(milliseconds) || milliseconds < 0.0 ||
    milliseconds > static_cast<double>(std::numeric_limits<int64_t>::max()) / 1'000'000.0)
  {
    throw std::invalid_argument(
            std::string(parameter_name) +
            " must be a non-negative finite value");
  }
  return static_cast<int64_t>(milliseconds * 1'000'000.0);
}

}  // namespace

VisionRosIngress::VisionRosIngress(const rclcpp::NodeOptions & options)
: Node("vision_ros_ingress", options), subscription_qos_(camera_subscription_qos()),
  sync_slop_ns_(milliseconds_to_ns(
      declare_parameter<double>(
        "sync_slop_ms", 10.0,
        rcl_interfaces::msg::ParameterDescriptor().set__description(
          "ApproximateTime maximum interval in milliseconds. Negative values are invalid; "
          "The Arm Cell Vision supported-profile default is 10 ms; 0 is reserved for "
          "explicit diagnostic or test configuration. This is an adapter acceptance upper bound, "
          "not a guarantee that message_filters emits every eligible combination. Matching "
          "follows installed ROS 2 Humble ApproximateTime semantics.")),
      "sync_slop_ms")),
  ingress_({
    milliseconds_to_ns(declare_parameter<double>("settling_time_ms", 50.0), "settling_time_ms"),
    milliseconds_to_ns(declare_parameter<double>("request_timeout_ms", 500.0),
    "request_timeout_ms"),
    receipt_age_limit_ns(*this)}),
  clock_observer_(ingress_)
{
  const auto queue_size = declare_parameter<int>("sync_queue_size", 20);
  if (queue_size <= 0) {
    throw std::invalid_argument("sync_queue_size must be positive");
  }
  receipt_metadata_capacity_ = static_cast<size_t>(queue_size);
  callback_history_capacity_ = static_cast<size_t>(queue_size);
  const auto rgb_topic = declare_parameter<std::string>("rgb_topic", kRgbTopic);
  const auto depth_topic = declare_parameter<std::string>("depth_topic", kDepthTopic);
  const auto camera_info_topic = declare_parameter<std::string>(
    "camera_info_topic",
    kCameraInfoTopic);

  rgb_subscriber_.subscribe(this, rgb_topic, subscription_qos_.get_rmw_qos_profile());
  depth_subscriber_.subscribe(this, depth_topic, subscription_qos_.get_rmw_qos_profile());
  camera_info_subscription_ = create_subscription<CameraInfo>(
    camera_info_topic, subscription_qos_,
    std::bind(&VisionRosIngress::on_camera_info, this, std::placeholders::_1));
  rgb_subscriber_.registerCallback(
    [this](const Image::ConstSharedPtr & message) {
      record_receipt(rgb_receipts_, stamp_ns(message->header.stamp));
    });
  depth_subscriber_.registerCallback(
    [this](const Image::ConstSharedPtr & message) {
      record_receipt(depth_receipts_, stamp_ns(message->header.stamp));
    });
  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(queue_size), rgb_subscriber_, depth_subscriber_);
  synchronizer_->setMaxIntervalDuration(rclcpp::Duration::from_nanoseconds(sync_slop_ns_));
  synchronizer_->registerCallback(
    std::bind(
      &VisionRosIngress::on_synchronized, this, std::placeholders::_1,
      std::placeholders::_2));
  clock_subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS(),
    std::bind(&VisionRosIngress::on_clock, this, std::placeholders::_1));
}

IngressResult VisionRosIngress::begin_acquisition(int64_t watermark)
{
  return ingress_.begin_acquisition(watermark, monotonic_now_ns());
}

IngressResult VisionRosIngress::check_deadline()
{
  return ingress_.check_deadline(monotonic_now_ns());
}

IngressResult VisionRosIngress::reset_after_lifecycle()
{
  return ingress_.reset_after_lifecycle();
}

const std::optional<IngressResult> & VisionRosIngress::last_result() const {return last_result_;}
const rclcpp::QoS & VisionRosIngress::subscription_qos() const {return subscription_qos_;}
int64_t VisionRosIngress::sync_slop_ns() const {return sync_slop_ns_;}
bool VisionRosIngress::has_cached_calibration() const
{
  std::lock_guard<std::mutex> lock(calibration_mutex_);
  return cached_calibration_.has_value();
}

uint64_t VisionRosIngress::calibration_cache_generation() const
{
  std::lock_guard<std::mutex> lock(calibration_mutex_);
  return calibration_cache_generation_;
}

const std::vector<int64_t> & VisionRosIngress::synchronized_callback_rgb_stamps() const
{
  return synchronized_callback_rgb_stamps_;
}

std::array<size_t, 2> VisionRosIngress::receipt_metadata_sizes() const
{
  return {rgb_receipts_.size(), depth_receipts_.size()};
}

VisionIngressAcceptanceSnapshot VisionRosIngress::acceptance_snapshot() const
{
  std::lock_guard<std::mutex> lock(acceptance_mutex_);
  return acceptance_snapshot_;
}

int64_t VisionRosIngress::monotonic_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

int64_t VisionRosIngress::stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

rclcpp::QoS VisionRosIngress::camera_subscription_qos()
{
  return rclcpp::SensorDataQoS().keep_last(10).durability_volatile();
}

void VisionRosIngress::record_receipt(ReceiptMap & receipts, int64_t stamp)
{
  if (receipts.size() >= receipt_metadata_capacity_ && receipts.find(stamp) == receipts.end()) {
    receipts.erase(receipts.begin());
  }
  receipts.try_emplace(stamp, monotonic_now_ns());
}

int64_t VisionRosIngress::take_receipt(ReceiptMap & receipts, int64_t stamp)
{
  const auto entry = receipts.find(stamp);
  if (entry == receipts.end()) {
    return -1;
  }
  const auto receipt = entry->second;
  receipts.erase(entry);
  return receipt;
}

void VisionRosIngress::on_camera_info(const CameraInfo::ConstSharedPtr & camera_info)
{
  std::lock_guard<std::mutex> lock(calibration_mutex_);
  if (!VisionRosAdapter::has_valid_calibration(*camera_info)) {
    cached_calibration_.reset();
    return;
  }
  cached_calibration_ = CachedCalibration{
    *camera_info, monotonic_now_ns(), ++calibration_cache_generation_};
}

void VisionRosIngress::on_synchronized(
  const Image::ConstSharedPtr & rgb,
  const Image::ConstSharedPtr & depth)
{
  const auto rgb_stamp = stamp_ns(rgb->header.stamp);
  const auto depth_stamp = stamp_ns(depth->header.stamp);
  if (synchronized_callback_rgb_stamps_.size() >= callback_history_capacity_) {
    synchronized_callback_rgb_stamps_.erase(synchronized_callback_rgb_stamps_.begin());
  }
  synchronized_callback_rgb_stamps_.push_back(rgb_stamp);
  auto receipt_times = ReceiptTimes{
    take_receipt(rgb_receipts_, rgb_stamp),
    take_receipt(depth_receipts_, depth_stamp),
    -1};
  std::optional<CachedCalibration> calibration;
  {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    calibration = cached_calibration_;
  }
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.synchronized_callback_count++;
    acceptance_snapshot_.latest_rgb_stamp_ns = rgb_stamp;
    acceptance_snapshot_.latest_depth_stamp_ns = depth_stamp;
    acceptance_snapshot_.calibration_cache_valid = calibration.has_value();
    acceptance_snapshot_.calibration_cache_generation = calibration_cache_generation();
    if (receipt_times.rgb_monotonic_ns >= 0) {
      acceptance_snapshot_.latest_rgb_receipt_monotonic_ns = receipt_times.rgb_monotonic_ns;
    }
    if (receipt_times.depth_monotonic_ns >= 0) {
      acceptance_snapshot_.latest_depth_receipt_monotonic_ns = receipt_times.depth_monotonic_ns;
    }
    if (calibration) {
      acceptance_snapshot_.latest_camera_info_stamp_ns =
        stamp_ns(calibration->message.header.stamp);
      acceptance_snapshot_.latest_camera_info_receipt_monotonic_ns =
        calibration->receipt_monotonic_ns;
    }
    if (receipt_times.rgb_monotonic_ns >= 0 && receipt_times.depth_monotonic_ns >= 0) {
      const auto oldest = std::min(
        receipt_times.rgb_monotonic_ns, receipt_times.depth_monotonic_ns);
      const auto newest = std::max(
        receipt_times.rgb_monotonic_ns, receipt_times.depth_monotonic_ns);
      const auto now = monotonic_now_ns();
      acceptance_snapshot_.latest_oldest_receipt_age_ns = now - oldest;
      acceptance_snapshot_.latest_receipt_spread_ns = newest - oldest;
    }
  }
  if (!calibration) {
    last_result_.emplace(
      IngressResult{IngressStatus::kCalibrationUnavailable, std::nullopt});
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.last_status = IngressStatus::kCalibrationUnavailable;
    return;
  }
  receipt_times.camera_info_monotonic_ns = calibration->receipt_monotonic_ns;
  const auto candidate = VisionRosAdapter::to_candidate(
    *rgb, *depth, calibration->message, receipt_times);
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.latest_canonical_stamp_ns = candidate.common_stamp_ns;
  }
  if (!VisionRosAdapter::within_sync_slop(candidate, sync_slop_ns_)) {
    last_result_.emplace(
      IngressResult{IngressStatus::kSynchronizationSlopExceeded, std::nullopt});
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.last_status = IngressStatus::kSynchronizationSlopExceeded;
    return;
  }
  const auto result = ingress_.evaluate(candidate, monotonic_now_ns());
  if (result.status == IngressStatus::kIncompatibleMetadata) {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    if (cached_calibration_ && cached_calibration_->generation == calibration->generation) {
      cached_calibration_.reset();
    }
  }
  last_result_.emplace(result);
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.policy_delivery_count++;
    if (result.status == IngressStatus::kObservationReady) {
      acceptance_snapshot_.observation_ready_count++;
    }
    acceptance_snapshot_.last_status = result.status;
    acceptance_snapshot_.epoch_invalidated = ingress_.epoch_invalidated();
    if (result.status == IngressStatus::kTimeRollback) {
      acceptance_snapshot_.observation_rollback_observed = true;
    }
  }
}

void VisionRosIngress::on_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & clock)
{
  const auto clock_stamp = stamp_ns(clock->clock);
  const auto result = clock_observer_.observe(clock_stamp);
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.clock_sample_count++;
    acceptance_snapshot_.latest_clock_stamp_ns = clock_stamp;
  }
  if (result.status == IngressStatus::kTimeRollback ||
    result.status == IngressStatus::kEpochInvalidated)
  {
    last_result_.emplace(result);
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.last_status = result.status;
    acceptance_snapshot_.clock_rollback_observed = true;
    acceptance_snapshot_.epoch_invalidated = ingress_.epoch_invalidated();
  }
}

}  // namespace arm_cell_vision
