#include "arm_cell_vision/vision_ros_ingress.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
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
constexpr int64_t kMaximumReusablePairAgeNs = 200'000'000;

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
  reusable_pair_max_age_ns_(milliseconds_to_ns(
      declare_parameter<double>(
        "reusable_pair_max_age_ms", 200.0,
        rcl_interfaces::msg::ParameterDescriptor().set__description(
          "Maximum local receipt age for reusing the latest synchronized RGB-D input; "
          "this re-runs detection and never reuses a prior detection result.")),
      "reusable_pair_max_age_ms")),
  ingress_({
    milliseconds_to_ns(declare_parameter<double>("request_timeout_ms", 500.0),
    "request_timeout_ms"),
    receipt_age_limit_ns(*this)}),
  clock_observer_(ingress_)
{
  const auto queue_size = declare_parameter<int>("sync_queue_size", 20);
  if (queue_size <= 0) {
    throw std::invalid_argument("sync_queue_size must be positive");
  }
  if (reusable_pair_max_age_ns_ > kMaximumReusablePairAgeNs) {
    throw std::invalid_argument("reusable_pair_max_age_ms must not exceed 200 ms");
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
      const auto stamp = stamp_ns(message->header.stamp);
      const auto receipt = monotonic_now_ns();
      record_receipt(rgb_receipts_, stamp, receipt);
      record_request_sample("rgb", message->header.frame_id, stamp, receipt);
      std::lock_guard<std::mutex> lock(acceptance_mutex_);
      acceptance_snapshot_.rgb_message_count++;
      acceptance_snapshot_.latest_rgb_received_stamp_ns = stamp;
      acceptance_snapshot_.latest_rgb_received_monotonic_ns = receipt;
      if (acceptance_snapshot_.latest_depth_received_stamp_ns) {
        acceptance_snapshot_.latest_rgb_depth_delta_ns = std::llabs(
          stamp - *acceptance_snapshot_.latest_depth_received_stamp_ns);
      }
    });
  depth_subscriber_.registerCallback(
    [this](const Image::ConstSharedPtr & message) {
      const auto stamp = stamp_ns(message->header.stamp);
      const auto receipt = monotonic_now_ns();
      record_receipt(depth_receipts_, stamp, receipt);
      record_request_sample("depth", message->header.frame_id, stamp, receipt);
      std::lock_guard<std::mutex> lock(acceptance_mutex_);
      acceptance_snapshot_.depth_message_count++;
      acceptance_snapshot_.latest_depth_received_stamp_ns = stamp;
      acceptance_snapshot_.latest_depth_received_monotonic_ns = receipt;
      if (acceptance_snapshot_.latest_rgb_received_stamp_ns) {
        acceptance_snapshot_.latest_rgb_depth_delta_ns = std::llabs(
          *acceptance_snapshot_.latest_rgb_received_stamp_ns - stamp);
      }
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

IngressResult VisionRosIngress::begin_acquisition(
  int64_t watermark, std::optional<int64_t> request_start_monotonic_ns,
  std::optional<int64_t> request_timeout_ns)
{
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    last_result_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.cached_pair_reused = false;
  }
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    latest_observation_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    request_diagnostics_ = VisionIngressRequestDiagnostics{};
    request_diagnostics_.request_id = next_request_diagnostic_id_++;
    request_diagnostics_.request_start_monotonic_ns =
      request_start_monotonic_ns.value_or(monotonic_now_ns());
    request_diagnostics_.request_watermark_stamp_ns = watermark;
    request_diagnostics_.active = true;
    request_rgb_period_ = StreamPeriodState{};
    request_depth_period_ = StreamPeriodState{};
    request_rgb_stamps_.clear();
    request_depth_stamps_.clear();
  }
  const auto result = [&]() {
    std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
    return ingress_.begin_acquisition(
      watermark, request_start_monotonic_ns.value_or(monotonic_now_ns()), request_timeout_ns);
  }();
  if (result.status == IngressStatus::kWaiting) {
    std::optional<CachedObservation> cached;
    {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      cached = cached_observation_;
    }
    if (cached) {
      const auto now = monotonic_now_ns();
      const auto current_sensor_stamp = acceptance_snapshot().latest_clock_stamp_ns;
      const auto reusable = [&]() {
        std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
        return ingress_.evaluate_cached(
          cached->candidate, now, reusable_pair_max_age_ns_, current_sensor_stamp);
      }();
      {
        std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
        request_diagnostics_.cached_pair_available = true;
        request_diagnostics_.cached_pair_status = reusable.status;
      }
      if (reusable.status == IngressStatus::kObservationReady) {
        {
          std::lock_guard<std::mutex> lock(observation_mutex_);
          latest_observation_ = cached->observation;
        }
        set_last_result(reusable);
        std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
        request_diagnostics_.cached_pair_reused = true;
        request_diagnostics_.pre_request_pair_candidates = 1;
        request_diagnostics_.last_status = reusable.status;
        {
          std::lock_guard<std::mutex> acceptance_lock(acceptance_mutex_);
          acceptance_snapshot_.cached_pair_reused = true;
          acceptance_snapshot_.last_pair_observation_monotonic_ns = now;
        }
      }
    }
  }
  if (result.status != IngressStatus::kWaiting) {
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    request_diagnostics_.last_status = result.status;
  }
  return result;
}

IngressResult VisionRosIngress::check_deadline()
{
  const auto result = [&]() {
    std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
    return ingress_.check_deadline(monotonic_now_ns());
  }();
  if (result.status == IngressStatus::kRequestTimeout) {
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    if (request_diagnostics_.active) {
      request_diagnostics_.last_status = result.status;
    }
  }
  return result;
}

IngressResult VisionRosIngress::reset_after_lifecycle()
{
  {
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    request_diagnostics_.active = false;
  }
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    latest_observation_.reset();
    cached_observation_.reset();
    cached_observation_stamp_ns_.reset();
  }
  std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
  return ingress_.reset_after_lifecycle();
}

std::optional<IngressResult> VisionRosIngress::last_result() const
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  return last_result_;
}
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
  std::lock_guard<std::mutex> lock(receipt_mutex_);
  return {rgb_receipts_.size(), depth_receipts_.size()};
}

VisionIngressAcceptanceSnapshot VisionRosIngress::acceptance_snapshot() const
{
  std::lock_guard<std::mutex> lock(acceptance_mutex_);
  return acceptance_snapshot_;
}

VisionIngressRequestDiagnostics VisionRosIngress::finish_request_diagnostics()
{
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  request_diagnostics_.active = false;
  return request_diagnostics_;
}

void VisionRosIngress::mark_detector_processing_started()
{
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  if (request_diagnostics_.active) {
    request_diagnostics_.processing_started = true;
  }
}

std::optional<VisionObservation> VisionRosIngress::latest_observation() const
{
  std::lock_guard<std::mutex> lock(observation_mutex_);
  return latest_observation_;
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

void VisionRosIngress::record_receipt(
  ReceiptMap & receipts, int64_t stamp, int64_t receipt_monotonic_ns)
{
  std::lock_guard<std::mutex> lock(receipt_mutex_);
  if (receipts.size() >= receipt_metadata_capacity_ && receipts.find(stamp) == receipts.end()) {
    receipts.erase(receipts.begin());
  }
  receipts.try_emplace(stamp, receipt_monotonic_ns);
}

int64_t VisionRosIngress::take_receipt(ReceiptMap & receipts, int64_t stamp)
{
  std::lock_guard<std::mutex> lock(receipt_mutex_);
  const auto entry = receipts.find(stamp);
  if (entry == receipts.end()) {
    return -1;
  }
  const auto receipt = entry->second;
  receipts.erase(entry);
  return receipt;
}

void VisionRosIngress::record_request_sample(
  const char * stream, const std::string & frame_id, int64_t stamp_ns_value, int64_t receipt_ns)
{
  constexpr size_t kSampleHistoryCapacity = 64;
  constexpr size_t kSampleDetailCapacity = 64;
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  if (!request_diagnostics_.active) {
    return;
  }

  const bool is_rgb = stream[0] == 'r';
  auto & period = is_rgb ? request_rgb_period_ : request_depth_period_;
  auto & count = is_rgb ? request_diagnostics_.rgb_received : request_diagnostics_.depth_received;
  auto & latest_stamp = is_rgb ? request_diagnostics_.latest_rgb_stamp_ns :
    request_diagnostics_.latest_depth_stamp_ns;
  auto & latest_receipt = is_rgb ? request_diagnostics_.latest_rgb_receipt_monotonic_ns :
    request_diagnostics_.latest_depth_receipt_monotonic_ns;
  auto & stamps = is_rgb ? request_rgb_stamps_ : request_depth_stamps_;
  auto & opposite_stamps = is_rgb ? request_depth_stamps_ : request_rgb_stamps_;
  ++count;
  latest_stamp = stamp_ns_value;
  latest_receipt = receipt_ns;

  if (period.last_stamp_ns) {
    const auto value = stamp_ns_value - *period.last_stamp_ns;
    if (value > 0) {
      period.stamp_period_sum_ns += value;
      ++period.stamp_period_count;
    }
    if (is_rgb) {
      request_diagnostics_.rgb_stamp_period_latest_ns = value;
      if (period.stamp_period_count) {
        request_diagnostics_.rgb_stamp_period_mean_ns = period.stamp_period_sum_ns /
          static_cast<int64_t>(period.stamp_period_count);
      }
    } else {
      request_diagnostics_.depth_stamp_period_latest_ns = value;
      if (period.stamp_period_count) {
        request_diagnostics_.depth_stamp_period_mean_ns = period.stamp_period_sum_ns /
          static_cast<int64_t>(period.stamp_period_count);
      }
    }
  }
  if (period.last_receipt_ns) {
    const auto value = receipt_ns - *period.last_receipt_ns;
    if (value > 0) {
      period.receipt_period_sum_ns += value;
      ++period.receipt_period_count;
    }
    if (is_rgb) {
      request_diagnostics_.rgb_receipt_period_latest_ns = value;
      if (period.receipt_period_count) {
        request_diagnostics_.rgb_receipt_period_mean_ns = period.receipt_period_sum_ns /
          static_cast<int64_t>(period.receipt_period_count);
      }
    } else {
      request_diagnostics_.depth_receipt_period_latest_ns = value;
      if (period.receipt_period_count) {
        request_diagnostics_.depth_receipt_period_mean_ns = period.receipt_period_sum_ns /
          static_cast<int64_t>(period.receipt_period_count);
      }
    }
  }
  period.last_stamp_ns = stamp_ns_value;
  period.last_receipt_ns = receipt_ns;

  for (const auto other_stamp : opposite_stamps) {
    const auto delta = std::llabs(stamp_ns_value - other_stamp);
    if (!request_diagnostics_.minimum_cross_stream_delta_ns ||
      delta < *request_diagnostics_.minimum_cross_stream_delta_ns)
    {
      request_diagnostics_.minimum_cross_stream_delta_ns = delta;
      request_diagnostics_.minimum_cross_rgb_stamp_ns = is_rgb ? stamp_ns_value : other_stamp;
      request_diagnostics_.minimum_cross_depth_stamp_ns = is_rgb ? other_stamp : stamp_ns_value;
    }
  }
  if (stamps.size() >= kSampleHistoryCapacity) {
    stamps.erase(stamps.begin());
  }
  stamps.push_back(stamp_ns_value);
  if (request_diagnostics_.latest_rgb_stamp_ns && request_diagnostics_.latest_depth_stamp_ns) {
    request_diagnostics_.latest_stamp_delta_ns = std::llabs(
      *request_diagnostics_.latest_rgb_stamp_ns - *request_diagnostics_.latest_depth_stamp_ns);
  }
  if (request_diagnostics_.sample_details.size() < kSampleDetailCapacity) {
    request_diagnostics_.sample_details.push_back({stream, frame_id, stamp_ns_value, receipt_ns});
  } else {
    ++request_diagnostics_.sample_detail_dropped;
  }
}

std::optional<uint64_t> VisionRosIngress::record_request_pair(
  int64_t rgb_stamp_ns, int64_t depth_stamp_ns,
  int64_t rgb_receipt_ns, int64_t depth_receipt_ns)
{
  constexpr size_t kPairDetailCapacity = 32;
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  if (!request_diagnostics_.active) {
    return std::nullopt;
  }
  const auto sequence = next_pair_diagnostic_sequence_++;
  ++request_diagnostics_.sync_callbacks;
  request_diagnostics_.latest_pair_rgb_stamp_ns = rgb_stamp_ns;
  request_diagnostics_.latest_pair_depth_stamp_ns = depth_stamp_ns;
  request_diagnostics_.latest_pair_delta_ns = std::llabs(rgb_stamp_ns - depth_stamp_ns);
  const bool receipts_known = rgb_receipt_ns >= 0 && depth_receipt_ns >= 0;
  const bool pre_request = receipts_known &&
    (rgb_receipt_ns <= request_diagnostics_.request_start_monotonic_ns ||
    depth_receipt_ns <= request_diagnostics_.request_start_monotonic_ns);
  const bool watermark_passed = rgb_stamp_ns > request_diagnostics_.request_watermark_stamp_ns;
  if (!receipts_known) {
    ++request_diagnostics_.unknown_pair_receipt_count;
  } else if (pre_request) {
    ++request_diagnostics_.pre_request_pair_candidates;
  }
  if (!watermark_passed) {
    ++request_diagnostics_.watermark_rejected_pairs;
  }
  if (request_diagnostics_.pair_details.size() < kPairDetailCapacity) {
    request_diagnostics_.pair_details.push_back(
      {
        sequence, rgb_stamp_ns, depth_stamp_ns, rgb_receipt_ns, depth_receipt_ns,
        std::llabs(rgb_stamp_ns - depth_stamp_ns), pre_request, watermark_passed,
        std::nullopt, std::nullopt, std::nullopt});
  } else {
    ++request_diagnostics_.pair_detail_dropped;
  }
  return sequence;
}

void VisionRosIngress::complete_request_pair(
  std::optional<uint64_t> sequence, bool policy_evaluated, IngressStatus status)
{
  if (!sequence) {
    return;
  }
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  if (!request_diagnostics_.active) {
    return;
  }
  if (policy_evaluated) {
    ++request_diagnostics_.policy_evaluations;
  }
  const bool observation_ready = status == IngressStatus::kObservationReady;
  if (observation_ready) {
    ++request_diagnostics_.observation_ready;
  }
  request_diagnostics_.last_status = status;
  const auto pair = std::find_if(
    request_diagnostics_.pair_details.rbegin(), request_diagnostics_.pair_details.rend(),
    [sequence](const auto & value) {return value.sequence == *sequence;});
  if (pair != request_diagnostics_.pair_details.rend()) {
    pair->policy_evaluated = policy_evaluated;
    pair->observation_ready = observation_ready;
    pair->status = status;
  }
}

void VisionRosIngress::on_camera_info(const CameraInfo::ConstSharedPtr & camera_info)
{
  const auto receipt = monotonic_now_ns();
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.camera_info_message_count++;
    acceptance_snapshot_.latest_camera_info_received_monotonic_ns = receipt;
    acceptance_snapshot_.latest_camera_info_stamp_ns = stamp_ns(camera_info->header.stamp);
    acceptance_snapshot_.calibration_cache_valid =
      VisionRosAdapter::has_valid_calibration(*camera_info);
  }
  std::lock_guard<std::mutex> lock(calibration_mutex_);
  if (!VisionRosAdapter::has_valid_calibration(*camera_info)) {
    cached_calibration_.reset();
    {
      std::lock_guard<std::mutex> observation_lock(observation_mutex_);
      cached_observation_.reset();
      cached_observation_stamp_ns_.reset();
      latest_observation_.reset();
    }
    return;
  }
  cached_calibration_ = CachedCalibration{*camera_info, receipt, ++calibration_cache_generation_};
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
  const auto request_pair_sequence = record_request_pair(
    rgb_stamp, depth_stamp, receipt_times.rgb_monotonic_ns, receipt_times.depth_monotonic_ns);
  const auto synchronized_receipt = monotonic_now_ns();
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.latest_rgb_stamp_ns = rgb_stamp;
    acceptance_snapshot_.latest_depth_stamp_ns = depth_stamp;
    acceptance_snapshot_.last_synchronized_receipt_monotonic_ns = synchronized_receipt;
    acceptance_snapshot_.synchronized_callback_count++;
  }
  std::optional<CachedCalibration> calibration;
  {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    calibration = cached_calibration_;
  }
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
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
    set_last_result(IngressResult{IngressStatus::kCalibrationUnavailable, std::nullopt});
    complete_request_pair(
      request_pair_sequence, false, IngressStatus::kCalibrationUnavailable);
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.last_status = IngressStatus::kCalibrationUnavailable;
    return;
  }
  receipt_times.camera_info_monotonic_ns = calibration->receipt_monotonic_ns;
  const auto candidate = VisionRosAdapter::to_candidate(
    *rgb, *depth, calibration->message, receipt_times);
  if (VisionRosAdapter::within_sync_slop(candidate, sync_slop_ns_) &&
    SensorIngressPolicy::is_compatible_candidate(candidate))
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    if (!cached_observation_stamp_ns_ ||
      candidate.common_stamp_ns.value_or(-1) > *cached_observation_stamp_ns_)
    {
      cached_observation_stamp_ns_ = candidate.common_stamp_ns;
      cached_observation_ = CachedObservation{
        VisionObservation{*rgb, *depth, calibration->message}, candidate};
    }
  }
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.latest_canonical_stamp_ns = candidate.common_stamp_ns;
  }
  if (!VisionRosAdapter::within_sync_slop(candidate, sync_slop_ns_)) {
    set_last_result(IngressResult{IngressStatus::kSynchronizationSlopExceeded, std::nullopt});
    complete_request_pair(
      request_pair_sequence, false, IngressStatus::kSynchronizationSlopExceeded);
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.last_status = IngressStatus::kSynchronizationSlopExceeded;
    return;
  }
  const auto result = [&]() {
    std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
    return ingress_.evaluate(candidate, monotonic_now_ns());
  }();
  complete_request_pair(request_pair_sequence, true, result.status);
  if (result.status == IngressStatus::kIncompatibleMetadata) {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    if (cached_calibration_ && cached_calibration_->generation == calibration->generation) {
      cached_calibration_.reset();
    }
    std::lock_guard<std::mutex> observation_lock(observation_mutex_);
    cached_observation_.reset();
    cached_observation_stamp_ns_.reset();
    latest_observation_.reset();
  }
  if (result.status == IngressStatus::kObservationReady) {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    latest_observation_ = VisionObservation{*rgb, *depth, calibration->message};
  }
  set_last_result(result);
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.policy_delivery_count++;
    if (result.status == IngressStatus::kObservationReady) {
      acceptance_snapshot_.observation_ready_count++;
      acceptance_snapshot_.last_pair_observation_monotonic_ns = monotonic_now_ns();
    }
    acceptance_snapshot_.last_status = result.status;
    {
      std::lock_guard<std::mutex> policy_lock(sensor_policy_mutex_);
      acceptance_snapshot_.epoch_invalidated = ingress_.epoch_invalidated();
    }
    if (result.status == IngressStatus::kTimeRollback) {
      acceptance_snapshot_.observation_rollback_observed = true;
    }
  }
}

void VisionRosIngress::on_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & clock)
{
  const auto clock_stamp = stamp_ns(clock->clock);
  bool epoch_invalidated = false;
  const auto result = [&]() {
    std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
    const auto observed = clock_observer_.observe(clock_stamp);
    epoch_invalidated = ingress_.epoch_invalidated();
    return observed;
  }();
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.clock_sample_count++;
    acceptance_snapshot_.latest_clock_stamp_ns = clock_stamp;
  }
  if (result.status == IngressStatus::kTimeRollback ||
    result.status == IngressStatus::kEpochInvalidated)
  {
    {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      cached_observation_.reset();
      cached_observation_stamp_ns_.reset();
      latest_observation_.reset();
    }
    set_last_result(result);
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.last_status = result.status;
    acceptance_snapshot_.clock_rollback_observed = true;
    acceptance_snapshot_.epoch_invalidated = epoch_invalidated;
  }
}

void VisionRosIngress::set_last_result(const IngressResult & result)
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  last_result_.reset();
  last_result_.emplace(result);
}

}  // namespace arm_cell_vision
