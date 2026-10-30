#include "arm_cell_vision/vision_ros_ingress.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <rcutils/logging.h>
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
          "Maximum local receipt age for reusing a cached Depth observation as a new "
          "detection input. Independent of Calibration sync_slop_ms.")),
      "reusable_pair_max_age_ms")),
  ingress_({
    milliseconds_to_ns(declare_parameter<double>("request_timeout_ms", 500.0),
    "request_timeout_ms"),
    receipt_age_limit_ns(*this)}),
  clock_observer_(ingress_)
{
  const bool requested_capture = declare_parameter<bool>(
    "diagnostics.capture_enabled", false,
    rcl_interfaces::msg::ParameterDescriptor().set__description(
      "Capture bounded request RGB-D sample/pair evidence in Snapshot metadata."));
  detailed_request_capture_enabled_ = requested_capture;
  const auto queue_size = declare_parameter<int>("sync_queue_size", 20);
  if (queue_size <= 0) {
    throw std::invalid_argument("sync_queue_size must be positive");
  }
  if (reusable_pair_max_age_ns_ > kMaximumReusablePairAgeNs) {
    throw std::invalid_argument("reusable_pair_max_age_ms must not exceed 200 ms");
  }
  receipt_metadata_capacity_ = static_cast<size_t>(queue_size);
  calibration_callback_history_capacity_ = static_cast<size_t>(queue_size);
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
      {
        std::lock_guard<std::mutex> lock(latest_rgb_mutex_);
        latest_rgb_ = message;
      }
      record_request_sample("rgb", message->header.frame_id, stamp, receipt);
      std::lock_guard<std::mutex> lock(acceptance_mutex_);
      acceptance_snapshot_.rgb_message_count++;
      acceptance_snapshot_.latest_rgb_received_stamp_ns = stamp;
      acceptance_snapshot_.latest_rgb_received_monotonic_ns = receipt;
    });
  depth_subscriber_.registerCallback(
    [this](const Image::ConstSharedPtr & message) {
      const auto stamp = stamp_ns(message->header.stamp);
      const auto receipt = monotonic_now_ns();
      record_receipt(depth_receipts_, stamp, receipt);
      record_request_sample("depth", message->header.frame_id, stamp, receipt);
      {
        std::lock_guard<std::mutex> lock(acceptance_mutex_);
        acceptance_snapshot_.depth_message_count++;
        acceptance_snapshot_.latest_depth_received_stamp_ns = stamp;
        acceptance_snapshot_.latest_depth_received_monotonic_ns = receipt;
      }
      on_depth(message, receipt);
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
    last_depth_freshness_stamp_ns_.reset();
    last_depth_freshness_receipt_ns_.reset();
    last_depth_freshness_max_delta_ns_.reset();
    last_depth_freshness_.reset();
    request_diagnostics_.request_id = next_request_diagnostic_id_++;
    request_diagnostics_.request_start_monotonic_ns =
      request_start_monotonic_ns.value_or(monotonic_now_ns());
    request_diagnostics_.request_watermark_stamp_ns = watermark;
    request_diagnostics_.active = true;
    request_diagnostics_.detailed_capture_enabled = detailed_request_capture_enabled_;
    request_rgb_period_ = StreamPeriodState{};
    request_depth_period_ = StreamPeriodState{};
    request_rgb_stamps_.clear();
    request_depth_stamps_.clear();
  }
  {
    const auto acceptance = acceptance_snapshot();
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    if (request_diagnostics_.active) {
      request_diagnostics_.clock_epoch_start = acceptance.clock_epoch;
    }
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
        {
          std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
          request_diagnostics_.cached_pair_reused = true;
          request_diagnostics_.last_status = reusable.status;
        }
        {
          std::lock_guard<std::mutex> acceptance_lock(acceptance_mutex_);
          acceptance_snapshot_.cached_pair_reused = true;
          acceptance_snapshot_.last_depth_observation_monotonic_ns = now;
        }
        auto reused_observation = cached->observation;
        if (reused_observation.rgb_receipt_monotonic_ns < 0 ||
          now < reused_observation.rgb_receipt_monotonic_ns ||
          now - reused_observation.rgb_receipt_monotonic_ns > reusable_pair_max_age_ns_)
        {
          reused_observation.rgb = Image{};
          reused_observation.rgb_receipt_monotonic_ns = -1;
        }
        std::function<void(const VisionObservation &)> observer;
        {
          std::lock_guard<std::mutex> lock(observer_mutex_);
          observer = observation_observer_;
        }
        if (observer) {observer(reused_observation);}
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

void VisionRosIngress::end_acquisition()
{
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    latest_observation_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    last_result_.reset();
  }
  std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
  ingress_.end_acquisition();
}

std::optional<IngressResult> VisionRosIngress::last_result() const
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  return last_result_;
}
const rclcpp::QoS & VisionRosIngress::subscription_qos() const {return subscription_qos_;}
int64_t VisionRosIngress::sync_slop_ns() const {return sync_slop_ns_;}
SnapshotRgbSelection VisionRosIngress::latest_snapshot_rgb(
  const Image & depth, const CameraInfo & camera_info) const
{
  Image::ConstSharedPtr image;
  {
    std::lock_guard<std::mutex> lock(latest_rgb_mutex_);
    image = latest_rgb_;
  }
  if (!image) {return SnapshotRgbSelection{std::nullopt, "not_received"};}
  if (depth.width == 0 || depth.height == 0 || depth.width != camera_info.width ||
    depth.height != camera_info.height)
  {
    return SnapshotRgbSelection{std::nullopt, "depth_camera_info_mismatch"};
  }
  if (depth.header.frame_id.empty() || camera_info.header.frame_id.empty() ||
    depth.header.frame_id != camera_info.header.frame_id ||
    image->header.frame_id != camera_info.header.frame_id)
  {
    return SnapshotRgbSelection{std::nullopt, "frame_mismatch"};
  }
  if (image->width == 0 || image->height == 0 || image->width != depth.width ||
    image->height != depth.height)
  {
    return SnapshotRgbSelection{std::nullopt, "dimensions_mismatch"};
  }
  if (image->encoding != "rgb8") {
    return SnapshotRgbSelection{std::nullopt, "encoding_unsupported"};
  }
  if (static_cast<size_t>(image->step) < static_cast<size_t>(image->width) * 3U) {
    return SnapshotRgbSelection{std::nullopt, "step_too_small"};
  }
  if (image->data.size() < static_cast<size_t>(image->step) * image->height) {
    return SnapshotRgbSelection{std::nullopt, "buffer_too_short"};
  }
  return SnapshotRgbSelection{*image, "available"};
}
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

VisionIngressPairFreshnessDiagnostic VisionRosIngress::evaluate_pair_source_stamp_freshness(
  const VisionObservation & observation, int64_t maximum_absolute_delta_ns)
{
  const auto clock = acceptance_snapshot();
  VisionIngressPairFreshnessDiagnostic evaluation;
  const auto rgb_source_stamp_ns = stamp_ns(observation.rgb.header.stamp);
  if (clock.latest_clock_stamp_ns) {
    const auto rgb_source_minus_clock_ns = rgb_source_stamp_ns - *clock.latest_clock_stamp_ns;
    evaluation.accepted = std::llabs(rgb_source_minus_clock_ns) <= maximum_absolute_delta_ns;
    if (detailed_request_capture_enabled_) {
      evaluation.rgb_source_minus_clock_ns = rgb_source_minus_clock_ns;
      evaluation.reason = evaluation.accepted ?
        "rgb_source_clock_delta_within_limit" : "rgb_source_clock_delta_exceeded";
    }
  } else {
    // Preserve the existing behavior when no /clock sample has been received.
    evaluation.accepted = true;
  }
  if (detailed_request_capture_enabled_) {
    evaluation.evaluation_steady_ns = monotonic_now_ns();
    evaluation.evaluation_clock_ns = clock.latest_clock_stamp_ns;
    evaluation.rgb_source_stamp_ns = rgb_source_stamp_ns;
    evaluation.depth_source_stamp_ns = stamp_ns(observation.depth.header.stamp);
    evaluation.maximum_absolute_delta_ns = maximum_absolute_delta_ns;
    evaluation.clock_received_steady_ns = clock.latest_clock_received_monotonic_ns;
    evaluation.clock_observation_status = clock.last_clock_observation_status;
    evaluation.clock_sample_relation = clock.last_clock_sample_relation;
    evaluation.clock_sample_count = clock.clock_sample_count;
    evaluation.clock_epoch = clock.clock_epoch;
    evaluation.clock_rollback_observed = clock.clock_rollback_observed;
    evaluation.epoch_invalidated = clock.epoch_invalidated;
    if (clock.latest_clock_stamp_ns) {
      evaluation.rgb_source_minus_clock_ns =
        rgb_source_stamp_ns - *clock.latest_clock_stamp_ns;
      evaluation.depth_source_minus_clock_ns =
        evaluation.depth_source_stamp_ns - *clock.latest_clock_stamp_ns;
    } else {
      evaluation.reason = "clock_not_received_gate_not_applied";
    }
    if (clock.latest_clock_received_monotonic_ns &&
      evaluation.evaluation_steady_ns >= *clock.latest_clock_received_monotonic_ns)
    {
      evaluation.clock_receipt_age_ns =
        evaluation.evaluation_steady_ns - *clock.latest_clock_received_monotonic_ns;
    }
    record_pair_source_stamp_freshness(observation, evaluation);
  }
  return evaluation;
}

VisionIngressPairFreshnessDiagnostic VisionRosIngress::evaluate_depth_source_stamp_freshness(
  const VisionObservation & observation, int64_t maximum_absolute_delta_ns)
{
  constexpr size_t kDepthFreshnessDetailCapacity = 32;
  const auto clock = acceptance_snapshot();
  VisionIngressPairFreshnessDiagnostic evaluation;
  const auto depth_stamp = stamp_ns(observation.depth.header.stamp);
  {
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    if (last_depth_freshness_ && last_depth_freshness_stamp_ns_ == depth_stamp &&
      last_depth_freshness_receipt_ns_ == observation.depth_receipt_monotonic_ns &&
      last_depth_freshness_max_delta_ns_ == maximum_absolute_delta_ns)
    {
      return *last_depth_freshness_;
    }
  }
  evaluation.accepted = !clock.latest_clock_stamp_ns ||
    std::llabs(depth_stamp - *clock.latest_clock_stamp_ns) <= maximum_absolute_delta_ns;
  evaluation.reason = !clock.latest_clock_stamp_ns ? "clock_not_received_gate_not_applied" :
    evaluation.accepted ? "depth_source_clock_delta_within_limit" :
    "depth_source_clock_delta_exceeded";
  if (detailed_request_capture_enabled_) {
    evaluation.evaluation_steady_ns = monotonic_now_ns();
    evaluation.evaluation_clock_ns = clock.latest_clock_stamp_ns;
    evaluation.rgb_source_stamp_ns = observation.rgb.width > 0 ?
      stamp_ns(observation.rgb.header.stamp) : 0;
    evaluation.depth_source_stamp_ns = depth_stamp;
    evaluation.maximum_absolute_delta_ns = maximum_absolute_delta_ns;
    if (clock.latest_clock_stamp_ns) {
      evaluation.depth_source_minus_clock_ns = depth_stamp - *clock.latest_clock_stamp_ns;
      if (observation.rgb.width > 0) {
        evaluation.rgb_source_minus_clock_ns =
          evaluation.rgb_source_stamp_ns - *clock.latest_clock_stamp_ns;
      }
    }
    evaluation.clock_sample_count = clock.clock_sample_count;
    evaluation.clock_epoch = clock.clock_epoch;
    evaluation.clock_rollback_observed = clock.clock_rollback_observed;
    evaluation.epoch_invalidated = clock.epoch_invalidated;
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    last_depth_freshness_stamp_ns_ = depth_stamp;
    last_depth_freshness_receipt_ns_ = observation.depth_receipt_monotonic_ns;
    last_depth_freshness_max_delta_ns_ = maximum_absolute_delta_ns;
    last_depth_freshness_ = evaluation;
    if (request_diagnostics_.active && request_diagnostics_.detailed_capture_enabled) {
      ++request_diagnostics_.depth_source_stamp_freshness_evaluated;
      if (evaluation.accepted) {
        ++request_diagnostics_.depth_source_stamp_freshness_accepted;
      } else {
        ++request_diagnostics_.depth_source_stamp_freshness_rejected;
      }
      if (request_diagnostics_.depth_source_freshness_details.size() <
        kDepthFreshnessDetailCapacity)
      {
        request_diagnostics_.depth_source_freshness_details.push_back(evaluation);
      }
    }
  }
  return evaluation;
}

void VisionRosIngress::record_pair_source_stamp_freshness(
  const VisionObservation & observation,
  const VisionIngressPairFreshnessDiagnostic & evaluation)
{
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  if (!request_diagnostics_.active || !request_diagnostics_.detailed_capture_enabled) {
    return;
  }
  ++request_diagnostics_.source_stamp_freshness_evaluated;
  if (evaluation.accepted) {
    ++request_diagnostics_.source_stamp_freshness_accepted;
  } else {
    ++request_diagnostics_.source_stamp_freshness_rejected;
    ++request_diagnostics_.source_stamp_freshness_clock_delta_rejected;
  }
  if (!evaluation.evaluation_clock_ns) {
    ++request_diagnostics_.source_stamp_freshness_clock_unavailable;
  }
  const auto pair = std::find_if(
    request_diagnostics_.pair_details.rbegin(), request_diagnostics_.pair_details.rend(),
    [&observation](const auto & value) {
      return !value.freshness_evaluation &&
             value.rgb_stamp_ns == stamp_ns(observation.rgb.header.stamp) &&
             value.depth_stamp_ns == stamp_ns(observation.depth.header.stamp) &&
             value.rgb_receipt_monotonic_ns == observation.rgb_receipt_monotonic_ns &&
             value.depth_receipt_monotonic_ns == observation.depth_receipt_monotonic_ns;
    });
  if (pair != request_diagnostics_.pair_details.rend()) {
    pair->freshness_evaluation = evaluation;
  }
}

VisionIngressRequestDiagnostics VisionRosIngress::finish_request_diagnostics()
{
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  request_diagnostics_.active = false;
  return request_diagnostics_;
}

bool VisionRosIngress::detailed_capture_enabled() const
{
  return detailed_request_capture_enabled_;
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

void VisionRosIngress::set_observation_observer(
  std::function<void(const VisionObservation &)> observer)
{
  std::lock_guard<std::mutex> lock(observer_mutex_);
  observation_observer_ = std::move(observer);
}

void VisionRosIngress::set_synchronized_observer(
  std::function<void(const VisionObservation &)> observer)
{
  std::lock_guard<std::mutex> lock(observer_mutex_);
  synchronized_observer_ = std::move(observer);
}

void VisionRosIngress::set_observation_invalidator(std::function<void()> invalidator)
{
  std::lock_guard<std::mutex> lock(observer_mutex_);
  observation_invalidator_ = std::move(invalidator);
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

uint64_t VisionRosIngress::record_request_sample(
  const char * stream, const std::string & frame_id, int64_t stamp_ns_value, int64_t receipt_ns)
{
  constexpr size_t kSampleHistoryCapacity = 64;
  constexpr size_t kSampleDetailCapacity = 64;
  constexpr size_t kFrameIdDetailMaxBytes = 128;
  std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
  const auto request_id = request_diagnostics_.request_id;
  if (!request_diagnostics_.active) {
    return request_id;
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
  if (detailed_request_capture_enabled_ && frame_id.size() <= kFrameIdDetailMaxBytes &&
    request_diagnostics_.sample_details.size() < kSampleDetailCapacity)
  {
    request_diagnostics_.sample_details.push_back({stream, frame_id, stamp_ns_value, receipt_ns});
  } else if (detailed_request_capture_enabled_) {
    ++request_diagnostics_.sample_detail_dropped;
  }
  return request_id;
}

std::optional<uint64_t> VisionRosIngress::record_request_pair(
  int64_t rgb_stamp_ns, int64_t depth_stamp_ns,
  int64_t rgb_receipt_ns, int64_t depth_receipt_ns,
  const std::string & rgb_frame_id, const std::string & depth_frame_id)
{
  constexpr size_t kPairDetailCapacity = 32;
  constexpr size_t kFrameIdDetailMaxBytes = 128;
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
  }
  if (!watermark_passed) {
    ++request_diagnostics_.watermark_rejected_pairs;
  }
  if (detailed_request_capture_enabled_ &&
    rgb_frame_id.size() <= kFrameIdDetailMaxBytes &&
    depth_frame_id.size() <= kFrameIdDetailMaxBytes &&
    request_diagnostics_.pair_details.size() < kPairDetailCapacity)
  {
    request_diagnostics_.pair_details.push_back(
      {
        sequence, rgb_frame_id, depth_frame_id, rgb_stamp_ns, depth_stamp_ns,
        rgb_receipt_ns, depth_receipt_ns,
        std::llabs(rgb_stamp_ns - depth_stamp_ns), pre_request, watermark_passed,
        std::nullopt, std::nullopt, std::nullopt, std::nullopt});
  } else if (detailed_request_capture_enabled_) {
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
  if (synchronized_callback_rgb_stamps_.size() >= calibration_callback_history_capacity_) {
    synchronized_callback_rgb_stamps_.erase(synchronized_callback_rgb_stamps_.begin());
  }
  synchronized_callback_rgb_stamps_.push_back(rgb_stamp);
  const auto rgb_receipt = take_receipt(rgb_receipts_, rgb_stamp);
  const auto depth_receipt = take_receipt(depth_receipts_, depth_stamp);
  const auto sequence = record_request_pair(
    rgb_stamp, depth_stamp, rgb_receipt, depth_receipt,
    rgb->header.frame_id, depth->header.frame_id);
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.latest_rgb_stamp_ns = rgb_stamp;
    acceptance_snapshot_.latest_depth_stamp_ns = depth_stamp;
    acceptance_snapshot_.latest_rgb_receipt_monotonic_ns = rgb_receipt;
    acceptance_snapshot_.latest_depth_receipt_monotonic_ns = depth_receipt;
    const auto now = monotonic_now_ns();
    const auto oldest_receipt = std::min(rgb_receipt, depth_receipt);
    acceptance_snapshot_.latest_oldest_receipt_age_ns = now >= oldest_receipt ?
      now - oldest_receipt : 0;
    acceptance_snapshot_.latest_receipt_spread_ns = std::llabs(rgb_receipt - depth_receipt);
    acceptance_snapshot_.last_synchronized_receipt_monotonic_ns = now;
    acceptance_snapshot_.synchronized_callback_count++;
  }
  complete_request_pair(sequence, false, IngressStatus::kWaiting);
  std::optional<CachedCalibration> calibration;
  {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    calibration = cached_calibration_;
  }
  if (calibration) {
    std::function<void(const VisionObservation &)> observer;
    {
      std::lock_guard<std::mutex> lock(observer_mutex_);
      observer = synchronized_observer_;
    }
    if (observer) {
      observer(VisionObservation{
        *rgb, *depth, calibration->message, rgb_receipt, depth_receipt});
    }
  }
}

void VisionRosIngress::on_depth(
  const Image::ConstSharedPtr & depth, int64_t receipt_monotonic_ns)
{
  const auto depth_stamp = stamp_ns(depth->header.stamp);
  std::optional<CachedCalibration> calibration;
  {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    calibration = cached_calibration_;
  }
  if (!calibration) {
    std::function<void()> invalidator;
    {
      std::lock_guard<std::mutex> lock(observer_mutex_);
      invalidator = observation_invalidator_;
    }
    if (invalidator) {invalidator();}
    set_last_result(IngressResult{IngressStatus::kCalibrationUnavailable, std::nullopt});
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.calibration_cache_valid = false;
    acceptance_snapshot_.last_status = IngressStatus::kCalibrationUnavailable;
    return;
  }

  VisionObservation observation;
  observation.depth = *depth;
  observation.camera_info = calibration->message;
  observation.depth_receipt_monotonic_ns = receipt_monotonic_ns;
  observation.rgb_presentation_status = "deferred_to_snapshot";
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.latest_depth_stamp_ns = depth_stamp;
    acceptance_snapshot_.latest_depth_receipt_monotonic_ns = receipt_monotonic_ns;
    acceptance_snapshot_.latest_camera_info_stamp_ns =
      stamp_ns(calibration->message.header.stamp);
    acceptance_snapshot_.latest_camera_info_receipt_monotonic_ns =
      calibration->receipt_monotonic_ns;
  }

  SynchronizedObservationCandidate candidate;
  candidate.depth = SensorSample{
    depth_stamp, receipt_monotonic_ns, static_cast<int32_t>(depth->width),
    static_cast<int32_t>(depth->height), depth->header.frame_id};
  candidate.camera_info = SensorSample{
    stamp_ns(calibration->message.header.stamp), calibration->receipt_monotonic_ns,
    static_cast<int32_t>(calibration->message.width),
    static_cast<int32_t>(calibration->message.height), calibration->message.header.frame_id};
  candidate.common_stamp_ns = depth_stamp;
  const bool compatible = SensorIngressPolicy::is_compatible_candidate(candidate);
  bool sensor_epoch_valid = false;
  {
    std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
    sensor_epoch_valid = !ingress_.epoch_invalidated();
  }
  if (!compatible || !sensor_epoch_valid) {
    std::function<void()> invalidator;
    {
      std::lock_guard<std::mutex> lock(observer_mutex_);
      invalidator = observation_invalidator_;
    }
    if (invalidator) {invalidator();}
    if (!compatible) {
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      if (cached_calibration_ && cached_calibration_->generation == calibration->generation) {
        cached_calibration_.reset();
      }
      std::lock_guard<std::mutex> observation_lock(observation_mutex_);
      cached_observation_.reset();
      cached_observation_stamp_ns_.reset();
      latest_observation_.reset();
    }
    set_last_result(IngressResult{
      compatible ? IngressStatus::kEpochInvalidated : IngressStatus::kIncompatibleMetadata,
      std::nullopt});
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.last_status = compatible ? IngressStatus::kEpochInvalidated :
      IngressStatus::kIncompatibleMetadata;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    latest_observation_ = observation;
    if (!cached_observation_stamp_ns_ || depth_stamp > *cached_observation_stamp_ns_) {
      cached_observation_stamp_ns_ = depth_stamp;
      cached_observation_ = CachedObservation{observation, candidate};
    }
  }
  std::function<void(const VisionObservation &)> observer;
  {
    std::lock_guard<std::mutex> lock(observer_mutex_);
    observer = observation_observer_;
  }
  if (observer) {observer(observation);}

  const auto result = [&]() {
      std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
      return ingress_.evaluate(candidate, monotonic_now_ns());
    }();
  set_last_result(result);
  {
    std::lock_guard<std::mutex> lock(request_diagnostics_mutex_);
    if (request_diagnostics_.active) {
      ++request_diagnostics_.policy_evaluations;
      if (result.status == IngressStatus::kObservationReady) {
        ++request_diagnostics_.observation_ready;
      }
      request_diagnostics_.last_status = result.status;
    }
  }
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    acceptance_snapshot_.calibration_cache_valid = true;
    acceptance_snapshot_.calibration_cache_generation = calibration->generation;
    acceptance_snapshot_.latest_camera_info_stamp_ns =
      stamp_ns(calibration->message.header.stamp);
    acceptance_snapshot_.latest_camera_info_receipt_monotonic_ns =
      calibration->receipt_monotonic_ns;
    acceptance_snapshot_.latest_canonical_stamp_ns = depth_stamp;
    acceptance_snapshot_.policy_delivery_count++;
    if (result.status == IngressStatus::kObservationReady) {
      acceptance_snapshot_.observation_ready_count++;
      acceptance_snapshot_.last_depth_observation_monotonic_ns = monotonic_now_ns();
    }
    acceptance_snapshot_.last_status = result.status;
    std::lock_guard<std::mutex> policy_lock(sensor_policy_mutex_);
    acceptance_snapshot_.epoch_invalidated = ingress_.epoch_invalidated();
  }
}

void VisionRosIngress::on_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & clock)
{
  const auto clock_stamp = stamp_ns(clock->clock);
  const auto receipt_monotonic_ns = monotonic_now_ns();
  bool epoch_invalidated = false;
  const auto result = [&]() {
      std::lock_guard<std::mutex> lock(sensor_policy_mutex_);
      const auto observed = clock_observer_.observe(clock_stamp);
      epoch_invalidated = ingress_.epoch_invalidated();
      return observed;
    }();
  {
    std::lock_guard<std::mutex> lock(acceptance_mutex_);
    const auto previous_stamp = acceptance_snapshot_.latest_clock_stamp_ns;
    if (!previous_stamp) {
      acceptance_snapshot_.last_clock_sample_relation =
        VisionClockSampleRelation::kFirstSample;
    } else if (clock_stamp < *previous_stamp) {
      acceptance_snapshot_.last_clock_sample_relation =
        VisionClockSampleRelation::kBackward;
    } else if (clock_stamp == *previous_stamp) {
      acceptance_snapshot_.last_clock_sample_relation =
        VisionClockSampleRelation::kUnchanged;
    } else {
      acceptance_snapshot_.last_clock_sample_relation =
        VisionClockSampleRelation::kAdvanced;
    }
    acceptance_snapshot_.clock_sample_count++;
    acceptance_snapshot_.latest_clock_stamp_ns = clock_stamp;
    acceptance_snapshot_.latest_clock_received_monotonic_ns = receipt_monotonic_ns;
    acceptance_snapshot_.last_clock_observation_status = result.status;
    if (result.status == IngressStatus::kTimeRollback) {
      ++acceptance_snapshot_.clock_epoch;
    }
  }
  if (result.status == IngressStatus::kTimeRollback ||
    result.status == IngressStatus::kEpochInvalidated)
  {
    std::function<void()> invalidator;
    {
      std::lock_guard<std::mutex> lock(observer_mutex_);
      invalidator = observation_invalidator_;
    }
    if (invalidator) {invalidator();}
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
