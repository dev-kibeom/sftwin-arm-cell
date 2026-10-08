#include "arm_cell_vision/detect_target_service.hpp"

#include <chrono>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

namespace arm_cell_vision
{
namespace
{

using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;

int64_t duration_ns(const builtin_interfaces::msg::Duration & duration)
{
  if (duration.sec < 0 || duration.nanosec >= 1'000'000'000U) {
    return -1;
  }
  const auto seconds = static_cast<int64_t>(duration.sec);
  if (seconds > std::numeric_limits<int64_t>::max() / 1'000'000'000LL) {
    return -1;
  }
  const auto result = seconds * 1'000'000'000LL + duration.nanosec;
  return result < 0 ? -1 : result;
}

void copy_result(const DetectionResult & source, DetectTargetService::Response & target)
{
  target.result_code.value = source.result_code;
  target.target_pose = source.target_pose;
  target.has_target_pose = source.has_target_pose;
  target.has_target_yaw = source.has_target_yaw;
  target.estimated_height_m = source.estimated_height_m;
  target.has_estimated_height = source.has_estimated_height;
  target.diagnostic_detail = source.diagnostic_detail;
}

const char * ingress_status_name(IngressStatus status)
{
  switch (status) {
    case IngressStatus::kWaiting: return "waiting";
    case IngressStatus::kNoActiveAcquisition: return "no_active_acquisition";
    case IngressStatus::kObservationReady: return "observation_ready";
    case IngressStatus::kDuplicateObservation: return "duplicate_observation";
    case IngressStatus::kSynchronizationSlopExceeded: return "sync_tolerance_exceeded";
    case IngressStatus::kStaleObservation: return "stale_observation";
    case IngressStatus::kReceiptStaleObservation: return "receipt_stale_observation";
    case IngressStatus::kCalibrationUnavailable: return "camera_info_unavailable_or_invalid";
    case IngressStatus::kIncompatibleMetadata: return "geometry_or_calibration_incompatible";
    case IngressStatus::kMalformedObservation: return "malformed_observation";
    case IngressStatus::kRequestTimeout: return "request_deadline";
    case IngressStatus::kTimeRollback: return "sensor_time_rollback";
    case IngressStatus::kEpochInvalidated: return "sensor_epoch_invalidated";
  }
  return "unknown_ingress_status";
}

std::string optional_ms(const std::optional<int64_t> & value)
{
  if (!value) {
    return "unavailable";
  }
  std::ostringstream output;
  output << std::fixed << std::setprecision(3) <<
    static_cast<double>(*value) / 1'000'000.0;
  return output.str();
}

std::string optional_stamp(const std::optional<int64_t> & value)
{
  return value ? std::to_string(*value) : "unavailable";
}

std::string optional_status(const std::optional<IngressStatus> & value)
{
  return value ? ingress_status_name(*value) : "none";
}

void log_rgbd_request_summary(
  const rclcpp::Logger & logger, const VisionIngressRequestDiagnostics & snapshot)
{
  const auto period = [](const std::optional<int64_t> & mean,
      const std::optional<int64_t> & latest) {
      return "mean=" + optional_ms(mean) + ",latest=" + optional_ms(latest);
    };
  const auto rgb_stamp_period = period(
    snapshot.rgb_stamp_period_mean_ns, snapshot.rgb_stamp_period_latest_ns);
  const auto rgb_receipt_period = period(
    snapshot.rgb_receipt_period_mean_ns, snapshot.rgb_receipt_period_latest_ns);
  const auto depth_stamp_period = period(
    snapshot.depth_stamp_period_mean_ns, snapshot.depth_stamp_period_latest_ns);
  const auto depth_receipt_period = period(
    snapshot.depth_receipt_period_mean_ns, snapshot.depth_receipt_period_latest_ns);
  const auto latest_pair_rgb_stamp = optional_stamp(snapshot.latest_pair_rgb_stamp_ns);
  const auto latest_pair_depth_stamp = optional_stamp(snapshot.latest_pair_depth_stamp_ns);
  const auto latest_delta = optional_ms(snapshot.latest_stamp_delta_ns);
  const auto minimum_delta = optional_ms(snapshot.minimum_cross_stream_delta_ns);
  const auto minimum_rgb_stamp = optional_stamp(snapshot.minimum_cross_rgb_stamp_ns);
  const auto minimum_depth_stamp = optional_stamp(snapshot.minimum_cross_depth_stamp_ns);
  const auto actual_pair_delta = optional_ms(snapshot.latest_pair_delta_ns);
  const auto last_status = optional_status(snapshot.last_status);
  const auto cached_pair_status = optional_status(snapshot.cached_pair_status);
  const bool ingress_observed = snapshot.rgb_received > 0 || snapshot.depth_received > 0;
  RCLCPP_INFO(
    logger,
    "RGB-D sync summary: request=%llu start_monotonic_ns=%lld watermark_ns=%lld "
    "ingress_during_request=%s rgb_received=%llu depth_received=%llu "
    "rgb_stamp_period_ms{%s} rgb_receipt_period_ms{%s} "
    "depth_stamp_period_ms{%s} depth_receipt_period_ms{%s} "
    "latest_received_rgb_depth_delta_ms=%s minimum_cross_stream_delta_ms=%s "
    "minimum_cross_rgb_stamp_ns=%s minimum_cross_depth_stamp_ns=%s "
    "sync_callbacks=%llu "
    "latest_pair_rgb_stamp_ns=%s latest_pair_depth_stamp_ns=%s actual_pair_delta_ms=%s "
    "pre_request_pair_candidates=%llu cached_pair_available=%s cached_pair_reused=%s "
    "cached_pair_status=%s unknown_pair_receipts=%llu "
    "watermark_rejected_pairs=%llu policy_evaluations=%llu observation_ready=%llu last_status=%s "
    "processing_started=%s source_capture_identity=unavailable "
    "sample_detail_dropped=%llu pair_detail_dropped=%llu",
    static_cast<unsigned long long>(snapshot.request_id),
    static_cast<long long>(snapshot.request_start_monotonic_ns),
    static_cast<long long>(snapshot.request_watermark_stamp_ns),
    ingress_observed ? "true" : "false",
    static_cast<unsigned long long>(snapshot.rgb_received),
    static_cast<unsigned long long>(snapshot.depth_received), rgb_stamp_period.c_str(),
    rgb_receipt_period.c_str(), depth_stamp_period.c_str(), depth_receipt_period.c_str(),
    latest_delta.c_str(), minimum_delta.c_str(), minimum_rgb_stamp.c_str(),
    minimum_depth_stamp.c_str(),
    static_cast<unsigned long long>(snapshot.sync_callbacks), latest_pair_rgb_stamp.c_str(),
    latest_pair_depth_stamp.c_str(), actual_pair_delta.c_str(),
    static_cast<unsigned long long>(snapshot.pre_request_pair_candidates),
    snapshot.cached_pair_available ? "true" : "false",
    snapshot.cached_pair_reused ? "true" : "false",
    cached_pair_status.c_str(),
    static_cast<unsigned long long>(snapshot.unknown_pair_receipt_count),
    static_cast<unsigned long long>(snapshot.watermark_rejected_pairs),
    static_cast<unsigned long long>(snapshot.policy_evaluations),
    static_cast<unsigned long long>(snapshot.observation_ready), last_status.c_str(),
    snapshot.processing_started ? "true" : "false",
    static_cast<unsigned long long>(snapshot.sample_detail_dropped),
    static_cast<unsigned long long>(snapshot.pair_detail_dropped));

  for (const auto & sample : snapshot.sample_details) {
    const auto elapsed_ms = static_cast<double>(
      sample.receipt_monotonic_ns - snapshot.request_start_monotonic_ns) / 1'000'000.0;
    RCLCPP_DEBUG(
      logger,
      "RGB-D sync sample: request=%llu stream=%s frame_id=%s stamp_ns=%lld "
      "receipt_monotonic_ns=%lld "
      "request_elapsed_ms=%.3f",
      static_cast<unsigned long long>(snapshot.request_id), sample.stream.c_str(),
      sample.frame_id.c_str(),
      static_cast<long long>(sample.stamp_ns),
      static_cast<long long>(sample.receipt_monotonic_ns), elapsed_ms);
  }
  for (const auto & pair : snapshot.pair_details) {
    const auto status = optional_status(pair.status);
    const auto evaluated = pair.policy_evaluated ? (*pair.policy_evaluated ? "true" : "false") :
      "unavailable";
    const auto observation_ready = pair.observation_ready ?
      (*pair.observation_ready ? "true" : "false") : "unavailable";
    RCLCPP_DEBUG(
      logger,
      "RGB-D sync pair: request=%llu pair=%llu rgb_stamp_ns=%lld depth_stamp_ns=%lld "
      "rgb_receipt_monotonic_ns=%lld depth_receipt_monotonic_ns=%lld delta_ms=%.3f "
      "pre_request_sample=%s watermark_passed=%s policy_evaluated=%s "
      "observation_ready=%s status=%s",
      static_cast<unsigned long long>(snapshot.request_id),
      static_cast<unsigned long long>(pair.sequence),
      static_cast<long long>(pair.rgb_stamp_ns), static_cast<long long>(pair.depth_stamp_ns),
      static_cast<long long>(pair.rgb_receipt_monotonic_ns),
      static_cast<long long>(pair.depth_receipt_monotonic_ns),
      static_cast<double>(pair.delta_ns) / 1'000'000.0,
      pair.pre_request_sample ? "true" : "false", pair.watermark_passed ? "true" : "false",
      evaluated, observation_ready, status.c_str());
  }
}

struct RgbdRequestSummaryGuard
{
  VisionRosIngress & ingress;
  rclcpp::Logger logger;

  ~RgbdRequestSummaryGuard()
  {
    log_rgbd_request_summary(logger, ingress.finish_request_diagnostics());
  }
};

std::string sensor_timing_detail(
  const VisionIngressAcceptanceSnapshot & snapshot, int64_t now_ns,
  bool status_observed_during_request)
{
  const auto age_ms = [now_ns](const std::optional<int64_t> & receipt) -> std::string {
      if (!receipt || now_ns < *receipt) {
        return "unavailable";
      }
      std::ostringstream value;
      value << std::fixed << std::setprecision(1) <<
        static_cast<double>(now_ns - *receipt) / 1'000'000.0 << "ms";
      return value.str();
    };
  std::ostringstream detail;
  detail << "latest_age_ms{rgb=" << age_ms(snapshot.latest_rgb_received_monotonic_ns)
         << ",depth=" << age_ms(snapshot.latest_depth_received_monotonic_ns)
         << ",camera_info=" << age_ms(snapshot.latest_camera_info_received_monotonic_ns)
         << "}; latest_received_rgb_depth_delta_ms=";
  if (snapshot.latest_rgb_depth_delta_ns) {
    detail << std::fixed << std::setprecision(3) <<
      static_cast<double>(*snapshot.latest_rgb_depth_delta_ns) / 1'000'000.0;
  } else {
    detail << "unavailable";
  }
  detail << "; last_ingress_status=" <<
    (status_observed_during_request && snapshot.last_status ?
  ingress_status_name(*snapshot.last_status) : "none_during_request");
  return detail.str();
}

std::string timeout_detail(
  const VisionIngressAcceptanceSnapshot & snapshot,
  const VisionIngressAcceptanceSnapshot & request_start,
  int64_t now_ns)
{
  const bool rgb_ever = snapshot.rgb_message_count > 0;
  const bool depth_ever = snapshot.depth_message_count > 0;
  const bool rgb_during_request = snapshot.rgb_message_count > request_start.rgb_message_count;
  const bool depth_during_request = snapshot.depth_message_count >
    request_start.depth_message_count;
  const bool new_pair_during_request =
    snapshot.synchronized_callback_count > request_start.synchronized_callback_count;
  std::string reason;
  if (!rgb_ever) {
    reason = "no RGB observed";
  } else if (!rgb_during_request) {
    reason = "no RGB received during request; pre-request samples are not fresh evidence";
  } else if (!depth_ever) {
    reason = "no depth observed";
  } else if (!depth_during_request) {
    reason = "no depth received during request; pre-request samples are not fresh evidence";
  } else if (!snapshot.calibration_cache_valid) {
    reason = "CameraInfo unavailable/invalid";
  } else if (!new_pair_during_request) {
    reason = "RGB/depth observed but no pair within sync tolerance";
  } else if (snapshot.policy_delivery_count > request_start.policy_delivery_count &&
    snapshot.last_status &&
    (*snapshot.last_status == IngressStatus::kStaleObservation ||
    *snapshot.last_status == IngressStatus::kReceiptStaleObservation))
  {
    reason = "synchronized observation exists but is stale";
  } else if (snapshot.policy_delivery_count > request_start.policy_delivery_count &&
    snapshot.last_status &&
    *snapshot.last_status == IngressStatus::kIncompatibleMetadata)
  {
    reason = "observation rejected by geometry/calibration compatibility";
  } else if (snapshot.last_pair_observation_monotonic_ns &&
    (!request_start.last_pair_observation_monotonic_ns ||
    *snapshot.last_pair_observation_monotonic_ns >
    *request_start.last_pair_observation_monotonic_ns))
  {
    reason = "synchronized observation exists but was rejected or unavailable";
  } else {
    reason = "no usable synchronized observation";
  }
  std::ostringstream detail;
  detail << "no valid observation before timeout: " << reason
         << "; observed{rgb=" << (rgb_during_request ? "received" : "no_rgb")
         << ",depth=" << (depth_during_request ? "received" : "no_depth")
         << ",camera_info=" <<
    (snapshot.calibration_cache_valid ? "valid" : "unavailable_or_invalid")
         << "}; " << sensor_timing_detail(snapshot, now_ns, new_pair_during_request)
         << "; processing_elapsed_ms=not_started";
  return detail.str();
}

}  // namespace

DetectTargetService::DetectTargetService(
  VisionRosIngress & ingress,
  DetectTargetProcessor processor,
  SteadyNow steady_now,
  TransformLookup transform_lookup,
  std::vector<DetectorProfile> detector_profiles)
: ingress_(ingress), processor_(std::move(processor)), steady_now_(std::move(steady_now)),
  transform_lookup_(std::move(transform_lookup)),
  detector_profiles_(std::move(detector_profiles))
{
}

rclcpp::Service<arm_cell_interfaces::srv::DetectTarget>::SharedPtr
DetectTargetService::advertise(rclcpp::Node & node)
{
  logger_ = node.get_logger();
  try {
    diagnostic_publisher_ = node.create_publisher<arm_cell_interfaces::msg::VisionDiagnostic>(
      "/vision/diagnostics", rclcpp::QoS(1).reliable().transient_local());
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger_, "vision diagnostic output unavailable; DetectTarget remains available: %s",
      error.what());
  }
  if (diagnostic_publisher_) {
    arm_cell_interfaces::msg::VisionDiagnostic initial;
    initial.state = arm_cell_interfaces::msg::VisionDiagnostic::IDLE;
    initial.result_code = ResultCode::DETECT_RESULT_INVALID_RESULT;
    try {
      diagnostic_publisher_->publish(initial);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        logger_, "vision diagnostic output unavailable; DetectTarget remains available: %s",
        error.what());
      diagnostic_publisher_.reset();
    }
  }
  callback_group_ = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  service_server_ = node.create_service<arm_cell_interfaces::srv::DetectTarget>(
    "/vision/detect_target",
    [this](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<arm_cell_interfaces::srv::DetectTarget::Request> request,
      std::shared_ptr<arm_cell_interfaces::srv::DetectTarget::Response> response) {
      handle(*request, *response);
    }, rmw_qos_profile_services_default, callback_group_);
  return service_server_;
}

void DetectTargetService::handle(const Request & request, Response & response)
{
  bool diagnostic_warning_logged = false;
  const auto publish_diagnostic = [this, &request, &diagnostic_warning_logged](
    const DetectionResult & detection, uint8_t state,
    const std_msgs::msg::Header & source_header,
    const std::string & diagnostic_detail = {}) {
      if (!diagnostic_publisher_) {
        return;
      }
      arm_cell_interfaces::msg::VisionDiagnostic diagnostic;
      diagnostic.state = state;
      diagnostic.target_id = request.target_id;
      diagnostic.source_rgb_header = source_header;
      diagnostic.diagnostic_detail = diagnostic_detail;
      diagnostic.result_code = detection.result_code;
      diagnostic.valid = detection.result_code ==
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
      diagnostic.centroid_pixel_valid = detection.centroid_pixel_valid;
      diagnostic.centroid_pixel_x = detection.centroid_pixel_x;
      diagnostic.centroid_pixel_y = detection.centroid_pixel_y;
      diagnostic.object_region_valid = detection.object_region_valid;
      diagnostic.object_region_x = detection.object_region_x;
      diagnostic.object_region_y = detection.object_region_y;
      diagnostic.object_region_width = detection.object_region_width;
      diagnostic.object_region_height = detection.object_region_height;
      diagnostic.support_region_valid = detection.support_region_valid;
      diagnostic.support_region = detection.support_region;
      diagnostic.yaw_available = detection.has_target_yaw;
      try {
        diagnostic_publisher_->publish(diagnostic);
      } catch (const std::exception &) {
        if (!diagnostic_warning_logged) {
          diagnostic_warning_logged = true;
          RCLCPP_WARN(
            logger_, "vision diagnostic publication failed; DetectTarget result is unaffected");
        }
      }
    };
  DetectionResult empty_diagnostic;
  empty_diagnostic.result_code = ResultCode::DETECT_RESULT_INVALID_RESULT;
  const std_msgs::msg::Header empty_header;
  publish_diagnostic(
    empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::DETECTING,
    empty_header);
  const auto log_result = [&]() {
      const auto code = response.result_code.value;
      if (code == ResultCode::DETECT_RESULT_SUCCESS) {
        RCLCPP_INFO(
          logger_, "target detection succeeded target_id=%s",
          request.target_id.c_str());
      } else if (code == ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND) {
        RCLCPP_INFO(
          logger_, "target detection found no available target target_id=%s diagnostic=%s",
          request.target_id.c_str(), response.diagnostic_detail.c_str());
      } else if (code == ResultCode::DETECT_RESULT_TIMEOUT) {
        RCLCPP_WARN(
          logger_, "target detection unavailable target_id=%s result_code=%u diagnostic=%s",
          request.target_id.c_str(), code, response.diagnostic_detail.c_str());
      } else if (code == ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET) {
        RCLCPP_WARN(
          logger_,
          "target detection temporarily unavailable target_id=%s result_code=%u diagnostic=%s",
          request.target_id.c_str(), code, response.diagnostic_detail.c_str());
      } else {
        RCLCPP_ERROR(
          logger_, "target detection failed target_id=%s result_code=%u diagnostic=%s",
          request.target_id.c_str(), code, response.diagnostic_detail.c_str());
      }
    };
  const auto timeout_ns = duration_ns(request.timeout);
  if (timeout_ns < 0) {
    response.result_code.value = ResultCode::DETECT_RESULT_INVALID_RESULT;
    response.diagnostic_detail = "timeout must be a non-negative duration";
    publish_diagnostic(
      empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
      empty_header);
    log_result();
    return;
  }
  const auto request_start = steady_now_();
  const RgbdRequestSummaryGuard summary_guard{ingress_, logger_};
  const auto append_vision_elapsed = [&]() {
      std::ostringstream timing;
      timing << "vision_total_elapsed_ms=" << std::fixed << std::setprecision(1)
             << static_cast<double>(steady_now_() - request_start) / 1'000'000.0;
      if (!response.diagnostic_detail.empty()) {
        response.diagnostic_detail += "; ";
      }
      response.diagnostic_detail += timing.str();
    };
  const auto request_deadline = timeout_ns > std::numeric_limits<int64_t>::max() - request_start ?
    std::numeric_limits<int64_t>::max() : request_start + timeout_ns;
  const auto snapshot = ingress_.acceptance_snapshot();
  const auto watermark = snapshot.latest_canonical_stamp_ns.value_or(0);
  const auto begin = ingress_.begin_acquisition(watermark, request_start, timeout_ns);
  if (begin.status != IngressStatus::kWaiting) {
    response.result_code.value = ResultCode::DETECT_RESULT_SENSOR_ERROR;
    response.diagnostic_detail = "unable to begin vision acquisition";
    append_vision_elapsed();
    empty_diagnostic.result_code = response.result_code.value;
    publish_diagnostic(
      empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
      empty_header);
    log_result();
    return;
  }

  while (steady_now_() < request_deadline) {
    const auto observation = ingress_.latest_observation();
    if (observation) {
      const auto observation_snapshot = ingress_.acceptance_snapshot();
      const auto acquisition_wait_ms = observation_snapshot.cached_pair_reused ?
        std::optional<double>(0.0) :
        observation_snapshot.last_pair_observation_monotonic_ns &&
        *observation_snapshot.last_pair_observation_monotonic_ns >= request_start ?
        std::optional<double>(
        static_cast<double>(
          *observation_snapshot.last_pair_observation_monotonic_ns - request_start) /
        1'000'000.0) : std::nullopt;
      const auto append_acquisition_wait = [&]() {
          if (!acquisition_wait_ms) {
            return;
          }
          std::ostringstream timing;
          timing << "acquisition_wait_ms=" << std::fixed << std::setprecision(1)
                 << *acquisition_wait_ms;
          if (!response.diagnostic_detail.empty()) {
            response.diagnostic_detail += "; ";
          }
          response.diagnostic_detail += timing.str();
        };
      const auto transform = transform_lookup_(observation->depth.header.frame_id, 0);
      if (!transform) {
        response.result_code.value = ResultCode::DETECT_RESULT_TF_ERROR;
        response.diagnostic_detail = "camera optical to base_link transform unavailable";
        append_acquisition_wait();
        append_vision_elapsed();
        empty_diagnostic.result_code = response.result_code.value;
        publish_diagnostic(
          empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
          observation->rgb.header);
        log_result();
        return;
      }
      if (steady_now_() >= request_deadline) {
        response.result_code.value = ResultCode::DETECT_RESULT_TIMEOUT;
        const auto timeout_snapshot = ingress_.acceptance_snapshot();
        response.diagnostic_detail = "processing deadline expired before detector start; " +
          sensor_timing_detail(timeout_snapshot, steady_now_(), true) +
          "; processing_elapsed_ms=not_started";
        append_acquisition_wait();
        append_vision_elapsed();
        empty_diagnostic.result_code = response.result_code.value;
        publish_diagnostic(
          empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
          observation->rgb.header, response.diagnostic_detail);
        log_result();
        return;
      }
      publish_diagnostic(
        empty_diagnostic,
        arm_cell_interfaces::msg::VisionDiagnostic::DETECTING,
        observation->rgb.header);
      ingress_.mark_detector_processing_started();
      const auto processing_start = steady_now_();
      const auto result = processor_.process(
        {observation->rgb, observation->depth, observation->camera_info, *transform},
        request.target_id, detector_profiles_);
      const auto processing_elapsed_ms =
        static_cast<double>(steady_now_() - processing_start) / 1'000'000.0;
      auto diagnostic_result = result;
      diagnostic_result.source_rgb_header = observation->rgb.header;
      if (steady_now_() >= request_deadline) {
        response.result_code.value = ResultCode::DETECT_RESULT_TIMEOUT;
        const auto processing_snapshot = ingress_.acceptance_snapshot();
        std::ostringstream detail;
        detail << "processing started but request deadline expired; processing_elapsed_ms="
               << std::fixed << std::setprecision(1) << processing_elapsed_ms << "; "
               << sensor_timing_detail(processing_snapshot, steady_now_(), true);
        response.diagnostic_detail = detail.str();
        diagnostic_result.result_code = response.result_code.value;
        diagnostic_result.has_target_pose = false;
        diagnostic_result.has_target_yaw = false;
        diagnostic_result.diagnostic_detail = response.diagnostic_detail;
      } else {
        copy_result(result, response);
      }
      if (steady_now_() < request_deadline) {
        std::ostringstream timing;
        timing << "processing_elapsed_ms=" << std::fixed << std::setprecision(1)
               << processing_elapsed_ms;
        if (!response.diagnostic_detail.empty()) {
          response.diagnostic_detail += "; ";
        }
        response.diagnostic_detail += timing.str();
      }
      append_acquisition_wait();
      append_vision_elapsed();
      publish_diagnostic(
        diagnostic_result,
        arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
        observation->rgb.header, response.diagnostic_detail);
      log_result();
      return;
    }
    const auto current = ingress_.last_result();
    if (current && current->status == IngressStatus::kObservationReady) {
      response.result_code.value = ResultCode::DETECT_RESULT_SENSOR_ERROR;
      response.diagnostic_detail = "observation payload unavailable";
      append_vision_elapsed();
      empty_diagnostic.result_code = response.result_code.value;
      publish_diagnostic(
        empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
        empty_header);
      log_result();
      return;
    }
    if (current && current->status == IngressStatus::kRequestTimeout) {
      response.result_code.value = ResultCode::DETECT_RESULT_TIMEOUT;
      const auto timeout_snapshot = ingress_.acceptance_snapshot();
      response.diagnostic_detail = timeout_detail(timeout_snapshot, snapshot, steady_now_());
      append_vision_elapsed();
      empty_diagnostic.result_code = response.result_code.value;
      publish_diagnostic(
        empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
        empty_header, response.diagnostic_detail);
      log_result();
      return;
    }
    ingress_.check_deadline();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  response.result_code.value = ResultCode::DETECT_RESULT_TIMEOUT;
  const auto timeout_snapshot = ingress_.acceptance_snapshot();
  response.diagnostic_detail = timeout_detail(timeout_snapshot, snapshot, steady_now_());
  append_vision_elapsed();
  empty_diagnostic.result_code = response.result_code.value;
  publish_diagnostic(
    empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
    empty_header, response.diagnostic_detail);
  log_result();
}

}  // namespace arm_cell_vision
