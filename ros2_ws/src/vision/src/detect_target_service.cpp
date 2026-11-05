#include "arm_cell_vision/detect_target_service.hpp"
#include "arm_cell_vision/vision_detector_snapshot.hpp"

#include <algorithm>
#include <chrono>
#include <atomic>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

namespace arm_cell_vision
{
namespace
{

std::atomic<uint64_t> next_acquisition_generation{1};

using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;

const char * result_code_name(uint8_t code)
{
  switch (code) {
    case ResultCode::DETECT_RESULT_SUCCESS: return "DETECT_RESULT_SUCCESS";
    case ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND: return "DETECT_RESULT_OBJECT_NOT_FOUND";
    case ResultCode::DETECT_RESULT_TIMEOUT: return "DETECT_RESULT_TIMEOUT";
    case ResultCode::DETECT_RESULT_SENSOR_ERROR: return "DETECT_RESULT_SENSOR_ERROR";
    case ResultCode::DETECT_RESULT_TF_ERROR: return "DETECT_RESULT_TF_ERROR";
    case ResultCode::DETECT_RESULT_GEOMETRY_ERROR: return "DETECT_RESULT_GEOMETRY_ERROR";
    case ResultCode::DETECT_RESULT_INVALID_RESULT: return "DETECT_RESULT_INVALID_RESULT";
    case ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET:
      return "DETECT_RESULT_TEMPORARY_INVALID_TARGET";
    case ResultCode::DETECT_RESULT_BUSY: return "DETECT_RESULT_BUSY";
    default: return "UNKNOWN";
  }
}

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
  target.diagnostic_detail = source.diagnostic_detail;
  target.scene_generation = source.scene_generation;
  if (source.result_code == ResultCode::DETECT_RESULT_SUCCESS ||
    source.result_code == ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET ||
    source.result_code == ResultCode::DETECT_RESULT_GEOMETRY_ERROR)
  {
    target.disturbance_candidates = source.disturbance_candidates;
  }
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

struct RgbdRequestSummaryGuard
{
  VisionRosIngress & ingress;

  ~RgbdRequestSummaryGuard()
  {
    (void)ingress.finish_request_diagnostics();
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
         << "}; last_ingress_status=" <<
    (status_observed_during_request && snapshot.last_status ?
  ingress_status_name(*snapshot.last_status) : "none_during_request");
  return detail.str();
}

std::string timeout_detail(
  const VisionIngressAcceptanceSnapshot & snapshot,
  const VisionIngressAcceptanceSnapshot & request_start,
  int64_t now_ns)
{
  const bool rgb_during_request = snapshot.rgb_message_count > request_start.rgb_message_count;
  const bool depth_ever = snapshot.depth_message_count > 0;
  const bool depth_during_request = snapshot.depth_message_count > request_start.depth_message_count;
  std::string reason;
  if (!depth_ever) {
    reason = "no depth observed";
  } else if (!depth_during_request) {
    reason = "no depth received during request; pre-request samples are not fresh evidence";
  } else if (!snapshot.calibration_cache_valid) {
    reason = "CameraInfo unavailable/invalid";
  } else if (snapshot.policy_delivery_count > request_start.policy_delivery_count &&
    snapshot.last_status &&
    (*snapshot.last_status == IngressStatus::kStaleObservation ||
    *snapshot.last_status == IngressStatus::kReceiptStaleObservation))
  {
    reason = "depth observation exists but is stale";
  } else if (snapshot.policy_delivery_count > request_start.policy_delivery_count &&
    snapshot.last_status &&
    *snapshot.last_status == IngressStatus::kIncompatibleMetadata)
  {
    reason = "depth observation rejected by geometry/calibration compatibility";
  } else if (snapshot.last_depth_observation_monotonic_ns &&
    (!request_start.last_depth_observation_monotonic_ns ||
    *snapshot.last_depth_observation_monotonic_ns >
    *request_start.last_depth_observation_monotonic_ns))
  {
    reason = "depth observation exists but was rejected or unavailable";
  } else {
    reason = "no usable depth observation";
  }
  std::ostringstream detail;
  detail << "acquisition deadline expired: " << reason
         << "; observed{rgb=" << (rgb_during_request ? "received" : "no_rgb")
         << ",depth=" << (depth_during_request ? "received" : "no_depth")
         << ",camera_info=" <<
    (snapshot.calibration_cache_valid ? "valid" : "unavailable_or_invalid")
         << "}; " << sensor_timing_detail(snapshot, now_ns,
    snapshot.policy_delivery_count > request_start.policy_delivery_count);
  return detail.str();
}

}  // namespace

std::string make_acquisition_id(const std::string & vision_run_id, uint64_t generation)
{
  return vision_run_id + "-acq-" + std::to_string(generation);
}

DetectTargetService::DetectTargetService(
  VisionRosIngress & ingress,
  SteadyNow steady_now,
  AcquisitionBegin acquisition_begin,
  AcquisitionEnd acquisition_end,
  AcquisitionResultLookup acquisition_result_lookup,
  AcquisitionDiagnosticLookup acquisition_diagnostic_lookup,
  AcquisitionTerminal acquisition_terminal,
  std::string vision_run_id,
  SupportNormalLookup support_normal_lookup)
: ingress_(ingress), steady_now_(std::move(steady_now)),
  acquisition_begin_(std::move(acquisition_begin)), acquisition_end_(std::move(acquisition_end)),
  acquisition_result_lookup_(std::move(acquisition_result_lookup)),
  acquisition_diagnostic_lookup_(std::move(acquisition_diagnostic_lookup)),
  acquisition_terminal_(std::move(acquisition_terminal)),
  support_normal_lookup_(std::move(support_normal_lookup)),
  vision_run_id_(vision_run_id.empty() ? make_vision_run_id() : std::move(vision_run_id))
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
  bool use_sim_time = false;
  node.get_parameter_or("use_sim_time", use_sim_time, false);
  if (use_sim_time) {
    mislocalize_service_ = node.create_service<std_srvs::srv::SetBool>(
      "/vision/test_scenario/mislocalize_next",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(scenario_mutex_);
        if (request->data && mislocalize_armed_) {
          response->success = false;
          response->message = "a Vision mislocalization is already pending";
          return;
        }
        if (request->data) {
          mislocalize_after_generation_ = next_acquisition_generation.load();
          mislocalize_armed_ = true;
          response->message = "next eligible successful Acquisition armed";
        } else {
          mislocalize_armed_ = false;
          response->message = "pending Vision mislocalization cleared";
        }
        response->success = true;
      }, rmw_qos_profile_services_default, callback_group_);
  }
  return service_server_;
}

void DetectTargetService::set_mislocalize_next(bool active)
{
  std::lock_guard<std::mutex> lock(scenario_mutex_);
  if (active) {
    if (mislocalize_armed_) {return;}
    mislocalize_after_generation_ = next_acquisition_generation.load();
    mislocalize_armed_ = true;
  } else {
    mislocalize_armed_ = false;
  }
}

bool DetectTargetService::mislocalize_pending() const
{
  std::lock_guard<std::mutex> lock(scenario_mutex_);
  return mislocalize_armed_;
}

void DetectTargetService::handle(const Request & request, Response & response)
{
  const auto request_start = steady_now_();
  bool diagnostic_warning_logged = false;
  const auto publish_diagnostic = [this, &request, &response, &diagnostic_warning_logged](
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
      diagnostic.result_code = state == arm_cell_interfaces::msg::VisionDiagnostic::IDLE ?
        response.result_code.value : detection.result_code;
      diagnostic.valid = diagnostic.result_code ==
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
      diagnostic.support_region_valid = detection.support_region_valid;
      diagnostic.support_region = detection.support_region;
      try {
        diagnostic_publisher_->publish(diagnostic);
      } catch (const std::exception & error) {
        if (!diagnostic_warning_logged) {
          diagnostic_warning_logged = true;
          RCLCPP_WARN(
            logger_, "Vision diagnostic publication failed; DetectTarget result is unaffected cause=%s",
            error.what());
        }
      }
    };
  DetectionResult empty_diagnostic;
  empty_diagnostic.result_code = ResultCode::DETECT_RESULT_INVALID_RESULT;
  const std_msgs::msg::Header empty_header;
  const auto log_result = [&](const std::string & failure_cause) {
      const auto code = response.result_code.value;
      if (code == ResultCode::DETECT_RESULT_SUCCESS) {
        RCLCPP_DEBUG(logger_, "Vision found a target observation");
      } else if (code == ResultCode::DETECT_RESULT_OBJECT_NOT_FOUND) {
        RCLCPP_DEBUG(logger_,
                     "Vision found no target matching the requested profile");
      } else if (code == ResultCode::DETECT_RESULT_TIMEOUT) {
        RCLCPP_WARN(
            logger_,
            "Vision acquisition did not finish before its deadline result=%s cause=%s "
            "target=%s algorithm=%s",
            result_code_name(code), failure_cause.c_str(), request.target_id.c_str(),
            request.algorithm.c_str());
      } else if (code == ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET) {
        RCLCPP_WARN(
            logger_,
            "Vision could not use the current target observation result=%s cause=%s",
            result_code_name(code), failure_cause.c_str());
      } else if (code == ResultCode::DETECT_RESULT_BUSY) {
        RCLCPP_DEBUG(
          logger_, "Vision request could not start result=%s cause=%s",
          result_code_name(code), failure_cause.c_str());
      } else {
        RCLCPP_ERROR(
            logger_,
            "Vision could not fulfill the detection request result=%s cause=%s",
            result_code_name(code), failure_cause.c_str());
      }
    };
  if (request.algorithm != "cuboid_top_face") {
    response.result_code.value = ResultCode::DETECT_RESULT_INVALID_RESULT;
    response.diagnostic_detail = "unsupported DetectTarget.algorithm: '" + request.algorithm + "'";
    log_result("unsupported DetectTarget.algorithm");
    return;
  }
  const auto timeout_ns = duration_ns(request.timeout);
  if (timeout_ns < 0) {
    response.result_code.value = ResultCode::DETECT_RESULT_INVALID_RESULT;
    response.diagnostic_detail = "timeout must be a non-negative duration";
    log_result("timeout must be a non-negative duration");
    return;
  }
  std::unique_lock<std::mutex> acquisition_lock(acquisition_mutex_, std::try_to_lock);
  if (!acquisition_lock.owns_lock()) {
    response.result_code.value = ResultCode::DETECT_RESULT_BUSY;
    response.acquisition_id.clear();
    response.diagnostic_detail = "request contention: another DetectTarget acquisition is active";
    log_result("another DetectTarget acquisition is active");
    return;
  }
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
  const RgbdRequestSummaryGuard summary_guard{ingress_};
  const auto snapshot = ingress_.acceptance_snapshot();
  const auto watermark = snapshot.latest_canonical_stamp_ns.value_or(0);
  const auto acquisition_generation = next_acquisition_generation.fetch_add(1);
  const auto acquisition_begin_result = acquisition_begin_ ? acquisition_begin_(
    request.target_id, acquisition_generation, request_start, watermark, request.algorithm) :
    ResultCode::DETECT_RESULT_SUCCESS;
  if (acquisition_begin_result != ResultCode::DETECT_RESULT_SUCCESS) {
    response.result_code.value = acquisition_begin_result;
    response.acquisition_id.clear();
    response.diagnostic_detail = acquisition_begin_result == ResultCode::DETECT_RESULT_BUSY ?
      "request contention: another Vision acquisition is active" :
      acquisition_begin_result == ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET ?
      "support calibration is not READY; DetectTarget request was not acquired" :
      "requested detector profile is unavailable";
    append_vision_elapsed();
    if (acquisition_begin_result != ResultCode::DETECT_RESULT_BUSY) {
      empty_diagnostic.result_code = response.result_code.value;
      publish_diagnostic(
        empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE, empty_header,
        response.diagnostic_detail);
    }
    log_result(acquisition_begin_result == ResultCode::DETECT_RESULT_BUSY ?
      "another Vision acquisition is active" :
      acquisition_begin_result == ResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET ?
      "support Calibration is not READY" : "requested detector profile is unavailable");
    return;
  }
  response.acquisition_id = make_acquisition_id(vision_run_id_, acquisition_generation);
  publish_diagnostic(
    empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::DETECTING,
    empty_header);
  struct AcquisitionEndGuard
  {
    VisionRosIngress & ingress;
    AcquisitionEnd end;
    ~AcquisitionEndGuard()
    {
      if (end) {end();}
      ingress.end_acquisition();
    }
  } acquisition_end_guard{ingress_, acquisition_end_};
  int64_t terminal_decision_ns = 0;
  SharedAcquisitionResult acquisition_result;
  struct AcquisitionTerminalGuard
  {
    const std::string & target_id;
    uint64_t generation;
    const Response & response;
    const SharedAcquisitionResult & acquisition_result;
    AcquisitionTerminal terminal;
    rclcpp::Logger logger;
    VisionRosIngress & ingress;
    int64_t request_deadline_ns;
    int64_t request_timeout_ns;
    int64_t sync_slop_ns;
    const int64_t & decision_ns;
    SteadyNow steady_now;
    ~AcquisitionTerminalGuard() noexcept
    {
      try {
        if (terminal) {
          const auto terminal_time = decision_ns == 0 ? steady_now() : decision_ns;
          VisionRequestDiagnosticRecord record;
          record.ingress = ingress.finish_request_diagnostics();
          record.clock = ingress.acceptance_snapshot();
          record.deadline_monotonic_ns = request_deadline_ns;
          record.decision_monotonic_ns = terminal_time;
          record.timeout_ns = request_timeout_ns;
          record.sync_slop_ns = sync_slop_ns;
          record.result_code = response.result_code.value;
          terminal(target_id, generation, response, acquisition_result, record);
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(logger,
          "vision terminal snapshot callback failed cause=%s", error.what());
      } catch (...) {
        RCLCPP_WARN(logger,
          "vision terminal snapshot callback failed cause=unknown_exception");
      }
    }
  } terminal_guard{request.target_id, acquisition_generation, response, acquisition_result,
    acquisition_terminal_, logger_, ingress_, request_deadline, timeout_ns,
    ingress_.sync_slop_ns(), terminal_decision_ns, steady_now_};
  const auto begin = ingress_.begin_acquisition(watermark, request_start, timeout_ns);
  if (begin.status != IngressStatus::kWaiting) {
    response.result_code.value = ResultCode::DETECT_RESULT_SENSOR_ERROR;
    response.diagnostic_detail = "unable to begin vision acquisition";
    append_vision_elapsed();
    empty_diagnostic.result_code = response.result_code.value;
    publish_diagnostic(
      empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
      empty_header);
    log_result("unable to begin Vision acquisition");
    return;
  }

  while (true) {
    const auto deadline_check_ns = steady_now_();
    if (deadline_check_ns >= request_deadline) {
      terminal_decision_ns = deadline_check_ns;
      break;
    }
    const auto retained = acquisition_result_lookup_ ?
      acquisition_result_lookup_(request.target_id, acquisition_generation) :
      SharedAcquisitionResult{};
    const auto * detection = retained ? &retained->detection : nullptr;
    const auto qualification_check_ns = steady_now_();
    if (retained && detection && qualification_check_ns < request_deadline &&
      retained->generation == acquisition_generation &&
      retained->target_id == request.target_id &&
      detection->scene_generation == acquisition_generation &&
      detection->result_code != ResultCode::DETECT_RESULT_INVALID_RESULT)
    {
      acquisition_result = retained;
      terminal_decision_ns = qualification_check_ns;
      copy_result(*detection, response);
      for (size_t i = 0; i < response.disturbance_candidates.size(); ++i) {
        response.disturbance_candidates[i].candidate_id = "disturbance-" +
          std::to_string(i + 1);
      }
      if (!response.disturbance_candidates.empty()) {
        using Disturbance = arm_cell_interfaces::msg::DisturbanceCandidate;
        const auto count_category = [&](uint8_t category) {
            return std::count_if(
              response.disturbance_candidates.begin(), response.disturbance_candidates.end(),
              [category](const auto & candidate) {return candidate.category == category;});
          };
        std::ostringstream disturbance_summary;
        disturbance_summary << "; disturbance_candidates=" <<
          response.disturbance_candidates.size() << "{inclined_surface=" <<
          count_category(Disturbance::CATEGORY_INCLINED_SURFACE) <<
          ",elevated_unresolved=" <<
          count_category(Disturbance::CATEGORY_ELEVATED_UNRESOLVED) <<
          ",other_unresolved=" <<
          count_category(Disturbance::CATEGORY_OTHER_UNRESOLVED) << "}";
        response.diagnostic_detail += disturbance_summary.str();
      }
      auto failure_cause = detection->failure_cause;
      if (response.result_code.value == ResultCode::DETECT_RESULT_SUCCESS) {
        const auto & candidates = detection->ordered_candidates;
        response.material_candidates.clear();
        response.material_candidates.reserve(candidates.size());
        uint32_t candidate_sequence = 1;
        for (size_t i = 0; i < candidates.size(); ++i) {
          const auto & source = candidates[i];
          arm_cell_interfaces::msg::PickCandidate value;
          value.acquisition_id = response.acquisition_id;
          value.candidate_id = "candidate-" + std::to_string(candidate_sequence++);
          value.target_pose = source.target_pose;
          value.top_height_tolerance_m =
            detection->algorithm == "cuboid_top_face" && detection->applied_profile ?
            detection->applied_profile->cuboid.height_tolerance_m : 0.0;
          value.orientation_symmetry_order = source.orientation_symmetry_order;
          value.estimated_height_m = source.estimated_height_m;
          value.has_estimated_height = source.has_estimated_height;
          value.has_observed_rectangle = source.has_observed_rectangle;
          value.rectangle_long_side_m = source.rectangle_long_side_m;
          value.rectangle_short_side_m = source.rectangle_short_side_m;
          if (source.confidence_grade == "HIGH" || source.confidence_grade == "MEDIUM")
          {
            response.material_candidates.push_back(std::move(value));
          }
        }
        if (response.material_candidates.empty()) {
          response.result_code.value = ResultCode::DETECT_RESULT_GEOMETRY_ERROR;
          response.diagnostic_detail += "; no_high_or_medium_candidate_after_classification";
        }
      }
      if (response.result_code.value == ResultCode::DETECT_RESULT_SUCCESS &&
        !response.material_candidates.empty())
      {
        std::lock_guard<std::mutex> lock(scenario_mutex_);
        if (mislocalize_armed_ && acquisition_generation >= mislocalize_after_generation_) {
          const auto support_normal = support_normal_lookup_ ?
            support_normal_lookup_(detection->source_rgb_header.frame_id) :
            std::optional<geometry_msgs::msg::Vector3>{};
          if (support_normal) {
            const double norm = std::sqrt(
              support_normal->x * support_normal->x +
              support_normal->y * support_normal->y +
              support_normal->z * support_normal->z);
            if (std::isfinite(norm) && norm > 1.0e-6) {
              const auto dx = support_normal->x / norm * 0.1;
              const auto dy = support_normal->y / norm * 0.1;
              const auto dz = support_normal->z / norm * 0.1;
              std::ostringstream scenario_detail;
              scenario_detail << std::fixed << std::setprecision(6)
                << "mislocalization_applied=true; offset_base_m=("
                << dx << ',' << dy << ',' << dz << "); acquisition_id="
                << response.acquisition_id;
              for (auto & candidate : response.material_candidates) {
                const auto original = candidate.target_pose.pose.position;
                candidate.target_pose.pose.position.x += dx;
                candidate.target_pose.pose.position.y += dy;
                candidate.target_pose.pose.position.z += dz;
                const auto & returned = candidate.target_pose.pose.position;
                scenario_detail << "; candidate_id=" << candidate.candidate_id <<
                  "; original_base_xyz=(" << original.x << ',' << original.y << ',' << original.z <<
                  "); returned_base_xyz=(" << returned.x << ',' << returned.y << ',' << returned.z << ')';
              }
              response.diagnostic_detail += "; " + scenario_detail.str();
              mislocalize_armed_ = false;
            } else {
              response.diagnostic_detail +=
                "; mislocalization_pending=true; support normal is invalid";
            }
          } else {
            response.diagnostic_detail +=
              "; mislocalization_pending=true; calibrated support normal unavailable";
          }
        }
      }
      append_vision_elapsed();
      publish_diagnostic(
        *detection, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
        detection->source_rgb_header, response.diagnostic_detail);
      log_result(failure_cause);
      return;
    }
    const auto current = ingress_.last_result();
    if (current && current->status == IngressStatus::kRequestTimeout) {
      terminal_decision_ns = steady_now_();
      break;
    }
    ingress_.check_deadline();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (terminal_decision_ns == 0) {terminal_decision_ns = steady_now_();}
  response.result_code.value = ResultCode::DETECT_RESULT_TIMEOUT;
  const auto timeout_snapshot = ingress_.acceptance_snapshot();
  response.diagnostic_detail = timeout_detail(timeout_snapshot, snapshot, terminal_decision_ns);
  AcquisitionDiagnosticSummary acquisition_summary;
  if (acquisition_diagnostic_lookup_) {
    acquisition_summary = acquisition_diagnostic_lookup_();
    if (!acquisition_summary.detail.empty()) {
      response.diagnostic_detail = acquisition_summary.detail + "; " + response.diagnostic_detail;
    }
  }
  append_vision_elapsed();
  empty_diagnostic.result_code = response.result_code.value;
  publish_diagnostic(
    empty_diagnostic, arm_cell_interfaces::msg::VisionDiagnostic::IDLE,
    empty_header, response.diagnostic_detail);
  log_result(acquisition_summary.failure_cause.empty() ?
    "Acquisition deadline expired" : acquisition_summary.failure_cause);
}

}  // namespace arm_cell_vision
