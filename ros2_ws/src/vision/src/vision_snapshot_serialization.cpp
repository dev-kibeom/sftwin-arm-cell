#include "vision_snapshot_internal.hpp"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include <shape_msgs/msg/solid_primitive.hpp>

namespace arm_cell_vision
{
namespace snapshot_detail
{
const char * event_name(uint8_t kind)
{
  switch (kind) {
    case 0: return "CALIBRATION_COMPLETE";
    case 1: return "DETECT_TARGET_PROGRESS";
    case 2: return "DETECT_TARGET_SUCCESS";
    case 3: return "DETECT_TARGET_FAILURE";
    case 4: return "CALIBRATION_PROGRESS";
    default: return "UNKNOWN";
  }
}

const char * result_code_name(uint8_t code)
{
  using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
  switch (code) {
    case Code::DETECT_RESULT_SUCCESS: return "DETECT_RESULT_SUCCESS";
    case Code::DETECT_RESULT_OBJECT_NOT_FOUND: return "DETECT_RESULT_OBJECT_NOT_FOUND";
    case Code::DETECT_RESULT_TIMEOUT: return "DETECT_RESULT_TIMEOUT";
    case Code::DETECT_RESULT_SENSOR_ERROR: return "DETECT_RESULT_SENSOR_ERROR";
    case Code::DETECT_RESULT_TF_ERROR: return "DETECT_RESULT_TF_ERROR";
    case Code::DETECT_RESULT_GEOMETRY_ERROR: return "DETECT_RESULT_GEOMETRY_ERROR";
    case Code::DETECT_RESULT_INVALID_RESULT: return "DETECT_RESULT_INVALID_RESULT";
    case Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET:
      return "DETECT_RESULT_TEMPORARY_INVALID_TARGET";
    default: return "UNKNOWN";
  }
}

std::string json_escape(const std::string & value)
{
  std::ostringstream out;
  for (const auto character : value) {
    switch (character) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(character) < 0x20) {out << ' ';} else {out << character;}
    }
  }
  return out.str();
}

void append_depth_scene_stability_json(
  std::ostringstream & json, const VisionDetectorSnapshotJob & job)
{
  const auto & final_state = job.depth_scene_stability;
  json << "{\"status\":\"" << depth_scene_status_name(final_state.status) <<
    "\",\"stable_depth_stamp_ns\":" << job.stable_depth_stamp_ns <<
    ",\"reason\":\"" << json_escape(final_state.reason) <<
    "\",\"last_reset_reason\":\"" << json_escape(final_state.last_reset_reason) <<
    "\",\"qualifying_observations\":" << final_state.qualifying_observations <<
    ",\"required_observations\":" << final_state.required_observations <<
    ",\"valid_depth_pixels\":" << final_state.valid_depth_pixels <<
    ",\"unobserved_depth_pixels\":" << final_state.unobserved_depth_pixels <<
    ",\"depth_difference_tolerance_m\":" << final_state.depth_difference_tolerance_m <<
    ",\"changed_pixel_ratio_tolerance\":" << final_state.changed_pixel_ratio_tolerance;
  if (job.depth_scene_diagnostics_enabled) {
    const auto append_comparison = [&json](const DepthSceneComparison & comparison) {
        json << "{\"comparable_pixels\":" << comparison.comparable_pixels <<
          ",\"changed_pixels\":" << comparison.changed_pixels <<
          ",\"changed_pixel_ratio\":";
        if (comparison.comparable_pixels == 0) {json << "null";}
        else {json << comparison.changed_pixel_ratio;}
        json <<
          ",\"absolute_difference_histogram_unit\":\"m\",\"bins\":[";
        for (size_t bin = 0; bin < comparison.absolute_difference_histogram.size(); ++bin) {
          if (bin) {json << ',';}
          json << "{\"range_m\":\"" << depth_difference_histogram_bin_label(bin) <<
            "\",\"pixels\":" << comparison.absolute_difference_histogram[bin] << '}';
        }
        json << "]}";
      };
    json << ",\"panorama\":\"scene_stability_panorama.png\",\"recent_observation_count\":" <<
      job.depth_scene_samples.size() << ",\"observations\":[";
    for (size_t i = 0; i < job.depth_scene_samples.size(); ++i) {
      if (i) {json << ',';}
      const auto & sample = job.depth_scene_samples[i];
      const auto & state = sample.state;
      json << "{\"index\":" << i + 1 << ",\"stamp_ns\":" << sample.stamp_ns <<
        ",\"receipt_monotonic_ns\":" << sample.receipt_monotonic_ns <<
        ",\"acquisition_elapsed_ms\":" << sample.acquisition_elapsed_ms <<
        ",\"processing_elapsed_ms\":" << sample.processing_elapsed_ms <<
        ",\"status\":\"" << depth_scene_status_name(state.status) <<
        "\",\"qualifying_observations\":" << state.qualifying_observations <<
        ",\"reason\":\"" << json_escape(state.reason) <<
        "\",\"last_reset_reason\":\"" << json_escape(state.last_reset_reason) <<
        "\",\"valid_depth_pixels\":" << state.valid_depth_pixels <<
        ",\"unobserved_depth_pixels\":" << state.unobserved_depth_pixels <<
        ",\"comparison_evidence_limited\":" <<
        (state.comparison_evidence_limited ? "true" : "false") <<
        ",\"depth_difference_tolerance_m\":" << state.depth_difference_tolerance_m <<
        ",\"changed_pixel_ratio_tolerance\":" << state.changed_pixel_ratio_tolerance <<
        ",\"reference_comparison\":";
      append_comparison(state.reference_comparison);
      json << ",\"previous_comparison\":";
      append_comparison(state.previous_comparison);
      json << '}';
    }
    json << ']';
  }
  json << '}';
}

void append_depth_validation_json(
  std::ostringstream & json, const SupportCalibrationSnapshot & calibration)
{
  json << ",\"depth_validation_samples\":[";
  for (size_t index = 0; index < calibration.recent_depth_validations.size(); ++index) {
    if (index != 0) {json << ',';}
    const auto & evidence = calibration.recent_depth_validations[index];
    json << "{\"valid_depth_sample_count\":" << evidence.valid_sample_count <<
      ",\"median_signed_residual_m\":";
    if (evidence.residual_available) {json << evidence.median_signed_residual_m;} else {json << "null";}
    json << ",\"minimum_signed_residual_m\":";
    if (evidence.residual_available) {json << evidence.minimum_signed_residual_m;} else {json << "null";}
    json << ",\"maximum_signed_residual_m\":";
    if (evidence.residual_available) {json << evidence.maximum_signed_residual_m;} else {json << "null";}
    json << ",\"support_contact_tolerance_m\":" << evidence.support_contact_tolerance_m <<
      ",\"configured_marker_height_m\":" << evidence.configured_marker_height_m <<
      ",\"accepted\":" << (evidence.accepted ? "true" : "false") <<
      ",\"reason\":\"" << json_escape(evidence.reason) << "\"}";
  }
  json << ']';
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
  return "unavailable";
}

const char * clock_sample_relation_name(VisionClockSampleRelation relation)
{
  switch (relation) {
    case VisionClockSampleRelation::kNotReceived: return "not_received";
    case VisionClockSampleRelation::kFirstSample: return "first_sample";
    case VisionClockSampleRelation::kAdvanced: return "advanced";
    case VisionClockSampleRelation::kUnchanged: return "unchanged";
    case VisionClockSampleRelation::kBackward: return "backward";
  }
  return "unavailable";
}

std::string request_diagnostics_json(const VisionRequestDiagnosticRecord & record)
{
  const auto & ingress = record.ingress;
  std::ostringstream json;
  json << "{\"request_id\":" << ingress.request_id << ",\"request_start_steady_ns\":" <<
    ingress.request_start_monotonic_ns << ",\"deadline_steady_ns\":" <<
    record.deadline_monotonic_ns << ",\"decision_steady_ns\":" <<
    record.decision_monotonic_ns << ",\"timeout_ns\":" << record.timeout_ns <<
    ",\"result_code\":" << static_cast<unsigned>(record.result_code) <<
    ",\"settling\":{\"status\":\"unavailable\",\"reason\":\"no Vision settling interval is applied or observed\"}" <<
    ",\"pairing\":{\"scope\":\"optional_rgbd_calibration_association\",\"sync_slop_ns\":" <<
    record.sync_slop_ns <<
    ",\"rgb_received\":" << ingress.rgb_received << ",\"depth_received\":" <<
    ingress.depth_received << ",\"sync_callbacks\":" << ingress.sync_callbacks <<
    ",\"policy_evaluations\":" << ingress.policy_evaluations <<
    ",\"observation_ready\":" << ingress.observation_ready
    << ",\"request_watermark_stamp_ns\":" << ingress.request_watermark_stamp_ns
    << ",\"minimum_cross_stream_delta_ns\":";
  if (ingress.minimum_cross_stream_delta_ns) {json << *ingress.minimum_cross_stream_delta_ns;} else {json << "null";}
  json << ",\"minimum_cross_rgb_stamp_ns\":";
  if (ingress.minimum_cross_rgb_stamp_ns) {json << *ingress.minimum_cross_rgb_stamp_ns;} else {json << "null";}
  json << ",\"minimum_cross_depth_stamp_ns\":";
  if (ingress.minimum_cross_depth_stamp_ns) {json << *ingress.minimum_cross_depth_stamp_ns;} else {json << "null";}
  json << ",\"latest_pair_rgb_stamp_ns\":";
  if (ingress.latest_pair_rgb_stamp_ns) {json << *ingress.latest_pair_rgb_stamp_ns;} else {json << "null";}
  json << ",\"latest_pair_depth_stamp_ns\":";
  if (ingress.latest_pair_depth_stamp_ns) {json << *ingress.latest_pair_depth_stamp_ns;} else {json << "null";}
  json << ",\"latest_pair_delta_ns\":";
  if (ingress.latest_pair_delta_ns) {json << *ingress.latest_pair_delta_ns;} else {json << "null";}
  json
    << ",\"cached_pair_available\":" << (ingress.cached_pair_available ? "true" : "false")
    << ",\"cached_pair_reused\":" << (ingress.cached_pair_reused ? "true" : "false")
    << ",\"cached_pair_status\":\"" <<
    (ingress.cached_pair_status ? ingress_status_name(*ingress.cached_pair_status) : "none")
    << "\",\"sample_details_dropped\":" << ingress.sample_detail_dropped
    << ",\"pair_details_dropped\":" << ingress.pair_detail_dropped << "},\"samples\":[";
  for (size_t index = 0; index < ingress.sample_details.size(); ++index) {
    const auto & sample = ingress.sample_details[index];
    if (index) {json << ',';}
    json << "{\"stream\":\"" << json_escape(sample.stream) << "\",\"frame_id\":\"" <<
      json_escape(sample.frame_id) << "\",\"source_stamp_ns\":" << sample.stamp_ns <<
      ",\"receipt_steady_ns\":" << sample.receipt_monotonic_ns << '}';
  }
  json << "],\"pairs\":[";
  for (size_t index = 0; index < ingress.pair_details.size(); ++index) {
    const auto & pair = ingress.pair_details[index];
    if (index) {json << ',';}
    json << "{\"sequence\":" << pair.sequence << ",\"rgb_frame_id\":\"" <<
      json_escape(pair.rgb_frame_id) << "\",\"depth_frame_id\":\"" <<
      json_escape(pair.depth_frame_id) << "\",\"rgb_stamp_ns\":" << pair.rgb_stamp_ns <<
      ",\"depth_stamp_ns\":" << pair.depth_stamp_ns << ",\"rgb_receipt_steady_ns\":" <<
      pair.rgb_receipt_monotonic_ns << ",\"depth_receipt_steady_ns\":" <<
      pair.depth_receipt_monotonic_ns << ",\"stamp_delta_ns\":" << pair.delta_ns
      << ",\"pre_request\":" << (pair.pre_request_sample ? "true" : "false")
      << ",\"watermark_passed\":" << (pair.watermark_passed ? "true" : "false")
      << ",\"policy_evaluated\":";
    if (pair.policy_evaluated) {json << (*pair.policy_evaluated ? "true" : "false");} else {json << "null";}
    json << ",\"observation_ready\":";
    if (pair.observation_ready) {json << (*pair.observation_ready ? "true" : "false");} else {json << "null";}
    json << ",\"status\":\"" <<
      (pair.status ? ingress_status_name(*pair.status) : "unavailable") << "\",\"freshness_evaluation\":";
    if (pair.freshness_evaluation) {
      const auto & freshness = *pair.freshness_evaluation;
      json << "{\"scope\":\"calibration_rgbd_association\","
           << "\"source_time_basis\":\"image_header_stamp_ros_time\","
           << "\"clock_time_basis\":\"ros_sim_time\","
           << "\"steady_time_basis\":\"local_steady_clock\","
           << "\"signed_delta_definition\":\"source_stamp_ns - evaluation_clock_ns\","
           << "\"gate_source\":\"rgb_source_stamp\",\"evaluation_steady_ns\":"
           << freshness.evaluation_steady_ns << ",\"evaluation_clock_ns\":";
      if (freshness.evaluation_clock_ns) {json << *freshness.evaluation_clock_ns;} else {json << "null";}
      json << ",\"rgb_source_stamp_ns\":" << freshness.rgb_source_stamp_ns
           << ",\"depth_source_stamp_ns\":" << freshness.depth_source_stamp_ns
           << ",\"rgb_source_minus_clock_ns\":";
      if (freshness.rgb_source_minus_clock_ns) {json << *freshness.rgb_source_minus_clock_ns;} else {json << "null";}
      json << ",\"depth_source_minus_clock_ns\":";
      if (freshness.depth_source_minus_clock_ns) {json << *freshness.depth_source_minus_clock_ns;} else {json << "null";}
      json << ",\"clock_received_steady_ns\":";
      if (freshness.clock_received_steady_ns) {json << *freshness.clock_received_steady_ns;} else {json << "null";}
      json << ",\"clock_receipt_age_ns\":";
      if (freshness.clock_receipt_age_ns) {json << *freshness.clock_receipt_age_ns;} else {json << "null";}
      json << ",\"maximum_absolute_delta_ns\":" << freshness.maximum_absolute_delta_ns
           << ",\"accepted\":" << (freshness.accepted ? "true" : "false")
           << ",\"reason\":\"" << json_escape(freshness.reason)
           << "\",\"clock_sample_relation\":\""
           << clock_sample_relation_name(freshness.clock_sample_relation)
           << "\",\"clock_observation_status\":\""
           << (freshness.clock_observation_status ?
      ingress_status_name(*freshness.clock_observation_status) : "not_received")
           << "\",\"clock_sample_count\":" << freshness.clock_sample_count
           << ",\"clock_epoch\":" << freshness.clock_epoch
           << ",\"clock_rollback_observed\":"
           << (freshness.clock_rollback_observed ? "true" : "false")
           << ",\"epoch_invalidated\":" << (freshness.epoch_invalidated ? "true" : "false")
           << '}';
    } else {
      json << "null";
    }
    json << '}';
  }
  json << "],\"clock_epoch\":{\"start\":" << ingress.clock_epoch_start <<
    ",\"decision\":" << record.clock.clock_epoch << ",\"latest_clock_stamp_ns\":";
  if (record.clock.latest_clock_stamp_ns) {json << *record.clock.latest_clock_stamp_ns;} else {json << "null";}
  json << ",\"latest_clock_received_steady_ns\":";
  if (record.clock.latest_clock_received_monotonic_ns) {json << *record.clock.latest_clock_received_monotonic_ns;} else {json << "null";}
  json << ",\"last_clock_sample_relation\":\""
       << clock_sample_relation_name(record.clock.last_clock_sample_relation)
       << "\",\"last_clock_observation_status\":\""
       << (record.clock.last_clock_observation_status ?
    ingress_status_name(*record.clock.last_clock_observation_status) : "not_received")
       << "\",\"clock_sample_count\":" << record.clock.clock_sample_count
       << ",\"clock_rollback_observed\":" <<
    (record.clock.clock_rollback_observed ? "true" : "false")
       << ",\"epoch_invalidated\":" << (record.clock.epoch_invalidated ? "true" : "false")
       << "},\"freshness\":{\"scope\":\"calibration_rgbd_association\","
       << "\"source_time_basis\":\"image_header_stamp_ros_time\","
       << "\"clock_time_basis\":\"ros_sim_time\","
       << "\"steady_time_basis\":\"local_steady_clock\","
       << "\"signed_delta_definition\":\"source_stamp_ns - evaluation_clock_ns\","
       << "\"gate_source\":\"rgb_source_stamp\",\"evaluated\":"
       << ingress.source_stamp_freshness_evaluated
       << ",\"accepted\":" << ingress.source_stamp_freshness_accepted
       << ",\"rejected\":" << ingress.source_stamp_freshness_rejected
       << ",\"clock_unavailable\":" << ingress.source_stamp_freshness_clock_unavailable
       << ",\"clock_delta_rejected\":" << ingress.source_stamp_freshness_clock_delta_rejected
       << "},\"depth_source_freshness\":{\"scope\":\"detect_target_depth_source\",\"evaluated\":" <<
    ingress.depth_source_stamp_freshness_evaluated << ",\"accepted\":" <<
    ingress.depth_source_stamp_freshness_accepted << ",\"rejected\":" <<
    ingress.depth_source_stamp_freshness_rejected << ",\"observations\":[";
  for (size_t index = 0; index < ingress.depth_source_freshness_details.size(); ++index) {
    if (index) {json << ',';}
    const auto & freshness = ingress.depth_source_freshness_details[index];
    json << "{\"depth_source_stamp_ns\":" << freshness.depth_source_stamp_ns <<
      ",\"evaluation_steady_ns\":" << freshness.evaluation_steady_ns <<
      ",\"evaluation_clock_ns\":";
    if (freshness.evaluation_clock_ns) {json << *freshness.evaluation_clock_ns;} else {json << "null";}
    json << ",\"depth_source_minus_clock_ns\":";
    if (freshness.depth_source_minus_clock_ns) {json << *freshness.depth_source_minus_clock_ns;} else {json << "null";}
    json << ",\"maximum_absolute_delta_ns\":" << freshness.maximum_absolute_delta_ns <<
      ",\"accepted\":" << (freshness.accepted ? "true" : "false") <<
      ",\"reason\":\"" << json_escape(freshness.reason) << "\",\"clock_epoch\":" <<
      freshness.clock_epoch << ",\"epoch_invalidated\":" <<
      (freshness.epoch_invalidated ? "true" : "false") << '}';
  }
  json << "]}}";
  return json.str();
}

std::string header_json(const std_msgs::msg::Header & header)
{
  std::ostringstream out;
  out << "{\"stamp\":{\"sec\":" << header.stamp.sec << ",\"nanosec\":" <<
    header.stamp.nanosec << "},\"frame_id\":\"" << json_escape(header.frame_id) << "\"}";
  return out.str();
}

bool is_renderable_presentation_rgb(const sensor_msgs::msg::Image & image)
{
  return image.width > 0 && image.height > 0 && !image.header.frame_id.empty() &&
         image.encoding == "rgb8" &&
         static_cast<size_t>(image.step) >= static_cast<size_t>(image.width) * 3U &&
         image.data.size() >= static_cast<size_t>(image.step) * image.height;
}

std::string presentation_rgb_status(const DetectionInput & input)
{
  const auto & image = input.rgb;
  if (is_renderable_presentation_rgb(image)) {return "available";}
  if (image.width == 0 || image.height == 0) {
    return input.rgb_presentation_status.empty() ? "not_received" :
           input.rgb_presentation_status;
  }
  if (image.encoding != "rgb8") {return "encoding_unsupported";}
  if (static_cast<size_t>(image.step) < static_cast<size_t>(image.width) * 3U) {
    return "step_too_small";
  }
  if (image.data.size() < static_cast<size_t>(image.step) * image.height) {
    return "buffer_too_short";
  }
  if (image.header.frame_id.empty()) {return "frame_id_missing";}
  return input.rgb_presentation_status.empty() ? "incompatible" :
         input.rgb_presentation_status;
}

void append_observation_identity_json(
  std::ostringstream & json, const DetectionInput & input, const char * depth_key)
{
  const auto & rgb = input.rgb;
  const bool rgb_available = is_renderable_presentation_rgb(rgb);
  const auto rgb_status = presentation_rgb_status(input);
  json << "{\"rgb_available\":" << (rgb_available ? "true" : "false") <<
    ",\"rgb_presentation_status\":\"" << json_escape(rgb_status) << "\",\"rgb\":";
  if (rgb_available) {json << header_json(rgb.header);} else {json << "null";}
  json << ",\"" << depth_key << "\":" << header_json(input.depth.header) << '}';
}

const char * disturbance_category_name(uint8_t category)
{
  using Candidate = arm_cell_interfaces::msg::DisturbanceCandidate;
  switch (category) {
    case Candidate::CATEGORY_INCLINED_SURFACE: return "INCLINED_SURFACE";
    case Candidate::CATEGORY_ELEVATED_UNRESOLVED: return "ELEVATED_UNRESOLVED";
    case Candidate::CATEGORY_OTHER_UNRESOLVED: return "OTHER_UNRESOLVED";
    default: return "UNKNOWN";
  }
}

std::string disturbance_provenance_json(const DisturbanceRegionDiagnostic & region)
{
  std::ostringstream out;
  out << "{\"reason\":\"" << json_escape(region.reason) << "\"";
  const auto append_ids = [&](const char * name, const std::vector<uint32_t> & ids) {
      out << ",\"" << name << "\":[";
      for (size_t i = 0; i < ids.size(); ++i) {
        if (i) {out << ',';} out << ids[i];
      }
      out << ']';
    };
  out << ",\"source_boundary_id\":" << region.source_boundary_id <<
    ",\"outline_id\":" << region.outline_id << ",\"patch_index\":" << region.patch_index <<
    ",\"rectangle_id\":" << region.rectangle_id << ",\"representative_method\":\"" <<
    json_escape(region.representative_method) << "\"";
  append_ids("hypothesis_ids", region.hypothesis_ids);
  out << '}';
  return out.str();
}

std::string diagnostic_result_summary_json(const DetectionResult & result, bool candidate_local = false)
{
  std::ostringstream out;
  out << std::setprecision(12) << "{\"algorithm\":\"" << json_escape(result.algorithm) <<
    "\",\"processing_stage\":\"" << json_escape(result.processing_stage) <<
    "\",\"result_code\":" << static_cast<unsigned>(result.result_code) <<
    ",\"result_name\":\"" << result_code_name(result.result_code) <<
    "\",\"diagnostic_detail\":\"" << json_escape(result.diagnostic_detail) <<
    "\",\"ordered_candidate_count\":" << result.ordered_candidates.size();
  if (candidate_local || result.algorithm != "cuboid_top_face") {
    out << ",\"has_target_pose\":" << (result.has_target_pose ? "true" : "false");
  }
  out << ",\"disturbance_candidates\":[";
  for (size_t i = 0; i < result.disturbance_candidates.size(); ++i) {
    if (i) {out << ',';}
    const auto & candidate = result.disturbance_candidates[i];
    out << "{\"candidate_id\":\"" << json_escape(candidate.candidate_id) <<
      "\",\"category\":\"" << disturbance_category_name(candidate.category) <<
      "\",\"frame_id\":\"" << json_escape(candidate.center.header.frame_id) <<
      "\",\"position_m\":[" << candidate.center.pose.position.x << ',' <<
      candidate.center.pose.position.y << ',' << candidate.center.pose.position.z <<
      "],\"observed_height_m\":" << candidate.observed_height_m;
    const auto region = std::find_if(result.disturbance_regions.begin(), result.disturbance_regions.end(),
      [&](const auto & item) {return item.candidate_id == candidate.candidate_id;});
    if (region != result.disturbance_regions.end()) {
      out << ",\"observation_provenance\":" << disturbance_provenance_json(*region);
    }
    out << '}';
  }
  out << "],\"unresolved_pixels\":" << result.unresolved_pixels;
  out << ",\"material_candidates\":[";
  for (size_t i = 0; i < result.ordered_candidates.size(); ++i) {
    if (i) {out << ',';}
    out << "{\"candidate_id\":\"candidate-" << i + 1 << "\",\"observation_candidate_id\":\"" <<
      json_escape(result.ordered_candidates[i].rawpart_selected_candidate) <<
      "\",\"evidence\":" << diagnostic_result_summary_json(result.ordered_candidates[i], true) << '}';
  }
  out << ']';
  if (candidate_local) {
    out << ",\"confidence_grade\":\"" << json_escape(result.confidence_grade) <<
      "\",\"confidence_reason\":\"" << json_escape(result.confidence_reason) << "\"";
  }
  if ((candidate_local || result.algorithm != "cuboid_top_face") && result.has_target_pose) {
    out << ",\"target_pose\":{\"frame_id\":\"" <<
      json_escape(result.target_pose.header.frame_id) << "\",\"stamp_ns\":" <<
      static_cast<int64_t>(result.target_pose.header.stamp.sec) * 1000000000LL +
      result.target_pose.header.stamp.nanosec << ",\"position_m\":[" <<
      result.target_pose.pose.position.x << ',' << result.target_pose.pose.position.y << ',' <<
      result.target_pose.pose.position.z << "],\"orientation_xyzw\":[" <<
      result.target_pose.pose.orientation.x << ',' << result.target_pose.pose.orientation.y << ',' <<
      result.target_pose.pose.orientation.z << ',' << result.target_pose.pose.orientation.w << "]}";
  }
  if (candidate_local) {
    out << ",\"orientation_symmetry_order\":" << unsigned(result.orientation_symmetry_order) <<
      ",\"has_estimated_height\":" << (result.has_estimated_height ? "true" : "false") <<
      ",\"estimated_height_m\":" << result.estimated_height_m <<
      ",\"has_observed_rectangle\":" << (result.has_observed_rectangle ? "true" : "false") <<
      ",\"rectangle_dimensions_m\":[" << result.rectangle_long_side_m << ',' <<
      result.rectangle_short_side_m << ']';
  }
  if (result.has_rawpart_diagnostics) {
    out << ",\"candidate_summary\":{\"workspace_xyz_points\":" <<
      result.rawpart_workspace_xyz_points << ",\"candidate_count\":" <<
      result.rawpart_candidate_count << ",\"profile_match_count\":" <<
      result.rawpart_profile_match_count << ",\"candidate_generation\":\"" <<
      json_escape(result.rawpart_candidate_generation) << "\",\"absence_confirmation_available\":" <<
      (result.rawpart_absence_confirmation_available ? "true" : "false") <<
      ",\"absence_confirmed\":" << (result.rawpart_absence_confirmed ? "true" : "false") <<
      ",\"partition_status\":\"" << json_escape(result.rawpart_candidate_generation) << "\"" <<
      ",\"rejected_by_reason\":{";
    bool first = true;
    for (const auto & [reason, count] : result.rawpart_rejected_by_reason) {
      if (!first) {out << ',';}
      first = false;
      out << "\"" << json_escape(reason) << "\":" << count;
    }
    out << "},\"candidates\":[";
    for (size_t i = 0; i < result.rawpart_candidate_diagnostics.size(); ++i) {
      if (i) {out << ',';}
      const auto & candidate = result.rawpart_candidate_diagnostics[i];
      out << "{\"candidate_id\":" << candidate.candidate_id <<
        ",\"confidence_grade\":\"" << json_escape(candidate.confidence_grade) <<
        "\",\"confidence_reason\":\"" << json_escape(candidate.confidence_reason) << "\"" <<
        ",\"profile_match\":" << (candidate.profile_match ? "true" : "false") <<
        ",\"rejection_reason\":\"" << json_escape(candidate.rejection_reason) << "\"";
      out <<
        ",\"position_m\":[" << candidate.position_x_m << ',' << candidate.position_y_m << ',' <<
        candidate.position_z_m << "],\"measured_dimensions_m\":[" << candidate.measured_width_m << ',' <<
        candidate.measured_height_m << ',' << candidate.measured_vertical_height_m <<
        "],\"expected_dimensions_m\":[" << candidate.expected_width_m << ',' <<
        candidate.expected_height_m << ',' << candidate.expected_vertical_height_m <<
        "],\"dimension_tolerance_m\":[" << candidate.width_tolerance_m << ',' <<
        candidate.height_tolerance_m << ',' << candidate.vertical_height_tolerance_m <<
        "],\"orientation_symmetry_order\":" <<
        static_cast<unsigned>(candidate.orientation_symmetry_order) <<
        ",\"observed_yaw_valid\":" << (candidate.observed_yaw_valid ? "true" : "false") <<
        (candidate.observed_yaw_valid ? ",\"observed_yaw_rad\":" : "") <<
        (candidate.observed_yaw_valid ? std::to_string(candidate.observed_yaw_rad) : std::string{}) <<
        ",\"inlier_fraction\":" << candidate.inlier_fraction << ",\"coverage\":" <<
        candidate.coverage << ",\"edge_support\":" << candidate.edge_support <<
        ",\"max_normalized_residual\":" << candidate.max_normalized_residual <<
        ",\"selection_residual\":" << candidate.selection_residual <<
        ",\"fitted_outline_pixels\":[";
      for (size_t p = 0; p < candidate.fitted_outline_pixels.size(); ++p) {
        if (p) {out << ',';}
        out << '[' << candidate.fitted_outline_pixels[p].x << ',' <<
          candidate.fitted_outline_pixels[p].y << ']';
      }
      out << "],\"selected_for_target\":" <<
        (candidate.selected_for_target ? "true" : "false") << '}';
    }
    out << "]}";
  }
  out << '}';
  return out.str();
}

}  // namespace snapshot_detail

using snapshot_detail::append_depth_scene_stability_json;
using snapshot_detail::append_depth_validation_json;
using snapshot_detail::append_observation_identity_json;
using snapshot_detail::diagnostic_result_summary_json;
using snapshot_detail::disturbance_category_name;
using snapshot_detail::event_name;
using snapshot_detail::header_json;
using snapshot_detail::is_renderable_presentation_rgb;
using snapshot_detail::json_escape;
using snapshot_detail::presentation_rgb_status;
using snapshot_detail::request_diagnostics_json;
using snapshot_detail::result_code_name;

std::string VisionDetectorSnapshotOutput::serialize_metadata(
  const VisionDetectorSnapshotJob & job,
  const arm_cell_interfaces::msg::VisionDetectorSnapshot & message,
  bool source_image_rendered,
  const std::vector<std::optional<cv::Point>> & disturbance_projections) const
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  const auto & disturbance_candidates = job.detection_result().disturbance_candidates;
  const size_t displayed_disturbance_count = job.has_detection_result ?
    disturbance_candidates.size() : 0U;
  std::ostringstream json;
  json << std::setprecision(12) << "{\"schema_version\":19,\"snapshot_id\":\"" <<
    json_escape(message.snapshot_id) << "\",\"revision\":" << message.revision <<
    ",\"event_kind\":\"" << event_name(message.event_kind) << "\",\"vision_run_id\":\"" <<
    json_escape(message.vision_run_id) << "\",\"calibration_id\":\"" <<
    json_escape(message.calibration_id) << "\",\"acquisition_id\":\"" <<
    json_escape(message.acquisition_id) << '"';
  if (message.event_kind == Message::DETECT_TARGET_SUCCESS ||
    message.event_kind == Message::DETECT_TARGET_FAILURE)
  {
    json << ",\"scene_generation\":" << job.terminal_scene_generation;
  }
  json << ",\"image_available\":" <<
    (message.image_png.data.empty() ? "false" : "true") <<
    ",\"source_image_rendered\":" << (source_image_rendered ? "true" : "false");
  json << ",\"observation\":";
  if (job.has_observation && job.observation_input()) {
    append_observation_identity_json(json, *job.observation_input(), "aligned_depth");
  } else {json << "null";}
  if (!job.target_id.empty()) {
    json << ",\"target_id\":\"" << json_escape(job.target_id) << "\"";
  }
  if (message.event_kind == Message::CALIBRATION_COMPLETE ||
    !job.calibration_data().recent_depth_validations.empty()) {
    json << ",\"calibration_summary\":{\"ready\":" <<
      (job.calibration_data().ready ? "true" : "false") << ",\"accepted_observations\":" <<
      job.calibration_data().accepted_frames << ",\"required_observations\":" <<
      job.calibration_data().required_frames << ",\"median_depth_residual_m\":" <<
      job.calibration_data().median_depth_residual_m;
    append_depth_validation_json(json, job.calibration_data());
    if (job.calibration_data().ready) {
      const auto & transform = job.calibration_data().tray_from_camera.transform;
      json << ",\"tray_from_camera\":{\"translation\":[" << transform.translation.x << ',' <<
        transform.translation.y << ',' << transform.translation.z << "],\"rotation_xyzw\":[" <<
        transform.rotation.x << ',' << transform.rotation.y << ',' << transform.rotation.z << ',' <<
        transform.rotation.w << "]}";
    }
    json << '}';
  }
  json << ",\"snapshot_style\":{\"candidate_color_rgb\":\"" <<
    style_.candidate_color_rgb << "\",\"candidate_line_width_px\":" <<
    style_.candidate_line_width_px << ",\"unresolved_color_rgb\":\"" <<
    style_.unresolved_color_rgb << "\",\"target_color_rgb\":\"" <<
    style_.target_color_rgb << "\",\"support_color_rgb\":\"" <<
    style_.support_color_rgb << "\",\"disturbance_inclined_color_rgb\":\"" <<
    style_.disturbance_inclined_color_rgb << "\",\"disturbance_elevated_color_rgb\":\"" <<
    style_.disturbance_elevated_color_rgb << "\",\"disturbance_other_color_rgb\":\"" <<
    style_.disturbance_other_color_rgb << "\",\"region_line_width_px\":" <<
    style_.region_line_width_px << ",\"measurement_color_rgb\":\"" <<
    style_.measurement_color_rgb << "\",\"marker_size_px\":" << style_.marker_size_px <<
    ",\"measurement_line_width_px\":" << style_.measurement_line_width_px <<
    ",\"show_annotation_text\":" << (style_.show_annotation_text ? "true" : "false") <<
    ",\"annotation_text_scale\":" << style_.annotation_text_scale <<
    ",\"primary_text_color_rgb\":\"" << style_.primary_text_color_rgb <<
    "\",\"detail_text_color_rgb\":\"" << style_.detail_text_color_rgb << "\"}";
  json << ",\"annotation\":{\"processing_label\":\"" <<
    json_escape(job.processing_stage_data()) << "\",\"target_id\":\"" <<
    json_escape(job.target_id) << "\",\"candidate_count_displayed\":" << job.detection_result().rawpart_candidate_count;
  if (job.detection_result().algorithm != "cuboid_top_face") {
    json << ",\"has_target_pose\":" <<
      (job.detection_result().has_target_pose ? "true" : "false");
  }
  if (job.detection_result().algorithm != "cuboid_top_face" &&
    job.detection_result().has_target_pose) {
    json << ",\"target_pose_frame_id\":\"" <<
      json_escape(job.detection_result().target_pose.header.frame_id) << "\",\"target_pose_position_m\":[" <<
      job.detection_result().target_pose.pose.position.x << ',' << job.detection_result().target_pose.pose.position.y << ',' <<
      job.detection_result().target_pose.pose.position.z << "],\"orientation_symmetry_order\":" <<
      static_cast<unsigned>(job.detection_result().orientation_symmetry_order);
  }
  json << ",\"disturbance_candidates\":[";
  for (size_t i = 0; i < displayed_disturbance_count; ++i) {
    if (i) {json << ',';}
    const auto & candidate = disturbance_candidates[i];
    const bool projected = i < disturbance_projections.size() && disturbance_projections[i].has_value();
    json << "{\"candidate_id\":\"" << json_escape(candidate.candidate_id) <<
      "\",\"category\":\"" << disturbance_category_name(candidate.category) <<
      "\",\"observed_height_m\":" << candidate.observed_height_m <<
      ",\"center\":{\"frame_id\":\"" << json_escape(candidate.center.header.frame_id) <<
      "\",\"stamp_ns\":" << static_cast<int64_t>(candidate.center.header.stamp.sec) * 1000000000LL +
      candidate.center.header.stamp.nanosec << ",\"position_m\":[" <<
      candidate.center.pose.position.x << ',' << candidate.center.pose.position.y << ',' <<
      candidate.center.pose.position.z << "]},\"projection_status\":\"" <<
      (projected ? "projected" : "unavailable") <<
      "\",\"rgb_pixel_xy\":";
    if (projected) {
      json << '[' << disturbance_projections[i]->x << ',' << disturbance_projections[i]->y << ']';
    } else {
      json << "null";
    }
    json << '}';
  }
  json << ']';
  json << '}';
  if (!job.cuboid_stage_images.empty()) {
    json << ",\"cuboid_stage_images\":[";
    for (size_t i = 0; i < job.cuboid_stage_images.size(); ++i) {
      if (i) {json << ',';}
      const auto & stage = job.cuboid_stage_images[i];
      json << "{\"stage\":\"" << json_escape(stage.stage) <<
        "\",\"status\":\"" << json_escape(stage.status) <<
        "\",\"relative_path\":\"" << json_escape(stage.filename) <<
        "\",\"candidate_ids\":[";
      bool first_candidate = true;
      for (const auto candidate_id : stage.candidate_ids) {
        if (!first_candidate) {json << ',';}
        first_candidate = false;
        json << candidate_id;
      }
      json << "]}";
    }
    json << ']';
  }
  {
    json << ",\"depth_scene_stability\":";
    append_depth_scene_stability_json(json, job);
  }
  json << '}';
  return json.str();
}

bool VisionDetectorSnapshotOutput::save_diagnostic_record(
  const VisionDetectorSnapshotJob & job, std::string & failure_cause,
  std::string & serialized_record)
{
  const bool acquisition_id_safe = !job.acquisition_id.empty() &&
    job.acquisition_id.find_first_not_of(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") == std::string::npos;
  std::error_code error;
  bool output_available = acquisition_id_safe;
  if (!acquisition_id_safe) {
    failure_cause = "acquisition_id unavailable or unsafe";
  } else {
    std::filesystem::create_directories(diagnostic_root_, error);
    if (error) {
      failure_cause = "create diagnostic directory: " + error.message();
      output_available = false;
    }
  }
  const auto record_path = output_available ?
    diagnostic_root_ / (job.acquisition_id + ".json") : std::filesystem::path();
  if (output_available && std::filesystem::exists(record_path, error)) {
    output_available = false;
    failure_cause = "diagnostic record destination already exists";
  }
  std::filesystem::path replay_path;
  std::filesystem::path boundary_path;
  bool replay_saved = false;
  bool boundary_saved = false;
  const auto replay = job.detection_result().cuboid_replay_evidence;
  const bool replay_present = static_cast<bool>(replay);
  const bool replay_shape_valid = replay_present && replay->rows > 0 && replay->columns > 0 &&
    replay->height_grid_m.type() == CV_64FC1 &&
    replay->height_grid_m.rows == static_cast<int>(replay->rows) &&
    replay->height_grid_m.cols == static_cast<int>(replay->columns) &&
    replay->height_grid_m.total() == static_cast<size_t>(replay->rows) * replay->columns &&
    (replay->image_boundary_mask.empty() ||
    (replay->image_boundary_mask.type() == CV_8UC1 &&
    replay->image_boundary_mask.rows == static_cast<int>(replay->rows) &&
    replay->image_boundary_mask.cols == static_cast<int>(replay->columns) &&
    replay->image_boundary_mask.total() == replay->height_grid_m.total()));
  std::string grid_status = replay_present ? "not_saved" : "not_captured";
  std::string mask_status = replay_present ? "not_saved" : "not_available";
  std::string replay_status = replay_present ? "pending" : "not_captured";
  std::string replay_reason;
  static std::atomic<uint64_t> temporary_file_sequence{0};
  const auto atomic_write = [&](
      const std::filesystem::path & path, size_t expected_bytes,
      const std::function<bool(std::ofstream &)> & write) {
      std::error_code io_error;
      if (std::filesystem::exists(path, io_error) || io_error) {
        failure_cause = io_error ? "inspect destination: " + io_error.message() :
          "destination already exists";
        return false;
      }
      auto temporary = path;
      temporary += ".tmp." + std::to_string(temporary_file_sequence.fetch_add(1));
      std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
      if (!stream || !write(stream)) {
        stream.close();
        std::filesystem::remove(temporary, io_error);
        failure_cause = "write temporary evidence failed";
        return false;
      }
      stream.flush();
      const bool stream_good = static_cast<bool>(stream);
      stream.close();
      if (!stream_good || stream.fail()) {
        std::filesystem::remove(temporary, io_error);
        failure_cause = "flush temporary evidence failed";
        return false;
      }
      const auto actual_bytes = std::filesystem::file_size(temporary, io_error);
      if (io_error || actual_bytes != expected_bytes) {
        std::filesystem::remove(temporary, io_error);
        failure_cause = "temporary evidence size mismatch";
        return false;
      }
      std::filesystem::rename(temporary, path, io_error);
      if (io_error) {
        std::filesystem::remove(temporary, error);
        failure_cause = "publish temporary evidence: " + io_error.message();
        return false;
      }
      const auto published_bytes = std::filesystem::file_size(path, io_error);
      if (io_error || published_bytes != expected_bytes) {
        std::filesystem::remove(path, io_error);
        failure_cause = "published evidence size mismatch";
        return false;
      }
      return true;
    };
  if (replay_present && !replay_shape_valid) {
    replay_status = "invalid";
    grid_status = "invalid";
    mask_status = replay->image_boundary_mask.empty() ? "not_available" : "invalid";
    replay_reason = "grid_or_mask_dimensions_or_type_mismatch";
  } else if (output_available && replay_shape_valid) {
    replay_path = diagnostic_root_ / (job.acquisition_id + ".height_grid.f64le");
    const size_t expected_grid_bytes = replay->height_grid_m.total() * sizeof(double);
    replay_saved = atomic_write(replay_path, expected_grid_bytes, [&](std::ofstream & out) {
        for (int row = 0; row < replay->height_grid_m.rows; ++row) {
          const auto * values = replay->height_grid_m.ptr<double>(row);
          for (int col = 0; col < replay->height_grid_m.cols; ++col) {
            uint64_t bits = 0;
            std::memcpy(&bits, &values[col], sizeof(bits));
            for (unsigned byte = 0; byte < 8; ++byte) {
              out.put(static_cast<char>((bits >> (byte * 8)) & 0xffU));
            }
          }
        }
        return static_cast<bool>(out);
      });
    if (replay_saved) {grid_status = "saved";} else {grid_status = "save_failed";replay_path.clear();}
    if (!replay->image_boundary_mask.empty()) {
      boundary_path = diagnostic_root_ / (job.acquisition_id + ".image_boundary.u8");
      const size_t expected_mask_bytes = replay->image_boundary_mask.total();
      boundary_saved = atomic_write(boundary_path, expected_mask_bytes, [&](std::ofstream & out) {
          for (int row = 0; row < replay->image_boundary_mask.rows; ++row) {
            out.write(reinterpret_cast<const char *>(replay->image_boundary_mask.ptr<uint8_t>(row)),
              replay->image_boundary_mask.cols);
          }
          return static_cast<bool>(out);
        });
      if (boundary_saved) {mask_status = "saved";} else {mask_status = "save_failed";boundary_path.clear();}
    } else {mask_status = "not_available";}
    const bool mask_required = !replay->image_boundary_mask.empty();
    replay_status = replay_saved && (!mask_required || boundary_saved) ? "saved" :
      (replay_saved || boundary_saved) ? "partial" : "save_failed";
    if (replay_status != "saved") {replay_reason = failure_cause;}
  } else if (replay_present) {
    replay_status = "save_failed";
    grid_status = "save_failed";
    mask_status = replay->image_boundary_mask.empty() ? "not_available" : "save_failed";
    replay_reason = failure_cause;
  } else if (job.has_detection_result) {
    replay_status = job.detection_result().result_code ==
      arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS ?
      "not_captured" : "unavailable";
    replay_reason = replay_status == "not_captured" ?
      "detailed_capture_not_selected" : "algorithm_did_not_retain_replay_grid";
  } else {
    replay_status = "not_observed";
    replay_reason = "no_representative_detection_observation";
  }
  std::ostringstream json;
  json << std::setprecision(17) << "{\"schema_version\":2,\"record_type\":\"vision_diagnostic\","
       << "\"record_storage_status\":\"" <<
    (output_available ? "saved" : "unavailable") << "\",\"record_storage_failure_reason\":";
  if (output_available) {json << "null";} else {
    json << "\"" << json_escape(failure_cause) << "\"";
  }
  json
       << ",\"vision_run_id\":\"" << json_escape(job.vision_run_id) << "\","
       << "\"acquisition_id\":\"" << json_escape(job.acquisition_id) << "\","
       << "\"live_test_session_id\":";
  if (live_test_session_id_.empty()) {json << "null";} else {
    json << "\"" << json_escape(live_test_session_id_) << "\"";
  }
  json << ",\"association_status\":\"" <<
    (live_test_session_id_.empty() ? "unassociated" : "associated") << "\","
       << "\"strategy\":\"" << json_escape(job.algorithm) << "\",\"target_id\":\"" <<
    json_escape(job.target_id) << "\",\"observation\":";
  if (job.observation_input()) {
    const bool rgb_available = is_renderable_presentation_rgb(job.observation_input()->rgb);
    json << "{\"rgb_available\":" << (rgb_available ? "true" : "false") <<
      ",\"rgb_presentation_status\":\"" <<
      json_escape(presentation_rgb_status(*job.observation_input())) << "\",\"rgb\":";
    if (rgb_available) {
      json << header_json(job.observation_input()->rgb.header);
    } else {json << "null";}
    json << ",\"depth\":" << header_json(job.observation_input()->depth.header) <<
      ",\"camera_info_frame_id\":\"" <<
      json_escape(job.observation_input()->camera_info.header.frame_id) << "\",\"camera_intrinsics_k\":[";
    for (size_t i = 0; i < job.observation_input()->camera_info.k.size(); ++i) {
      if (i) {json << ',';}
      json << job.observation_input()->camera_info.k[i];
    }
    json << "],\"camera_distortion_d\":[";
    for (size_t i = 0; i < job.observation_input()->camera_info.d.size(); ++i) {
      if (i) {json << ',';}
      json << job.observation_input()->camera_info.d[i];
    }
    const auto & optical = job.observation_input()->optical_to_base;
    json << "],\"optical_to_base\":{\"parent_frame\":\"" <<
      json_escape(optical.header.frame_id) << "\",\"child_frame\":\"" <<
      json_escape(optical.child_frame_id) << "\",\"translation_m\":[" <<
      optical.transform.translation.x << ',' << optical.transform.translation.y << ',' <<
      optical.transform.translation.z << "],\"rotation_xyzw\":[" <<
      optical.transform.rotation.x << ',' << optical.transform.rotation.y << ',' <<
      optical.transform.rotation.z << ',' << optical.transform.rotation.w << "]}}";
  } else {json << "null";}
  json << ",\"result_code\":" << static_cast<unsigned>(job.terminal_result_code) <<
       ",\"terminal_result\":{\"result_code\":" <<
    static_cast<unsigned>(job.terminal_result_code) << ",\"result_name\":\"" <<
    result_code_name(job.terminal_result_code) << "\",\"processing_stage\":\"" <<
    json_escape(job.processing_stage_data()) << "\",\"last_observation_stage\":\"" <<
    json_escape(job.last_observation_stage) << "\",\"diagnostic_detail\":\"" <<
    json_escape(job.terminal_diagnostic) << "\"},\"acquisition_selection\":{\"status\":\"" <<
    (job.terminal_selection_available ? "available" : "unavailable") << "\",\"reason\":\"" <<
    (job.terminal_selection_available ? "" : "terminal_result_not_success") <<
    "\",\"scene_generation\":" << job.terminal_scene_generation <<
    "},\"single_observation\":" <<
    (job.has_detection_result ? diagnostic_result_summary_json(job.detection_result()) : "null") <<
       ",\"applied_profile\":";
  if (job.selected_detector_profile()) {
    const auto & profile = *job.selected_detector_profile();
    json << "{\"profile_id\":\"" << json_escape(profile.target_id) <<
      "\",\"selection_source\":\"acquisition_target_id\",\"algorithm\":\"" <<
      json_escape(profile.algorithm) << "\",\"material_id\":\"" <<
      json_escape(profile.material_id) << "\",\"shape\":\"" << json_escape(profile.shape) <<
      "\",\"dimensions_m\":[" << profile.dimensions_m[0] << ',' <<
      profile.dimensions_m[1] << ',' << profile.dimensions_m[2] <<
      "],\"dimension_tolerance_m\":[" << profile.dimension_tolerance_m[0] << ',' <<
      profile.dimension_tolerance_m[1] << ',' << profile.dimension_tolerance_m[2] <<
      "],\"workspace_roi_min_m\":[" << profile.workspace_roi_min_m[0] << ',' <<
      profile.workspace_roi_min_m[1] << ',' << profile.workspace_roi_min_m[2] <<
      "],\"workspace_roi_max_m\":[" << profile.workspace_roi_max_m[0] << ',' <<
      profile.workspace_roi_max_m[1] << ',' << profile.workspace_roi_max_m[2] <<
      "],\"tray_roi_min_m\":[" << profile.tray_roi_min_m[0] << ',' <<
      profile.tray_roi_min_m[1] << "],\"tray_roi_max_m\":[" << profile.tray_roi_max_m[0] << ',' <<
      profile.tray_roi_max_m[1] << "],\"support_contact_tolerance_m\":" <<
      profile.support_contact_tolerance_m << ",\"top_surface_geometry_tolerance_m\":" <<
      profile.top_surface_geometry_tolerance_m << ",\"minimum_observation_coverage\":" <<
      profile.minimum_observation_coverage;
    if (profile.algorithm == "cuboid_top_face") {
      const auto & cuboid = profile.cuboid;
      json << ",\"cuboid\":{\"grid_resolution_m\":" << cuboid.grid_resolution_m <<
      ",\"height_tolerance_m\":" << cuboid.height_tolerance_m
      << ",\"line_residual_limit_px\":" << cuboid.line_residual_limit_px
      << ",\"minimum_line_support\":" << cuboid.minimum_line_support
      << '}';
    }
    json << '}';
  } else {json << "null";}
  json << ",\"profile_selection_status\":\"" <<
    (job.selected_detector_profile() ? "selected_for_representative_observation" : "unavailable") << "\"";
  json << ",\"support_calibration\":{\"ready\":" <<
    (job.calibration_data().ready ? "true" : "false") << ",\"accepted_observations\":" <<
    job.calibration_data().accepted_frames << ",\"required_observations\":" <<
    job.calibration_data().required_frames << ",\"median_depth_residual_m\":" <<
    job.calibration_data().median_depth_residual_m;
  const auto & tray_from_camera = job.calibration_data().tray_from_camera;
  json << ",\"tray_from_camera\":{\"parent_frame\":\"" <<
    json_escape(tray_from_camera.header.frame_id) << "\",\"child_frame\":\"" <<
    json_escape(tray_from_camera.child_frame_id) << "\",\"translation_m\":[" <<
    tray_from_camera.transform.translation.x << ',' << tray_from_camera.transform.translation.y << ',' <<
    tray_from_camera.transform.translation.z << "],\"rotation_xyzw\":[" <<
    tray_from_camera.transform.rotation.x << ',' << tray_from_camera.transform.rotation.y << ',' <<
    tray_from_camera.transform.rotation.z << ',' << tray_from_camera.transform.rotation.w << "]}}";
  json << ",\"depth_scene_stability\":";
  append_depth_scene_stability_json(json, job);
  json << ",\"evidence_scopes\":{\"terminal_result\":\"acquisition\","
       << "\"single_observation\":\"" <<
    (job.has_detection_result ? "representative_processed_depth_observation" : "unavailable") <<
    "\",\"stability\":\"depth_scene_acquisition_aggregate\",\"terminal_failure_causality\":\""
       << (job.terminal_result_code ==
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS ?
    "terminal_success" : job.has_detection_result ?
    "representative_observation_and_depth_scene_stability_must_be_considered_together" :
    "no_representative_observation_available") << "\",\"representative_observation_limit\":\"" <<
    (job.terminal_result_code ==
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS ?
    "not_applicable" : job.has_detection_result ?
    "one_observation_alone_cannot_establish_depth_scene_stability_or_terminal_failure_cause" :
    "no_representative_observation_available") << "\"}";
  const bool cuboid_strategy = job.algorithm == "cuboid_top_face";
  if (cuboid_strategy) {json << ",\"cuboid_height_patches\":[";}
  const auto append_rejection_counts = [&json](const auto & counts) {
      json << '{';
      bool first = true;
      for (const auto & [reason, count] : counts) {
        if (!first) {json << ',';}
        first = false;
        json << "\"" << json_escape(reason) << "\":" << count;
      }
      json << '}';
    };
  const auto append_optional_count = [&json](const std::optional<uint64_t> & count) {
      if (count) {json << *count;} else {json << "null";}
    };
  const auto append_plane_evidence = [&json](const CuboidHeightPatchDiagnostic & plane) {
      const auto append_rows = [&json](const auto & rows) {
          json << '[';
          for (size_t i = 0; i < rows.size(); ++i) {
            if (i) {json << ',';} json << '[';
            for (size_t j = 0; j < rows[i].size(); ++j) {
              if (j) {json << ',';} json << rows[i][j];
            }
            json << ']';
          }
          json << ']';
        };
      json << ",\"source_boundary_id\":" << plane.source_boundary_id <<
        ",\"plane_fitted\":" << (plane.plane_fitted ? "true" : "false") <<
        ",\"plane_status\":\"" << json_escape(plane.plane_status) << "\"" <<
        ",\"normal\":[" << plane.plane_normal[0] << ',' << plane.plane_normal[1] << ',' <<
        plane.plane_normal[2] << "],\"slope_rad\":" <<
        std::atan2(std::hypot(plane.plane_normal[0], plane.plane_normal[1]), plane.plane_normal[2]) <<
        ",\"plane_residual_m\":" << plane.plane_residual_m <<
        ",\"plane_height_span_m\":" << plane.plane_height_span_m << ",\"observed_cell_ids\":[";
      for (size_t i = 0; i < plane.observed_cell_ids.size(); ++i) {
        if (i) {json << ',';} json << plane.observed_cell_ids[i];
      }
      json << "],\"observed_contours_xy_m\":[";
      for (size_t i = 0; i < plane.observed_contours_xy.size(); ++i) {
        if (i) {json << ',';} json << '[';
        for (size_t j = 0; j < plane.observed_contours_xy[i].size(); ++j) {
          if (j) {json << ',';}
          const auto & point = plane.observed_contours_xy[i][j];
          json << '[' << point.x << ',' << point.y << ']';
        }
        json << ']';
      }
      json << "],\"plane_center_height_m\":[" << plane.plane_center_height_m[0] << ',' <<
        plane.plane_center_height_m[1] << ',' << plane.plane_center_height_m[2] <<
        "],\"observed_height_p05_p95_m\":[" << plane.observed_height_p05_p95_m[0] << ',' <<
        plane.observed_height_p05_p95_m[1] << ']';
      json << ",\"fitted_edge_support_cell_ids\":"; append_rows(plane.fitted_edge_support_cell_ids);
      json << ",\"fitted_edge_residual_px\":[";
      for (size_t i = 0; i < plane.fitted_edge_residual_px.size(); ++i) {
        if (i) {json << ',';} json << plane.fitted_edge_residual_px[i];
      }
      json << ']';
      json << ",\"fitted_edges_xy_support\":"; append_rows(plane.fitted_edges_xy_support);
      json << ",\"adjacent_planes_id_step_m_discontinuous\":"; append_rows(plane.adjacent_planes);
      json << ",\"rectangle_regions_xy_yaw_length_width\":"; append_rows(plane.rectangle_regions);
      json << ",\"inferred_partition_edges_xy_m\":"; append_rows(plane.inferred_partition_edges);
      json << ",\"corners_xy_convex_turn\":"; append_rows(plane.corners_xy_turn);
      json << ",\"rectangle_alternative_pairs\":"; append_rows(plane.rectangle_alternatives);
      json << ",\"fitted_edge_source_ids\":[";
      for (size_t i = 0; i < plane.fitted_edge_source_ids.size(); ++i) {
        if (i) {json << ',';} json << plane.fitted_edge_source_ids[i];
      }
      json << ']';
    };
  for (size_t i = 0; cuboid_strategy && i < job.detection_result().cuboid_height_patches.size(); ++i) {
    if (i) {json << ',';}
    const auto & patch = job.detection_result().cuboid_height_patches[i];
    json << "{\"height_m\":" << patch.height_m << ",\"cell_count\":" <<
      patch.cell_count << ",\"contour_count\":" << patch.contour_count <<
      ",\"contours_with_fitted_edges_count\":" << patch.contours_with_fitted_edges_count <<
      ",\"fitted_edge_count\":" << patch.fitted_edge_count <<
      ",\"edge_fit_attempt_count\":" << patch.edge_fit_attempt_count <<
      ",\"contour_rejected_by_reason_count_unit\":\"contour\",\"contour_rejected_by_reason\":";
    append_rejection_counts(patch.contour_rejected_by_reason);
    json << ",\"edge_fit_rejected_by_reason_count_unit\":\"edge_fit_attempt\",\"edge_fit_rejected_by_reason\":";
    append_rejection_counts(patch.edge_fit_rejected_by_reason);
    json << ",\"candidate_evaluation_attempt_count\":" << patch.candidate_evaluation_attempt_count <<
      ",\"candidate_evaluation_attempt_count_unit\":\"candidate_evaluation\""
      ",\"pre_object_rejected_count\":" << patch.pre_object_rejected_count <<
      ",\"pre_object_rejected_count_unit\":\"pre_object_rejection_event\""
      ",\"generated_hypothesis_count\":" << patch.generated_hypothesis_count <<
      ",\"generated_hypothesis_count_unit\":\"candidate_after_deduplication\""
      ",\"geometry_valid_before_deduplication_count\":" <<
      patch.geometry_valid_before_deduplication_count <<
      ",\"geometry_valid_before_deduplication_count_unit\":\"candidate_before_deduplication\""
      ",\"deduplicated_count\":" << patch.deduplicated_count <<
      ",\"deduplicated_count_unit\":\"candidate_removed\""
      ",\"deduplicated_by_reason_count_unit\":\"candidate_removed\",\"deduplicated_by_reason\":";
    append_rejection_counts(patch.deduplicated_by_reason);
    json << ",\"partitioned_candidate_count\":" << patch.partitioned_candidate_count <<
      ",\"partitioned_candidate_count_unit\":\"candidate\""
      ",\"confidence_grade\":\"" << json_escape(patch.confidence_grade) <<
      "\",\"confidence_reason\":\"" << json_escape(patch.confidence_reason) << "\"" <<
      ",\"partition_status\":\"" << json_escape(patch.partition_status) <<
      "\",\"partition_reason\":\"" << json_escape(patch.partition_reason) <<
      "\",\"rejection_reason_count_unit\":\"rejection_event\",\"hypothesis_rejected_by_reason\":";
    append_rejection_counts(patch.hypothesis_rejected_by_reason);
    json << ",\"geometry_rejected_by_reason\":";
    append_rejection_counts(patch.geometry_rejected_by_reason);
    append_plane_evidence(patch);
    json << '}';
  }
  if (cuboid_strategy) {
    json << "],\"cuboid_boundaries\":[";
    for (size_t i = 0; i < job.detection_result().cuboid_boundaries.size(); ++i) {
      if (i) {json << ',';}
      const auto & boundary = job.detection_result().cuboid_boundaries[i];
      json << "{\"cell_count\":" << boundary.cell_count;
      append_plane_evidence(boundary);
      json << '}';
    }
    json << "],\"cuboid_hypotheses\":[";
    for (size_t i = 0; i < job.detection_result().cuboid_hypotheses.size(); ++i) {
      if (i) {json << ',';}
      const auto & candidate = job.detection_result().cuboid_hypotheses[i];
      json << "{\"selection_state\":\"" << json_escape(candidate.selection_state) <<
        "\",\"selection_reason\":\"" << json_escape(candidate.selection_reason) <<
        "\",\"observed_support_cells\":" << candidate.observed_support_cells <<
        ",\"height_residual_normalized\":" << candidate.height_residual <<
        ",\"dimension_residual_normalized\":";
      if (candidate.dimension_observation_count) {json << candidate.dimension_residual;} else {json << "null";}
      json << ",\"dimension_observation_count\":" << candidate.dimension_observation_count <<
        ",\"rectangle_id\":" << candidate.rectangle_id <<
        ",\"source_boundary_id\":" << candidate.source_boundary_id <<
        ",\"outline_id\":" << candidate.outline_id << ",\"complete_iou\":";
      if (candidate.complete_iou) {json << *candidate.complete_iou;} else {json << "null";}
      json << ",\"competing_candidate_ids\":[";
      for (size_t ci = 0; ci < candidate.competing_candidate_ids.size(); ++ci) {
        if (ci) {json << ',';} json << candidate.competing_candidate_ids[ci];
      }
      json << "],\"observed_line_support_id_length_m\":[";
      bool first_line = true;
      for (const auto & line : candidate.observed_line_support_m) {
        if (!first_line) {json << ',';} first_line = false;
        json << '[' << line.first << ',' << line.second << ']';
      }
      json << "],\"candidate_id\":" << candidate.candidate_id <<
        ",\"patch_index\":" << candidate.patch_index <<
        ",\"confidence_grade\":\"" << json_escape(candidate.confidence_grade) <<
        "\",\"confidence_reason\":\"" << json_escape(candidate.confidence_reason) << "\"" <<
        ",\"anchor_xy_m\":[" << candidate.anchor_x_m << ',' << candidate.anchor_y_m <<
        "],\"observed_anchor_edge_lengths_m\":[" << candidate.observed_anchor_edge_lengths_m[0] <<
        ',' << candidate.observed_anchor_edge_lengths_m[1] <<
        "],\"observed_boundary_sides\":" << candidate.observed_boundary_sides <<
        ",\"boundary_side_identity_available\":false"
        ",\"tray_support_evidence\":\"" << json_escape(candidate.tray_support_evidence) << "\"" <<
        ",\"height_m\":" << candidate.height_m <<
        ",\"center_xy_m\":[" << candidate.center_x_m << ',' << candidate.center_y_m <<
        "],\"yaw_rad\":" << candidate.yaw_rad <<
        ",\"dimensions_m\":[" << candidate.length_m << ',' << candidate.width_m << ',' <<
        candidate.thickness_m << "],\"observed_area_ratio\":" <<
        candidate.observed_area_ratio;
      json << ",\"parallel_edge_distances_m\":[";
      for (size_t side = 0; side < candidate.parallel_edge_distances_m.size(); ++side) {
        if (side) {json << ',';}
        json << candidate.parallel_edge_distances_m[side];
      }
      json << ']';
      if (candidate.boundary_support) {
        json << ",\"boundary_support\":" << *candidate.boundary_support;
      }
      json << ",\"disposition\":\"" << json_escape(candidate.disposition) <<
        "\",\"reason\":\"" <<
        json_escape(candidate.reason) << "\",\"deduplicated_to_candidate_id\":" <<
        candidate.deduplicated_to_candidate_id << '}';
    }
    json << "],\"cuboid_stages\":[";
  }
  for (size_t i = 0; cuboid_strategy && i < job.detection_result().cuboid_stages.size(); ++i) {
    if (i) {json << ',';}
    const auto & stage = job.detection_result().cuboid_stages[i];
    json << "{\"stage\":\"" << json_escape(stage.stage) << "\",\"status\":\"" <<
      json_escape(stage.status) << "\",\"reason\":\"" << json_escape(stage.reason) <<
      "\",\"count_unit\":\"" << json_escape(stage.count_unit) << "\",\"input_count\":";
    append_optional_count(stage.input_count);
    json << ",\"output_count\":";
    append_optional_count(stage.output_count);
    json << ",\"rejected_count\":";
    append_optional_count(stage.rejected_count);
    if (stage.stage == "height_grid_and_workspace" ||
      stage.stage == "height_patch_formation")
    {
      json << ",\"auxiliary_counts\":";
      append_rejection_counts(stage.auxiliary_counts);
    }
    if (stage.stage == "contour_and_line_fitting" && stage.status == "computed") {
      json << ",\"fitted_edge_count\":" << stage.fitted_edge_count <<
        ",\"fitted_edge_count_unit\":\"fitted_edge\""
        ",\"edge_fit_attempt_count\":" <<
        stage.auxiliary_counts.at("edge_fit_attempt_count") <<
        ",\"edge_fit_attempt_count_unit\":\"polygon_edge_fit_attempt\""
        ",\"contour_rejected_by_reason_count_unit\":\"contour\","
        "\"contour_rejected_by_reason\":";
      append_rejection_counts(stage.rejected_by_reason);
      json << ",\"edge_fit_rejected_by_reason_count_unit\":\"edge_fit_attempt\","
        "\"edge_fit_rejected_by_reason\":";
      append_rejection_counts(stage.edge_fit_rejected_by_reason);
    } else if (stage.stage == "cuboid_hypothesis_generation" && stage.status == "computed") {
      json << ",\"candidate_evaluation_attempt_count\":" <<
        stage.candidate_evaluation_attempt_count <<
        ",\"candidate_evaluation_attempt_count_unit\":\"candidate_evaluation\","
        "\"pre_object_rejected_count\":" << stage.pre_object_rejected_count <<
        ",\"pre_object_rejected_count_unit\":\"pre_object_rejection_event\","
        "\"generated_after_deduplication_count\":" << stage.generated_count <<
        ",\"generated_after_deduplication_count_unit\":\"candidate\""
        ",\"deduplicated_count\":" << stage.deduplicated_count <<
        ",\"deduplicated_count_unit\":\"candidate_removed\","
        "\"deduplicated_by_reason_count_unit\":\"candidate_removed\","
        "\"deduplicated_by_reason\":";
      append_rejection_counts(stage.deduplicated_by_reason);
      json << ",\"rejection_reason_count_unit\":\"rejection_event\","
        "\"rejected_by_reason\":";
      append_rejection_counts(stage.rejected_by_reason);
    } else if (stage.stage == "dimension_and_geometry_validation" && stage.status == "computed") {
      json << ",\"candidate_evaluation_attempt_count\":" <<
        stage.candidate_evaluation_attempt_count <<
        ",\"candidate_evaluation_attempt_count_unit\":\"candidate_evaluation\","
        "\"rejected_candidate_evaluation_count\":" <<
        stage.rejected_candidate_evaluation_count <<
        ",\"rejected_candidate_evaluation_count_unit\":\"candidate_evaluation\""
        ",\"generated_before_deduplication_count\":" << stage.generated_count <<
        ",\"generated_before_deduplication_count_unit\":\"candidate\""
        ",\"deduplicated_count\":" << stage.deduplicated_count <<
        ",\"deduplicated_count_unit\":\"candidate_removed\","
        "\"deduplicated_by_reason_count_unit\":\"candidate_removed\","
        "\"deduplicated_by_reason\":";
      append_rejection_counts(stage.deduplicated_by_reason);
      json << ",\"rejection_reason_count_unit\":\"rejection_event\","
        "\"rejected_by_reason\":";
      append_rejection_counts(stage.rejected_by_reason);
    } else if (stage.stage == "partitioned_or_isolated_classification" &&
      stage.status == "computed")
    {
      json << ",\"outcome_status_count_unit\":\"height_patch\",\"outcome_by_status\":";
      append_rejection_counts(stage.outcome_by_status);
    } else if (stage.stage == "candidate_enumeration" && stage.status == "computed") {
      json << ",\"enumerated_candidate_count\":" <<
        stage.auxiliary_counts.at("enumerated_candidates") <<
        ",\"enumerated_candidate_count_unit\":\"candidate\"";
    }
    json << '}';
  }
  if (cuboid_strategy) {json << ']';}
  json << ",\"capture_status\":{\"request_detail\":\"" <<
    (job.request_diagnostics && job.request_diagnostics->ingress.detailed_capture_enabled ?
    "selected" : "lightweight_only") << "\",\"sample_details_dropped\":" <<
    (job.request_diagnostics ? job.request_diagnostics->ingress.sample_detail_dropped : 0) <<
    ",\"pair_details_dropped\":" <<
    (job.request_diagnostics ? job.request_diagnostics->ingress.pair_detail_dropped : 0) <<
    '}';
  if (cuboid_strategy) {json << ",\"replay_evidence\":";}
  if (cuboid_strategy) {
    json << "{\"status\":\"" << replay_status << "\",\"reason\":\"" <<
      json_escape(replay_reason) << "\",\"grid_status\":\"" << grid_status <<
      "\",\"grid_format\":\"float64_little_endian_row_major_nan_invalid\",\"grid_path\":\"" <<
      (replay_saved ? json_escape(replay_path.filename().string()) : "") <<
      "\",\"image_boundary_mask_status\":\"" << mask_status <<
      "\",\"image_boundary_mask_path\":\"" <<
      (boundary_saved ? json_escape(boundary_path.filename().string()) : "") <<
      "\",\"rows\":";
    if (replay_present) {json << replay->rows;} else {json << "null";}
    json << ",\"columns\":";
    if (replay_present) {json << replay->columns;} else {json << "null";}
    json << ",\"resolution_m\":";
    if (replay_present) {json << replay->resolution_m;} else {json << "null";}
    json << ",\"origin_xy_m\":";
    if (replay_present) {json << '[' << replay->origin_x_m << ',' << replay->origin_y_m << ']';}
    else {json << "null";}
    json << ",\"frame_id\":";
    if (replay_present) {json << '"' << json_escape(replay->frame_id) << '"';}
    else {json << "null";}
    json << ",\"validity_encoding\":\"finite_cells_valid_nan_invalid\"}";
  }
  json << ",\"depth_scene_stability\":";
  append_depth_scene_stability_json(json, job);
  json << ",\"request_diagnostics\":";
  if (job.request_diagnostics) {json << request_diagnostics_json(*job.request_diagnostics);} else {
    json << "null";
  }
  json << '}';
  const auto payload = json.str();
  serialized_record = payload;
  if (!output_available) {return false;}
  const bool saved = atomic_write(record_path, payload.size() + 1, [&](std::ofstream & out) {
      out.write(payload.data(), static_cast<std::streamsize>(payload.size()));
      out.put('\n');
      return static_cast<bool>(out);
    });
  if (!saved) {
    const auto status_key = std::string("\"record_storage_status\":\"saved\"");
    const auto status_position = serialized_record.find(status_key);
    if (status_position != std::string::npos) {
      serialized_record.replace(
        status_position, status_key.size(), "\"record_storage_status\":\"save_failed\"");
    }
    const auto reason_key = std::string("\"record_storage_failure_reason\":null");
    const auto reason_position = serialized_record.find(reason_key);
    if (reason_position != std::string::npos) {
      serialized_record.replace(
        reason_position, reason_key.size(), "\"record_storage_failure_reason\":\"" +
        json_escape(failure_cause) + "\"");
    }
    return false;
  }
  if (replay_present && replay_status != "saved") {
    failure_cause = "diagnostic saved but replay evidence status=" + replay_status;
    return false;
  }
  return true;
}


}  // namespace arm_cell_vision
