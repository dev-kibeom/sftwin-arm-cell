#include "arm_cell_vision/cuboid_top_face.hpp"
#include "cuboid_top_face_geometry.hpp"
#include "cuboid_unresolved.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace arm_cell_vision
{
namespace
{
using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
using DisturbanceCandidate = arm_cell_interfaces::msg::DisturbanceCandidate;
constexpr double pi = 3.14159265358979323846;
using namespace cuboid_geometry;
using namespace cuboid_unresolved;

struct PixelRayLookup
{
  struct Key
  {
    uint32_t width;
    uint32_t height;
    std::string frame_id;
    std::string distortion_model;
    std::array<double, 9> intrinsic;
    std::vector<double> distortion;

    bool matches(
      const sensor_msgs::msg::CameraInfo & info, uint32_t image_width,
      uint32_t image_height) const
    {
      return width == image_width && height == image_height &&
             frame_id == info.header.frame_id && distortion_model == info.distortion_model &&
             intrinsic == info.k && distortion == info.d;
    }
  };

  Key key;
  cv::Mat camera_matrix;
  cv::Mat distortion;
  std::vector<cv::Point2f> centers;
  std::vector<std::array<cv::Point2f, 4>> corners;
};

std::shared_ptr<const PixelRayLookup> pixel_ray_lookup(
  const sensor_msgs::msg::CameraInfo & info, uint32_t width, uint32_t height)
{
  // One process-wide calibration/resolution entry; the current observation holds its own reference.
  static constexpr size_t kCacheByteLimit = 64 * 1024 * 1024;
  static std::mutex cache_mutex;
  static std::shared_ptr<const PixelRayLookup> cached;
  {
    std::lock_guard<std::mutex> lock(cache_mutex);
    if (cached && cached->key.matches(info, width, height)) {return cached;}
    cached.reset();
  }

  auto lookup = std::make_shared<PixelRayLookup>();
  lookup->key = {width, height, info.header.frame_id, info.distortion_model, info.k, info.d};
  const size_t pixel_count = static_cast<size_t>(width) * height;
  lookup->centers.resize(pixel_count);
  lookup->corners.resize(pixel_count);
  constexpr uint32_t kRowsPerBatch = 16;
  std::vector<cv::Point2f> center_pixels;
  std::vector<cv::Point2f> corner_pixels;
  std::vector<cv::Point2f> normalized_centers;
  std::vector<cv::Point2f> normalized_corners;
  const std::array<cv::Point2f, 4> offsets{{
    {-0.5F, -0.5F}, {0.5F, -0.5F}, {0.5F, 0.5F}, {-0.5F, 0.5F}}};
  lookup->camera_matrix = (cv::Mat_<double>(3, 3) <<
    info.k[0], 0, info.k[2], 0, info.k[4], info.k[5], 0, 0, 1);
  lookup->distortion = info.d.empty() ? cv::Mat::zeros(1, 5, CV_64F) :
    cv::Mat(info.d, true);
  for (uint32_t row_base = 0; row_base < height; row_base += kRowsPerBatch) {
    const uint32_t batch_rows = std::min(kRowsPerBatch, height - row_base);
    const size_t batch_pixels = static_cast<size_t>(batch_rows) * width;
    center_pixels.clear(); corner_pixels.clear();
    center_pixels.reserve(batch_pixels); corner_pixels.reserve(batch_pixels * 4);
    for (uint32_t row = row_base; row < row_base + batch_rows; ++row) {
      for (uint32_t column = 0; column < width; ++column) {
        const cv::Point2f center(static_cast<float>(column), static_cast<float>(row));
        center_pixels.push_back(center);
        for (const auto & offset : offsets) {corner_pixels.push_back(center + offset);}
      }
    }
    cv::undistortPoints(
      center_pixels, normalized_centers, lookup->camera_matrix, lookup->distortion);
    cv::undistortPoints(
      corner_pixels, normalized_corners, lookup->camera_matrix, lookup->distortion);
    const size_t lookup_base = static_cast<size_t>(row_base) * width;
    for (size_t pixel = 0; pixel < batch_pixels; ++pixel) {
      lookup->centers[lookup_base + pixel] = normalized_centers[pixel];
      for (size_t corner = 0; corner < 4; ++corner) {
        lookup->corners[lookup_base + pixel][corner] = normalized_corners[4 * pixel + corner];
      }
    }
  }

  constexpr size_t kBytesPerPixel = sizeof(cv::Point2f) + sizeof(std::array<cv::Point2f, 4>);
  if (pixel_count > kCacheByteLimit / kBytesPerPixel) {
    std::lock_guard<std::mutex> lock(cache_mutex);
    if (cached && !cached->key.matches(info, width, height)) {cached.reset();}
    return lookup;
  }
  std::lock_guard<std::mutex> lock(cache_mutex);
  if (cached && cached->key.matches(info, width, height)) {return cached;}
  cached = lookup;
  return lookup;
}

size_t deduplicate_across_heights(
  std::vector<Fit> & candidates, double height_tolerance, double spacing,
  std::vector<CuboidHypothesisVisualization> & visualization)
{
  std::stable_sort(candidates.begin(), candidates.end(), [](const Fit & a, const Fit & b) {
      return a.candidate_id < b.candidate_id;
    });
  std::vector<Fit> retained;
  std::map<uint32_t, uint32_t> removed_to;
  for (auto & candidate : candidates) {
    auto duplicate = std::find_if(retained.begin(), retained.end(), [&](const Fit & previous) {
        if (!within_height_tolerance(candidate.top, previous.top, height_tolerance) ||
          candidate.face.length != previous.face.length ||
          candidate.face.width != previous.face.width ||
          candidate.face.thickness != previous.face.thickness)
        {return false;}
        if (cv::norm(candidate.center - previous.center) <= 2 * spacing) {
          const auto candidate_corners = corners(candidate);
          const auto previous_corners = corners(previous);
          const bool equivalent_pose = std::all_of(
            candidate_corners.begin(), candidate_corners.end(), [&](const auto & point) {
              return std::any_of(previous_corners.begin(), previous_corners.end(),
                [&](const auto & other) {return cv::norm(point - other) <= 2 * spacing;});
            });
          if (equivalent_pose) {return true;}
        }
        const int intersection_area = cv::countNonZero(candidate.mask & previous.mask);
        const size_t union_area = candidate.mask_cell_count + previous.mask_cell_count -
          static_cast<size_t>(intersection_area);
        return union_area > 0 && static_cast<double>(intersection_area) / union_area >= 0.90;
      });
    if (duplicate == retained.end()) {
      retained.push_back(std::move(candidate));
    } else {
      removed_to[candidate.candidate_id] = duplicate->candidate_id;
    }
  }
  for (auto & item : visualization) {
    if (!removed_to.count(item.candidate_id)) {continue;}
    item.disposition = "removed_by_deduplication";
    item.reason = "equivalent_top_across_height_patches";
    item.deduplicated_to_candidate_id = removed_to.at(item.candidate_id);
  }
  candidates = std::move(retained);
  return removed_to.size();
}

void inherit_normalized_confidence(std::vector<CuboidHypothesisVisualization> & hypotheses)
{
  for (auto & hypothesis : hypotheses) {
    if (hypothesis.disposition != "removed_by_deduplication" ||
      hypothesis.deduplicated_to_candidate_id == 0) {continue;}
    const auto representative = std::find_if(
      hypotheses.begin(), hypotheses.end(),
      [&](const auto & candidate) {
        return candidate.candidate_id == hypothesis.deduplicated_to_candidate_id;
      });
    if (representative != hypotheses.end()) {
      hypothesis.selection_state = representative->selection_state;
      hypothesis.selection_reason = "equivalent_normalized_hypothesis";
      hypothesis.confidence_grade = representative->confidence_grade;
      hypothesis.confidence_reason = "normalized_to_candidate_" +
        std::to_string(hypothesis.deduplicated_to_candidate_id);
    }
  }
}

cv::Mat missing_depth(const cv::Mat & heights)
{
  cv::Mat missing = cv::Mat::zeros(heights.size(), CV_8U);
  for (int row = 0; row < heights.rows; ++row) {
    for (int col = 0; col < heights.cols; ++col) {
      if (!std::isfinite(heights.at<double>(row, col))) {missing.at<uint8_t>(row, col) = 255;}
    }
  }
  return missing;
}


}  // namespace

std::shared_ptr<const std::vector<cv::Point2f>> normalized_camera_ray_lookup(
  const sensor_msgs::msg::CameraInfo & info, uint32_t width, uint32_t height)
{
  const auto lookup = pixel_ray_lookup(info, width, height);
  return {lookup, &lookup->centers};
}

void validate_detector_algorithm(const std::string & algorithm)
{
  if (algorithm != "cuboid_top_face") {
    throw std::invalid_argument(
            "DetectTarget.algorithm unsupported value: '" + algorithm + "'");
  }
}

void validate_cuboid_config(const CuboidTopFaceConfig & c)
{
  for (double v : {c.grid_resolution_m, c.height_tolerance_m, c.line_residual_limit_px})
  {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("RawPart.cuboid geometry parameters must be positive finite");
    }
  }
  if (!std::isfinite(c.minimum_line_support) || c.minimum_line_support <= 0 ||
    c.minimum_line_support > 1)
  {throw std::invalid_argument("RawPart.cuboid overlap/contact fraction is invalid");}
}

DetectionResult detect_cuboid_top_face(
  const CuboidHeightObservation & obs,
  const DetectorProfile & profile,
  bool retain_replay_evidence)
{
  validate_cuboid_config(profile.cuboid);
  DetectionResult result;
  result.algorithm = "cuboid_top_face";
  result.processing_stage = "cuboid_top_face_geometry";
  result.result_code = Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
  result.has_rawpart_diagnostics = true;
  if (obs.heights.empty() || obs.heights.type() != CV_64F || obs.spacing_m <= 0) {
    result.failure_cause = "invalid support-relative height observation";
    result.diagnostic_detail = "invalid support-relative height observation";
    for (const auto & stage_name : {
        "height_grid_and_workspace", "height_patch_formation", "contour_and_line_fitting",
        "cuboid_hypothesis_generation", "dimension_and_geometry_validation",
        "candidate_local_eligibility", "candidate_enumeration"})
    {
      CuboidStageDiagnostic stage;
      stage.stage = stage_name;
      stage.status = "not_computed";
      stage.reason = stage.stage == "height_grid_and_workspace" ?
        "invalid_height_observation" : "upstream_height_observation_invalid";
      stage.count_unit = "unavailable";
      result.cuboid_stages.push_back(std::move(stage));
    }
    return result;
  }
  constexpr size_t mask_byte_limit = 64 * 1024 * 1024;
  constexpr size_t transient_mask_slots = 16;
  const size_t mask_slots = mask_byte_limit / obs.heights.total();
  const size_t initial_operations = std::min<size_t>(20000,
    mask_slots > transient_mask_slots ? (mask_slots - transient_mask_slots) / 2 : 0);
  size_t search_operations = initial_operations;
  auto plane_map = build_plane_map(obs, profile, search_operations, retain_replay_evidence);
  const auto & patches = plane_map.regions;
  result.cuboid_boundaries = std::move(plane_map.boundaries);
  uint64_t valid_height_cells = 0, finite_grid_cells = 0, workspace_height_cells = 0;
  for (int r = 0; r < obs.heights.rows; ++r) {
    for (int c = 0; c < obs.heights.cols; ++c) {
      const double z = obs.heights.at<double>(r, c);
      if (std::isfinite(z)) {++finite_grid_cells;}
      if (std::isfinite(z) && z > profile.support_contact_tolerance_m) {++valid_height_cells;}
    }
  }
  for (const auto & patch : patches) {workspace_height_cells += patch.pixels.size();}
  result.rawpart_workspace_xyz_points = valid_height_cells;
  CuboidStageDiagnostic grid_stage;
  grid_stage.stage = "height_grid_and_workspace";
  grid_stage.count_unit = "cell";
  grid_stage.input_count = static_cast<uint64_t>(obs.heights.total());
  grid_stage.output_count = finite_grid_cells;
  grid_stage.rejected_count = *grid_stage.input_count - *grid_stage.output_count;
  grid_stage.auxiliary_counts["above_support_cells"] = valid_height_cells;
  grid_stage.auxiliary_counts["workspace_roi_cells"] = workspace_height_cells;
  if (valid_height_cells == 0) {grid_stage.reason = "no_valid_height_above_support";}
  result.cuboid_stages.push_back(std::move(grid_stage));
  CuboidStageDiagnostic patch_stage;
  patch_stage.stage = "height_patch_formation";
  patch_stage.count_unit = "multiple_explicit_units";
  patch_stage.status = "computed";
  patch_stage.auxiliary_counts["workspace_roi_cells"] = workspace_height_cells;
  patch_stage.auxiliary_counts["formed_height_patches"] = patches.size();
  patch_stage.reason = patches.empty() ? "no_connected_height_patches_in_workspace_roi" : "connected_height_patches_formed";
  result.cuboid_stages.push_back(std::move(patch_stage));
  CuboidStageDiagnostic line_stage;
  line_stage.stage = "contour_and_line_fitting";
  line_stage.count_unit = "contour";
  line_stage.input_count = 0;
  line_stage.output_count = 0;
  line_stage.rejected_count = 0;
  CuboidStageDiagnostic hypothesis_stage;
  hypothesis_stage.stage = "cuboid_hypothesis_generation";
  hypothesis_stage.count_unit = "multiple_explicit_units";
  hypothesis_stage.status = patches.empty() ? "not_computed" : "computed";
  hypothesis_stage.reason = patches.empty() ? "no_height_patches" : "";
  CuboidStageDiagnostic geometry_stage;
  geometry_stage.stage = "dimension_and_geometry_validation";
  geometry_stage.count_unit = "multiple_explicit_units";
  geometry_stage.status = patches.empty() ? "not_computed" : "computed";
  geometry_stage.reason = patches.empty() ? "no_height_patches" : "";
  CuboidStageDiagnostic eligibility_stage;
  eligibility_stage.stage = "candidate_local_eligibility";
  eligibility_stage.count_unit = "candidate";
  eligibility_stage.input_count = 0;
  eligibility_stage.output_count = 0;
  eligibility_stage.rejected_count = 0;
  eligibility_stage.status = patches.empty() ? "not_computed" : "computed";
  eligibility_stage.reason = patches.empty() ? "no_height_patches" : "";
  std::vector<ObservationRegion> partitions;
  std::vector<Fit> accepted;
  uint32_t next_candidate_id = 1;
  uint32_t patch_index = 0;
  // At most two full-grid masks per budgeted operation, bounded to 64 MiB.
  for (auto patch : patches) {
    {
      patch.cells = cv::Mat::zeros(obs.heights.size(), CV_8U);
      for (const auto & pixel : patch.pixels) {patch.cells.at<uint8_t>(pixel.y, pixel.x) = 255;}
    }
    auto fitted = reconstruct_plane(patch, patch_index, next_candidate_id, search_operations,
        obs, profile, retain_replay_evidence);
    plane_map.region_contours.push_back(std::move(fitted.occupancy_contours));
    partitions.insert(partitions.end(), std::make_move_iterator(fitted.partitions.begin()),
      std::make_move_iterator(fitted.partitions.end()));
    for (auto & outline : plane_map.outlines) {
      if (outline.source_boundary_id == patch.source_boundary_id &&
        std::find(fitted.unresolved_outline_ids.begin(), fitted.unresolved_outline_ids.end(),
          outline.outline_id) != fitted.unresolved_outline_ids.end()) {outline.unresolved = true;}
    }
    ++result.cuboid_stages[1].outcome_by_status[patch.status];
    for (const auto & count : fitted.counts) {hypothesis_stage.auxiliary_counts[count.first] += count.second;}
    auto & patch_diagnostic = fitted.diagnostic;
    auto generated = std::move(fitted.fits);
    result.cuboid_hypotheses.insert(result.cuboid_hypotheses.end(), fitted.hypotheses.begin(), fitted.hypotheses.end());
    for (auto & hypothesis : result.cuboid_hypotheses) {
      if (hypothesis.patch_index == patch_index &&
        hypothesis.disposition == "rejected_geometry_validation")
      {
        hypothesis.confidence_grade = "";
        hypothesis.confidence_reason = hypothesis.reason;
      }
    }
    const auto contours = patch_diagnostic.contour_count;
    const auto contours_with_edges = patch_diagnostic.contours_with_fitted_edges_count;
    const auto fitted_edges = patch_diagnostic.fitted_edge_count;
    const auto edge_fit_attempts = patch_diagnostic.edge_fit_attempt_count;
    const auto hypothesis_attempts = patch_diagnostic.candidate_evaluation_attempt_count;
    const auto pre_object_rejections = patch_diagnostic.pre_object_rejected_count;
    const auto generated_before_deduplication = patch_diagnostic.geometry_valid_before_deduplication_count;
    const auto deduplicated = patch_diagnostic.deduplicated_count;
    const auto hypothesis_rejections = patch_diagnostic.hypothesis_rejected_by_reason;
    const auto geometry_rejections = patch_diagnostic.geometry_rejected_by_reason;
    const auto deduplicated_by_reason = patch_diagnostic.deduplicated_by_reason;
    const auto contour_rejections = patch_diagnostic.contour_rejected_by_reason;
    const auto edge_fit_rejections = patch_diagnostic.edge_fit_rejected_by_reason;
    for (const auto & entry : hypothesis_rejections) {result.rawpart_rejected_by_reason[entry.first] += entry.second;}
    for (const auto & entry : geometry_rejections) {result.rawpart_rejected_by_reason[entry.first] += entry.second;}
    patch_diagnostic.contour_count = contours;
    patch_diagnostic.contours_with_fitted_edges_count = contours_with_edges;
    patch_diagnostic.fitted_edge_count = fitted_edges;
    patch_diagnostic.edge_fit_attempt_count = edge_fit_attempts;
    patch_diagnostic.contour_rejected_by_reason = contour_rejections;
    patch_diagnostic.edge_fit_rejected_by_reason = edge_fit_rejections;
    patch_diagnostic.candidate_evaluation_attempt_count = hypothesis_attempts;
    patch_diagnostic.pre_object_rejected_count = pre_object_rejections;
    patch_diagnostic.deduplicated_count = deduplicated;
    patch_diagnostic.deduplicated_by_reason = deduplicated_by_reason;
    patch_diagnostic.hypothesis_rejected_by_reason = hypothesis_rejections;
    patch_diagnostic.geometry_rejected_by_reason = geometry_rejections;
    *line_stage.input_count += contours;
    *line_stage.output_count += contours_with_edges;
    *line_stage.rejected_count += contours - contours_with_edges;
    line_stage.fitted_edge_count += fitted_edges;
    line_stage.auxiliary_counts["edge_fit_attempt_count"] += edge_fit_attempts;
    for (const auto & entry : edge_fit_rejections) {
      line_stage.edge_fit_rejected_by_reason[entry.first] += entry.second;
    }
    for (const auto & entry : contour_rejections) {
      line_stage.rejected_by_reason[entry.first] += entry.second;
    }
    hypothesis_stage.pre_object_rejected_count += pre_object_rejections;
    hypothesis_stage.deduplicated_count += deduplicated;
    for (const auto & entry : deduplicated_by_reason) {
      hypothesis_stage.deduplicated_by_reason[entry.first] += entry.second;
    }
    hypothesis_stage.generated_count += patch_diagnostic.generated_hypothesis_count;
    hypothesis_stage.candidate_evaluation_attempt_count += hypothesis_attempts;
    for (const auto & entry : hypothesis_rejections) {
      hypothesis_stage.rejected_by_reason[entry.first] += entry.second;
    }
    geometry_stage.deduplicated_count += deduplicated;
    for (const auto & entry : deduplicated_by_reason) {
      geometry_stage.deduplicated_by_reason[entry.first] += entry.second;
    }
    geometry_stage.candidate_evaluation_attempt_count += hypothesis_attempts;
    geometry_stage.generated_count += generated_before_deduplication;
    geometry_stage.pre_object_rejected_count += pre_object_rejections;
    for (const auto & entry : geometry_rejections) {
      geometry_stage.rejected_candidate_evaluation_count += entry.second;
    }
    for (const auto & entry : geometry_rejections) {
      geometry_stage.rejected_by_reason[entry.first] += entry.second;
    }
    patch_diagnostic.geometry_valid_before_deduplication_count = generated_before_deduplication;
    result.rawpart_candidate_count += patch_diagnostic.generated_hypothesis_count;
    for (const auto & fit : generated) {
      accepted.push_back(fit);
      for (auto & item : result.cuboid_hypotheses) {
        if (item.candidate_id == fit.candidate_id) {
          item.disposition = "eligible_candidate";
          item.reason = "candidate_local_geometry_and_observation_checks_passed";
        }
      }
    }
    for (auto & item : result.cuboid_hypotheses) {
      if (item.patch_index != patch_index ||
        item.disposition != "retained_after_deduplication") {continue;}
      item.disposition = "rejected_candidate_local";
      item.reason = "candidate_local_geometry_or_observation_check_failed";
    }
    patch_diagnostic.partitioned_candidate_count = generated.size();
    result.cuboid_height_patches.push_back(std::move(patch_diagnostic));
    *eligibility_stage.input_count += generated.size();
    ++patch_index;
  }
  const size_t cross_height_duplicates = deduplicate_across_heights(
    accepted, profile.cuboid.height_tolerance_m, obs.spacing_m, result.cuboid_hypotheses);
  hypothesis_stage.deduplicated_count += cross_height_duplicates;
  geometry_stage.deduplicated_count += cross_height_duplicates;
  if (cross_height_duplicates) {
    hypothesis_stage.deduplicated_by_reason["equivalent_top_across_height_patches"] +=
      cross_height_duplicates;
    geometry_stage.deduplicated_by_reason["equivalent_top_across_height_patches"] +=
      cross_height_duplicates;
  }
  result.rawpart_candidate_count -= std::min<uint64_t>(
    result.rawpart_candidate_count, cross_height_duplicates);
  const auto selections = select_hypotheses(accepted, obs, profile, search_operations);
  hypothesis_stage.auxiliary_counts["shared_evaluation_limit"] = initial_operations;
  hypothesis_stage.auxiliary_counts["shared_evaluations_used"] = initial_operations - search_operations;
  hypothesis_stage.auxiliary_counts["retained_fit_mask_bytes"] = 2 * accepted.size() * obs.heights.total();
  hypothesis_stage.auxiliary_counts["mask_byte_limit"] = mask_byte_limit;
  std::map<uint32_t, std::string> conflicts;
  std::vector<Fit> models;
  for (const auto & fit : accepted) {
    const auto & selection = selections.at(fit.candidate_id);
    if (selection.state == "SELECTED") {models.push_back(fit);}
    if (selection.state != "SELECTED") {
      conflicts[fit.candidate_id] = "identity_ambiguous";
      ++result.rawpart_rejected_by_reason[conflicts[fit.candidate_id]];
    }
    for (auto & item : result.cuboid_hypotheses) {
      if (item.candidate_id != fit.candidate_id) {continue;}
      item.selection_state = selection.state;
      item.selection_reason = selection.reason;
      item.competing_candidate_ids = selection.competitors;
      item.confidence_grade = selection.state == "UNRESOLVED" ? "LOW" :
        fit.is_partial_reconstruction ? "MEDIUM" : "HIGH";
      item.confidence_reason = selection.state == "UNRESOLVED" ? "distinct_pose_or_layout_hypotheses_remain_indistinguishable" :
        fit.is_partial_reconstruction ? "partial_top_surface_resolved_by_l_corner_height_and_recipe_evidence" :
        "top_surface_observed_and_individual_pose_resolved";
      if (selection.state != "SELECTED") {
        item.disposition = selection.state == "ALTERNATIVE" ? "alternative_interpretation" : "rejected_identity_ambiguous";
        item.reason = selection.reason;
      }
    }
  }
  accepted.erase(std::remove_if(accepted.begin(), accepted.end(), [&](const Fit & fit) {
      return conflicts.count(fit.candidate_id) != 0;
    }), accepted.end());
  // This counter is the post-normalization, post-conflict candidate set. Raw
  // proposal volume and discarded duplicates remain available in stage stats.
  result.rawpart_candidate_count = accepted.size();
  inherit_normalized_confidence(result.cuboid_hypotheses);
  if (accepted.empty()) {
    result.rawpart_candidate_generation = "no_eligible_candidates";
  } else {
    result.rawpart_candidate_generation = "unified_l_corner_candidate_eligibility";
  }
  for (auto & patch : result.cuboid_height_patches) {
    patch.partitioned_candidate_count = std::count_if(accepted.begin(), accepted.end(),
        [&](const Fit & fit) {return fit.patch_index == &patch - result.cuboid_height_patches.data();});
  }
  *eligibility_stage.output_count = accepted.size();
  *eligibility_stage.rejected_count = *eligibility_stage.input_count - *eligibility_stage.output_count;
  eligibility_stage.auxiliary_counts["evaluated_height_patches"] = patches.size();
  eligibility_stage.auxiliary_counts["candidates_rejected_for_conflict"] = conflicts.size();
  eligibility_stage.reason = accepted.empty() ? "no_individually_eligible_candidates" :
    "individually_eligible_candidates_retained_across_all_heights";
  std::stable_sort(result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
    [](const auto & a, const auto & b) {return a.candidate_id < b.candidate_id;});
  if (patches.empty()) {
    line_stage.status = "not_computed";
    line_stage.reason = "no_height_patches";
    line_stage.input_count.reset();line_stage.output_count.reset();line_stage.rejected_count.reset();
  } else {
    line_stage.status = "computed";
    line_stage.reason = *line_stage.input_count == 0 ? "no_contours" :
      line_stage.fitted_edge_count == 0 ? "no_fitted_lines" : "fitted_lines_available";
  }
  if (geometry_stage.candidate_evaluation_attempt_count == 0) {
    geometry_stage.status = "not_computed";
    geometry_stage.reason = patches.empty() ? "no_height_patches" : "no_candidate_proposals";
  }
  if (*eligibility_stage.input_count == 0) {
    eligibility_stage.status = "not_computed";
    eligibility_stage.reason = patches.empty() ? "no_height_patches" : "no_generated_candidates";
    eligibility_stage.input_count.reset();eligibility_stage.output_count.reset();
    eligibility_stage.rejected_count.reset();
    eligibility_stage.outcome_by_status.clear();
    eligibility_stage.rejected_by_reason.clear();
  }
  if (!patches.empty()) {
    hypothesis_stage.reason = hypothesis_stage.generated_count == 0 ?
      "no_hypotheses_survived_geometry_and_deduplication" : "hypotheses_generated";
    if (geometry_stage.status == "computed") {
      geometry_stage.reason = geometry_stage.generated_count == 0 ?
        "no_validated_hypotheses" : "hypotheses_validated";
    }
  }
  result.cuboid_stages.push_back(std::move(line_stage));
  result.cuboid_stages.push_back(std::move(hypothesis_stage));
  result.cuboid_stages.push_back(std::move(geometry_stage));
  result.cuboid_stages.push_back(std::move(eligibility_stage));
  const cv::Mat missing = (models.empty() && accepted.empty()) ?
    cv::Mat() : missing_depth(obs.heights);
  if (accepted.empty()) {
    const bool final_geometry_unresolved = std::any_of(
      result.cuboid_height_patches.begin(), result.cuboid_height_patches.end(),
      [](const auto & patch) {
        return (patch.contour_count > 0 && patch.cell_count >= 3) || patch.fitted_edge_count > 0 ||
               patch.hypothesis_rejected_by_reason.count("inclined_surface") > 0 ||
               patch.hypothesis_rejected_by_reason.count("mixed_or_inclined") > 0;
      });
    result.result_code = final_geometry_unresolved ? Code::DETECT_RESULT_GEOMETRY_ERROR :
      Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
    append_disturbance_regions(result, plane_map.outlines, partitions, obs, profile);
    result.failure_cause = "cuboid top geometry unresolved";
    result.diagnostic_detail = "cuboid top geometry unresolved; " +
      result.rawpart_candidate_generation + "; disturbance_candidates=" +
      std::to_string(result.disturbance_candidates.size());
    if (!result.rawpart_rejected_by_reason.empty()) {
      result.diagnostic_detail += "; rejection_counts={";
      for (const auto & rejection : result.rawpart_rejected_by_reason) {
        if (result.diagnostic_detail.back() != '{') {result.diagnostic_detail += ',';}
        result.diagnostic_detail += rejection.first + ":" + std::to_string(rejection.second);
      }
      result.diagnostic_detail += '}';
    }
    CuboidStageDiagnostic enumeration_stage;
    enumeration_stage.stage = "candidate_enumeration";
    enumeration_stage.status = "not_computed";
    enumeration_stage.reason = "no_individually_eligible_candidates";
    result.cuboid_stages.push_back(std::move(enumeration_stage));
    auto replay = std::make_shared<CuboidReplayEvidence>();
    replay->rows = static_cast<uint32_t>(obs.heights.rows);
    replay->columns = static_cast<uint32_t>(obs.heights.cols);
    replay->resolution_m = obs.spacing_m;
    replay->origin_x_m = obs.origin.x;
    replay->origin_y_m = obs.origin.y;
    replay->height_grid_m = obs.heights;
    if (!obs.image_boundary.empty()) {
      replay->image_boundary_mask = obs.image_boundary;
    }
    result.cuboid_replay_evidence = std::move(replay);
    return result;
  }
  std::shared_ptr<CuboidReplayEvidence> selected_replay;
  if (retain_replay_evidence) {
    selected_replay = std::make_shared<CuboidReplayEvidence>();
    selected_replay->rows = static_cast<uint32_t>(obs.heights.rows);
    selected_replay->columns = static_cast<uint32_t>(obs.heights.cols);
    selected_replay->resolution_m = obs.spacing_m;
    selected_replay->origin_x_m = obs.origin.x;
    selected_replay->origin_y_m = obs.origin.y;
    selected_replay->height_grid_m = obs.heights;
    if (!obs.image_boundary.empty()) {
      selected_replay->image_boundary_mask = obs.image_boundary;
    }
  }
  // Candidate IDs define enumeration only; they carry no grasp preference.
  std::stable_sort(accepted.begin(), accepted.end(), [](const Fit & a, const Fit & b) {
      return a.candidate_id < b.candidate_id;
    });
  for (auto & item : result.cuboid_hypotheses) {
    if (std::any_of(accepted.begin(), accepted.end(), [&](const Fit & fit) {
        return fit.candidate_id == item.candidate_id;
      })) {
      if (item.disposition == "eligible_candidate" ||
        item.disposition == "eligible_l_corner_hypothesis") {
        item.disposition = "qualified_geometry";
        item.reason = "candidate_geometry_and_observation_checks_passed";
      }
    }
  }
  CuboidStageDiagnostic enumeration_stage;
  enumeration_stage.stage = "candidate_enumeration";
  enumeration_stage.count_unit = "candidate";
  enumeration_stage.input_count = accepted.size();
  enumeration_stage.output_count = accepted.size();
  enumeration_stage.auxiliary_counts["enumerated_candidates"] = accepted.size();
  enumeration_stage.reason = "deterministic_candidate_id_enumeration";
  append_disturbance_regions(result, plane_map.outlines, partitions, obs, profile);
  const auto unresolved = unresolved_observation_cells(models, obs, profile, missing);
  result.cuboid_stages.push_back(std::move(enumeration_stage));
  result.unresolved_pixels = cv::countNonZero(unresolved);
  result.rawpart_profile_match_count = accepted.size();
  for (size_t i = 0; i < accepted.size(); ++i) {
    const auto & fit = accepted[i];
    DetectionResult candidate;
    candidate.algorithm = "cuboid_top_face";
    candidate.processing_stage = "target_pose_estimation";
    candidate.thickness_evidence = "material_face_hypothesis";
    candidate.result_code = Code::DETECT_RESULT_SUCCESS;
    candidate.has_target_pose = true;
    const auto model = handoff_model(fit);
    candidate.target_pose.pose.position.x = model.center.x;
    candidate.target_pose.pose.position.y = model.center.y;
    candidate.target_pose.pose.position.z = model.top;
    candidate.target_pose.pose.orientation.z = std::sin(model.yaw / 2);
    candidate.target_pose.pose.orientation.w = std::cos(model.yaw / 2);
    candidate.has_estimated_height = true;
    candidate.estimated_height_m = fit.face.thickness;
    const bool complete_top_observed = !fit.is_partial_reconstruction;
    candidate.has_observed_rectangle = fit.rectangle_measured;
    candidate.rectangle_long_side_m = std::max(fit.measured_length, fit.measured_width);
    candidate.rectangle_short_side_m = std::min(fit.measured_length, fit.measured_width);
    candidate.orientation_symmetry_order = std::abs(fit.face.length - fit.face.width) <
      1e-9 ? 4 : 2;
    candidate.observed_yaw_rad = model.yaw;
    candidate.profile_candidate_valid = true;
    candidate.profile_candidate_features =
    {model.center.x, model.center.y, model.top,
      fit.face.length, fit.face.width, fit.face.thickness};
    candidate.confidence_grade = complete_top_observed ? "HIGH" : "MEDIUM";
    candidate.confidence_reason = complete_top_observed ?
      "top_surface_observed_and_individual_pose_resolved" :
      "partial_top_surface_resolved_by_l_corner_height_and_recipe_evidence";
    for (auto & hypothesis : result.cuboid_hypotheses) {
      if (hypothesis.candidate_id == fit.candidate_id) {
        hypothesis.confidence_grade = candidate.confidence_grade;
        hypothesis.confidence_reason = candidate.confidence_reason;
      }
    }
    candidate.unresolved_pixels = result.unresolved_pixels;
    candidate.rawpart_selected_candidate = std::to_string(fit.candidate_id);
    candidate.diagnostic_detail = "cuboid_top_face; " +
      result.rawpart_candidate_generation + "; confidence=" +
      candidate.confidence_grade + "; " + candidate.confidence_reason;
    RawPartCandidateDiagnostic diagnostic;
    diagnostic.candidate_id = fit.candidate_id;
    diagnostic.confidence_grade = candidate.confidence_grade;
    diagnostic.confidence_reason = candidate.confidence_reason;
    diagnostic.position_x_m = model.center.x; diagnostic.position_y_m = model.center.y;
    diagnostic.position_z_m = model.top;
    diagnostic.measured_width_m = fit.rectangle_measured ? fit.measured_length : 0.0;
    diagnostic.measured_height_m = fit.rectangle_measured ? fit.measured_width : 0.0;
    diagnostic.expected_width_m = fit.face.length; diagnostic.expected_height_m = fit.face.width;
    diagnostic.expected_vertical_height_m = fit.face.thickness;
    diagnostic.measured_vertical_height_m = 0; // Own thickness is inferred, not measured stack elevation.
    diagnostic.coverage = fit.area; diagnostic.edge_support = fit.boundary.value();
    diagnostic.profile_match = true;
    diagnostic.orientation_symmetry_order = candidate.orientation_symmetry_order;
    diagnostic.observed_yaw_valid = true;
    diagnostic.observed_yaw_rad = fit.yaw;
    for (const auto & p : corners(fit)) {
      geometry_msgs::msg::Point32 point; point.x = p.x; point.y = p.y; point.z = fit.top;
      diagnostic.fitted_outline_support.push_back(point);
    }
    result.rawpart_candidate_diagnostics.push_back(diagnostic);
    result.ordered_candidates.push_back(std::move(candidate));
  }
  inherit_normalized_confidence(result.cuboid_hypotheses);
  result.result_code = Code::DETECT_RESULT_SUCCESS;
  result.processing_stage = "target_pose_estimation";
  result.diagnostic_detail = "cuboid_top_face; " + result.rawpart_candidate_generation;
  if (!result.cuboid_replay_evidence) {
    result.cuboid_replay_evidence = std::move(selected_replay);
  }
  return result;
}

DetectionResult process_cuboid_top_face(
  const DetectionInput & input, const DetectorProfile & profile,
  const tf2::Transform & support_T_camera)
{
  const double spacing = profile.cuboid.grid_resolution_m;
  const auto & lo = profile.workspace_roi_min_m;
  const auto & hi = profile.workspace_roi_max_m;
  const int cols = static_cast<int>(std::ceil((hi[0] - lo[0]) / spacing)) + 1;
  const int rows = static_cast<int>(std::ceil((hi[1] - lo[1]) / spacing)) + 1;
  if (rows <= 0 || cols <= 0 || static_cast<int64_t>(rows) * cols > 4000000) {
    throw std::invalid_argument("RawPart.cuboid grid exceeds bounded observation size");
  }
  CuboidHeightObservation obs{cv::Mat(
      rows, cols, CV_64F,
      cv::Scalar(std::numeric_limits<double>::quiet_NaN())), {lo[0], lo[1]}, spacing, {}};
  obs.image_boundary = cv::Mat::zeros(rows, cols, CV_8U);
  const auto & info = input.camera_info;
  // Only camera rays are cached; support calibration and Tray ROI remain current per observation.
  const auto rays = pixel_ray_lookup(info, input.depth.width, input.depth.height);
  const size_t pixel_count = static_cast<size_t>(input.depth.width) * input.depth.height;
  for (size_t i = 0; i < pixel_count; ++i) {
    const size_t r = i / input.depth.width, c = i % input.depth.width;
    float depth = 0;
    if (input.depth.encoding == "32FC1") {
      std::memcpy(&depth, input.depth.data.data() + r * input.depth.step + c * 4, 4);
    } else {
      uint16_t mm; std::memcpy(&mm, input.depth.data.data() + r * input.depth.step + c * 2, 2);
      depth = mm / 1000.0F;
    }
    if (!std::isfinite(depth) || depth <= 0) {continue;}
    const auto & center_ray = rays->centers[i];
    const auto p = support_T_camera * tf2::Vector3(
      center_ray.x * depth, center_ray.y * depth,
      depth);
    if (p.x() < lo[0] || p.x() > hi[0] || p.y() < lo[1] || p.y() > hi[1]) {continue;}
    std::vector<cv::Point2f> cell_polygon;
    for (size_t corner = 0; corner < 4; ++corner) {
      const auto & ray = rays->corners[i][corner];
      const auto point = support_T_camera * tf2::Vector3(ray.x * depth, ray.y * depth, depth);
      cell_polygon.emplace_back((point.x() - lo[0]) / spacing, (point.y() - lo[1]) / spacing);
    }
    const cv::Rect bounds = cv::boundingRect(cell_polygon) & cv::Rect(0, 0, cols, rows);
    for (int y = bounds.y; y < bounds.y + bounds.height; ++y) {
      for (int x = bounds.x; x < bounds.x + bounds.width; ++x) {
        if (cv::pointPolygonTest(cell_polygon, cv::Point2f(x, y), false) < 0) {continue;}
        auto & z = obs.heights.at<double>(y, x);
        if (!std::isfinite(z) || p.z() > z) {z = p.z();}
        if (r == 0 || c == 0 || r + 1 == input.depth.height || c + 1 == input.depth.width) {
          obs.image_boundary.at<uint8_t>(y, x) = 255;
        }
      }
    }
    // Retain a valid subcell sample even if its footprint contains no grid center.
    const int x = std::lround((p.x() - lo[0]) / spacing),
      y = std::lround((p.y() - lo[1]) / spacing);
    auto & z = obs.heights.at<double>(y, x);
    if (!std::isfinite(z) || p.z() > z) {z = p.z();}
    if (r == 0 || c == 0 || r + 1 == input.depth.height || c + 1 == input.depth.width) {
      obs.image_boundary.at<uint8_t>(y, x) = 255;
    }
  }
  auto result = detect_cuboid_top_face(obs, profile, input.detailed_diagnostics_enabled);
  if (result.cuboid_replay_evidence) {
    result.cuboid_replay_evidence->frame_id = "Tray";
  }
  tf2::Quaternion q(input.optical_to_base.transform.rotation.x,
    input.optical_to_base.transform.rotation.y,
    input.optical_to_base.transform.rotation.z, input.optical_to_base.transform.rotation.w);
  q.normalize();
  const auto & t = input.optical_to_base.transform.translation;
  const tf2::Transform base_T_support =
    tf2::Transform(q, {t.x, t.y, t.z}) * support_T_camera.inverse();
  auto transform_pose = [&](geometry_msgs::msg::Pose & pose) {
      const auto p = base_T_support *
        tf2::Vector3(pose.position.x, pose.position.y, pose.position.z);
      tf2::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z,
        pose.orientation.w);
      rotation = base_T_support.getRotation() * rotation; rotation.normalize();
      pose.position.x = p.x();pose.position.y = p.y();pose.position.z = p.z();
      pose.orientation.x = rotation.x();pose.orientation.y = rotation.y();
      pose.orientation.z = rotation.z();pose.orientation.w = rotation.w();
    };
  for (auto & candidate : result.disturbance_candidates) {
    candidate.center.header = input.depth.header;
    candidate.center.header.frame_id = "base_link";
    transform_pose(candidate.center.pose);
  }
  if (input.rgb.width > 0 && input.rgb.height > 0 &&
    input.rgb.header.frame_id == input.depth.header.frame_id && input.rgb.encoding == "rgb8" &&
    static_cast<size_t>(input.rgb.step) >= static_cast<size_t>(input.rgb.width) * 3U &&
    input.rgb.data.size() >= static_cast<size_t>(input.rgb.step) * input.rgb.height)
  {
    result.source_rgb_header = input.rgb.header;
  }
  auto finalize = [&](DetectionResult & candidate) {
      candidate.source_rgb_header = result.source_rgb_header;
      candidate.target_pose.header = input.depth.header;
      candidate.target_pose.header.frame_id = "base_link";
      transform_pose(candidate.target_pose.pose);
    };
  for (auto & diagnostic : result.rawpart_candidate_diagnostics) {
    std::vector<cv::Point3f> camera_points;
    for (const auto & p : diagnostic.fitted_outline_support) {
      const auto camera = support_T_camera.inverse() * tf2::Vector3(p.x, p.y, p.z);
      camera_points.emplace_back(camera.x(), camera.y(), camera.z());
    }
    if (!camera_points.empty()) {
      std::vector<cv::Point2f> projected;
      cv::projectPoints(
        camera_points, cv::Vec3d(0, 0, 0), cv::Vec3d(
          0, 0,
          0), rays->camera_matrix, rays->distortion, projected);
      for (const auto & pixel : projected) {
        geometry_msgs::msg::Point32 p; p.x = pixel.x;p.y = pixel.y;
        diagnostic.fitted_outline_pixels.push_back(p);
      }
    }
  }
  for (auto & candidate : result.ordered_candidates) {
    finalize(candidate);
  }
  return result;
}
}  // namespace arm_cell_vision
