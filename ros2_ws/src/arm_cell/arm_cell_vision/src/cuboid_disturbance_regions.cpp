#include "cuboid_top_face_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace arm_cell_vision::cuboid_geometry
{
namespace
{
using Hypothesis = CuboidHypothesisVisualization;

const Hypothesis & representative(const Hypothesis & hypothesis, const DetectionResult & result)
{
  const auto * current = &hypothesis;
  for (size_t i = 0; current->deduplicated_to_candidate_id && i < result.cuboid_hypotheses.size();
    ++i)
  {
    const auto next = std::find_if(
      result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
      [&](const auto & h) {return h.candidate_id == current->deduplicated_to_candidate_id;});
    if (next == result.cuboid_hypotheses.end()) {break;}
    current = &*next;
  }
  return *current;
}
bool confirmed(const Hypothesis & h)
{
  return h.selection_state == "SELECTED" &&
         (h.confidence_grade == "HIGH" || h.confidence_grade == "MEDIUM");
}
bool unresolved(const Hypothesis & h)
{
  return h.selection_state != "REJECTED" &&
         (h.confidence_grade == "LOW" || h.selection_state == "UNRESOLVED");
}
std::vector<cv::Point2f> hypothesis_footprint(const Hypothesis & h)
{
  if (!std::isfinite(h.center_x_m) || !std::isfinite(h.center_y_m) ||
    !std::isfinite(h.yaw_rad) || !std::isfinite(h.length_m) || !std::isfinite(h.width_m) ||
    h.length_m <= 0 || h.width_m <= 0) {return {};}
  Fit fit;
  fit.center = {h.center_x_m, h.center_y_m}; fit.yaw = h.yaw_rad;
  fit.face = {h.length_m, h.width_m, h.thickness_m, 0, 0};
  return corners(fit);
}

void append_candidate(
  DetectionResult & result, const ObservationRegion & region,
  const std::vector<const Hypothesis *> & hypotheses, const CuboidHeightObservation & obs)
{
  auto xy = region.observed_centroid;
  std::string method = "outline_centroid";
  if (!hypotheses.empty()) {
    auto intersection = hypothesis_footprint(*hypotheses.front());
    for (size_t i = 1; !intersection.empty() && i < hypotheses.size(); ++i) {
      const auto footprint = hypothesis_footprint(*hypotheses[i]);
      std::vector<cv::Point2f> next;
      if (footprint.empty() || cv::intersectConvexConvex(intersection, footprint, next) <= 0) {
        intersection.clear(); break;
      }
      intersection = std::move(next);
    }
    // Cuboid footprints are convex polygons, so their intersection is convex
    // as well. Compute one area centroid; never split it into new candidates.
    if (!intersection.empty()) {
      const auto moments = cv::moments(intersection);
      if (std::isfinite(moments.m00) && moments.m00 != 0) {
        xy = {moments.m10 / moments.m00, moments.m01 / moments.m00};
        method = hypotheses.size() == 1 ? "single_hypothesis" : "footprint_intersection";
        if (hypotheses.size() == 1) {
          xy = {hypotheses[0]->center_x_m, hypotheses[0]->center_y_m};
        }
      }
    }
  }
  if (!std::isfinite(xy.x) || !std::isfinite(xy.y)) {return;}
  double height = region.mean_observed_height.value_or(std::numeric_limits<double>::quiet_NaN());
  const double column = (xy.x - obs.origin.x) / obs.spacing_m;
  const double row = (xy.y - obs.origin.y) / obs.spacing_m;
  const auto belongs_to_observation = [&](double col, double row_index) {
      if (!region.observed_boundary || region.observed_boundary->empty() ||
        cv::pointPolygonTest(*region.observed_boundary, cv::Point2f(col, row_index), false) < 0)
      {return false;}
      const auto point = obs.origin + cv::Point2d(col, row_index) * obs.spacing_m;
      return region.partition_boundary.empty() ||
             cv::pointPolygonTest(region.partition_boundary, point, false) >= 0;
    };
  if (column >= -0.5 && column < obs.heights.cols - 0.5 &&
    row >= -0.5 && row < obs.heights.rows - 0.5)
  {
    const int x = cvRound(column), y = cvRound(row);
    // Both XY and the sampled grid center must belong to this observed region;
    // a nearby cell on another object/partition is not this target's evidence.
    if (belongs_to_observation(column, row) && belongs_to_observation(x, y)) {
      const double measured = obs.heights.at<double>(y, x);
      if (std::isfinite(measured)) {height = measured;}
    }
  }
  if (!std::isfinite(height)) {return;}
  arm_cell_interfaces::msg::DisturbanceCandidate candidate;
  candidate.candidate_id = "disturbance-" +
    std::to_string(result.disturbance_candidates.size() + 1);
  candidate.category = region.category;
  candidate.center.pose.position.x = xy.x; candidate.center.pose.position.y = xy.y;
  candidate.center.pose.position.z = height; candidate.center.pose.orientation.w = 1;
  candidate.observed_height_m = height;
  DisturbanceRegionDiagnostic diagnostic;
  diagnostic.candidate_id = candidate.candidate_id;
  diagnostic.source_boundary_id = region.source_boundary_id;
  diagnostic.outline_id = region.outline_id;
  diagnostic.patch_index = region.patch_index; diagnostic.rectangle_id = region.rectangle_id;
  diagnostic.representative_method = method;
  diagnostic.reason = hypotheses.empty() ? "existing_region_geometry_not_reconstructed" :
    "existing_region_low_or_unresolved_hypotheses";
  for (const auto * h : hypotheses) {
    diagnostic.hypothesis_ids.push_back(h->candidate_id);
  }
  result.disturbance_candidates.push_back(std::move(candidate));
  result.disturbance_regions.push_back(std::move(diagnostic));
}
}  // namespace

void append_disturbance_regions(
  DetectionResult & result, const std::vector<ObservationRegion> & outlines,
  const std::vector<ObservationRegion> & partitions,
  const CuboidHeightObservation & observation, const DetectorProfile &)
{
  for (const auto & outline : outlines) {
    std::vector<const Hypothesis *> related;
    bool has_confirmed = false;
    for (const auto & h : result.cuboid_hypotheses) {
      if (h.source_boundary_id != outline.source_boundary_id ||
        h.outline_id != outline.outline_id)
      {
        continue;
      }
      const auto & selected = representative(h, result);
      has_confirmed |= confirmed(selected);
      related.push_back(&h);
    }
    if (!has_confirmed) {
      std::vector<const Hypothesis *> hypotheses;
      for (const auto * original : related) {
        const auto & h = representative(*original, result);
        if (unresolved(h) && std::none_of(
            hypotheses.begin(), hypotheses.end(),
            [&](const auto * item) {return item->candidate_id == h.candidate_id;}))
        {hypotheses.push_back(&h);}
      }
      if (outline.unresolved || !hypotheses.empty()) {
        append_candidate(result, outline, hypotheses, observation);
      }
      continue;
    }
    // A confirmed object forbids its entire parent outline. Only an existing,
    // independently resolved partition with explicit hypothesis correspondence
    // can locate a separate unresolved target; do not subtract its cells.
    for (const auto & partition : partitions) {
      if (!partition.independent || partition.source_boundary_id != outline.source_boundary_id ||
        partition.outline_id != outline.outline_id) {continue;}
      const bool uncertain_correspondence = std::any_of(
        related.begin(), related.end(),
        [&](const auto * h) {
          const bool is_confirmed = confirmed(representative(*h, result));
          return is_confirmed && h->patch_index == partition.patch_index && h->rectangle_id == 0;
        });
      if (uncertain_correspondence) {continue;}
      std::vector<const Hypothesis *> hypotheses;
      bool partition_confirmed = false, has_other_interpretation = false;
      for (const auto * h : related) {
        if (h->patch_index != partition.patch_index || h->rectangle_id != partition.rectangle_id) {
          continue;
        }
        const auto & selected = representative(*h, result);
        partition_confirmed |= confirmed(selected);
        has_other_interpretation |= selected.selection_state == "ALTERNATIVE";
        if (unresolved(selected) && std::none_of(
            hypotheses.begin(), hypotheses.end(),
            [&](const auto * item) {return item->candidate_id == selected.candidate_id;}))
        {hypotheses.push_back(&selected);}
      }
      if (!partition_confirmed && (!hypotheses.empty() ||
        (partition.unresolved && !has_other_interpretation)))
      {
        append_candidate(result, partition, hypotheses, observation);
      }
    }
  }
}
}  // namespace arm_cell_vision::cuboid_geometry
