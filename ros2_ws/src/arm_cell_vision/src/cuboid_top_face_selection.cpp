#include "cuboid_top_face_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <set>
#include <opencv2/imgproc.hpp>

namespace arm_cell_vision
{
namespace cuboid_geometry
{
bool materially_overlaps(const Fit & a, const Fit & b, double spacing)
{
  std::vector<cv::Point2f> intersection;
  const double overlap = cv::intersectConvexConvex(corners(a), corners(b), intersection);
  const double one_cell_band = spacing * std::min(
    {a.face.length, a.face.width, b.face.length, b.face.width});
  return overlap > one_cell_band;
}

// Compare only incompatible interpretations. Cell IDs and line indices refer to
// the original measurement, never to a filled Recipe or partition mask.
int compare_observation(
  const Fit & a, const Fit & b,
  const CuboidHeightObservation & obs, const DetectorProfile & profile)
{
  const double epsilon = std::max(
    obs.spacing_m / std::sqrt(2.0),
    profile.cuboid.line_residual_limit_px * obs.spacing_m);
  cv::Mat observed = cv::Mat::zeros(obs.heights.size(), CV_8U);
  for (int y = 0; y < observed.rows; ++y) {
    for (int x = 0; x < observed.cols; ++x) {
      const double z = obs.heights.at<double>(y, x);
      if (std::isfinite(z) &&
        (within_height_tolerance(z, a.top, profile.cuboid.height_tolerance_m) ||
        within_height_tolerance(z, b.top, profile.cuboid.height_tolerance_m)))
      {
        observed.at<uint8_t>(y, x) = 255;
      }
    }
  }
  cv::Mat core;
  const int band = static_cast<int>(std::ceil(epsilon / obs.spacing_m));
  cv::erode(observed, core, cv::Mat::ones(2 * band + 1, 2 * band + 1, CV_8U));
  if (a.observed_cell_ids.size() != b.observed_cell_ids.size()) {
    std::vector<uint32_t> difference;
    std::set_symmetric_difference(
      a.observed_cell_ids.begin(), a.observed_cell_ids.end(),
      b.observed_cell_ids.begin(), b.observed_cell_ids.end(), std::back_inserter(difference));
    const bool outside_band = std::any_of(
      difference.begin(), difference.end(),
      [&](uint32_t id) {
        return core.at<uint8_t>(id / obs.heights.cols, id % obs.heights.cols) != 0;
      });
    if (outside_band) {return a.observed_cell_ids.size() > b.observed_cell_ids.size() ? 1 : -1;}
  }
  // Compare the same original line once, including its endpoint uncertainty.
  auto lengths = a.observed_line_support_m;
  for (const auto & line : b.observed_line_support_m) {
    lengths.emplace(line.first, 0);
  }
  bool beyond_endpoint_band = false;
  double total_a = 0, total_b = 0;
  for (const auto & line : lengths) {
    const auto ai = a.observed_line_support_m.find(line.first);
    const auto bi = b.observed_line_support_m.find(line.first);
    const double al = ai == a.observed_line_support_m.end() ? 0 : ai->second;
    const double bl = bi == b.observed_line_support_m.end() ? 0 : bi->second;
    total_a += al; total_b += bl;
    beyond_endpoint_band |= std::abs(al - bl) > 2 * epsilon;
  }
  if (beyond_endpoint_band && std::abs(total_a - total_b) > 2 * epsilon) {
    return total_a > total_b ? 1 : -1;
  }
  if (std::abs(a.height_residual - b.height_residual) > 1) {
    return a.height_residual < b.height_residual ? 1 : -1;
  }
  if (a.dimension_observation_count && b.dimension_observation_count &&
    std::abs(a.dimension_residual - b.dimension_residual) > 1)
  {
    return a.dimension_residual < b.dimension_residual ? 1 : -1;
  }
  if (a.complete_iou && b.complete_iou && a.patch_index == b.patch_index &&
    a.rectangle_id == b.rectangle_id)
  {
    // Boundary-band-only differences are already a tie in actual support.
    const int outside = cv::countNonZero((a.mask ^ b.mask) & core);
    if (outside && std::abs(*a.complete_iou - *b.complete_iou) > 1e-9) {
      return *a.complete_iou > *b.complete_iou ? 1 : -1;
    }
  }
  // A locally observed support plane can break an otherwise unresolved face/
  // thickness tie. Do not prefer ground thickness from top elevation alone.
  if (a.support_height_m && b.support_height_m &&
    std::abs(*a.support_height_m - *b.support_height_m) <= profile.cuboid.height_tolerance_m)
  {
    const auto supported = [&](const Fit & fit) {
        return std::abs(fit.top - fit.face.thickness - *fit.support_height_m) <=
               profile.support_contact_tolerance_m;
      };
    if (supported(a) != supported(b)) {return supported(a) ? 1 : -1;}
  }
  return 0;
}

std::map<uint32_t, Selection> select_hypotheses(
  const std::vector<Fit> & candidates, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, size_t & operations)
{
  std::map<uint32_t, Selection> selections;
  std::vector<std::pair<uint32_t, uint32_t>> ties;
  std::map<uint32_t, std::vector<uint32_t>> stronger;
  for (const auto & fit : candidates) {
    selections[fit.candidate_id] = {};
  }
  const double tolerance = profile.cuboid.height_tolerance_m;
  for (size_t i = 0; i < candidates.size(); ++i) {
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      const auto & a = candidates[i]; const auto & b = candidates[j];
      if (!materially_overlaps(a, b, obs.spacing_m)) {continue;}
      const double overlap_z = std::min(a.top, b.top) -
        std::max(a.top - a.face.thickness, b.top - b.face.thickness);
      if (overlap_z <= tolerance) {continue;}
      auto & sa = selections[a.candidate_id]; auto & sb = selections[b.candidate_id];
      sa.competitors.push_back(b.candidate_id); sb.competitors.push_back(a.candidate_id);
      int comparison = 0;
      if (operations > 0) {--operations; comparison = compare_observation(a, b, obs, profile);}
      if (comparison == 0) {ties.emplace_back(a.candidate_id, b.candidate_id);} else {
        const auto loser = comparison > 0 ? b.candidate_id : a.candidate_id;
        const auto winner = comparison > 0 ? a.candidate_id : b.candidate_id;
        stronger[loser].push_back(winner);
      }
    }
  }
  // A discarded spanning proposal must not suppress an independently supported
  // neighbour. Resolve strict local dominance before propagating tied ambiguity;
  // this compares hypotheses and never searches combinations of object layouts.
  std::set<uint32_t> active;
  for (const auto & fit : candidates) {
    active.insert(fit.candidate_id);
  }
  while (!active.empty()) {
    std::vector<uint32_t> roots;
    for (uint32_t id : active) {
      const auto & better = stronger[id];
      if (std::none_of(
          better.begin(), better.end(), [&](uint32_t other) {
            return active.count(other);
          }))
      {
        roots.push_back(id);
      }
    }
    if (roots.empty()) {
      for (uint32_t id : active) {
        selections[id].state = "UNRESOLVED";
        selections[id].reason = "observation_uncertainty_prevents_strict_dominance";
      }
      break;
    }
    std::vector<uint32_t> alternatives;
    for (uint32_t id : active) {
      const auto & better = stronger[id];
      if (std::any_of(
          roots.begin(), roots.end(), [&](uint32_t root) {
            return std::find(better.begin(), better.end(), root) != better.end();
          })) {alternatives.push_back(id);}
    }
    for (uint32_t id : alternatives) {
      selections[id].state = "ALTERNATIVE";
      selections[id].reason = "competing_interpretation_has_stronger_observation_support";
      active.erase(id);
    }
    for (uint32_t id : roots) {
      active.erase(id);
    }
  }
  for (const auto & tie : ties) {
    auto & a = selections[tie.first]; auto & b = selections[tie.second];
    if (a.state == "ALTERNATIVE" || b.state == "ALTERNATIVE") {continue;}
    a.state = b.state = "UNRESOLVED";
    a.reason = b.reason = operations == 0 ? "selection_evaluation_budget_exhausted" :
      "distinct_pose_or_layout_hypotheses_remain_indistinguishable";
  }
  return selections;
}

}  // namespace cuboid_geometry
}  // namespace arm_cell_vision
