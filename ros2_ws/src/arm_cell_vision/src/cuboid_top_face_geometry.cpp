#include "cuboid_top_face_geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <opencv2/imgproc.hpp>

namespace arm_cell_vision
{
namespace cuboid_geometry
{
constexpr double pi = 3.14159265358979323846;
cv::Point2d axis(double yaw) {return {std::cos(yaw), std::sin(yaw)};}
cv::Point2d normal(double yaw) {return {-std::sin(yaw), std::cos(yaw)};}

std::vector<cv::Point2f> corners(const Fit & fit)
{
  std::vector<cv::Point2f> result;
  for (const auto & signs : std::array<cv::Point2d, 4>{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}}) {
    const auto p = fit.center + axis(fit.yaw) * (signs.x * fit.face.length / 2) +
      normal(fit.yaw) * (signs.y * fit.face.width / 2);
    result.emplace_back(p.x, p.y);
  }
  return result;
}

cv::Mat footprint(const Fit & fit, const CuboidHeightObservation & obs)
{
  cv::Mat mask = cv::Mat::zeros(obs.heights.size(), CV_8U);
  std::vector<cv::Point> polygon;
  for (const auto & p : corners(fit)) {
    polygon.emplace_back(
      std::lround((p.x - obs.origin.x) / obs.spacing_m),
      std::lround((p.y - obs.origin.y) / obs.spacing_m));
  }
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygon}, cv::Scalar(255));
  return mask;
}

bool within_height_tolerance(double a, double b, double tolerance)
{
  const double floating_point_slack = 2 * std::numeric_limits<double>::epsilon() *
    std::max({1.0, std::abs(a), std::abs(b)});
  return std::abs(a - b) <= tolerance + floating_point_slack;
}

namespace
{
std::vector<Face> faces_impl(const DetectorProfile & profile)
{
  std::vector<Face> result;
  for (int height_axis = 0; height_axis < 3; ++height_axis) {
    int length_axis = (height_axis + 1) % 3, width_axis = (height_axis + 2) % 3;
    if (profile.dimensions_m[length_axis] < profile.dimensions_m[width_axis]) {
      std::swap(length_axis, width_axis);
    }
    Face f{profile.dimensions_m[length_axis], profile.dimensions_m[width_axis],
      profile.dimensions_m[height_axis],
      profile.dimension_tolerance_m[length_axis], profile.dimension_tolerance_m[width_axis]};
    const auto same = [&](const Face & other) {
        return std::abs(f.length - other.length) < 1e-9 &&
               std::abs(f.width - other.width) < 1e-9 &&
               std::abs(f.thickness - other.thickness) < 1e-9;
      };
    if (std::none_of(result.begin(), result.end(), same)) {result.push_back(f);}
  }
  return result;
}

double median(std::vector<double> values)
{
  if (values.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2 != 0) {return *middle;}
  const double upper = *middle;
  const double lower = *std::max_element(values.begin(), middle);
  return (lower + upper) / 2;
}

bool has_spatial_height_trend(
  const std::vector<cv::Point> & component, const cv::Mat & heights, bool along_x,
  double tolerance)
{
  int minimum = std::numeric_limits<int>::max();
  int maximum = std::numeric_limits<int>::min();
  for (const auto & pixel : component) {
    const int coordinate = along_x ? pixel.x : pixel.y;
    minimum = std::min(minimum, coordinate);
    maximum = std::max(maximum, coordinate);
  }
  if (maximum <= minimum) {return false;}
  std::array<std::vector<double>, 3> bands;
  const int span = maximum - minimum + 1;
  for (const auto & pixel : component) {
    const int coordinate = along_x ? pixel.x : pixel.y;
    const size_t band = std::min<size_t>(2, (3 * (coordinate - minimum)) / span);
    bands[band].push_back(heights.at<double>(pixel.y, pixel.x));
  }
  if (std::any_of(bands.begin(), bands.end(), [](const auto & band) {return band.empty();})) {
    return false;
  }
  const std::array<double, 3> level{{median(bands[0]), median(bands[1]), median(bands[2])}};
  const bool increasing = level[2] - level[0] > tolerance &&
    level[1] >= level[0] - tolerance && level[2] >= level[1] - tolerance;
  const bool decreasing = level[0] - level[2] > tolerance &&
    level[1] <= level[0] + tolerance && level[2] <= level[1] + tolerance;
  return increasing || decreasing;
}

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return 0;}
  std::sort(values.begin(), values.end());
  const double index = fraction * (values.size() - 1);
  const auto low = static_cast<size_t>(index);
  return values[low] + (index - low) *
         (values[static_cast<size_t>(std::ceil(index))] - values[low]);
}

double geometry_tolerance(const DetectorProfile & profile)
{
  return profile.top_surface_geometry_tolerance_m > 0 ?
         profile.top_surface_geometry_tolerance_m : profile.cuboid.height_tolerance_m;
}

void measure_plane(PlaneRegion & patch, const CuboidHeightObservation & obs,
  const DetectorProfile & profile)
{
  std::vector<double> heights;
  cv::Point2d mean;
  double mean_z = 0;
  for (const auto & pixel : patch.pixels) {
    const double z = obs.heights.at<double>(pixel);
    heights.push_back(z); mean += cv::Point2d(pixel); mean_z += z;
  }
  patch.height = median(heights);
  mean *= 1.0 / heights.size(); mean_z /= heights.size();
  cv::Matx22d covariance(0, 0, 0, 0); cv::Vec2d cross(0, 0);
  for (size_t i = 0; i < patch.pixels.size(); ++i) {
    const auto pixel = patch.pixels[i];
    const auto d = (cv::Point2d(pixel) - mean) * obs.spacing_m;
    covariance(0, 0) += d.x * d.x; covariance(0, 1) += d.x * d.y;
    covariance(1, 0) += d.x * d.y; covariance(1, 1) += d.y * d.y;
    cross[0] += d.x * (heights[i] - mean_z); cross[1] += d.y * (heights[i] - mean_z);
  }
  cv::Vec2d gradient(0, 0); cv::solve(covariance, cross, gradient, cv::DECOMP_SVD);
  double plane_low = std::numeric_limits<double>::infinity(), plane_high = -plane_low;
  for (const auto & pixel : patch.pixels) {
    const auto offset = (cv::Point2d(pixel) - mean) * obs.spacing_m;
    const double predicted = offset.x * gradient[0] + offset.y * gradient[1];
    plane_low = std::min(plane_low, predicted); plane_high = std::max(plane_high, predicted);
  }
  const double span = plane_high - plane_low;
  const double spread = percentile(heights, 0.95) - percentile(heights, 0.05);
  double squared_residual = 0;
  for (size_t i = 0; i < patch.pixels.size(); ++i) {
    const auto offset = (cv::Point2d(patch.pixels[i]) - mean) * obs.spacing_m;
    squared_residual += std::pow(heights[i] - mean_z -
      offset.x * gradient[0] - offset.y * gradient[1], 2);
  }
  patch.normal = cv::Vec3d(-gradient[0], -gradient[1], 1);
  patch.normal /= cv::norm(patch.normal);
  patch.residual_m = std::sqrt(squared_residual / heights.size());
  patch.height_span_m = span;
  cv::Mat eigenvalues;
  cv::eigen(covariance, eigenvalues);
  patch.plane_fitted = heights.size() >= 3 && eigenvalues.at<double>(0) > 0 &&
    eigenvalues.at<double>(1) > std::numeric_limits<double>::epsilon() * eigenvalues.at<double>(0);
  patch.center_height = {obs.origin.x + mean.x * obs.spacing_m,
    obs.origin.y + mean.y * obs.spacing_m, mean_z};
  patch.observed_height_p05_p95 = {percentile(heights, 0.05), percentile(heights, 0.95)};
  patch.status = !patch.plane_fitted ? "unresolved_insufficient_support" :
    std::max(spread, span) <= geometry_tolerance(profile) ? "horizontal" : "mixed_or_inclined";
}

std::vector<ContourEvidence> extract_contours_impl(
  const PlaneRegion & patch, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, CuboidHeightPatchDiagnostic & diagnostic,
  bool retain_visualization_evidence,
  std::vector<ContourEvidence> * occupancy_contours = nullptr)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(patch.cells.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::vector<ContourEvidence> result;
  for (size_t contour_index = 0; contour_index < contours.size(); ++contour_index) {
    const auto & contour = contours[contour_index];
    ++diagnostic.contour_count;
    if (retain_visualization_evidence) {
      std::vector<geometry_msgs::msg::Point32> saved;
      saved.reserve(contour.size());
      for (const auto & point : contour) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(obs.origin.x + point.x * obs.spacing_m);
        p.y = static_cast<float>(obs.origin.y + point.y * obs.spacing_m);
        saved.push_back(p);
      }
      diagnostic.observed_contours_xy.push_back(std::move(saved));
    }
    auto retain_narrow_boundary = [&]() {
        if (!occupancy_contours || contour.size() < 3) {return;}
        ContourEvidence occupancy;
        occupancy.points = contour;
        cv::approxPolyDP(contour, occupancy.vertices, 0.0, true);
        if (occupancy.vertices.size() >= 3) {occupancy_contours->push_back(std::move(occupancy));}
      };
    if (contour.size() < 4) {
      retain_narrow_boundary();
      ++diagnostic.contour_rejected_by_reason["too_few_contour_points"]; continue;
    }
    ContourEvidence evidence;
    evidence.outline_id = result.size() + 1;
    evidence.points = contour;
    cv::approxPolyDP(contour, evidence.vertices, profile.cuboid.line_residual_limit_px, true);
    if (evidence.vertices.size() < 3) {
      retain_narrow_boundary();
      ++diagnostic.contour_rejected_by_reason["fewer_than_three_observed_vertices"]; continue;
    }
    if (occupancy_contours) {occupancy_contours->push_back(evidence);}
    for (size_t i = 0; i < evidence.vertices.size(); ++i) {
      const auto previous = evidence.vertices[(i + evidence.vertices.size() - 1) % evidence.vertices.size()];
      const auto current = evidence.vertices[i];
      const auto next = evidence.vertices[(i + 1) % evidence.vertices.size()];
      const cv::Point2d a(current - previous), b(next - current);
      const double turn = (a.x * b.y - a.y * b.x) * cv::contourArea(contour, true);
      diagnostic.corners_xy_turn.push_back({obs.origin.x + current.x * obs.spacing_m,
        obs.origin.y + current.y * obs.spacing_m, turn > 0 ? 1.0 : turn < 0 ? -1.0 : 0.0});
      ++diagnostic.edge_fit_attempt_count;
      const auto first = evidence.vertices[i];
      const auto last = evidence.vertices[(i + 1) % evidence.vertices.size()];
      if (cv::norm(last - first) < 2) {
        ++diagnostic.edge_fit_rejected_by_reason["approximated_edge_too_short"]; continue;
      }
      const auto start = std::find(contour.begin(), contour.end(), first) - contour.begin();
      const auto end = std::find(contour.begin(), contour.end(), last) - contour.begin();
      std::vector<cv::Point2f> arc;
      for (size_t offset = 0; offset <= (end + contour.size() - start) % contour.size(); ++offset) {
        arc.emplace_back(contour[(start + offset) % contour.size()]);
      }
      const size_t lo = std::lround(0.05 * (arc.size() - 1));
      const size_t hi = std::max(lo + 1, static_cast<size_t>(std::lround(0.95 * (arc.size() - 1))));
      if (hi >= arc.size()) {
        ++diagnostic.edge_fit_rejected_by_reason["insufficient_interior_points"]; continue;
      }
      std::vector<cv::Point2f> support(arc.begin() + lo, arc.begin() + hi + 1);
      const double fraction = static_cast<double>(support.size()) / arc.size();
      cv::Vec4f line;
      cv::fitLine(support, line, cv::DIST_L2, 0, 0.01, 0.01);
      const cv::Point2d u(line[0], line[1]), center(line[2], line[3]);
      const cv::Point2d n(-u.y, u.x);
      double residual = 0, low = std::numeric_limits<double>::infinity(), high = -low;
      for (const auto & pixel : support) {
        residual += std::pow((cv::Point2d(pixel) - center).dot(n), 2);
      }
      if (fraction < profile.cuboid.minimum_line_support ||
        std::sqrt(residual / support.size()) > profile.cuboid.line_residual_limit_px)
      {
        ++diagnostic.edge_fit_rejected_by_reason["insufficient_line_support_or_residual"]; continue;
      }
      for (const auto & pixel : arc) {
        const double t = (cv::Point2d(pixel) - center).dot(u);
        low = std::min(low, t); high = std::max(high, t);
      }
      evidence.lines.push_back(
        {obs.origin + (center + u * low) * obs.spacing_m,
          obs.origin + (center + u * high) * obs.spacing_m, fraction,
          std::sqrt(residual / support.size())});
      auto & oriented_segment = evidence.lines.back();
      oriented_segment.source_line_id = (static_cast<uint64_t>(patch.source_boundary_id) << 32) |
        diagnostic.fitted_edge_count;
      diagnostic.fitted_edge_source_ids.push_back(oriented_segment.source_line_id);
      diagnostic.fitted_edge_residual_px.push_back(oriented_segment.residual_px);
      if (retain_visualization_evidence) {
        std::vector<uint32_t> source_cells;
        for (const auto & pixel : support) {source_cells.push_back(cvRound(pixel.y) * obs.heights.cols + cvRound(pixel.x));}
        diagnostic.fitted_edge_support_cell_ids.push_back(std::move(source_cells));
      }
      if ((oriented_segment.last - oriented_segment.first).dot(cv::Point2d(last - first)) < 0) {
        std::swap(oriented_segment.first, oriented_segment.last);
      }
      if (retain_visualization_evidence) {
        const auto & segment = evidence.lines.back();
        diagnostic.fitted_edges_xy_support.push_back({segment.first.x, segment.first.y,
          segment.last.x, segment.last.y, fraction});
      }
      ++diagnostic.fitted_edge_count;
    }
    if (!evidence.lines.empty()) {++diagnostic.contours_with_fitted_edges_count;} else {
      ++diagnostic.contour_rejected_by_reason["no_fitted_edge"];
    }
    evidence.observed_boundary = std::make_shared<const std::vector<cv::Point>>(evidence.points);
    result.push_back(std::move(evidence));
  }
  return result;
}

Fit rectangle_fit_impl(
  cv::Point2d center, double yaw, const Face & face, const PlaneRegion & patch,
  const CuboidHeightObservation & obs, uint32_t patch_index)
{
  Fit fit;
  fit.center = center; fit.yaw = yaw; fit.face = face; fit.top = patch.height;
  fit.patch_index = patch_index; fit.mask = footprint(fit, obs);
  const int area = cv::countNonZero(fit.mask);
  fit.mask_cell_count = static_cast<size_t>(area);
  fit.area = area ? static_cast<double>(cv::countNonZero(fit.mask & patch.cells)) / area : 0;
  return fit;
}

bool same_face(const Fit & a, const Fit & b)
{
  return a.face.length == b.face.length && a.face.width == b.face.width &&
         a.face.thickness == b.face.thickness;
}

bool equivalent(const Fit & a, const Fit & b, double spacing)
{
  if (!same_face(a, b)) {return false;}
  if (a.complete_iou && b.complete_iou && a.rectangle_id == b.rectangle_id) {return true;}
  if (cv::norm(a.center - b.center) > 2 * spacing) {return false;}
  const auto ac = corners(a), bc = corners(b);
  for (const auto & p : ac) {
    double distance = std::numeric_limits<double>::infinity();
    for (const auto & q : bc) {
      distance = std::min(distance, static_cast<double>(cv::norm(p - q)));
    }
    if (distance > 2 * spacing) {return false;}
  }
  return true;
}

CuboidHypothesisVisualization visualization(const Fit & fit, const std::string & reason)
{
  CuboidHypothesisVisualization item;
  item.candidate_id = fit.candidate_id; item.patch_index = fit.patch_index;
  item.height_m = fit.top; item.center_x_m = fit.center.x; item.center_y_m = fit.center.y;
  item.yaw_rad = fit.yaw; item.length_m = fit.face.length; item.width_m = fit.face.width;
  item.thickness_m = fit.face.thickness; item.observed_area_ratio = fit.area;
  item.boundary_support = fit.boundary;
  item.disposition =
    reason.empty() ? "retained_after_deduplication" : "rejected_geometry_validation";
  item.reason = reason;
  item.selection_state = reason.empty() ? "" : "REJECTED";
  item.selection_reason = reason;
  item.source_boundary_id = fit.source_boundary_id;
  item.outline_id = fit.outline_id;
  item.observed_support_cells = fit.observed_cell_ids.size();
  item.observed_line_support_m = fit.observed_line_support_m;
  item.height_residual = fit.height_residual;
  item.dimension_residual = fit.dimension_residual;
  item.dimension_observation_count = fit.dimension_observation_count;
  item.complete_iou = fit.complete_iou;
  item.rectangle_id = fit.rectangle_id;
  return item;
}

}  // namespace

static std::vector<PlaneRegion> connected_observations(
  const CuboidHeightObservation & obs, const DetectorProfile & profile,
  const cv::Mat * allowed_cells)
{
  cv::Mat available = cv::Mat::zeros(obs.heights.size(), CV_8U);
  for (int r = 0; r < obs.heights.rows; ++r) {
    for (int c = 0; c < obs.heights.cols; ++c) {
      const double z = obs.heights.at<double>(r, c);
      const auto xy = obs.origin + cv::Point2d(c, r) * obs.spacing_m;
      if ((!allowed_cells || allowed_cells->at<uint8_t>(r, c)) &&
        std::isfinite(z) && z > profile.support_contact_tolerance_m &&
        z >= profile.workspace_roi_min_m[2] && z <= profile.workspace_roi_max_m[2] &&
        xy.x >= profile.workspace_roi_min_m[0] && xy.x <= profile.workspace_roi_max_m[0] &&
        xy.y >= profile.workspace_roi_min_m[1] && xy.y <= profile.workspace_roi_max_m[1])
      {available.at<uint8_t>(r, c) = 255;}
    }
  }
  std::vector<PlaneRegion> patches;
  uint32_t pick_boundary_id = 0;
  for (int r = 0; r < available.rows; ++r) {
    for (int c = 0; c < available.cols; ++c) {
      if (!available.at<uint8_t>(r, c)) {continue;}
      PlaneRegion patch;
      std::vector<cv::Point> pending{{c, r}};
      available.at<uint8_t>(r, c) = 0;
      while (!pending.empty()) {
        const auto pixel = pending.back(); pending.pop_back(); patch.pixels.push_back(pixel);
        const double z = obs.heights.at<double>(pixel.y, pixel.x);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int x = pixel.x + dx, y = pixel.y + dy;
            if (x < 0 || x >= available.cols || y < 0 || y >= available.rows ||
              available.at<uint8_t>(y, x) != 255) {continue;}
            if (!within_height_tolerance(
                z, obs.heights.at<double>(y, x),
                profile.cuboid.height_tolerance_m)) {continue;}
            available.at<uint8_t>(y, x) = 0; pending.push_back({x, y});
          }
        }
      }
      patch.source_boundary_id = pick_boundary_id++;
      patches.push_back(std::move(patch));
    }
  }
  return patches;
}

// Accumulate scalar statistics on an already identified region. This is an
// observation lookup, not another extraction, raster partition or cell set.
static ObservationRegion observation_region(
  const ContourEvidence & contour, const PlaneRegion & plane,
  const CuboidHeightObservation & obs, const DetectorProfile & profile,
  const std::vector<cv::Point2f> & partition = {})
{
  ObservationRegion region;
  region.source_boundary_id = plane.source_boundary_id;
  region.outline_id = contour.outline_id;
  region.observed_boundary = contour.observed_boundary;
  region.partition_boundary = partition;
  const auto moments = cv::moments(contour.points);
  if (moments.m00 != 0) {
    region.observed_centroid = obs.origin +
      cv::Point2d(moments.m10 / moments.m00, moments.m01 / moments.m00) * obs.spacing_m;
  }
  cv::Point3d sum;
  size_t count = 0;
  for (const auto & pixel : plane.pixels) {
    if (cv::pointPolygonTest(contour.points, pixel, false) < 0) {continue;}
    const auto xy = obs.origin + cv::Point2d(pixel) * obs.spacing_m;
    if (!partition.empty() && cv::pointPolygonTest(partition, xy, false) < 0) {continue;}
    const double z = obs.heights.at<double>(pixel);
    if (!std::isfinite(z)) {continue;}
    sum += cv::Point3d(xy.x, xy.y, z); ++count;
  }
  if (count) {
    region.mean_observed_height = sum.z / count;
    // Equal-area observed grid cells provide the observed area centroid of a
    // partition, without treating its inferred rectangle interior as observed.
    if (!partition.empty()) {region.observed_centroid = {sum.x / count, sum.y / count};}
  }
  using Candidate = arm_cell_interfaces::msg::DisturbanceCandidate;
  region.category = has_positive_inclined_surface_evidence(plane, obs,
      profile.cuboid.height_tolerance_m) ? Candidate::CATEGORY_INCLINED_SURFACE :
    plane.status == "horizontal" ? Candidate::CATEGORY_OTHER_UNRESOLVED :
    Candidate::CATEGORY_ELEVATED_UNRESOLVED;
  return region;
}

std::vector<PlaneRegion> extract_plane_regions(
  const CuboidHeightObservation & obs, const DetectorProfile & profile,
  const cv::Mat * allowed_cells)
{
  auto regions = connected_observations(obs, profile, allowed_cells);
  for (auto & region : regions) {measure_plane(region, obs, profile);}
  std::stable_sort(regions.begin(), regions.end(),
    [](const auto & a, const auto & b) {return a.height > b.height;});
  return regions;
}

PlaneMap build_plane_map(const CuboidHeightObservation & obs, const DetectorProfile & profile,
  size_t & operations, bool retain_evidence)
{
  PlaneMap map;
  auto components = connected_observations(obs, profile, nullptr);
  const int radius = std::max(2, static_cast<int>(std::ceil(profile.cuboid.line_residual_limit_px)));
  const double tolerance = geometry_tolerance(profile);
  for (auto & component : components) {
    component.cells = cv::Mat::zeros(obs.heights.size(), CV_8U);
    for (const auto & pixel : component.pixels) {component.cells.at<uint8_t>(pixel) = 255;}
    CuboidHeightPatchDiagnostic boundary;
    boundary.source_boundary_id = component.source_boundary_id;
    boundary.cell_count = component.pixels.size();
    // Extraction precedes plane interpretation, including failed/nonplanar regions.
    component.source_contours = std::make_shared<const std::vector<ContourEvidence>>(
      extract_contours_impl(component, obs, profile, boundary, retain_evidence));
    measure_plane(component, obs, profile);
    boundary.height_m = component.height;
    for (const auto & contour : *component.source_contours) {
      map.outlines.push_back(observation_region(contour, component, obs, profile));
    }
    if (retain_evidence) {
      for (const auto & pixel : component.pixels) {
        boundary.observed_cell_ids.push_back(pixel.y * obs.heights.cols + pixel.x);
      }
    }
    map.boundaries.push_back(std::move(boundary));
    const auto local_plane_at = [&](const cv::Point & pixel) {
        PlaneRegion local;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const cv::Point q = pixel + cv::Point(dx, dy);
            if (q.x >= 0 && q.y >= 0 && q.x < obs.heights.cols && q.y < obs.heights.rows &&
              component.cells.at<uint8_t>(q)) {local.pixels.push_back(q);}
          }
        }
        measure_plane(local, obs, profile);
        return local;
      };
    // A small global RMS (or a horizontal classification) cannot rule out a
    // local plane change. Reuse the existing fit/normal comparison on overlapping
    // window probes before allocating the dense normal map and growing regions.
    bool distinct_local_planes = false;
    bool have_local_normal = false;
    std::array<cv::Vec2d, 4> extreme_normals;
    const auto bounds = cv::boundingRect(component.pixels);
    for (int y = bounds.y + radius; component.plane_fitted &&
      y < bounds.y + bounds.height - radius && !distinct_local_planes; y += 2 * radius)
    {
      for (int x = bounds.x + radius; x < bounds.x + bounds.width - radius; x += 2 * radius) {
        if (!component.cells.at<uint8_t>(y, x)) {continue;}
        const auto local = local_plane_at({x, y});
        // Clipped or missing support must not magnify noise over a nominal full
        // window baseline. Such evidence stays with the original component.
        if (local.pixels.size() != static_cast<size_t>((2 * radius + 1) * (2 * radius + 1)) ||
          !local.plane_fitted || local.residual_m > tolerance) {continue;}
        const cv::Vec2d gradient(-local.normal[0] / local.normal[2], -local.normal[1] / local.normal[2]);
        if (!have_local_normal) {extreme_normals.fill(gradient); have_local_normal = true;}
        for (const auto & previous : extreme_normals) {
          if (cv::norm(gradient - previous) * 2 * radius * obs.spacing_m > tolerance) {
            distinct_local_planes = true; break;
          }
        }
        if (distinct_local_planes) {break;}
        for (size_t axis = 0; axis < 2; ++axis) {
          if (gradient[axis] < extreme_normals[2 * axis][axis]) {extreme_normals[2 * axis] = gradient;}
          if (gradient[axis] > extreme_normals[2 * axis + 1][axis]) {extreme_normals[2 * axis + 1] = gradient;}
        }
      }
    }
    if (!distinct_local_planes) {
      if (component.residual_m > tolerance) {component.status = "unresolved_plane_partition";}
      component.cells.release();
      component.source_contours.reset();
      map.regions.push_back(std::move(component));
      continue;
    }
    // Local plane normals distinguish a ramp continuously touching a horizontal
    // top. No Recipe or object count is used. Labels own measured cells once;
    // residual cells remain an unresolved region with their original boundary.
    cv::Mat normals(obs.heights.size(), CV_64FC2, cv::Scalar(0, 0));
    cv::Mat fitted = cv::Mat::zeros(obs.heights.size(), CV_8U);
    for (const auto & pixel : component.pixels) {
      const auto local = local_plane_at(pixel);
      if (!local.plane_fitted || local.residual_m > tolerance) {continue;}
      normals.at<cv::Vec2d>(pixel) = {-local.normal[0] / local.normal[2],
        -local.normal[1] / local.normal[2]};
      fitted.at<uint8_t>(pixel) = 255;
    }
    cv::Mat remaining = component.cells.clone();
    for (const auto & seed : component.pixels) {
      if (!remaining.at<uint8_t>(seed) || !fitted.at<uint8_t>(seed)) {continue;}
      if (operations == 0) {break;}
      --operations;
      const auto gradient = normals.at<cv::Vec2d>(seed);
      const double seed_height = obs.heights.at<double>(seed);
      PlaneRegion region;
      region.source_boundary_id = component.source_boundary_id;
      region.source_contours = component.source_contours;
      std::vector<cv::Point> pending{seed};
      remaining.at<uint8_t>(seed) = 0;
      while (!pending.empty()) {
        const auto pixel = pending.back(); pending.pop_back(); region.pixels.push_back(pixel);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const auto q = pixel + cv::Point(dx, dy);
            if (q.x < 0 || q.y < 0 || q.x >= remaining.cols || q.y >= remaining.rows ||
              !remaining.at<uint8_t>(q) || !fitted.at<uint8_t>(q)) {continue;}
            const auto delta_gradient = normals.at<cv::Vec2d>(q) - gradient;
            const double normal_difference = cv::norm(delta_gradient) * 2 * radius * obs.spacing_m;
            const auto offset = cv::Point2d(q - seed) * obs.spacing_m;
            const double predicted = seed_height + offset.x * gradient[0] + offset.y * gradient[1];
            if (normal_difference > tolerance ||
              std::abs(obs.heights.at<double>(q) - predicted) > tolerance) {continue;}
            remaining.at<uint8_t>(q) = 0; pending.push_back(q);
          }
        }
      }
      measure_plane(region, obs, profile);
      map.regions.push_back(std::move(region));
    }
    if (cv::countNonZero(remaining)) {
      PlaneRegion unresolved;
      unresolved.source_boundary_id = component.source_boundary_id;
      unresolved.source_contours = component.source_contours;
      cv::findNonZero(remaining, unresolved.pixels);
      measure_plane(unresolved, obs, profile);
      unresolved.status = "unresolved_plane_partition";
      map.regions.push_back(std::move(unresolved));
    }
  }
  std::stable_sort(map.regions.begin(), map.regions.end(),
    [](const auto & a, const auto & b) {return a.height > b.height;});
  cv::Mat labels(obs.heights.size(), CV_32S, cv::Scalar(-1));
  for (size_t i = 0; i < map.regions.size(); ++i) {
    for (const auto & pixel : map.regions[i].pixels) {labels.at<int>(pixel) = i;}
  }
  std::set<std::pair<int, int>> contacts;
  for (int y = 0; y < labels.rows; ++y) {
    for (int x = 0; x < labels.cols; ++x) {
      const int a = labels.at<int>(y, x);
      if (a < 0) {continue;}
      for (const auto & d : std::array<cv::Point, 2>{{{1, 0}, {0, 1}}}) {
        if (x + d.x >= labels.cols || y + d.y >= labels.rows) {continue;}
        const int b = labels.at<int>(y + d.y, x + d.x);
        if (b < 0 || a == b || !contacts.emplace(std::min(a, b), std::max(a, b)).second) {continue;}
        const double step = obs.heights.at<double>(y + d.y, x + d.x) - obs.heights.at<double>(y, x);
        map.regions[a].adjacency.push_back({static_cast<double>(b), step,
          std::abs(step) > profile.cuboid.height_tolerance_m ? 1.0 : 0.0});
        map.regions[b].adjacency.push_back({static_cast<double>(a), -step,
          std::abs(step) > profile.cuboid.height_tolerance_m ? 1.0 : 0.0});
      }
    }
  }
  // Some coherent ramps have per-cell steps above the height continuity
  // tolerance, leaving existing plane regions one cell wide. Reconcile only
  // adjacent unfitted regions and retain them only if the existing plane fit
  // and every original observation support the same inclined plane.
  std::vector<bool> visited(map.regions.size(), false);
  for (size_t seed = 0; seed < map.regions.size(); ++seed) {
    if (visited[seed] || map.regions[seed].plane_fitted) {continue;}
    std::vector<size_t> pending{seed}, members;
    visited[seed] = true;
    while (!pending.empty()) {
      const auto index = pending.back(); pending.pop_back(); members.push_back(index);
      for (const auto & edge : map.regions[index].adjacency) {
        const auto next = static_cast<size_t>(edge[0]);
        if (next >= map.regions.size() || visited[next] || map.regions[next].plane_fitted) {continue;}
        visited[next] = true; pending.push_back(next);
      }
    }
    if (members.size() < 3) {continue;}
    PlaneRegion combined;
    for (const auto index : members) {
      combined.pixels.insert(combined.pixels.end(), map.regions[index].pixels.begin(),
        map.regions[index].pixels.end());
    }
    measure_plane(combined, obs, profile);
    if (!combined.plane_fitted || combined.status != "mixed_or_inclined" ||
      combined.residual_m > tolerance ||
      !has_positive_inclined_surface_evidence(combined, obs, profile.cuboid.height_tolerance_m)) {continue;}
    bool consistent = true;
    for (const auto & pixel : combined.pixels) {
      const auto xy = obs.origin + cv::Point2d(pixel) * obs.spacing_m;
      const double predicted = combined.center_height[2] -
        (combined.normal[0] * (xy.x - combined.center_height[0]) +
        combined.normal[1] * (xy.y - combined.center_height[1])) / combined.normal[2];
      if (std::abs(obs.heights.at<double>(pixel) - predicted) > tolerance) {
        consistent = false; break;
      }
    }
    if (!consistent) {continue;}
    combined.cells = cv::Mat::zeros(obs.heights.size(), CV_8U);
    for (const auto & pixel : combined.pixels) {combined.cells.at<uint8_t>(pixel) = 255;}
    CuboidHeightPatchDiagnostic ignored;
    std::vector<ContourEvidence> contours;
    extract_contours_impl(combined, obs, profile, ignored, false, &contours);
    combined.cells.release();
    map.recovered_inclined_regions.push_back(std::move(combined));
    map.recovered_inclined_contours.push_back(std::move(contours));
  }
  return map;
}

bool has_positive_inclined_surface_evidence(
  const PlaneRegion & patch, const CuboidHeightObservation & obs, double tolerance)
{
  return patch.status == "mixed_or_inclined" &&
         (has_spatial_height_trend(patch.pixels, obs.heights, true, tolerance) ||
         has_spatial_height_trend(patch.pixels, obs.heights, false, tolerance));
}

namespace
{
struct Corner
{
  cv::Point2d anchor;
  cv::Point2d first_axis;
  cv::Point2d second_axis;
  double first_edge_length;
  double second_edge_length;
};

bool supported_outer_corner(
  const Corner & corner, const PlaneRegion & patch,
  const CuboidHeightObservation & observation, double spacing)
{
  const auto inside = corner.first_axis + corner.second_axis;
  for (const double cells : {1.0, 2.0, 3.0}) {
    const auto point = corner.anchor + inside * (cells * spacing);
    const int x = cvRound((point.x - observation.origin.x) / spacing);
    const int y = cvRound((point.y - observation.origin.y) / spacing);
    if (x >= 0 && x < patch.cells.cols && y >= 0 && y < patch.cells.rows &&
      patch.cells.at<uint8_t>(y, x) != 0)
    {return true;}
  }
  return false;
}

std::vector<Corner> local_corners(
  const ContourEvidence & contour, const PlaneRegion & patch,
  const CuboidHeightObservation & observation, const DetectorProfile & profile,
  CuboidHeightPatchDiagnostic & diagnostic)
{
  std::vector<Corner> result;
  const double epsilon = profile.cuboid.line_residual_limit_px * observation.spacing_m;
  for (size_t i = 0; i < contour.lines.size(); ++i) {
    const auto & first = contour.lines[i];
    const auto d1 = first.last - first.first;
    const double length1 = cv::norm(d1);
    if (length1 <= 0) {continue;}
    const auto u = d1 * (1.0 / length1);
    for (size_t j = i + 1; j < contour.lines.size(); ++j) {
      const auto & second = contour.lines[j];
      if (!first.observed || !second.observed) {continue;}
      const auto d2 = second.last - second.first;
      const double length2 = cv::norm(d2);
      if (length2 <= 0) {continue;}
      const auto v = d2 * (1.0 / length2);
      const double determinant = u.x * v.y - u.y * v.x;
      if (std::abs(determinant) < 1e-8) {continue;}
      const auto delta = second.first - first.first;
      const double t = (delta.x * v.y - delta.y * v.x) / determinant;
      const auto anchor = first.first + u * t;
      const std::array<double, 2> first_projection{{
        (first.first - anchor).dot(u), (first.last - anchor).dot(u)}};
      const std::array<double, 2> second_projection{{
        (second.first - anchor).dot(v), (second.last - anchor).dot(v)}};
      const auto first_near = std::abs(first_projection[0]) <= std::abs(first_projection[1]) ?
        first.first : first.last;
      const auto second_near = std::abs(second_projection[0]) <= std::abs(second_projection[1]) ?
        second.first : second.last;
      const double gap = cv::norm(first_near - second_near);
      const auto endpoint_tolerance = [&](double length) {
          const double angle_tolerance = std::atan2(epsilon, length);
          // Include the fit residual, its angular uncertainty over the
          // observed edge, and one grid cell of endpoint raster quantization.
          return epsilon + length * std::tan(angle_tolerance) + observation.spacing_m;
        };
      const double first_endpoint_tolerance = endpoint_tolerance(length1);
      const double second_endpoint_tolerance = endpoint_tolerance(length2);
      if (gap > std::hypot(first_endpoint_tolerance, second_endpoint_tolerance) ||
        std::min(std::abs(first_projection[0]), std::abs(first_projection[1])) >
        first_endpoint_tolerance ||
        std::min(std::abs(second_projection[0]), std::abs(second_projection[1])) >
        second_endpoint_tolerance)
      {continue;}
      if (std::asin(std::min(1.0, std::abs(u.dot(v)))) >
        std::atan2(epsilon, length1) + std::atan2(epsilon, length2))
      {
        ++diagnostic.pre_object_rejected_count;
        ++diagnostic.hypothesis_rejected_by_reason["nonorthogonal_corner_edges"];
        continue;
      }
      const auto inward1 = std::abs(first_projection[0]) > std::abs(first_projection[1]) ?
        u * std::copysign(1.0, first_projection[0]) : u * std::copysign(1.0, first_projection[1]);
      const auto inward2 = std::abs(second_projection[0]) > std::abs(second_projection[1]) ?
        v * std::copysign(1.0, second_projection[0]) : v * std::copysign(1.0, second_projection[1]);
      const double edge1 = std::max(
        std::abs(first_projection[0]), std::abs(first_projection[1]));
      const double edge2 = std::max(
        std::abs(second_projection[0]), std::abs(second_projection[1]));
      Corner corner{anchor, inward1, inward2, edge1, edge2};
      if (!supported_outer_corner(corner, patch, observation, observation.spacing_m)) {continue;}
      const auto intervening_length = [&](size_t from, size_t to) {
          double length = 0;
          for (size_t index = (from + 1) % contour.lines.size(); index != to;
            index = (index + 1) % contour.lines.size()) {
            length = std::max(length, cv::norm(contour.lines[index].last - contour.lines[index].first));
          }
          return length;
        };
      const double endpoint_uncertainty = std::hypot(first_endpoint_tolerance,
        second_endpoint_tolerance);
      const bool follows = intervening_length(i, j) <= endpoint_uncertainty;
      const bool precedes = intervening_length(j, i) <= endpoint_uncertainty;
      if (!follows && !precedes) {continue;}
      const double turn = (follows ? 1.0 : -1.0) * (u.x * v.y - u.y * v.x);
      if (turn * cv::contourArea(contour.points, true) <= 0) {continue;}
      result.push_back(corner);
    }
  }
  return result;
}
struct RectangleRegion
{
  cv::Point2d center;
  double yaw;
  double length;
  double width;
  bool closed_observed;
  std::array<bool, 4> observed_sides{};
};

// Orthogonal coordinates come exclusively from fitted lines and measured
// vertices. Concavities can extend a partition, but cannot create an anchor.
std::vector<RectangleRegion> partition_rectangles(
  const ContourEvidence & contour, const std::vector<Corner> & anchors,
  const PlaneRegion & plane, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, size_t & operations)
{
  std::vector<RectangleRegion> rectangles;
  const double epsilon = std::max(obs.spacing_m / std::sqrt(2.0),
    profile.cuboid.line_residual_limit_px * obs.spacing_m);
  for (const auto & anchor : anchors) {
    const double yaw = std::atan2(anchor.first_axis.y, anchor.first_axis.x);
    const auto u = axis(yaw);
    const auto v = normal(yaw) * (normal(yaw).dot(anchor.second_axis) >= 0 ? 1 : -1);
    std::vector<double> lengths, widths;
    // Opposing coordinate lines must be measured and parallel to one of the
    // anchor axes. Arbitrary vertex projections (e.g. a bevel endpoint) do not
    // support a new perpendicular partition side.
    for (const auto & line : contour.lines) {
      if (!line.observed) {continue;}
      const auto first = line.first - anchor.anchor;
      const auto last = line.last - anchor.anchor;
      if (std::abs(first.dot(u) - last.dot(u)) <= epsilon &&
        std::abs(first.dot(v) - last.dot(v)) > 2 * epsilon) {
        const double offset = (first.dot(u) + last.dot(u)) / 2;
        if (offset > 2 * epsilon) {lengths.push_back(offset);}
      }
      if (std::abs(first.dot(v) - last.dot(v)) <= epsilon &&
        std::abs(first.dot(u) - last.dot(u)) > 2 * epsilon) {
        const double offset = (first.dot(v) + last.dot(v)) / 2;
        if (offset > 2 * epsilon) {widths.push_back(offset);}
      }
    }
    const auto unique = [&](std::vector<double> & values) {
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end(),
          [&](double a, double b) {return std::abs(a - b) <= epsilon;}), values.end());
      };
    unique(lengths); unique(widths);
    for (double length : lengths) {
      for (double width : widths) {
        if (operations == 0) {return rectangles;}
        --operations;
        RectangleRegion rectangle{anchor.anchor + u * length / 2 + v * width / 2,
          yaw, length, width, false};
        Fit geometric;
        geometric.center = rectangle.center; geometric.yaw = yaw;
        geometric.face = {length, width, 0, 0, 0};
        const auto mask = footprint(geometric, obs);
        cv::Mat interior;
        const int band = static_cast<int>(std::ceil(epsilon / obs.spacing_m));
        cv::erode(mask, interior, cv::Mat::ones(2 * band + 1, 2 * band + 1, CV_8U));
        if (cv::countNonZero(interior) == 0 || cv::countNonZero(interior & ~plane.cells)) {continue;}
        // All four corners need measured support near their intersection. A wide
        // missing L/U interior never becomes an observed rectangle here.
        bool supported = true;
        for (const auto & point : corners(geometric)) {
          const int x = cvRound((point.x - obs.origin.x) / obs.spacing_m);
          const int y = cvRound((point.y - obs.origin.y) / obs.spacing_m);
          if (x < 0 || y < 0 || x >= plane.cells.cols || y >= plane.cells.rows ||
            !plane.cells.at<uint8_t>(y, x)) {supported = false; break;}
        }
        if (!supported) {continue;}
        const auto points = corners(geometric);
        uint32_t observed_sides = 0;
        for (size_t side = 0; side < 4; ++side) {
          const cv::Point2d first(points[side]), last(points[(side + 1) % 4]);
          const auto direction = (last - first) / cv::norm(last - first);
          double support = 0;
          for (const auto & line : contour.lines) {
            if (!line.observed) {continue;}
            const auto a = line.first - first, b = line.last - first;
            const cv::Point2d n(-direction.y, direction.x);
            if (std::abs(a.dot(n)) > epsilon || std::abs(b.dot(n)) > epsilon) {continue;}
            support += std::max(0.0, std::min(cv::norm(last - first),
              std::max(a.dot(direction), b.dot(direction))) -
              std::max(0.0, std::min(a.dot(direction), b.dot(direction))));
          }
          if (support >= cv::norm(last - first) - 2 * epsilon) {
            ++observed_sides; rectangle.observed_sides[side] = true;
          }
        }
        rectangle.closed_observed = observed_sides == 4;
        const bool duplicate = std::any_of(rectangles.begin(), rectangles.end(),
          [&](const auto & previous) {
            return cv::norm(previous.center - rectangle.center) <= epsilon &&
              std::abs(previous.length * previous.width - length * width) <= epsilon * epsilon;
          });
        if (!duplicate) {rectangles.push_back(rectangle);}
      }
    }
  }
  // Retain maximal geometric regions; overlapping maxima remain alternatives.
  const auto all = rectangles;
  rectangles.erase(std::remove_if(rectangles.begin(), rectangles.end(), [&](const auto & small) {
      return std::any_of(all.begin(), all.end(), [&](const auto & large) {
        if (large.length * large.width <= small.length * small.width + epsilon * epsilon) {return false;}
        Fit a, b;
        a.center = small.center; a.yaw = small.yaw; a.face = {small.length, small.width, 0, 0, 0};
        b.center = large.center; b.yaw = large.yaw; b.face = {large.length, large.width, 0, 0, 0};
        std::vector<cv::Point2f> intersection;
        return cv::intersectConvexConvex(corners(a), corners(b), intersection) >=
          small.length * small.width - epsilon * epsilon;
      });
    }), rectangles.end());
  return rectangles;
}

// A long observed exterior side can span touching objects. Only Recipe-length
// continuations from an actual convex anchor are tested; no internal seam or
// additional object pose is generated by this one-dimensional check.
bool recipe_continuation(double remaining, const std::vector<Face> & catalog,
  double top, const DetectorProfile & profile, size_t & budget, bool & exhausted,
  double confirmed_cross_extent = 0, size_t depth = 0)
{
  if (depth >= 128) {exhausted = true; return false;}
  for (const auto & face : catalog) {
    if (top - face.thickness < -profile.support_contact_tolerance_m) {continue;}
    for (int orientation = 0; orientation < 2; ++orientation) {
      const auto side = orientation == 0 ? std::make_pair(face.length, face.length_tolerance) :
        std::make_pair(face.width, face.width_tolerance);
      const double cross = orientation == 0 ? face.width : face.length;
      const double cross_tolerance = orientation == 0 ? face.width_tolerance : face.length_tolerance;
      if (confirmed_cross_extent > 0 &&
        std::abs(cross - confirmed_cross_extent) > cross_tolerance) {continue;}
      if (std::abs(remaining) <= side.second) {return true;}
      if (remaining < side.first - side.second) {continue;}
      if (budget == 0) {exhausted = true; return false;}
      --budget;
      if (std::abs(remaining - side.first) <= side.second ||
        recipe_continuation(remaining - side.first, catalog, top, profile, budget, exhausted, confirmed_cross_extent, depth + 1)) {
        return true;
      }
      if (exhausted) {return false;}
    }
  }
  return false;
}

// A nearby lower top is not necessarily below this object. Require one
// continuous observed region along opposing original boundary segments; do not
// combine disconnected neighbours or use an inferred/clipped edge as evidence.
static std::optional<double> observed_lower_support_height(
  cv::Mat ring, const CuboidHeightObservation & obs, const DetectorProfile & profile,
  double top, const std::array<std::vector<const Segment *>, 4> & observed_sides)
{
  std::vector<cv::Point> seeds;
  double lowest = std::numeric_limits<double>::infinity();
  double highest = -std::numeric_limits<double>::infinity();
  for (int y = 0; y < ring.rows; ++y) {
    for (int x = 0; x < ring.cols; ++x) {
      if (!ring.at<uint8_t>(y, x)) {continue;}
      const double z = obs.heights.at<double>(y, x);
      if (!std::isfinite(z) || z >= top - profile.cuboid.height_tolerance_m) {
        ring.at<uint8_t>(y, x) = 0; continue;
      }
      seeds.emplace_back(x, y);
      lowest = std::min(lowest, z); highest = std::max(highest, z);
    }
  }
  if (seeds.empty() || highest - lowest > profile.support_contact_tolerance_m) {
    return std::nullopt;
  }
  const double band = std::ceil(profile.cuboid.line_residual_limit_px) * obs.spacing_m;
  std::optional<double> support_height;
  for (const auto & seed : seeds) {
    if (!ring.at<uint8_t>(seed)) {continue;}
    ring.at<uint8_t>(seed) = 0;
    std::vector<cv::Point> pending{seed};
    std::array<bool, 4> sides{};
    double sum = 0;
    double region_lowest = std::numeric_limits<double>::infinity();
    double region_highest = -std::numeric_limits<double>::infinity();
    size_t count = 0;
    while (!pending.empty()) {
      const auto pixel = pending.back(); pending.pop_back();
      const double z = obs.heights.at<double>(pixel);
      sum += z; ++count;
      region_lowest = std::min(region_lowest, z); region_highest = std::max(region_highest, z);
      const auto xy = obs.origin + cv::Point2d(pixel) * obs.spacing_m;
      for (size_t side = 0; side < sides.size(); ++side) {
        for (const auto * line : observed_sides[side]) {
          const auto delta = line->last - line->first;
          const double length = cv::norm(delta);
          const auto direction = delta / length;
          const auto offset = xy - line->first;
          const double along = offset.dot(direction);
          if (along >= 0 && along <= length &&
            std::abs(offset.dot(cv::Point2d(-direction.y, direction.x))) <= band)
          {sides[side] = true;}
        }
      }
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const cv::Point next = pixel + cv::Point(dx, dy);
          if (next.x < 0 || next.y < 0 || next.x >= ring.cols || next.y >= ring.rows ||
            !ring.at<uint8_t>(next) || !within_height_tolerance(
              z, obs.heights.at<double>(next), profile.cuboid.height_tolerance_m)) {continue;}
          ring.at<uint8_t>(next) = 0; pending.push_back(next);
        }
      }
    }
    if (region_highest - region_lowest > profile.cuboid.height_tolerance_m ||
      !((sides[0] && sides[1]) || (sides[2] && sides[3]))) {continue;}
    if (support_height) {return std::nullopt;}
    support_height = sum / count;
  }
  return support_height;
}

// Every hypothesis starts at an observed convex corner. Additional connected
// contour edges validate that same hypothesis; they never create a second path.
std::string evaluate_outline(
  Fit & fit, const Corner & corner, const std::vector<Corner> & anchors,
  const ContourEvidence & contour, const PlaneRegion & patch, const CuboidHeightObservation & obs,
  const DetectorProfile & profile)
{
  const double epsilon = profile.cuboid.line_residual_limit_px * obs.spacing_m;
  const double tolerance = profile.cuboid.height_tolerance_m;
  fit.anchor = corner.anchor;
  fit.observed_center = fit.center;
  fit.observed_top = patch.height;
  fit.measured_length = fit.face.length;
  fit.measured_width = fit.face.width;
  fit.tray_support_evidence = std::abs(fit.top - fit.face.thickness) <=
    profile.support_contact_tolerance_m ? "recipe_bottom_consistent_with_tray;direct_support_not_confirmed" :
    "direct_tray_support_not_confirmed";
  if (fit.top - fit.face.thickness < -profile.support_contact_tolerance_m) {
    return "modeled_bottom_below_support";
  }
  const auto u = axis(fit.yaw), v = normal(fit.yaw);
  std::array<double, 4> support{};
  std::array<std::vector<const Segment *>, 4> observed_sides;
  for (const auto & line : contour.lines) {
    if (!line.observed) {continue;}
    const auto delta = line.last - line.first;
    const double length = cv::norm(delta);
    if (length <= 0) {continue;}
    const auto direction = delta * (1.0 / length);
    const double parallel_tolerance = std::atan2(epsilon, length);
    for (size_t side = 0; side < 4; ++side) {
      const auto along = side < 2 ? u : v;
      const auto across = side < 2 ? v : u;
      const double span = side < 2 ? fit.face.length : fit.face.width;
      const double offset = (side % 2 ? 1 : -1) *
        (side < 2 ? fit.face.width : fit.face.length) / 2;
      if (std::asin(std::min(1.0, std::abs(direction.dot(across)))) > parallel_tolerance) {
        continue;
      }
      const auto a = line.first - fit.center, b = line.last - fit.center;
      if (std::abs(a.dot(across) - offset) > epsilon ||
        std::abs(b.dot(across) - offset) > epsilon) {continue;}
      const double low = std::min(a.dot(along), b.dot(along));
      const double high = std::max(a.dot(along), b.dot(along));
      const double supported_length = std::max(0.0, std::min(span / 2, high) -
        std::max(-span / 2, low));
      support[side] += supported_length / span;
      if (supported_length > 0) {observed_sides[side].push_back(&line);}
      fit.observed_line_support_m[line.source_line_id] = std::max(
        fit.observed_line_support_m[line.source_line_id], supported_length);
    }
  }
  // Opposite parallel edges connected through the observed L arms form the
  // additional U/rectangle evidence. Use their measured separation, not an
  // area score or a preferred proposal source.
  for (int arm = 0; arm < 2; ++arm) {
    const auto along = arm == 0 ? corner.first_axis : corner.second_axis;
    const double length = arm == 0 ? corner.first_edge_length : corner.second_edge_length;
    const auto endpoint = corner.anchor + along * length;
    const bool confirmed_outer_turn = std::any_of(anchors.begin(), anchors.end(),
      [&](const Corner & other) {
        return cv::norm(other.anchor - endpoint) <= 2 * epsilon + obs.spacing_m;
      });
    const bool length_axis = std::abs(along.dot(u)) > std::abs(along.dot(v));
    const double expected = length_axis ? fit.face.length : fit.face.width;
    const double allowed = fit.complete_iou ? std::max(2 * epsilon,
      length_axis ? fit.face.length_tolerance : fit.face.width_tolerance) :
      length_axis ? fit.face.length_tolerance : fit.face.width_tolerance;
    for (const auto & extra : contour.lines) {
      const auto delta = extra.last - extra.first;
      const double span = cv::norm(delta);
      if (span <= 0 || std::asin(std::min(1.0, std::abs((delta / span).dot(along)))) >
        std::atan2(epsilon, span)) {continue;}
      if (std::min(cv::norm(extra.first - endpoint), cv::norm(extra.last - endpoint)) >
        2 * epsilon + obs.spacing_m) {continue;}
      const double distance = ((extra.first + extra.last) * 0.5 - corner.anchor).dot(along);
      if (distance <= epsilon) {continue;}
      fit.parallel_edge_distances.push_back(distance);
      // A longer connected side can belong to touching material. Its exact
      // Recipe continuation is checked in the same anchored generation loop.
      if (confirmed_outer_turn && distance > expected + allowed && length <= expected + allowed) {
        return "connected_parallel_edge_distance_mismatch";
      }
    }
  }
  fit.observed_boundary_sides = std::count_if(support.begin(), support.end(),
    [](double value) {return value > 0;});
  fit.boundary = std::accumulate(support.begin(), support.end(), 0.0) / 4;
  // Internal contour holes are not additional object boundaries. Reuse the
  // existing line-fit raster uncertainty without filling the observation.
  cv::Mat observed_tolerant;
  const int residual_cells = static_cast<int>(std::ceil(profile.cuboid.line_residual_limit_px));
  const auto uncertainty_kernel = cv::Mat::ones(2 * residual_cells + 1,
    2 * residual_cells + 1, CV_8U);
  cv::dilate(patch.cells, observed_tolerant, uncertainty_kernel);
  // Only observed lower cells beside the observed outline can support a
  // thickness interpretation. Neither missing cells nor inferred edges are ground.
  cv::Mat support_ring;
  cv::dilate(fit.mask, support_ring, uncertainty_kernel);
  support_ring &= observed_tolerant & ~fit.mask;
  fit.support_height_m = observed_lower_support_height(
    support_ring, obs, profile, fit.top, observed_sides);
  if (fit.support_height_m) {
    fit.tray_support_evidence +=
      std::abs(fit.top - fit.face.thickness - *fit.support_height_m) <=
      profile.support_contact_tolerance_m ? ";observed_lower_plane_matches_bottom" :
      ";observed_lower_plane_does_not_match_bottom";
  }
  cv::Mat interior;
  cv::erode(fit.mask, interior, uncertainty_kernel);
  bool partial = cv::countNonZero(interior & ~observed_tolerant) > 0;
  for (const auto & point : corners(fit)) {
    if (point.x < std::max(obs.origin.x, profile.workspace_roi_min_m[0]) - epsilon ||
      point.y < std::max(obs.origin.y, profile.workspace_roi_min_m[1]) - epsilon ||
      point.x > std::min(obs.origin.x + (obs.heights.cols - 1) * obs.spacing_m,
        profile.workspace_roi_max_m[0]) + epsilon ||
      point.y > std::min(obs.origin.y + (obs.heights.rows - 1) * obs.spacing_m,
        profile.workspace_roi_max_m[1]) + epsilon) {partial = true;}
  }
  if (!obs.image_boundary.empty() && cv::countNonZero(interior & obs.image_boundary)) {
    partial = true;
  }
  for (int y = 0; y < obs.heights.rows; ++y) {
    for (int x = 0; x < obs.heights.cols; ++x) {
      if (!interior.at<uint8_t>(y, x)) {continue;}
      const double z = obs.heights.at<double>(y, x);
      if (!std::isfinite(z)) {continue;}
      if (z < fit.top - tolerance) {
        return "measured_interior_height_contradiction";
      }
      // Upper observations occlude the top; lower observations contradict it.
      if (z > fit.top + tolerance) {fit.covered_by_upper_surface = true; partial = true;}
    }
  }
  for (const auto & pixel : patch.pixels) {
    if (!fit.mask.at<uint8_t>(pixel)) {continue;}
    const double z = obs.heights.at<double>(pixel);
    if (!within_height_tolerance(z, fit.top, tolerance)) {continue;}
    fit.observed_cell_ids.push_back(pixel.y * obs.heights.cols + pixel.x);
    fit.height_residual += std::abs(z - fit.top) / tolerance;
  }
  std::sort(fit.observed_cell_ids.begin(), fit.observed_cell_ids.end());
  if (!fit.observed_cell_ids.empty()) {fit.height_residual /= fit.observed_cell_ids.size();}
  fit.area = fit.mask_cell_count ? static_cast<double>(fit.observed_cell_ids.size()) / fit.mask_cell_count : 0;
  fit.is_partial_reconstruction = partial;
  if (partial) {
    fit.complete_iou.reset(); fit.dimension_observation_count = 0;
  }
  fit.rectangle_measured = !partial && fit.observed_boundary_sides == 4;
  // Reuse observed-cell geometric refinement for a fully visible top. Missing
  // internal seams retain estimated rectangle provenance even when HIGH.
  if (!partial) {
    double low_u = std::numeric_limits<double>::infinity(), high_u = -low_u;
    double low_v = low_u, high_v = -low_u;
    std::vector<double> heights;
    for (const auto & pixel : patch.pixels) {
      if (!fit.mask.at<uint8_t>(pixel.y, pixel.x)) {continue;}
      const auto relative = obs.origin + cv::Point2d(pixel) * obs.spacing_m - fit.center;
      low_u = std::min(low_u, relative.dot(u)); high_u = std::max(high_u, relative.dot(u));
      low_v = std::min(low_v, relative.dot(v)); high_v = std::max(high_v, relative.dot(v));
      heights.push_back(obs.heights.at<double>(pixel.y, pixel.x));
    }
    if (!heights.empty() &&
      std::abs(high_u - low_u - fit.face.length) <= fit.face.length_tolerance &&
      std::abs(high_v - low_v - fit.face.width) <= fit.face.width_tolerance) {
      fit.measured_length = high_u - low_u;
      fit.measured_width = high_v - low_v;
      fit.observed_center = fit.center + u * ((low_u + high_u) / 2) + v * ((low_v + high_v) / 2);
      fit.observed_top = std::accumulate(heights.begin(), heights.end(), 0.0) / heights.size();
    }
  }
  cv::dilate(fit.mask, fit.tolerant, cv::Mat::ones(3, 3, CV_8U));
  return {};
}

PlaneFits reconstruct_plane_impl(
  const PlaneRegion & patch, uint32_t patch_index, uint32_t & next_id,
  size_t & search_operations, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, bool retain_visualization_evidence)
{
  PlaneFits result;
  auto & diagnostic = result.diagnostic;
  diagnostic.height_m = patch.height;
  diagnostic.cell_count = patch.pixels.size();
  diagnostic.partition_status = "orthogonal_outline_reconstruction";
  if (retain_visualization_evidence) {
    for (const auto & pixel : patch.pixels) {
      geometry_msgs::msg::Point32 point;
      point.x = obs.origin.x + pixel.x * obs.spacing_m;
      point.y = obs.origin.y + pixel.y * obs.spacing_m;
      diagnostic.observed_cells_xy.push_back(point);
      diagnostic.observed_cell_ids.push_back(pixel.y * obs.heights.cols + pixel.x);
    }
  }
  diagnostic.source_boundary_id = patch.source_boundary_id;
  diagnostic.plane_normal = {patch.normal[0], patch.normal[1], patch.normal[2]};
  diagnostic.plane_center_height_m = patch.center_height;
  diagnostic.observed_height_p05_p95_m = patch.observed_height_p05_p95;
  diagnostic.plane_residual_m = patch.residual_m;
  diagnostic.plane_height_span_m = patch.height_span_m;
  diagnostic.plane_status = patch.status;
  diagnostic.adjacent_planes = patch.adjacency;
  diagnostic.plane_fitted = patch.plane_fitted;
  auto contours = extract_contours_impl(
    patch, obs, profile, diagnostic, retain_visualization_evidence, &result.occupancy_contours);
  const size_t local_contour_count = contours.size();
  if (patch.source_contours) {
    for (auto & contour : contours) {
      const auto source = std::find_if(patch.source_contours->begin(), patch.source_contours->end(),
        [&](const auto & original) {
          return cv::pointPolygonTest(original.points, contour.points.front(), false) >= 0;
        });
      contour.outline_id = source == patch.source_contours->end() ? 0 : source->outline_id;
    }
    const double epsilon = profile.cuboid.line_residual_limit_px * obs.spacing_m;
    size_t edge_index = 0;
    for (auto & contour : contours) {
      for (auto & line : contour.lines) {
        line.observed = false;
        for (const auto & source : *patch.source_contours) {
          for (const auto & original : source.lines) {
            const auto u = (original.last - original.first) / cv::norm(original.last - original.first);
            const cv::Point2d n(-u.y, u.x);
            const double original_length = cv::norm(original.last - original.first);
            const double first_along = (line.first - original.first).dot(u);
            const double last_along = (line.last - original.first).dot(u);
            if (std::abs((line.first - original.first).dot(n)) <= epsilon &&
              std::abs((line.last - original.first).dot(n)) <= epsilon &&
              std::min(first_along, last_along) >= -epsilon &&
              std::max(first_along, last_along) <= original_length + epsilon) {
              line.observed = true; line.source_line_id = original.source_line_id;
            }
          }
        }
        diagnostic.fitted_edge_source_ids[edge_index++] = line.observed ? line.source_line_id :
          std::numeric_limits<uint64_t>::max();
        if (!line.observed) {diagnostic.inferred_partition_edges.push_back({
          line.first.x, line.first.y, line.last.x, line.last.y});}
      }
    }
    // Reconstruction can also use the original observed anchors. A plane split
    // supplies local measured support, never an exclusive new object gate.
    contours.insert(contours.end(), patch.source_contours->begin(), patch.source_contours->end());
  }
  const bool inclined_plane = patch.status == "mixed_or_inclined" && patch.plane_fitted &&
    patch.residual_m <= geometry_tolerance(profile);
  if (patch.pixels.size() < 3 || inclined_plane) {
    if (inclined_plane) {
      for (const auto & contour : contours) {
        result.unresolved_outline_ids.push_back(contour.outline_id);
      }
    }
    diagnostic.partition_reason = patch.status;
    diagnostic.hypothesis_rejected_by_reason[patch.status]++;
    if (has_positive_inclined_surface_evidence(patch, obs, profile.cuboid.height_tolerance_m)) {
      diagnostic.hypothesis_rejected_by_reason["inclined_surface"]++;
    }
    return result;
  }
  for (const auto & contour : contours) {
    const auto contour_start = result.fits.size();
    const auto hypothesis_start = result.hypotheses.size();
    const auto anchors = local_corners(contour, patch, obs, profile, diagnostic);
    result.counts["convex_l_anchors"] += anchors.size();
    const auto rectangles = partition_rectangles(contour, anchors, patch, obs, profile, search_operations);
    for (const auto & rectangle : rectangles) {
      diagnostic.rectangle_regions.push_back({rectangle.center.x, rectangle.center.y,
        rectangle.yaw, rectangle.length, rectangle.width});
      if (!rectangle.closed_observed) {
        Fit region; region.center = rectangle.center; region.yaw = rectangle.yaw;
        region.face = {rectangle.length, rectangle.width, 0, 0, 0};
        const auto points = corners(region);
        for (size_t side = 0; side < 4; ++side) {
          if (rectangle.observed_sides[side]) {continue;}
          diagnostic.inferred_partition_edges.push_back({points[side].x, points[side].y,
            points[(side + 1) % 4].x, points[(side + 1) % 4].y});
        }
      }
    }
    const size_t rectangle_base = diagnostic.rectangle_regions.size() - rectangles.size();
    for (size_t i = 0; i < rectangles.size(); ++i) {
      for (size_t j = i + 1; j < rectangles.size(); ++j) {
        Fit a, b;
        a.center = rectangles[i].center; a.yaw = rectangles[i].yaw;
        a.face = {rectangles[i].length, rectangles[i].width, 0, 0, 0};
        b.center = rectangles[j].center; b.yaw = rectangles[j].yaw;
        b.face = {rectangles[j].length, rectangles[j].width, 0, 0, 0};
        std::vector<cv::Point2f> intersection;
        if (cv::intersectConvexConvex(corners(a), corners(b), intersection) > obs.spacing_m * obs.spacing_m) {
          diagnostic.rectangle_alternatives.push_back({static_cast<uint32_t>(rectangle_base + i + 1),
            static_cast<uint32_t>(rectangle_base + j + 1)});
        }
      }
    }
    // Retain references to local partition results. Original-boundary
    // reconstruction proposals remain linked to their outline, not extra regions.
    if (static_cast<size_t>(&contour - contours.data()) < local_contour_count) {
      for (size_t ri = 0; ri < rectangles.size(); ++ri) {
        Fit rectangle;
        rectangle.center = rectangles[ri].center; rectangle.yaw = rectangles[ri].yaw;
        rectangle.face = {rectangles[ri].length, rectangles[ri].width, 0, 0, 0};
        auto region = observation_region(contour, patch, obs, profile, corners(rectangle));
        region.patch_index = patch_index;
        region.rectangle_id = rectangle_base + ri + 1;
        region.independent = std::none_of(diagnostic.rectangle_alternatives.begin(),
          diagnostic.rectangle_alternatives.end(), [&](const auto & pair) {
            return pair[0] == region.rectangle_id || pair[1] == region.rectangle_id;
          });
        region.unresolved = true;
        result.partitions.push_back(std::move(region));
      }
    }
    if (anchors.empty()) {
      ++diagnostic.hypothesis_rejected_by_reason["insufficient_convex_l_evidence"];
      diagnostic.confidence_grade = "LOW";
      diagnostic.confidence_reason = "observed_outline_has_no_confirmed_convex_l_anchor";
    }
    const auto catalog = faces_impl(profile);
    bool exhausted = false;
    for (const auto & anchor : anchors) {
      for (const auto & face : catalog) {
        for (int mapping = 0; mapping < 2; ++mapping) {
          if (search_operations == 0) {exhausted = true; break;}
          --search_operations;
          const auto along = mapping == 0 ? anchor.first_axis : anchor.second_axis;
          const auto across = mapping == 0 ? anchor.second_axis : anchor.first_axis;
          const double yaw = std::atan2(along.y, along.x);
          const auto orthogonal = normal(yaw) * (normal(yaw).dot(across) >= 0 ? 1.0 : -1.0);
          auto fit = rectangle_fit_impl(anchor.anchor + along * (face.length / 2) +
            orthogonal * (face.width / 2), yaw, face, patch, obs, patch_index);
          PlaneRegion local_plane;
          const PlaneRegion * support_plane = &patch;
          if (patch.status != "horizontal") {
            // An incomplete Plane Map is evidence, not an object gate. Test the
            // same observed Anchor against its own footprint using all measured
            // source cells there, without filling or selecting height inliers.
            for (const auto & pixel : patch.pixels) {
              if (fit.mask.at<uint8_t>(pixel)) {local_plane.pixels.push_back(pixel);}
            }
            if (!local_plane.pixels.empty()) {
              measure_plane(local_plane, obs, profile);
              local_plane.cells = patch.cells;
              support_plane = &local_plane;
              fit.top = local_plane.height;
            }
          }
          fit.source_boundary_id = patch.source_boundary_id;
          fit.outline_id = contour.outline_id;
          fit.candidate_id = next_id++;
          ++diagnostic.candidate_evaluation_attempt_count;
          for (size_t ri = 0; ri < rectangles.size(); ++ri) {
            const auto & rectangle = rectangles[ri];
            const double epsilon = std::max(obs.spacing_m / std::sqrt(2.0),
              profile.cuboid.line_residual_limit_px * obs.spacing_m);
            Fit region; region.center = rectangle.center; region.yaw = rectangle.yaw;
            region.face = {rectangle.length, rectangle.width, 0, 0, 0};
            std::vector<cv::Point2f> intersection;
            const double area = cv::intersectConvexConvex(corners(fit), corners(region), intersection);
            if (area <= 0 || cv::norm(fit.center - rectangle.center) > epsilon ||
              std::abs(fit.face.length * fit.face.width - rectangle.length * rectangle.width) >
              epsilon * 2 * (rectangle.length + rectangle.width)) {continue;}
            fit.rectangle_id = diagnostic.rectangle_regions.size() - rectangles.size() + ri + 1;
            if (rectangle.closed_observed) {
              const auto observed_mask = footprint(region, obs);
              cv::Mat inner, outer;
              const int band = static_cast<int>(std::ceil(epsilon / obs.spacing_m));
              const auto kernel = cv::Mat::ones(2 * band + 1, 2 * band + 1, CV_8U);
              cv::erode(observed_mask, inner, kernel); cv::dilate(observed_mask, outer, kernel);
              if (cv::countNonZero((observed_mask ^ fit.mask) & ~(outer ^ inner))) {continue;}
              fit.complete_iou = std::clamp(area / (fit.face.length * fit.face.width +
                rectangle.length * rectangle.width - area), 0.0, 1.0);
              const bool same_axes = std::abs(axis(fit.yaw).dot(axis(rectangle.yaw))) >
                std::abs(axis(fit.yaw).dot(normal(rectangle.yaw)));
              const double measured_length = same_axes ? rectangle.length : rectangle.width;
              const double measured_width = same_axes ? rectangle.width : rectangle.length;
              fit.dimension_residual = (std::abs(measured_length - face.length) / face.length_tolerance +
                std::abs(measured_width - face.width) / face.width_tolerance) / 2;
              fit.dimension_observation_count = 2;
            }
            break;
          }
          std::string reason;
          if (support_plane->status != "horizontal") {
            reason = has_positive_inclined_surface_evidence(*support_plane, obs,
              profile.cuboid.height_tolerance_m) ? "inclined_surface" : "unresolved_candidate_plane";
          }
          for (int side_index = 0; side_index < 2; ++side_index) {
            const double observed_side = (mapping == side_index) ?
              anchor.first_edge_length : anchor.second_edge_length;
            const double expected = side_index == 0 ? face.length : face.width;
            const double allowed = fit.complete_iou ? std::max(
              side_index == 0 ? face.length_tolerance : face.width_tolerance,
              2 * std::max(obs.spacing_m / std::sqrt(2.0), profile.cuboid.line_residual_limit_px * obs.spacing_m)) :
              side_index == 0 ? face.length_tolerance : face.width_tolerance;
            if (observed_side <= expected + allowed) {continue;}
            const double other_observed = (mapping == side_index) ?
              anchor.second_edge_length : anchor.first_edge_length;
            const double other_expected = side_index == 0 ? face.width : face.length;
            const double other_tolerance = side_index == 0 ? face.width_tolerance : face.length_tolerance;
            const double confirmed_cross_extent = std::abs(other_observed - other_expected) <=
              other_tolerance ? other_observed : 0;
            if (!recipe_continuation(observed_side - expected,
                catalog, patch.height, profile, search_operations, exhausted,
                confirmed_cross_extent)) {
              reason = exhausted ? "orthogonal_outline_search_limit" :
                "observed_edge_exceeds_recipe_side_without_supported_continuation";
              break;
            }
          }
          if (reason.empty()) {reason = evaluate_outline(fit, anchor, anchors, contour, *support_plane, obs, profile);}
          auto hypothesis = visualization(fit, reason);
          hypothesis.anchor_x_m = anchor.anchor.x;
          hypothesis.anchor_y_m = anchor.anchor.y;
          hypothesis.observed_anchor_edge_lengths_m = {
            anchor.first_edge_length, anchor.second_edge_length};
          hypothesis.observed_boundary_sides = fit.observed_boundary_sides;
          hypothesis.tray_support_evidence = fit.tray_support_evidence;
          hypothesis.parallel_edge_distances_m = fit.parallel_edge_distances;
          if (!reason.empty()) {
            ++diagnostic.geometry_rejected_by_reason[reason];
            result.hypotheses.push_back(std::move(hypothesis));
            continue;
          }
          ++diagnostic.geometry_valid_before_deduplication_count;
          const auto duplicate = std::find_if(result.fits.begin() + contour_start,
            result.fits.end(), [&](const Fit & other) {return equivalent(fit, other, obs.spacing_m);});
          if (duplicate != result.fits.end()) {
            ++diagnostic.deduplicated_count;
            ++diagnostic.deduplicated_by_reason["equivalent_convex_l_pose"];
            hypothesis.disposition = "removed_by_deduplication";
            hypothesis.reason = "equivalent_convex_l_pose";
            hypothesis.deduplicated_to_candidate_id = duplicate->candidate_id;
          } else {
            result.fits.push_back(std::move(fit));
          }
          result.hypotheses.push_back(std::move(hypothesis));
        }
        if (exhausted) {break;}
      }
      if (exhausted) {break;}
    }
    if (exhausted) {
      ++diagnostic.hypothesis_rejected_by_reason["orthogonal_outline_search_limit"];
      ++result.counts["search_limit_contours"];
      ++result.counts["search_limit_regions"];
      diagnostic.confidence_grade = "LOW";
      diagnostic.confidence_reason = "unexamined_competing_hypotheses_remain";
      for (size_t i = hypothesis_start; i < result.hypotheses.size(); ++i) {
        auto & hypothesis = result.hypotheses[i];
        if (hypothesis.disposition == "rejected_geometry_validation" &&
          hypothesis.reason != "orthogonal_outline_search_limit") {continue;}
        hypothesis.selection_state = "UNRESOLVED";
        hypothesis.selection_reason = "orthogonal_outline_search_limit";
        hypothesis.disposition = "unresolved_search_limit";
        hypothesis.reason = "orthogonal_outline_search_limit";
        hypothesis.confidence_grade = "LOW";
        hypothesis.confidence_reason = "unexamined_competing_hypotheses_remain";
      }
      result.fits.resize(contour_start);
    } else {++result.counts["completed_contours"];}
    if (result.fits.size() == contour_start) {
      result.unresolved_outline_ids.push_back(contour.outline_id);
    }
  }
  if (result.fits.empty() && diagnostic.confidence_grade.empty()) {
    diagnostic.confidence_grade = "LOW";
    diagnostic.confidence_reason = "observed_geometry_not_individually_resolved";
  }
  diagnostic.generated_hypothesis_count = result.fits.size();
  diagnostic.partitioned_candidate_count = result.fits.size();
  diagnostic.partition_reason = result.fits.empty() ? "no_resolved_convex_l_hypotheses" :
    "convex_l_hypotheses_with_connected_orthogonal_edge_evidence";
  return result;
}

}  // namespace

PlaneFits reconstruct_plane(
  const PlaneRegion & patch, uint32_t patch_index, uint32_t & next_id,
  size_t & search_operations, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, bool retain_visualization_evidence)
{return reconstruct_plane_impl(patch, patch_index, next_id, search_operations, obs, profile,
    retain_visualization_evidence);}
}  // namespace cuboid_geometry
}  // namespace arm_cell_vision
