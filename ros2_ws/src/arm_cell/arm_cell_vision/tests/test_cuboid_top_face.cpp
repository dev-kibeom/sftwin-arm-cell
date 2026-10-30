#include <cmath>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <cstring>
#include <limits>
#include <iostream>
#include <numeric>
#include <set>
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include "arm_cell_vision/cuboid_top_face.hpp"
#include "../src/cuboid_top_face_geometry.hpp"
#include "../src/cuboid_unresolved.hpp"

namespace
{
using arm_cell_vision::CuboidHeightObservation;
using arm_cell_vision::DetectionResult;
using arm_cell_vision::DetectorProfile;
using arm_cell_vision::detect_cuboid_top_face;
using Code = arm_cell_interfaces::msg::DetectTargetResultCode;

DetectorProfile profile()
{
  DetectorProfile p;
  p.algorithm = "cuboid_top_face";
  p.dimensions_m = {0.08, 0.05, 0.03};
  p.dimension_tolerance_m = {0.0035, 0.0035, 0.0035};
  p.workspace_roi_min_m = {-0.3, -0.3, 0.005};
  p.workspace_roi_max_m = {0.3, 0.3, 0.2};
  p.support_contact_tolerance_m = 0.0025;
  p.cuboid.height_tolerance_m = 0.0025;
  return p;
}
CuboidHeightObservation scene()
{
  return {cv::Mat::zeros(240, 240, CV_64F), {-0.3, -0.3}, 0.0025};
}
// Independent Figure 4 test scene renderer: polygon coordinates round to grid, depth is max top.
void cuboid(
  CuboidHeightObservation & obs, double x, double y, double yaw = 0,
  double bottom = 0, double length = 0.08, double width = 0.05, double thickness = 0.03)
{
  std::vector<cv::Point> polygon;
  for (const auto s : std::array<cv::Point2d, 4>{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}}) {
    double px = x + s.x * length / 2 * std::cos(yaw) - s.y * width / 2 * std::sin(yaw);
    double py = y + s.x * length / 2 * std::sin(yaw) + s.y * width / 2 * std::cos(yaw);
    polygon.emplace_back(
      std::lround((px - obs.origin.x) / obs.spacing_m),
      std::lround((py - obs.origin.y) / obs.spacing_m));
  }
  cv::Mat mask = cv::Mat::zeros(obs.heights.size(), CV_8U);
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygon}, cv::Scalar(255));
  obs.heights.setTo(bottom + thickness, mask & (obs.heights < bottom + thickness));
}

void l_outline_cuboid(
  CuboidHeightObservation & obs, double yaw = 0.0,
  double horizontal_length = 0.045, double vertical_length = 0.03,
  double strip_width = 0.01, double bottom = 0.0)
{
  const double length = 0.08, width = 0.05, thickness = 0.03;
  const double x0 = -length / 2, y0 = -width / 2;
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      const auto point = obs.origin + cv::Point2d(col, row) * obs.spacing_m;
      const double x = point.x * std::cos(yaw) + point.y * std::sin(yaw);
      const double y = -point.x * std::sin(yaw) + point.y * std::cos(yaw);
      if (x >= x0 && x <= -x0 && y >= y0 && y <= -y0) {
        obs.heights.at<double>(row, col) = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
  auto paint = [&](double x1, double y1, double x2, double y2) {
      std::vector<cv::Point> polygon;
      for (const auto & p : std::array<cv::Point2d, 4>{{{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}}}) {
        const double x = p.x * std::cos(yaw) - p.y * std::sin(yaw);
        const double y = p.x * std::sin(yaw) + p.y * std::cos(yaw);
        polygon.emplace_back(
          std::lround((x - obs.origin.x) / obs.spacing_m),
          std::lround((y - obs.origin.y) / obs.spacing_m));
      }
      cv::Mat mask = cv::Mat::zeros(obs.heights.size(), CV_8U);
      cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygon}, cv::Scalar(255));
      obs.heights.setTo(bottom + thickness, mask);
    };
  paint(x0, y0, x0 + horizontal_length, y0 + strip_width);
  paint(x0, y0, x0 + strip_width, y0 + vertical_length);
}
void partial_rectangle_at_recipe_corner(
  CuboidHeightObservation & obs, double visible_length, double visible_width,
  double top = 0.03, double x0 = -0.04, double y0 = -0.025)
{
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      const auto point = obs.origin + cv::Point2d(col, row) * obs.spacing_m;
      if (point.x >= x0 && point.x <= x0 + 0.08 &&
        point.y >= y0 && point.y <= y0 + 0.05) {
        obs.heights.at<double>(row, col) = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
  std::vector<cv::Point> polygon;
  for (const auto & point : std::array<cv::Point2d, 4>{{
      {x0, y0}, {x0 + visible_length, y0},
      {x0 + visible_length, y0 + visible_width}, {x0, y0 + visible_width}}})
  {
    polygon.emplace_back(
      std::lround((point.x - obs.origin.x) / obs.spacing_m),
      std::lround((point.y - obs.origin.y) / obs.spacing_m));
  }
  cv::Mat mask = cv::Mat::zeros(obs.heights.size(), CV_8U);
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygon}, cv::Scalar(255));
  obs.heights.setTo(top, mask);
}
void partial_rotated_rectangle_at_recipe_corner(
  CuboidHeightObservation & obs, double visible_length, double visible_width, double yaw)
{
  constexpr double corner_x = -0.04, corner_y = -0.025;
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      const auto point = obs.origin + cv::Point2d(col, row) * obs.spacing_m;
      const double x = point.x * c + point.y * s, y = -point.x * s + point.y * c;
      if (x >= corner_x && x <= corner_x + 0.08 &&
        y >= corner_y && y <= corner_y + 0.05) {
        obs.heights.at<double>(row, col) = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
  std::vector<cv::Point> polygon;
  for (const auto & point : std::array<cv::Point2d, 4>{{
      {corner_x, corner_y}, {corner_x + visible_length, corner_y},
      {corner_x + visible_length, corner_y + visible_width},
      {corner_x, corner_y + visible_width}}})
  {
    const double x = point.x * c - point.y * s;
    const double y = point.x * s + point.y * c;
    polygon.emplace_back(
      std::lround((x - obs.origin.x) / obs.spacing_m),
      std::lround((y - obs.origin.y) / obs.spacing_m));
  }
  cv::Mat mask = cv::Mat::zeros(obs.heights.size(), CV_8U);
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygon}, cv::Scalar(255));
  obs.heights.setTo(0.03, mask);
}
void add_height_variation(
  CuboidHeightObservation & obs, double x_per_cell, double y_per_cell, double noise_amplitude)
{
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      auto & height = obs.heights.at<double>(row, col);
      if (height <= 0) {continue;}
      const int noise_code = (row * 17 + col * 31) % 11 - 5;
      height += (col - 120) * x_per_cell + (row - 120) * y_per_cell +
        noise_code * noise_amplitude;
    }
  }
}
TEST(CuboidTopFace, SmallWholePlaneResidualDoesNotHideLocallyDistinctRamp)
{
  namespace geometry = arm_cell_vision::cuboid_geometry;
  for (bool narrow_contact : {false, true}) {
    auto obs = scene();
    cuboid(obs, 0, 0);
    const int y_min = narrow_contact ? 118 : 110;
    const int y_max = narrow_contact ? 122 : 130;
    for (int y = y_min; y <= y_max; ++y) {
      for (int x = 137; x <= 141; ++x) {
        obs.heights.at<double>(y, x) = 0.03 + (x - 136) * 0.0012;
      }
    }
    // Identical deterministic Depth noise on both sides of the contact.
    for (int y = 110; y <= 130; ++y) {
      for (int x = 104; x <= 141; ++x) {
        if (obs.heights.at<double>(y, x) > 0) {
          obs.heights.at<double>(y, x) += 0.0002 * (((x * 13 + y * 7) % 5) - 2) / 2;
        }
      }
    }
    const auto original = obs.heights.clone();
    const auto p = profile();
    const auto connected = geometry::extract_plane_regions(obs, p);
    ASSERT_EQ(connected.size(), 1U);
    const auto & whole = connected.front();
    ASSERT_TRUE(whole.plane_fitted);
    ASSERT_LE(whole.residual_m, p.cuboid.height_tolerance_m);
    EXPECT_EQ(whole.status, narrow_contact ? "horizontal" : "mixed_or_inclined");
    cv::Mat flat_cells = cv::Mat::zeros(obs.heights.size(), CV_8U);
    flat_cells(cv::Rect(104, 110, 33, 21)).setTo(255);
    cv::Mat ramp_cells = cv::Mat::zeros(obs.heights.size(), CV_8U);
    ramp_cells(cv::Rect(137, y_min, 5, y_max - y_min + 1)).setTo(255);
    const auto flat = geometry::extract_plane_regions(obs, p, &flat_cells);
    const auto ramp = geometry::extract_plane_regions(obs, p, &ramp_cells);
    ASSERT_EQ(flat.size(), 1U); ASSERT_EQ(ramp.size(), 1U);
    ASSERT_TRUE(flat.front().plane_fitted); ASSERT_TRUE(ramp.front().plane_fitted);
    ASSERT_LE(ramp.front().residual_m, p.cuboid.height_tolerance_m);
    const auto gradient = [](const auto & plane) {
        return cv::Vec2d(-plane.normal[0] / plane.normal[2], -plane.normal[1] / plane.normal[2]);
      };
    const int radius = std::max(2, static_cast<int>(std::ceil(p.cuboid.line_residual_limit_px)));
    ASSERT_GT(cv::norm(gradient(flat.front()) - gradient(ramp.front())) * 2 * radius * obs.spacing_m,
      p.cuboid.height_tolerance_m);
    std::cout << "LOCAL_PLANE_REPRO narrow_contact=" << narrow_contact <<
      " whole_status=" << whole.status << " whole_rms_m=" << whole.residual_m <<
      " tolerance_m=" << p.cuboid.height_tolerance_m << std::endl;
    size_t budget = 20000;
    const auto map = geometry::build_plane_map(obs, p, budget, true);
    EXPECT_GE(map.regions.size(), 2U);
    EXPECT_TRUE(std::any_of(map.regions.begin(), map.regions.end(),
      [](const auto & plane) {return plane.status == "horizontal";}));
    EXPECT_TRUE(std::any_of(map.regions.begin(), map.regions.end(),
      [](const auto & plane) {return plane.status == "mixed_or_inclined";}));
    ASSERT_EQ(map.boundaries.size(), 1U);
    EXPECT_EQ(map.boundaries.front().cell_count, whole.pixels.size());
    EXPECT_EQ(cv::norm(original, obs.heights, cv::NORM_INF), 0);
    const auto result = detect_cuboid_top_face(obs, p);
    EXPECT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS);
    EXPECT_EQ(result.ordered_candidates.size(), 1U);
    if (result.ordered_candidates.empty()) {continue;}
    EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.x, 0, obs.spacing_m);
    EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.y, 0, obs.spacing_m);
  }
}

TEST(CuboidTopFace, LocalPlaneDecisionKeepsNoisySingleSurfacesAndSubToleranceBendTogether)
{
  namespace geometry = arm_cell_vision::cuboid_geometry;
  for (double noise_amplitude : {0.0002, 0.0008}) {
    for (int surface = 0; surface < 3; ++surface) {
      auto obs = scene();
      cuboid(obs, 0, 0);
      for (int y = 110; y <= 130; ++y) {
        for (int x = 104; x <= 136; ++x) {
          const double local_x = (x - 120) * obs.spacing_m;
          const double noise = noise_amplitude * (((x * 13 + y * 7) % 5) - 2) / 2;
          obs.heights.at<double>(y, x) += noise + (surface ? 0.2 * local_x : 0) +
            (surface == 2 ? 0.02 * std::abs(local_x) : 0);
        }
      }
      size_t budget = 20000;
      const auto map = geometry::build_plane_map(obs, profile(), budget, true);
      ASSERT_EQ(map.regions.size(), 1U) << "surface=" << surface;
      EXPECT_EQ(map.regions.front().status, surface ? "mixed_or_inclined" : "horizontal");
      const auto result = detect_cuboid_top_face(obs, profile());
      EXPECT_EQ(result.result_code, surface ? Code::DETECT_RESULT_GEOMETRY_ERROR : Code::DETECT_RESULT_SUCCESS);
      EXPECT_EQ(result.ordered_candidates.size(), surface ? 0U : 1U);
    }
  }
}

TEST(CuboidTopFace, PlaneMapSeparatesConnectedSurfacesWithoutRecipeAndPreservesCellOwnership)
{
  auto obs = scene();
  cuboid(obs, 0, 0);
  for (int y = 110; y <= 130; ++y) {
    for (int x = 137; x <= 168; ++x) {obs.heights.at<double>(y, x) = 0.03 + (x - 136) * 0.001;}
  }
  const auto original = obs.heights.clone();
  auto p = profile();
  p.dimensions_m = {0.19, 0.17, 0.11};  // Plane Map must ignore Recipe.
  size_t budget = 20000;
  const auto map = arm_cell_vision::cuboid_geometry::build_plane_map(obs, p, budget, true);
  ASSERT_EQ(map.boundaries.size(), 1U);
  ASSERT_GE(map.regions.size(), 2U);
  cv::Mat owned = cv::Mat::zeros(obs.heights.size(), CV_8U);
  bool horizontal = false, inclined = false, adjacent = false;
  for (const auto & plane : map.regions) {
    horizontal |= plane.status == "horizontal";
    inclined |= plane.status == "mixed_or_inclined" && plane.plane_fitted;
    adjacent |= !plane.adjacency.empty();
    EXPECT_TRUE(std::isfinite(plane.residual_m));
    for (const auto & pixel : plane.pixels) {EXPECT_EQ(owned.at<uint8_t>(pixel)++, 0);}
  }
  EXPECT_TRUE(horizontal); EXPECT_TRUE(inclined); EXPECT_TRUE(adjacent);
  EXPECT_EQ(cv::countNonZero(owned), cv::countNonZero(obs.heights > 0));
  EXPECT_EQ(cv::norm(original, obs.heights, cv::NORM_INF), 0);
}



TEST(CuboidTopFace, PartialLDoesNotBecomeAClosedObservedRectangleOrUseCompleteIou)
{
  auto obs = scene();
  l_outline_cuboid(obs, 0, 0.065, 0.04);
  const auto result = detect_cuboid_top_face(obs, profile(), true);
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS);
  EXPECT_EQ(result.ordered_candidates.at(0).confidence_grade, "MEDIUM");
  for (const auto & hypothesis : result.cuboid_hypotheses) {
    if (hypothesis.selection_state == "SELECTED") {EXPECT_FALSE(hypothesis.complete_iou);}
  }
  // Narrow observed arms may themselves partition into geometric rectangles;
  // the unobserved Recipe interior must never be completed at this stage.
  for (const auto & region : result.cuboid_height_patches.front().rectangle_regions) {
    EXPECT_LT(region[3] * region[4], 0.08 * 0.05 / 2);
  }
}

// These candidate models are controlled geometry, independent of proposal
// generation. Measurements and spatial compatibility determine selection.
TEST(CuboidTopFace, SelectionComparesOnlyActual3dCompetitorsAndKeepsIndependentObjects)
{
  namespace geometry = arm_cell_vision::cuboid_geometry;
  auto obs = scene();
  cuboid(obs, 0, 0, 0, 0.03);
  const auto make_fit = [&](uint32_t id, double x, double top, double thickness, double length) {
      geometry::Fit fit;
      fit.candidate_id = id; fit.center = {x, 0}; fit.yaw = 0;
      fit.top = top; fit.face = {length, 0.05, thickness, 0.0035, 0.0035};
      fit.mask = geometry::footprint(fit, obs);
      for (int y = 0; y < obs.heights.rows; ++y) {
        for (int x_cell = 0; x_cell < obs.heights.cols; ++x_cell) {
          if (fit.mask.at<uint8_t>(y, x_cell) && std::abs(obs.heights.at<double>(y, x_cell) - top) < 1e-8) {
            fit.observed_cell_ids.push_back(y * obs.heights.cols + x_cell);
          }
        }
      }
      return fit;
    };
  const auto complete = make_fit(1, 0, 0.06, 0.03, 0.08);
  const auto shorter = make_fit(2, -0.01, 0.06, 0.03, 0.06);
  const auto independent = make_fit(3, 0.15, 0.03, 0.03, 0.08);
  const auto stacked = make_fit(4, 0, 0.03, 0.03, 0.08);
  size_t budget = 20000;
  const auto selected = geometry::select_hypotheses({complete, shorter, independent, stacked}, obs, profile(), budget);
  EXPECT_EQ(selected.at(1).state, "SELECTED");
  EXPECT_EQ(selected.at(2).state, "ALTERNATIVE");
  EXPECT_EQ(selected.at(3).state, "SELECTED");
  EXPECT_EQ(selected.at(4).state, "SELECTED");  // XY overlap, touching vertical intervals.
  auto different_thickness = complete;
  different_thickness.candidate_id = 5; different_thickness.face.thickness = 0.05;
  budget = 20000;
  const auto tied = geometry::select_hypotheses({complete, different_thickness, independent}, obs, profile(), budget);
  EXPECT_EQ(tied.at(1).state, "UNRESOLVED");
  EXPECT_EQ(tied.at(5).state, "UNRESOLVED");
  EXPECT_EQ(tied.at(3).state, "SELECTED");
  budget = 0;
  const auto exhausted = geometry::select_hypotheses({complete, shorter, independent}, obs, profile(), budget);
  EXPECT_EQ(exhausted.at(1).state, "UNRESOLVED");
  EXPECT_EQ(exhausted.at(2).state, "UNRESOLVED");
  EXPECT_EQ(exhausted.at(3).state, "SELECTED");
  // A discarded spanning interpretation cannot discard the independent object
  // at its other end. Left and right volumes only touch in 3D.
  obs.heights.setTo(0);
  cuboid(obs, -0.04, 0, 0, 0.03);
  cuboid(obs, 0.07, 0, 0, 0.03, 0.02);
  const auto left = make_fit(6, -0.04, 0.06, 0.03, 0.08);
  const auto spanning = make_fit(7, 0, 0.06, 0.03, 0.09);
  const auto right = make_fit(8, 0.04, 0.06, 0.03, 0.08);
  budget = 20000;
  const auto neighbours = geometry::select_hypotheses({spanning, right, left}, obs, profile(), budget);
  EXPECT_EQ(neighbours.at(6).state, "SELECTED");
  EXPECT_EQ(neighbours.at(7).state, "ALTERNATIVE");
  EXPECT_EQ(neighbours.at(8).state, "SELECTED");

}

TEST(CuboidTopFace, FullyObservedSmallBoundaryErrorHasIouWithoutChangingConfidenceMeaning)
{
  auto obs = scene();
  cuboid(obs, 0, 0, 0, 0, 0.0825, 0.05);
  const auto result = detect_cuboid_top_face(obs, profile(), true);
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS);
  EXPECT_EQ(result.ordered_candidates.at(0).confidence_grade, "HIGH");
  const auto selected = std::find_if(result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
    [](const auto & h) {return h.selection_state == "SELECTED" && h.disposition != "removed_by_deduplication";});
  ASSERT_NE(selected, result.cuboid_hypotheses.end());
  ASSERT_TRUE(selected->complete_iou);
  EXPECT_GT(*selected->complete_iou, 0.8);
  EXPECT_LE(*selected->complete_iou, 1.0);
}

TEST(CuboidTopFace, ConvexBevelDoesNotInventRectanglePartitionFromVertexCoordinates)
{
  auto obs = scene();
  const std::vector<cv::Point> polygon{{100, 100}, {140, 100}, {140, 120}, {120, 140}, {100, 140}};
  cv::Mat mask = cv::Mat::zeros(obs.heights.size(), CV_8U);
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygon}, cv::Scalar(255));
  obs.heights.setTo(0.03, mask);
  const auto result = detect_cuboid_top_face(obs, profile(), true);
  ASSERT_EQ(result.cuboid_height_patches.size(), 1U);
  EXPECT_TRUE(result.cuboid_height_patches.front().rectangle_regions.empty());
  EXPECT_FALSE(result.cuboid_height_patches.front().observed_contours_xy.empty());
}

TEST(CuboidTopFace, RemoteCollinearBoundaryCannotTurnPlanePartitionIntoObservedAnchor)
{
  namespace geometry = arm_cell_vision::cuboid_geometry;
  auto obs = scene();
  geometry::ContourEvidence source;
  source.points = {{100, 100}, {140, 100}, {140, 110}, {164, 110},
    {164, 130}, {140, 130}, {140, 140}, {100, 140}};
  source.vertices = source.points;
  cv::Mat parent = cv::Mat::zeros(obs.heights.size(), CV_8U);
  cv::fillPoly(parent, std::vector<std::vector<cv::Point>>{source.points}, cv::Scalar(255));
  obs.heights.setTo(0.03, parent);
  for (size_t i = 0; i < source.points.size(); ++i) {
    geometry::Segment line;
    line.first = obs.origin + cv::Point2d(source.points[i]) * obs.spacing_m;
    line.last = obs.origin + cv::Point2d(source.points[(i + 1) % source.points.size()]) * obs.spacing_m;
    line.support = 1; line.residual_px = 0; line.source_line_id = i;
    source.lines.push_back(line);
  }
  // This plane subregion has one real exterior side. Its top/bottom edges
  // happen to be collinear with remote exterior segments of another lobe.
  geometry::PlaneRegion plane;
  plane.height = 0.03; plane.status = "horizontal";
  plane.cells = cv::Mat::zeros(obs.heights.size(), CV_8U);
  plane.cells(cv::Rect(100, 110, 33, 21)).setTo(255);
  cv::findNonZero(plane.cells, plane.pixels);
  plane.source_contours = std::make_shared<const std::vector<geometry::ContourEvidence>>(
    std::vector<geometry::ContourEvidence>{source});
  size_t budget = 20000; uint32_t id = 1;
  const auto result = geometry::reconstruct_plane(plane, 0, id, budget, obs, profile(), true);
  EXPECT_TRUE(result.fits.empty());
  EXPECT_FALSE(result.diagnostic.inferred_partition_edges.empty());
}

TEST(CuboidTopFace, SeparatedEqualHeightTopsHaveDistinctConnectedPatches)
{
  auto obs = scene();
  cuboid(obs, -0.12, 0);
  cuboid(obs, 0.12, 0);
  const auto r = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(r.result_code, Code::DETECT_RESULT_SUCCESS);
  EXPECT_EQ(r.ordered_candidates.size(), 2U);
  EXPECT_EQ(r.cuboid_height_patches.size(), 2U);
  uint64_t cells = 0;
  for (const auto & patch : r.cuboid_height_patches) {cells += patch.cell_count;}
  EXPECT_EQ(cells, static_cast<uint64_t>(cv::countNonZero(obs.heights > 0)));
}



TEST(CuboidTopFace, LaterContourSearchLimitPreservesCompletedIndependentGeometry)
{
  namespace geometry = arm_cell_vision::cuboid_geometry;
  auto obs = scene();
  cuboid(obs, -0.12, -0.08);
  cuboid(obs, 0.12, 0.08);
  const cv::Mat observed = obs.heights > 0;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(observed.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  ASSERT_EQ(contours.size(), 2U);
  const auto patch_for = [&](const cv::Mat & cells) {
      geometry::PlaneRegion patch;
      patch.cells = cells;
      cv::findNonZero(cells, patch.pixels);
      patch.height = 0.03;
      patch.status = "horizontal";
      return patch;
    };
  const auto first_bounds = cv::boundingRect(contours.front());
  const auto first_center = obs.origin + cv::Point2d(
    first_bounds.x + (first_bounds.width - 1) / 2.0,
    first_bounds.y + (first_bounds.height - 1) / 2.0) * obs.spacing_m;
  std::array<size_t, 2> costs{};
  geometry::PlaneFits first_complete;
  for (size_t i = 0; i < contours.size(); ++i) {
    cv::Mat cells = cv::Mat::zeros(observed.size(), CV_8U);
    cv::drawContours(cells, contours, static_cast<int>(i), cv::Scalar(255), -1);
    size_t budget = 20000;
    uint32_t id = 1;
    auto complete = geometry::reconstruct_plane(patch_for(cells), 0, id, budget, obs, profile());
    ASSERT_EQ(complete.fits.size(), 1U);
    costs[i] = 20000 - budget;
    if (i == 0) {first_complete = std::move(complete);}
  }
  // Exercise the contour-processing boundary directly with one patch containing
  // two independent contours. No production patch connectivity or Profile is changed.
  // Test exhaustion before proposals and after the later contour's validation.
  for (size_t budget : {costs[0], costs[0] + costs[1] - 1}) {
    const bool later_validation_started = budget > costs[0];
    uint32_t id = 1;
    const auto result = geometry::reconstruct_plane(patch_for(observed), 0, id, budget, obs, profile());
    ASSERT_EQ(result.fits.size(), 1U);
    const auto & retained = result.fits.front();
    EXPECT_EQ(retained.candidate_id, first_complete.fits.front().candidate_id);
    EXPECT_NEAR(cv::norm(retained.center - first_center), 0, obs.spacing_m);
    EXPECT_NEAR(retained.measured_length, 0.08, profile().dimension_tolerance_m[0]);
    EXPECT_NEAR(retained.measured_width, 0.05, profile().dimension_tolerance_m[1]);
    const auto record = std::find_if(result.hypotheses.begin(), result.hypotheses.end(),
      [&](const auto & h) {return h.candidate_id == retained.candidate_id;});
    ASSERT_NE(record, result.hypotheses.end());
    EXPECT_EQ(record->disposition, "retained_after_deduplication");
    EXPECT_EQ(result.diagnostic.hypothesis_rejected_by_reason.count("orthogonal_outline_search_limit"), 1U);
    EXPECT_EQ(result.counts.at("search_limit_regions"), 1U);
    EXPECT_EQ(result.counts.at("completed_contours"), 1U);
    EXPECT_EQ(result.counts.at("search_limit_contours"), 1U);
    EXPECT_EQ(result.diagnostic.partitioned_candidate_count, 1U);
    if (later_validation_started) {
      EXPECT_GT(std::count_if(result.hypotheses.begin(), result.hypotheses.end(),
          [](const auto & h) {return h.reason == "orthogonal_outline_search_limit";}), 0);
    }
    std::cout << "SEARCH_LIMIT_DIAGNOSTIC " << result.diagnostic.partition_reason << std::endl;
    for (const auto & h : result.hypotheses) {
      if (h.reason == "orthogonal_outline_search_limit") {
        EXPECT_EQ(h.disposition, "unresolved_search_limit");
        EXPECT_NE(h.candidate_id, retained.candidate_id);
      }
    }
  }
}

TEST(CuboidTopFace, NoisyTopCellsBelongToOnePatchAcrossAbsoluteHeightBins)
{
  auto obs = scene();
  cuboid(obs, 0, 0, 0.0, 0.0012);
  add_height_variation(obs, 0, 0, 0.0001);
  const auto r = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(r.result_code, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(r.cuboid_height_patches.size(), 1U);
  EXPECT_EQ(r.cuboid_height_patches[0].cell_count,
    static_cast<uint64_t>(cv::countNonZero(obs.heights > 0)));
}



TEST(CuboidTopFace, FittedHeightVariationWithinToleranceRemainsHorizontal)
{
  auto obs = scene(); cuboid(obs, 0, 0);
  // Actual x extent varies by 2.24 mm; using the bounding diagonal would
  // incorrectly inflate that variation beyond the 2.5 mm profile tolerance.
  add_height_variation(obs, 0.00007, 0, 0);
  const auto r = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(r.result_code, Code::DETECT_RESULT_SUCCESS);
  EXPECT_EQ(r.ordered_candidates.size(), 1U);
  EXPECT_EQ(r.rawpart_rejected_by_reason.count("inclined_surface"), 0U);
}

TEST(CuboidTopFace, TinyRecipeSideContinuationsStopWithoutPromotingAnIncompleteSearch)
{
  auto obs = scene(); cuboid(obs, 0, 0, 0, 0, 0.15, 0.15, 0.12);
  auto p = profile(); p.dimensions_m = {0.08, 0.05, 1e-7};
  p.dimension_tolerance_m = {0.0035, 0.0035, 1e-9};
  const auto r = detect_cuboid_top_face(obs, p);
  EXPECT_NE(r.result_code, Code::DETECT_RESULT_SUCCESS);
  EXPECT_TRUE(r.ordered_candidates.empty());
  EXPECT_GT(r.rawpart_rejected_by_reason.count("orthogonal_outline_search_limit"), 0U);
}



// A connected low surface must not inherit its neighbor's high ceiling.














TEST(CuboidTopFace, OneContourCanProduceSeveralIndependentlyCountedFittedEdges)
{
  auto obs = scene(); cuboid(obs, 0, 0);
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  ASSERT_EQ(result.cuboid_height_patches.size(), 1U);
  const auto & patch = result.cuboid_height_patches.front();
  EXPECT_EQ(patch.contour_count, 1U);
  EXPECT_GT(patch.fitted_edge_count, 1U);
  uint64_t rejected_edges = 0;
  for (const auto & entry : patch.edge_fit_rejected_by_reason) {rejected_edges += entry.second;}
  EXPECT_EQ(patch.edge_fit_attempt_count, patch.fitted_edge_count + rejected_edges);
  EXPECT_TRUE(patch.contour_rejected_by_reason.empty());
  ASSERT_GT(result.cuboid_stages.size(), 4U);
  const auto & geometry = result.cuboid_stages[4];
  EXPECT_EQ(
    geometry.candidate_evaluation_attempt_count,
    geometry.rejected_candidate_evaluation_count + geometry.generated_count);
  const auto & line = result.cuboid_stages[2];
  EXPECT_EQ(line.status, "computed");
  EXPECT_EQ(line.reason, "fitted_lines_available");
  EXPECT_EQ(line.input_count, 1U);
  EXPECT_EQ(line.output_count, 1U);
  EXPECT_EQ(line.rejected_count, 0U);
}

TEST(CuboidTopFace, LineStageStatusAndReasonMatchContourAndEdgeCounts)
{
  auto no_patches = scene();
  no_patches.heights.setTo(0.0);
  const auto empty_result = detect_cuboid_top_face(no_patches, profile());
  const auto & empty_line = empty_result.cuboid_stages[2];
  EXPECT_EQ(empty_line.status, "not_computed");
  EXPECT_EQ(empty_line.reason, "no_height_patches");
  EXPECT_FALSE(empty_line.input_count);
  EXPECT_FALSE(empty_line.output_count);
  EXPECT_FALSE(empty_line.rejected_count);

  // A measured point boundary survives extraction, but cannot supply fitted lines.
  auto insufficient = scene();
  insufficient.heights.setTo(std::numeric_limits<double>::quiet_NaN());
  insufficient.heights.at<double>(120, 120) = 0.03;
  const auto insufficient_result = detect_cuboid_top_face(insufficient, profile());
  const auto & no_contour_line = insufficient_result.cuboid_stages[2];
  EXPECT_EQ(no_contour_line.status, "computed");
  EXPECT_EQ(no_contour_line.reason, "no_fitted_lines");
  EXPECT_EQ(no_contour_line.input_count, 1U);
  EXPECT_EQ(no_contour_line.output_count, 0U);
  EXPECT_EQ(no_contour_line.rejected_count, 1U);

  auto no_fitted_edges = insufficient;
  no_fitted_edges.heights.at<double>(120, 121) = 0.03;
  no_fitted_edges.heights.at<double>(120, 122) = 0.03;
  const auto no_edges_result = detect_cuboid_top_face(no_fitted_edges, profile());
  const auto & no_edge_line = no_edges_result.cuboid_stages[2];
  EXPECT_EQ(no_edge_line.status, "computed");
  EXPECT_EQ(no_edge_line.reason, "no_fitted_lines");
  EXPECT_EQ(no_edge_line.input_count, 1U);
  EXPECT_EQ(no_edge_line.output_count, 0U);
  EXPECT_EQ(no_edge_line.rejected_count, 1U);
  EXPECT_FALSE(no_edge_line.rejected_by_reason.empty());
}

TEST(CuboidTopFace, ZeroCandidateEvidenceSeparatesStagesAndRetainsActualHeightGrid)
{
  auto obs = scene();
  obs.heights.setTo(std::numeric_limits<double>::quiet_NaN());
  obs.heights.at<double>(120, 120) = 0.03;
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  ASSERT_EQ(result.cuboid_stages.size(), 7U);
  EXPECT_EQ(result.cuboid_stages[0].stage, "height_grid_and_workspace");
  EXPECT_EQ(result.cuboid_stages[0].output_count, 1U);
  EXPECT_EQ(result.cuboid_stages[1].auxiliary_counts.at("formed_height_patches"), 1U);
  EXPECT_EQ(result.cuboid_stages[2].reason, "no_fitted_lines");
  EXPECT_EQ(result.cuboid_stages[3].rejected_by_reason.at("unresolved_insufficient_support"), 1U);
  EXPECT_FALSE(result.cuboid_stages[4].input_count);
  EXPECT_EQ(result.cuboid_stages[5].reason, "no_generated_candidates");
  EXPECT_EQ(result.cuboid_stages[6].status, "not_computed");
  ASSERT_EQ(result.cuboid_height_patches.size(), 1U);
  EXPECT_DOUBLE_EQ(result.cuboid_height_patches[0].height_m, 0.03);
  EXPECT_EQ(result.cuboid_height_patches[0].cell_count, 1U);
  EXPECT_EQ(result.cuboid_height_patches[0].partition_status, "orthogonal_outline_reconstruction");
  EXPECT_EQ(
    result.cuboid_height_patches[0].partition_reason,
    "unresolved_insufficient_support");
  EXPECT_EQ(result.cuboid_stages[5].status, "not_computed");
  EXPECT_EQ(result.cuboid_stages[5].reason, "no_generated_candidates");
  EXPECT_NE(
    result.cuboid_height_patches[0].hypothesis_rejected_by_reason.find(
      "unresolved_insufficient_support"),
    result.cuboid_height_patches[0].hypothesis_rejected_by_reason.end());
  ASSERT_TRUE(result.cuboid_replay_evidence);
  EXPECT_EQ(result.cuboid_replay_evidence->rows, 240U);
  EXPECT_EQ(result.cuboid_replay_evidence->columns, 240U);
  EXPECT_DOUBLE_EQ(result.cuboid_replay_evidence->resolution_m, obs.spacing_m);
  EXPECT_EQ(result.cuboid_replay_evidence->height_grid_m.total(), obs.heights.total());
  EXPECT_TRUE(std::isnan(result.cuboid_replay_evidence->height_grid_m.at<double>(0, 0)));
  EXPECT_DOUBLE_EQ(result.cuboid_replay_evidence->height_grid_m.at<double>(120, 120), 0.03);
}

TEST(CuboidTopFace, InvalidHeightObservationLeavesAllGeometryStagesNotComputed)
{
  const auto result = detect_cuboid_top_face(CuboidHeightObservation{}, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  ASSERT_EQ(result.cuboid_stages.size(), 7U);
  for (const auto & stage : result.cuboid_stages) {
    EXPECT_EQ(stage.status, "not_computed") << stage.stage;
    EXPECT_FALSE(stage.input_count) << stage.stage;
    EXPECT_FALSE(stage.output_count) << stage.stage;
    EXPECT_FALSE(stage.rejected_count) << stage.stage;
  }
  EXPECT_EQ(result.cuboid_stages.front().reason, "invalid_height_observation");
}

TEST(CuboidTopFace, MultipleHeightPatchesKeepTheirOwnRejectionReasons)
{
  auto obs = scene();
  cuboid(obs, 0.0, 0.0, 0.0, 0.0, 0.15, 0.15, 0.03);
  obs.heights.at<double>(30, 30) = 0.06;
  const auto result = detect_cuboid_top_face(obs, profile());
  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR);
  ASSERT_EQ(result.cuboid_height_patches.size(), 2U);
  EXPECT_DOUBLE_EQ(result.cuboid_height_patches[0].height_m, 0.06);
  EXPECT_NE(
    result.cuboid_height_patches[0].hypothesis_rejected_by_reason.find(
      "unresolved_insufficient_support"),
    result.cuboid_height_patches[0].hypothesis_rejected_by_reason.end());
  EXPECT_DOUBLE_EQ(result.cuboid_height_patches[1].height_m, 0.03);
  EXPECT_FALSE(result.cuboid_height_patches[1].geometry_rejected_by_reason.empty());
  EXPECT_EQ(result.cuboid_stages[3].stage, "cuboid_hypothesis_generation");
  EXPECT_EQ(result.cuboid_stages[4].stage, "dimension_and_geometry_validation");
  for (const auto & patch : result.cuboid_height_patches) {
    EXPECT_EQ(patch.candidate_evaluation_attempt_count, patch.geometry_valid_before_deduplication_count +
      std::accumulate(patch.geometry_rejected_by_reason.begin(),
      patch.geometry_rejected_by_reason.end(), uint64_t{0},
      [](uint64_t total, const auto & entry) {return total + entry.second;}));
    EXPECT_EQ(patch.generated_hypothesis_count, patch.geometry_valid_before_deduplication_count -
      patch.deduplicated_count);
    EXPECT_EQ(patch.deduplicated_count, std::accumulate(
      patch.deduplicated_by_reason.begin(), patch.deduplicated_by_reason.end(), uint64_t{0},
      [](uint64_t total, const auto & entry) {return total + entry.second;}));
  }
  const auto attempts = std::accumulate(
    result.cuboid_height_patches.begin(), result.cuboid_height_patches.end(), uint64_t{0},
    [](uint64_t total, const auto & patch) {return total + patch.candidate_evaluation_attempt_count;});
  const auto geometrically_valid = std::accumulate(
    result.cuboid_height_patches.begin(), result.cuboid_height_patches.end(), uint64_t{0},
    [](uint64_t total, const auto & patch) {return total + patch.geometry_valid_before_deduplication_count;});
  ASSERT_EQ(result.cuboid_hypotheses.size(), attempts);
  std::set<uint32_t> attempt_ids;
  size_t rejected_geometry_attempts = 0;
  for (const auto & hypothesis : result.cuboid_hypotheses) {
    EXPECT_TRUE(attempt_ids.insert(hypothesis.candidate_id).second);
    if (hypothesis.disposition == "rejected_geometry_validation") {
      ++rejected_geometry_attempts;
      EXPECT_FALSE(hypothesis.reason.empty());
    }
  }
  EXPECT_EQ(rejected_geometry_attempts, attempts - geometrically_valid);
  const auto pre_object_rejected = std::accumulate(
    result.cuboid_height_patches.begin(), result.cuboid_height_patches.end(), uint64_t{0},
    [](uint64_t total, const auto & patch) {return total + patch.pre_object_rejected_count;});
  EXPECT_EQ(result.cuboid_stages[4].candidate_evaluation_attempt_count, attempts);
  EXPECT_EQ(result.cuboid_stages[4].generated_count, geometrically_valid);
  EXPECT_EQ(result.cuboid_stages[4].rejected_candidate_evaluation_count,
    attempts - geometrically_valid);
  EXPECT_EQ(result.cuboid_stages[3].pre_object_rejected_count, pre_object_rejected);
  EXPECT_FALSE(result.cuboid_stages[4].input_count);
  EXPECT_FALSE(result.cuboid_stages[4].output_count);
  EXPECT_EQ(result.cuboid_stages[5].input_count.value_or(0),
    std::accumulate(result.cuboid_height_patches.begin(), result.cuboid_height_patches.end(),
      uint64_t{0}, [](uint64_t total, const auto & patch) {
        return total + patch.generated_hypothesis_count;
      }));
}

TEST(CuboidTopFace, BoundarySupportIsUnavailableBeforeGeometryValidationReachesIt)
{
  namespace geometry = arm_cell_vision::cuboid_geometry;
  auto observation = scene();
  cuboid(observation, 0.0, 0.0);
  auto detector_profile = profile();
  detector_profile.dimensions_m = {0.15, 0.12, 0.10};
  geometry::PlaneRegion patch;
  patch.height = 0.03;
  patch.status = "horizontal";
  patch.cells = observation.heights > 0;
  cv::findNonZero(patch.cells, patch.pixels);
  uint32_t next_candidate_id = 1;
  size_t search_operations = 20000;

  const auto result = geometry::reconstruct_plane(
    patch, 0, next_candidate_id, search_operations, observation, detector_profile, true);
  const auto rejected = std::find_if(
    result.hypotheses.begin(), result.hypotheses.end(), [](const auto & hypothesis) {
      return hypothesis.reason == "modeled_bottom_below_support";
    });

  ASSERT_NE(rejected, result.hypotheses.end());
  EXPECT_FALSE(rejected->boundary_support.has_value());
}

TEST(CuboidTopFace, BoundarySupportRetainsTheComputedValue)
{
  namespace geometry = arm_cell_vision::cuboid_geometry;
  auto observation = scene();
  cuboid(observation, 0.0, 0.0);
  auto detector_profile = profile();
  geometry::PlaneRegion patch;
  patch.height = 0.03;
  patch.status = "horizontal";
  patch.cells = observation.heights > 0;
  cv::findNonZero(patch.cells, patch.pixels);
  uint32_t next_candidate_id = 1;
  size_t search_operations = 20000;

  const auto result = geometry::reconstruct_plane(
    patch, 0, next_candidate_id, search_operations, observation, detector_profile, true);
  const auto retained = std::find_if(
    result.hypotheses.begin(), result.hypotheses.end(), [](const auto & hypothesis) {
      return hypothesis.disposition == "retained_after_deduplication";
    });

  ASSERT_NE(retained, result.hypotheses.end());
  ASSERT_TRUE(retained->boundary_support.has_value());
  EXPECT_NEAR(*retained->boundary_support, 1.0, 1e-8);
}

TEST(CuboidTopFace, DetailedSelectionRetainsTheActualHeightGrid)
{
  auto obs = scene(); cuboid(obs, 0.0, 0.0);
  const auto lightweight = detect_cuboid_top_face(obs, profile(), false);
  const auto result = detect_cuboid_top_face(obs, profile(), true);
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(result.result_code, lightweight.result_code);
  ASSERT_EQ(result.ordered_candidates.size(), lightweight.ordered_candidates.size());
  for (size_t index = 0; index < result.ordered_candidates.size(); ++index) {
    const auto & detailed_candidate = result.ordered_candidates[index];
    const auto & light_candidate = lightweight.ordered_candidates[index];
    EXPECT_DOUBLE_EQ(detailed_candidate.target_pose.pose.position.x,
      light_candidate.target_pose.pose.position.x);
    EXPECT_DOUBLE_EQ(detailed_candidate.target_pose.pose.position.y,
      light_candidate.target_pose.pose.position.y);
    EXPECT_DOUBLE_EQ(detailed_candidate.target_pose.pose.position.z,
      light_candidate.target_pose.pose.position.z);
    EXPECT_DOUBLE_EQ(detailed_candidate.target_pose.pose.orientation.z,
      light_candidate.target_pose.pose.orientation.z);
    EXPECT_DOUBLE_EQ(detailed_candidate.target_pose.pose.orientation.w,
      light_candidate.target_pose.pose.orientation.w);
  }
  EXPECT_FALSE(lightweight.cuboid_replay_evidence);
  ASSERT_TRUE(result.cuboid_replay_evidence);
  ASSERT_EQ(result.cuboid_replay_evidence->height_grid_m.total(), obs.heights.total());
  EXPECT_EQ(result.cuboid_replay_evidence->height_grid_m.data, obs.heights.data);
  EXPECT_EQ(result.cuboid_replay_evidence->image_boundary_mask.data, obs.image_boundary.data);
  EXPECT_DOUBLE_EQ(
    result.cuboid_replay_evidence->height_grid_m.at<double>(120, 120),
    obs.heights.at<double>(120, 120));
  ASSERT_EQ(result.cuboid_height_patches.size(), 1U);
  EXPECT_GT(result.cuboid_height_patches.front().fitted_edge_count,
    result.cuboid_height_patches.front().contour_count);
  EXPECT_EQ(result.cuboid_stages[2].input_count,
    result.cuboid_height_patches.front().contour_count);
  EXPECT_EQ(result.cuboid_stages[2].output_count,
    result.cuboid_height_patches.front().contours_with_fitted_edges_count);
  EXPECT_EQ(result.cuboid_stages[2].fitted_edge_count,
    result.cuboid_height_patches.front().fitted_edge_count);
  EXPECT_EQ(result.cuboid_height_patches.front().observed_cells_xy.size(),
    result.cuboid_height_patches.front().cell_count);
  EXPECT_EQ(result.cuboid_height_patches.front().observed_contours_xy.size(),
    result.cuboid_height_patches.front().contour_count);
  EXPECT_EQ(result.cuboid_height_patches.front().fitted_edges_xy_support.size(),
    result.cuboid_height_patches.front().fitted_edge_count);
  EXPECT_TRUE(lightweight.cuboid_height_patches.front().observed_cells_xy.empty());
  EXPECT_TRUE(lightweight.cuboid_height_patches.front().observed_contours_xy.empty());
  EXPECT_TRUE(lightweight.cuboid_height_patches.front().fitted_edges_xy_support.empty());
  EXPECT_EQ(result.cuboid_stages[5].stage, "candidate_local_eligibility");
  EXPECT_EQ(result.cuboid_stages[5].output_count, 1U);
  EXPECT_TRUE(result.cuboid_stages[5].rejected_by_reason.empty());
}

namespace
{
void measure_capture_path(
  const std::string & name, const CuboidHeightObservation & observation,
  bool detailed, size_t repetitions, uint8_t expected_result)
{
  std::vector<uint64_t> elapsed_us;
  elapsed_us.reserve(repetitions);
  size_t retained_bytes = 0;
  for (size_t iteration = 0; iteration < repetitions; ++iteration) {
    const auto started = std::chrono::steady_clock::now();
    const auto result = detect_cuboid_top_face(observation, profile(), detailed);
    const auto finished = std::chrono::steady_clock::now();
    EXPECT_EQ(result.result_code, expected_result);
    elapsed_us.push_back(static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(finished - started).count()));
    if (result.cuboid_replay_evidence) {
      EXPECT_EQ(result.cuboid_replay_evidence->height_grid_m.data, observation.heights.data);
      EXPECT_EQ(result.cuboid_replay_evidence->image_boundary_mask.data,
        observation.image_boundary.data);
      retained_bytes = result.cuboid_replay_evidence->height_grid_m.total() * sizeof(double) +
        result.cuboid_replay_evidence->image_boundary_mask.total();
    }
  }
  std::sort(elapsed_us.begin(), elapsed_us.end());
  std::cout << "VISION_REPLAY_PERF case=" << name << " repetitions=" << repetitions
            << " median_us=" << elapsed_us[elapsed_us.size() / 2]
            << " p95_us=" << elapsed_us[(elapsed_us.size() - 1) * 95 / 100]
            << " capture_payload_copy_bytes=0 retained_grid_and_mask_bytes=" << retained_bytes
            << "\n";
}
TEST(CuboidTopFace, ReconstructsFromLocallySupportedCornerOnConvexPartialContour)
{
  auto observation = scene();
  partial_rectangle_at_recipe_corner(observation, 0.06, 0.01);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  ASSERT_FALSE(result.ordered_candidates.empty()) << result.diagnostic_detail;
  EXPECT_EQ(result.rawpart_candidate_generation, "unified_l_corner_candidate_eligibility");
  EXPECT_FALSE(result.ordered_candidates.at(0).has_observed_rectangle);
  EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.x, 0.0, 2 * observation.spacing_m);
  EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.y, 0.0, 2 * observation.spacing_m);
  EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_long_side_m, 0.08, 1e-6);
  EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_short_side_m, 0.05, 1e-6);
  EXPECT_EQ(result.ordered_candidates.at(0).orientation_symmetry_order, 2);
  ASSERT_EQ(result.ordered_candidates.size(), 1U);
  EXPECT_EQ(result.ordered_candidates.front().confidence_grade, "MEDIUM");
  EXPECT_NE(result.ordered_candidates.front().confidence_reason.find("partial_top_surface"),
    std::string::npos);
  EXPECT_TRUE(result.ordered_candidates.at(0).has_estimated_height);
  EXPECT_NEAR(result.ordered_candidates.at(0).estimated_height_m, 0.03, 1e-6);
}

TEST(CuboidTopFace, ReconstructsFromTwoPartialEdgesWithoutClosedLContour)
{
  auto observation = scene();
  l_outline_cuboid(observation, 0.0, 0.0225, 0.015, 0.0075);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  ASSERT_FALSE(result.ordered_candidates.empty()) << result.diagnostic_detail;
  EXPECT_EQ(result.rawpart_candidate_generation, "unified_l_corner_candidate_eligibility");
  EXPECT_FALSE(result.ordered_candidates.at(0).has_observed_rectangle);
  EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_long_side_m, 0.08, 1e-6);
  EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_short_side_m, 0.05, 1e-6);
}

TEST(CuboidTopFace, ReconstructsAcrossSmallRasterGapAtObservedOuterCorner)
{
  auto observation = scene();
  l_outline_cuboid(observation, 0.0, 0.06, 0.02, 0.0075);
  // Simulate a short depth dropout at the shared corner. Both orthogonal
  // fitted edges and interior patch support remain, but their endpoints no
  // longer touch in the raster.
  observation.heights(cv::Rect(104, 110, 3, 3)).setTo(std::numeric_limits<double>::quiet_NaN());
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  ASSERT_FALSE(result.ordered_candidates.empty()) << result.diagnostic_detail;
  const auto corner_hypothesis = std::find_if(
    result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(), [&](const auto & h) {
      return std::hypot(h.center_x_m, h.center_y_m) <= 2 * observation.spacing_m &&
             h.disposition != "rejected_geometry_validation";
    });
  EXPECT_NE(corner_hypothesis, result.cuboid_hypotheses.end());
  EXPECT_EQ(result.rawpart_candidate_generation, "unified_l_corner_candidate_eligibility");
  EXPECT_FALSE(result.ordered_candidates.at(0).has_observed_rectangle);
}

TEST(CuboidTopFace, ReconstructedRecipeFootprintAndTopReferenceFollowRotatedCorners)
{
  const double half_pi = std::acos(-1.0) / 2;
  for (const double yaw : {-2.4, -0.73, -0.73 + half_pi, -0.28, 0.31,
      1.7, 2.8})
  {
    auto observation = scene();
    // The observed L, Recipe dimensions and measured surrounding Tray constrain
    // the unknown completion. Check the same normalized pose at every rotation.
    partial_rotated_rectangle_at_recipe_corner(observation, 0.06, 0.02, yaw);
    const auto result = detect_cuboid_top_face(observation, profile(), true);
    ASSERT_FALSE(result.ordered_candidates.empty()) << "yaw=" << yaw << " " << result.diagnostic_detail;
    const auto selected = std::find_if(
      result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(), [&result](const auto & h) {
        return h.confidence_grade == "MEDIUM" &&
               std::hypot(h.center_x_m - result.ordered_candidates.at(0).target_pose.pose.position.x,
                 h.center_y_m - result.ordered_candidates.at(0).target_pose.pose.position.y) < 1e-9;
      });
    ASSERT_NE(selected, result.cuboid_hypotheses.end());
    // The returned Recipe footprint, Top
    // Reference, and representative axis are internally consistent; the
    // separate unrotated fixture checks actual shape accuracy where known.
    EXPECT_FALSE(result.ordered_candidates.at(0).has_observed_rectangle);
    EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.x, selected->center_x_m, 1e-9);
    EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.y, selected->center_y_m, 1e-9);
    EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.z, selected->height_m, 1e-9);
    EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_long_side_m, std::max(selected->length_m, selected->width_m), 1e-6);
    EXPECT_NEAR(result.ordered_candidates.at(0).rectangle_short_side_m, std::min(selected->length_m, selected->width_m), 1e-6);
    const auto q = result.ordered_candidates.at(0).target_pose.pose.orientation;
    const double pose_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    EXPECT_NEAR(std::remainder(pose_yaw - selected->yaw_rad, std::acos(-1.0)), 0.0, 1e-9);
    EXPECT_EQ(result.ordered_candidates.at(0).orientation_symmetry_order, 2);
  }
}

TEST(CuboidTopFace, BiasedShortEdgeOutsideExistingOrthogonalityToleranceDoesNotInventPose)
{
  auto observation = scene();
  partial_rotated_rectangle_at_recipe_corner(observation, 0.06, 0.02,
    0.31 + std::acos(-1.0) / 2);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  EXPECT_FALSE(result.has_target_pose);
  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR);
  ASSERT_FALSE(result.cuboid_height_patches.empty());
  EXPECT_GT(result.cuboid_height_patches.front().hypothesis_rejected_by_reason.count(
      "nonorthogonal_corner_edges"), 0U);
  EXPECT_EQ(result.cuboid_height_patches.front().confidence_grade, "LOW");
}



TEST(CuboidTopFace, FullyObservedTopTouchingGridBoundaryRemainsHigh)
{
  auto observation = scene();
  cuboid(observation, 0.2575, 0);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  ASSERT_EQ(result.ordered_candidates.size(), 1U) << result.diagnostic_detail;
  EXPECT_EQ(result.ordered_candidates.front().confidence_grade, "HIGH");
}

TEST(CuboidTopFace, ShortEdgesDoNotDetermineRecipeLongAndShortSideMapping)
{
  auto observation = scene();
  partial_rectangle_at_recipe_corner(observation, 0.02, 0.02);
  for (int y = 110; y <= 142; ++y) {
    for (int x = 104; x <= 136; ++x) {
      if (observation.heights.at<double>(y, x) == 0) {
        observation.heights.at<double>(y, x) = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
  const auto first = detect_cuboid_top_face(observation, profile(), true);
  const auto repeated = detect_cuboid_top_face(observation, profile(), true);
  EXPECT_FALSE(first.has_target_pose) << first.diagnostic_detail;
  EXPECT_EQ(first.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_FALSE(first.disturbance_candidates.empty());
  EXPECT_EQ(first.diagnostic_detail, repeated.diagnostic_detail);
}

TEST(CuboidTopFace, StackedTopElevationDoesNotForceTraySupportedThickness)
{
  auto observation = scene();
  l_outline_cuboid(observation, 0.31, 0.0225, 0.015, 0.0075, 0.09);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_FALSE(result.has_target_pose);
  EXPECT_TRUE(std::any_of(result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
    [](const auto & h) {
      return std::abs(h.height_m - 0.12) < 1e-6 && std::abs(h.thickness_m - 0.08) < 1e-6 &&
             h.confidence_grade == "LOW";
    }));
  EXPECT_FALSE(result.disturbance_candidates.empty());
}



TEST(CuboidTopFace, RejectsOnlyRecipeHypothesesDirectlyContradictedByObservedEdgeLengths)
{
  auto observation = scene();
  partial_rectangle_at_recipe_corner(observation, 0.085, 0.06);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  EXPECT_FALSE(result.has_target_pose) << result.diagnostic_detail;
  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR);
  ASSERT_TRUE(result.rawpart_rejected_by_reason.count("observed_edge_exceeds_recipe_side_without_supported_continuation"));
  EXPECT_GT(result.rawpart_rejected_by_reason.at("observed_edge_exceeds_recipe_side_without_supported_continuation"), 0U);
}

TEST(CuboidTopFace, ReconstructedCandidateHasEstimatedProvenanceAndDeterministicPose)
{
  auto observation = scene();
  partial_rectangle_at_recipe_corner(observation, 0.06, 0.01);
  const auto first = detect_cuboid_top_face(observation, profile(), true);
  const auto repeated = detect_cuboid_top_face(observation, profile(), true);
  ASSERT_FALSE(first.ordered_candidates.empty());
  ASSERT_EQ(first.ordered_candidates.size(), 1U);
  const auto & candidate = first.ordered_candidates.front();
  EXPECT_FALSE(candidate.has_observed_rectangle);
  EXPECT_TRUE(first.rawpart_candidate_diagnostics.front().observed_yaw_valid);
  EXPECT_DOUBLE_EQ(first.rawpart_candidate_diagnostics.front().measured_width_m, 0.0);
  EXPECT_EQ(first.ordered_candidates.at(0).rawpart_selected_candidate, repeated.ordered_candidates.at(0).rawpart_selected_candidate);
  EXPECT_EQ(first.ordered_candidates.at(0).target_pose.pose.position.x, repeated.ordered_candidates.at(0).target_pose.pose.position.x);
  EXPECT_EQ(first.ordered_candidates.at(0).target_pose.pose.position.y, repeated.ordered_candidates.at(0).target_pose.pose.position.y);
  EXPECT_EQ(first.ordered_candidates.at(0).target_pose.pose.position.z, repeated.ordered_candidates.at(0).target_pose.pose.position.z);
  EXPECT_EQ(first.ordered_candidates.at(0).target_pose.pose.orientation.z, repeated.ordered_candidates.at(0).target_pose.pose.orientation.z);
}

TEST(CuboidTopFace, MissingLocalCornerEvidenceRemainsGeometryFailure)
{
  auto observation = scene();
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  EXPECT_FALSE(result.has_target_pose);
  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_NE(result.diagnostic_detail.find("no_eligible_candidates"), std::string::npos);
  EXPECT_EQ(result.rawpart_candidate_generation, "no_eligible_candidates");
}

TEST(CuboidTopFace, FootprintCanCrossWorkspaceObservationBoundaryWhenCenterIsAllowed)
{
  auto observation = scene();
  partial_rectangle_at_recipe_corner(observation, 0.06, 0.01, 0.03, 0.24, -0.025);
  // The true Recipe edge continues past the Height Grid boundary; only the
  // observed raster is retained, so the missing cells remain UNKNOWN.
  EXPECT_GT(cv::countNonZero(observation.heights.col(observation.heights.cols - 1) > 0), 0);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  ASSERT_FALSE(result.ordered_candidates.empty()) << result.diagnostic_detail;
  EXPECT_EQ(result.ordered_candidates.at(0).confidence_grade, "MEDIUM");
  EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.x, 0.28, 2 * observation.spacing_m);
  EXPECT_GT(result.ordered_candidates.at(0).rectangle_long_side_m / 2 + result.ordered_candidates.at(0).target_pose.pose.position.x, 0.3);
}

TEST(CuboidTopFace, ImageBoundaryClippingKeepsUniquelyReconstructedMediumCandidate)
{
  auto observation = scene();
  // Render only the part of a target that remains inside the camera crop.
  partial_rectangle_at_recipe_corner(observation, 0.04, 0.01, 0.03, -0.04, -0.025);
  observation.image_boundary = cv::Mat::zeros(observation.heights.size(), CV_8U);
  observation.image_boundary.col(120).setTo(255);
  observation.heights.colRange(120, observation.heights.cols).setTo(std::numeric_limits<double>::quiet_NaN());
  EXPECT_GT(cv::countNonZero(observation.heights.col(119) > 0), 0);
  EXPECT_EQ(cv::countNonZero(observation.heights.colRange(120, observation.heights.cols) > 0), 0);
  const auto result = detect_cuboid_top_face(observation, profile(), true);
  ASSERT_FALSE(result.ordered_candidates.empty()) << result.diagnostic_detail;
  EXPECT_EQ(result.ordered_candidates.at(0).confidence_grade, "MEDIUM");
  EXPECT_FALSE(result.ordered_candidates.at(0).has_observed_rectangle);
  EXPECT_NEAR(result.ordered_candidates.at(0).target_pose.pose.position.x, 0.0, 2 * observation.spacing_m);
}

TEST(CuboidTopFace, RoiBoundaryAloneDoesNotRejectReconstructedCandidate)
{
  auto observation = scene();
  partial_rectangle_at_recipe_corner(observation, 0.03, 0.015, 0.03, 0.02, 0.02);
  arm_cell_vision::cuboid_geometry::PlaneRegion patch;
  patch.height = 0.03;
  patch.status = "horizontal";
  patch.cells = cv::Mat::zeros(observation.heights.size(), CV_8U);
  for (int y = 0; y < observation.heights.rows; ++y) {
    for (int x = 0; x < observation.heights.cols; ++x) {
      if (observation.heights.at<double>(y, x) > 0) {
        patch.cells.at<uint8_t>(y, x) = 255;
        patch.pixels.emplace_back(x, y);
      }
    }
  }
  auto p = profile();
  p.workspace_roi_min_m[0] = 0.15;
  p.workspace_roi_max_m[0] = 0.2;
  uint32_t next_candidate_id = 1;
  size_t budget = 20000;
  const auto result = arm_cell_vision::cuboid_geometry::reconstruct_plane(
    patch, 0, next_candidate_id, budget, observation, p, true);
  EXPECT_FALSE(result.fits.empty());
  EXPECT_EQ(result.diagnostic.geometry_rejected_by_reason.count("workspace_roi_exceeded"), 0U);
}



}  // namespace

TEST(CuboidTopFacePerf, LightweightSuccess)
{
  auto obs = scene(); cuboid(obs, 0.0, 0.0);
  obs.image_boundary = cv::Mat::zeros(obs.heights.size(), CV_8UC1);
  measure_capture_path("lightweight_success", obs, false, 12, Code::DETECT_RESULT_SUCCESS);
}

TEST(CuboidTopFacePerf, DetailedSuccess)
{
  auto obs = scene(); cuboid(obs, 0.0, 0.0);
  obs.image_boundary = cv::Mat::zeros(obs.heights.size(), CV_8UC1);
  measure_capture_path("detailed_success", obs, true, 12, Code::DETECT_RESULT_SUCCESS);
}

TEST(CuboidTopFacePerf, RepeatedZeroCandidateReplay)
{
  auto obs = scene();
  obs.image_boundary = cv::Mat::zeros(obs.heights.size(), CV_8UC1);
  obs.heights.setTo(std::numeric_limits<double>::quiet_NaN());
  obs.heights.at<double>(120, 120) = 0.03;
  measure_capture_path(
    "repeated_zero_candidate_replay", obs, false, 100,
    Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
}


TEST(CuboidTopFace, NotebookParallelRotatedPair)
{
  const double yaw = 12 * 3.141592653589793 / 180;
  auto obs = scene(); cuboid(obs, 0, 0, yaw); cuboid(
    obs, 0.08 * std::cos(yaw), 0.08 * std::sin(
      yaw), yaw);
  const auto r = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(r.ordered_candidates.size(), 2U) << r.diagnostic_detail;
}


TEST(CuboidTopFace, ContinuousInclinedTopIsRejectedWithoutPoseReconstruction)
{
  auto obs = scene(); cuboid(obs, 0, 0);
  const auto detector_profile = profile();
  add_height_variation(obs, 0.00032, 0, 0);
  const auto result = detect_cuboid_top_face(obs, detector_profile);
  EXPECT_NE(result.result_code, Code::DETECT_RESULT_SUCCESS);
  EXPECT_EQ(result.ordered_candidates.size(), 0U);
  EXPECT_GT(result.rawpart_rejected_by_reason.count("inclined_surface"), 0U);
  EXPECT_TRUE(result.cuboid_hypotheses.empty());
  ASSERT_EQ(result.cuboid_height_patches.size(), 1U);
  EXPECT_GT(result.cuboid_height_patches[0].hypothesis_rejected_by_reason.count("inclined_surface"), 0U);
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  EXPECT_EQ(
    result.disturbance_candidates.front().category,
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_INCLINED_SURFACE);
  EXPECT_NEAR(result.disturbance_candidates.front().center.pose.position.x, 0.0, 0.001);
  EXPECT_NEAR(result.disturbance_candidates.front().center.pose.position.y, 0.0, 0.001);
  EXPECT_NEAR(result.disturbance_candidates.front().observed_height_m, 0.03, 1e-8);
  EXPECT_EQ(result.disturbance_candidates.front().candidate_id, "disturbance-1");
}

TEST(CuboidTopFace, SelectedObjectDoesNotPromoteIsolatedBoundaryResiduals)
{
  auto obs = scene();
  cuboid(obs, 0, 0);
  // Separate raster speckles near both long edges, without object geometry.
  for (int x = 106; x <= 134; x += 4) {
    obs.heights.at<double>(108, x) = 0.031;
    obs.heights.at<double>(132, x) = 0.031;
  }
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(result.ordered_candidates.size(), 1U);
  EXPECT_TRUE(result.disturbance_candidates.empty());
}

TEST(CuboidTopFace, UnresolvedSingleCellDoesNotEstablishDisturbanceGeometry)
{
  auto obs = scene();
  obs.heights.setTo(std::numeric_limits<double>::quiet_NaN());
  obs.heights.at<double>(120, 120) = 0.03;

  const auto result = detect_cuboid_top_face(obs, profile());

  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_TRUE(result.disturbance_candidates.empty());
}

TEST(CuboidTopFace, HorizontalUnmatchedContourRemainsOtherUnresolvedEvidence)
{
  auto obs = scene();
  obs.heights.setTo(std::numeric_limits<double>::quiet_NaN());
  cv::rectangle(obs.heights, {96, 96, 20, 20}, cv::Scalar(0.03), cv::FILLED);
  cv::rectangle(obs.heights, {108, 96, 20, 12}, cv::Scalar(0.03), cv::FILLED);
  auto detector_profile = profile();
  detector_profile.dimensions_m = {0.15, 0.12, 0.10};

  double sum_z = 0;
  size_t observed_cells = 0;
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      const double height = obs.heights.at<double>(row, col);
      if (!std::isfinite(height)) {continue;}
      sum_z += height;
      ++observed_cells;
    }
  }

  const auto result = detect_cuboid_top_face(obs, detector_profile);

  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_TRUE(result.ordered_candidates.empty());
  EXPECT_FALSE(result.rawpart_rejected_by_reason.empty());
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  const auto & disturbance = result.disturbance_candidates.front();
  EXPECT_EQ(
    disturbance.category,
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED);
  // Shoelace centroid of the known L outline (including its raster diagonal),
  // not the cell mean or bounding-box center.
  EXPECT_NEAR(
    disturbance.center.pose.position.x, obs.origin.x + (324691.0 / 2961) * obs.spacing_m, 1e-12);
  EXPECT_NEAR(
    disturbance.center.pose.position.y, obs.origin.y + (309223.0 / 2961) * obs.spacing_m, 1e-12);
  EXPECT_NEAR(
    disturbance.center.pose.position.z, sum_z / observed_cells, 1e-12);
  EXPECT_NEAR(disturbance.observed_height_m, sum_z / observed_cells, 1e-12);
}

TEST(CuboidTopFace, IndependentIsolatedCellsDoNotEstablishDisturbanceGeometry)
{
  auto obs = scene();
  obs.heights.at<double>(80, 80) = 0.03;
  obs.heights.at<double>(145, 145) = 0.06;

  const auto result = detect_cuboid_top_face(obs, profile());

  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_TRUE(result.disturbance_candidates.empty());
}
TEST(CuboidTopFace, InclinesAlongEitherGridAxisAndAcrossBothAxesAreRejectedWithNoise)
{
  for (const auto & gradients : std::array<std::array<double, 2>, 3>{{
      {{0.00032, 0}}, {{0, 0.00032}}, {{0.00012, 0.00012}}}})
  {
    auto obs = scene(); cuboid(obs, 0, 0);
    add_height_variation(obs, gradients[0], gradients[1], 0.00005);
    const auto result = detect_cuboid_top_face(obs, profile());
    EXPECT_NE(result.result_code, Code::DETECT_RESULT_SUCCESS) << gradients[0] << "," << gradients[1];
    EXPECT_TRUE(result.ordered_candidates.empty()) << gradients[0] << "," << gradients[1];
    EXPECT_GT(result.rawpart_rejected_by_reason.count("inclined_surface"), 0U)
      << gradients[0] << "," << gradients[1] << "; " << result.diagnostic_detail;
  }
}
TEST(CuboidTopFace, SmallHeightNoiseRetainsHorizontalTop)
{
  auto obs = scene(); cuboid(obs, 0, 0);
  add_height_variation(obs, 0, 0, 0.00005);
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  ASSERT_EQ(result.ordered_candidates.size(), 1U);
  EXPECT_NEAR(result.ordered_candidates.front().target_pose.pose.position.z, 0.03, 1e-8);
}

TEST(CuboidTopFace, DiscontinuousAdjacentStepIsNotClassifiedAsInclined)
{
  auto obs = scene();
  cuboid(obs, -0.04, 0);
  cuboid(obs, 0.04, 0, 0, 0.03);
  add_height_variation(obs, 0, 0, 0.00005);
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  EXPECT_EQ(result.ordered_candidates.size(), 2U);
  EXPECT_EQ(result.rawpart_rejected_by_reason.count("inclined_surface"), 0U);
  EXPECT_TRUE(std::any_of(result.ordered_candidates.begin(), result.ordered_candidates.end(),
      [](const auto & candidate) {
          return std::abs(candidate.target_pose.pose.position.z - 0.03) <=
                 profile().cuboid.height_tolerance_m;
      }));
  EXPECT_TRUE(std::any_of(result.ordered_candidates.begin(), result.ordered_candidates.end(),
      [](const auto & candidate) {
        return std::abs(candidate.target_pose.pose.position.z - 0.06) < 1e-8;
      }));
}
TEST(CuboidTopFace, InclinedPartWithASeparateHeightStepKeepsIndependentHorizontalCandidate)
{
  auto obs = scene(); cuboid(obs, -0.05, 0); cuboid(obs, 0.05, 0, 0, 0.03);
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      auto & height = obs.heights.at<double>(row, col);
      const double x = obs.origin.x + col * obs.spacing_m;
      if (height > 0 && x < 0) {height += (col - 100) * 0.00032;}
    }
  }
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  ASSERT_EQ(result.ordered_candidates.size(), 1U);
  EXPECT_NEAR(result.ordered_candidates.front().target_pose.pose.position.x, 0.05, 0.005);
  EXPECT_NEAR(result.ordered_candidates.front().target_pose.pose.position.z, 0.06, 1e-8);
  EXPECT_GT(result.rawpart_rejected_by_reason.count("inclined_surface"), 0U);
}
TEST(CuboidTopFace, LocalDiscontinuityDoesNotEraseSlopeEvidenceElsewhereOnCandidate)
{
  auto obs = scene(); cuboid(obs, -0.05, 0); cuboid(obs, 0.05, 0, 0, 0.03);
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      auto & height = obs.heights.at<double>(row, col);
      const double x = obs.origin.x + col * obs.spacing_m;
      if (height <= 0 || x >= 0) {continue;}
      height += (col - 100) * 0.00032;
      if (col == 99) {height += 0.01;}
    }
  }
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  ASSERT_EQ(result.ordered_candidates.size(), 1U);
  EXPECT_NEAR(result.ordered_candidates.front().target_pose.pose.position.x, 0.05, 0.005);
  EXPECT_NEAR(result.ordered_candidates.front().target_pose.pose.position.z, 0.06, 1e-8);
  EXPECT_GT(result.rawpart_rejected_by_reason.count("inclined_surface"), 0U);
}


TEST(CuboidTopFace, StartupSelection)
{
  EXPECT_THROW(
    arm_cell_vision::validate_detector_algorithm("legacy_rectangle"), std::invalid_argument);
  EXPECT_NO_THROW(arm_cell_vision::validate_detector_algorithm("cuboid_top_face"));
  EXPECT_THROW(arm_cell_vision::validate_detector_algorithm(""), std::invalid_argument);
  EXPECT_THROW(
    arm_cell_vision::validate_detector_algorithm(
      "Cuboid_top_face"), std::invalid_argument);
}

TEST(CuboidTopFace, HiddenStackSupportDoesNotDependOnUnrelatedLowerTop)
{
  auto obs = scene(); cuboid(obs, 0, 0, 0, 0.03);
  const auto r = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(r.result_code, Code::DETECT_RESULT_SUCCESS) << r.diagnostic_detail;
  EXPECT_NEAR(r.ordered_candidates.at(0).target_pose.pose.position.z, 0.06, 1e-8);
  EXPECT_NEAR(r.ordered_candidates.at(0).estimated_height_m, 0.03, 1e-8);
}

TEST(CuboidTopFace, ConflictingOwnThicknessInterpretationsStayUnresolved)
{
  auto p = profile(); p.dimensions_m = {0.08, 0.05, 0.051};
  auto obs = scene(); cuboid(obs, 0, 0, 0, 0.07, 0.08, 0.05, 0.051);
  const auto r = detect_cuboid_top_face(obs, p);
  EXPECT_EQ(r.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR) << r.diagnostic_detail;
  EXPECT_TRUE(r.ordered_candidates.empty());
  EXPECT_FALSE(r.disturbance_candidates.empty());
}

TEST(CuboidTopFace, AlternatingNearBinBoundaryHeightsFormOnePhysicalTop)
{
  auto obs = scene(); cuboid(obs, 0, 0);
  for (int row = 0; row < obs.heights.rows; ++row) {
    for (int col = 0; col < obs.heights.cols; ++col) {
      auto & height = obs.heights.at<double>(row, col);
      if (height > 0) {height = row % 2 ? 0.0313 : 0.0312;}
    }
  }
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS) << result.diagnostic_detail;
  EXPECT_EQ(result.cuboid_height_patches.size(), 1U);
  ASSERT_EQ(result.ordered_candidates.size(), 1U);
  EXPECT_EQ(result.rawpart_candidate_count, 1U);
  EXPECT_EQ(result.rawpart_candidate_diagnostics[0].greedy_rank, 0U);
  EXPECT_EQ(result.cuboid_height_patches[0].cell_count,
    static_cast<uint64_t>(cv::countNonZero(obs.heights > 0)));
}

TEST(CuboidTopFace, OpposingClearanceRanksSurroundedCandidateAfterExposedOnes)
{
  auto obs = scene();cuboid(obs, 0, 0);cuboid(obs, -0.17, 0);
  // Lower observed obstacles constrain both opposing pairs without hiding the top.
  cuboid(obs, -0.05, 0, 0, 0, 0.01, 0.08, 0.02);cuboid(obs, 0.05, 0, 0, 0, 0.01, 0.08, 0.02);
  cuboid(obs, 0, -0.04, 0, 0, 0.10, 0.01, 0.02);cuboid(obs, 0, 0.04, 0, 0, 0.10, 0.01, 0.02);
  const auto result = detect_cuboid_top_face(obs, profile());
  ASSERT_EQ(result.ordered_candidates.size(), 2U) << result.diagnostic_detail;
  const auto & last = result.ordered_candidates.back();
  EXPECT_NEAR(last.profile_candidate_features[0], 0, 0.0025);
  EXPECT_NEAR(last.profile_candidate_features[1], 0, 0.0025);
}














// Optional export keeps the exact same controlled input available to the
// untouched notebook and to before/after visualization outside the repository.









}  // namespace

namespace
{
using arm_cell_vision::CuboidHypothesisVisualization;
using namespace arm_cell_vision::cuboid_geometry;
ObservationRegion outline_region(uint32_t id = 1)
{
  ObservationRegion region;
  region.outline_id = id; region.observed_centroid = {0, 0};
  region.observed_boundary = std::make_shared<const std::vector<cv::Point>>(
    std::vector<cv::Point>{{0, 0}, {239, 0}, {239, 239}, {0, 239}});
  region.mean_observed_height = 0.04; region.unresolved = true;
  return region;
}
CuboidHypothesisVisualization outline_hypothesis(
  uint32_t id, double x, double y, double length = 0.04, double width = 0.02)
{
  CuboidHypothesisVisualization h;
  h.candidate_id = id; h.outline_id = 1;
  h.center_x_m = x; h.center_y_m = y;
  h.length_m = length; h.width_m = width;
  h.selection_state = "UNRESOLVED"; h.confidence_grade = "LOW";
  return h;
}
}

TEST(CuboidTopFace, LiveAcquisition20261024PreservesGroundTruthPoseAndCompetingFaceEvidence)
{
  const std::string prefix = std::string(CUBOID_FIXTURE_DIR) +
    "/../occupancy_replay/arm-cell-live-20261024-acq-11";
  CuboidHeightObservation observation{cv::Mat(161, 241, CV_64F), {-0.3, -0.2}, 0.0025,
    cv::Mat(161, 241, CV_8U)};
  std::ifstream grid(prefix + ".height_grid.f64le", std::ios::binary);
  ASSERT_TRUE(grid.read(reinterpret_cast<char *>(observation.heights.data),
    observation.heights.total() * sizeof(double)));
  std::ifstream mask(prefix + ".image_boundary.u8", std::ios::binary);
  ASSERT_TRUE(mask.read(reinterpret_cast<char *>(observation.image_boundary.data),
    observation.image_boundary.total()));

  auto rawpart = profile();
  rawpart.dimensions_m = {0.07, 0.07, 0.08};
  rawpart.dimension_tolerance_m = {0.01, 0.01, 0.01};
  rawpart.workspace_roi_min_m = {-0.3, -0.2, 0.01};
  rawpart.workspace_roi_max_m = {0.3, 0.2, 1.0};
  rawpart.support_contact_tolerance_m = 0.01;
  rawpart.top_surface_geometry_tolerance_m = 0.01;
  rawpart.cuboid.height_tolerance_m = 0.003;
  rawpart.cuboid.line_residual_limit_px = 1.25;

  const auto result = detect_cuboid_top_face(observation, rawpart, true);
  EXPECT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS);
  EXPECT_FALSE(result.ordered_candidates.empty());
  EXPECT_FALSE(result.disturbance_candidates.empty());
  EXPECT_GT(result.rawpart_rejected_by_reason.count("connected_parallel_edge_distance_mismatch"), 0U);
  // Ground Truth is an evaluation oracle only. It must not choose the Runtime
  // candidate or discard a different, still-consistent physical face.
  const auto ground_truth_compatible = std::find_if(result.cuboid_hypotheses.begin(),
    result.cuboid_hypotheses.end(), [](const auto & hypothesis) {
      return std::hypot(hypothesis.center_x_m - 0.21580078,
        hypothesis.center_y_m + 0.01609743) < 0.003 &&
        std::abs(std::remainder(hypothesis.yaw_rad - 1.05716223, std::acos(-1.0))) < 0.04 &&
        std::abs(hypothesis.length_m - 0.08) < 1e-8 &&
        std::abs(hypothesis.width_m - 0.07) < 1e-8 &&
        std::abs(hypothesis.thickness_m - 0.07) < 1e-8 &&
        hypothesis.disposition != "removed_by_deduplication";
    });
  ASSERT_NE(ground_truth_compatible, result.cuboid_hypotheses.end());
  const auto & measured = *ground_truth_compatible;
  EXPECT_EQ(measured.selection_state, "SELECTED");
  EXPECT_EQ(measured.confidence_grade, "MEDIUM");
  // Isaac recorded RawPart 5 at sim time 173.116666667 s, 7.25 s before
  // this stable acquisition. Preserve the original Tray-frame Pose oracle.
  EXPECT_NEAR(measured.center_x_m, 0.21580078, 0.003);
  EXPECT_NEAR(measured.center_y_m, -0.01609743, 0.003);
  EXPECT_NEAR(measured.height_m, 0.07143482, 0.0025);
  EXPECT_NEAR(std::remainder(measured.yaw_rad - 1.05716223, std::acos(-1.0)), 0.0, 0.04);
  EXPECT_FALSE(measured.parallel_edge_distances_m.empty());
  const auto competing = std::find_if(result.cuboid_hypotheses.begin(),
    result.cuboid_hypotheses.end(), [&](const auto & hypothesis) {
      return std::hypot(hypothesis.center_x_m - measured.center_x_m,
        hypothesis.center_y_m - measured.center_y_m) < rawpart.dimensions_m[0] &&
        std::abs(hypothesis.height_m - measured.height_m) < rawpart.cuboid.height_tolerance_m &&
        std::abs(hypothesis.length_m - 0.07) < 1e-8 &&
        std::abs(hypothesis.width_m - 0.07) < 1e-8 &&
        std::abs(hypothesis.thickness_m - 0.08) < 1e-8 &&
        hypothesis.selection_state == "ALTERNATIVE" &&
        hypothesis.disposition != "removed_by_deduplication";
    });
  ASSERT_NE(competing, result.cuboid_hypotheses.end());
  EXPECT_GT(measured.observed_support_cells, competing->observed_support_cells);
  EXPECT_NE(competing->confidence_grade, "LOW");
  EXPECT_FALSE(competing->selection_reason.empty());
  for (const auto & candidate : result.ordered_candidates) {
    EXPECT_NE(candidate.rawpart_selected_candidate, std::to_string(competing->candidate_id));
  }
  std::cout << "LIVE_REPLAY ground_truth_evaluation_only=true center_xy_m=" << measured.center_x_m <<
    "," << measured.center_y_m << " top_m=" << measured.height_m << " yaw_rad=" << measured.yaw_rad <<
    " gt_compatible_id=" << measured.candidate_id << " competing_face_id=" << competing->candidate_id <<
    " classification=MEDIUM result=SUCCESS\n";
}


TEST(CuboidTopFace, Live20261027BoundaryResidualsDoNotBecomeDisturbanceCandidates)
{
  const std::string prefix = std::string(CUBOID_FIXTURE_DIR) +
    "/../occupancy_replay/arm-cell-live-20261027-acq-1";
  CuboidHeightObservation obs{cv::Mat(129, 193, CV_64F), {-0.24, -0.16}, 0.0025,
    cv::Mat(129, 193, CV_8U)};
  std::ifstream grid(prefix + ".height_grid.f64le", std::ios::binary);
  ASSERT_TRUE(grid.read(reinterpret_cast<char *>(obs.heights.data), obs.heights.total() * 8));
  std::ifstream mask(prefix + ".image_boundary.u8", std::ios::binary);
  ASSERT_TRUE(mask.read(reinterpret_cast<char *>(obs.image_boundary.data), obs.heights.total()));
  auto p = profile();
  p.dimensions_m = {0.07, 0.07, 0.08}; p.dimension_tolerance_m = {0.01, 0.01, 0.01};
  p.workspace_roi_min_m = {-0.24, -0.16, 0.01}; p.workspace_roi_max_m = {0.24, 0.16, 1};
  p.support_contact_tolerance_m = 0.01; p.top_surface_geometry_tolerance_m = 0.01;
  p.cuboid.height_tolerance_m = 0.003; p.cuboid.line_residual_limit_px = 1.25;
  const auto result = detect_cuboid_top_face(obs, p, true);
  ASSERT_EQ(result.result_code, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(result.disturbance_regions.size(), result.disturbance_candidates.size());
  for (const auto & region : result.disturbance_regions) {
    EXPECT_GT(region.outline_id, 0U);
    EXPECT_TRUE(std::any_of(result.cuboid_boundaries.begin(), result.cuboid_boundaries.end(),
      [&](const auto & boundary) {return boundary.source_boundary_id == region.source_boundary_id;}));
    EXPECT_TRUE(region.representative_method == "footprint_intersection" ||
      region.representative_method == "single_hypothesis" || region.representative_method == "outline_centroid");
  }
  // The recorded point for Boundary 97 lies outside its observed outline;
  // the nearest cell belongs to Boundary 118 (height about 0.13814 m).
  // Its own 489 observed cells have the independently recorded mean below.
  const auto scoped = std::find_if(result.disturbance_regions.begin(), result.disturbance_regions.end(),
    [](const auto & region) {return region.source_boundary_id == 97 && region.outline_id == 1;});
  if (scoped != result.disturbance_regions.end()) {
    const auto index = static_cast<size_t>(scoped - result.disturbance_regions.begin());
    EXPECT_NEAR(result.disturbance_candidates[index].observed_height_m, 0.06844092788516641, 1e-12);
  } else {
    // A resolved covered object no longer needs a disturbance-only region.
    EXPECT_TRUE(std::any_of(result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
      [](const auto & h) {return h.source_boundary_id == 97 && h.selection_state == "SELECTED";}));
  }
  // Frozen PICK/Occupancy output predates covered candidates. Verify the new
  // semantic invariants instead of freezing changed candidate ordering/counts.
  ASSERT_FALSE(result.ordered_candidates.empty());
  std::set<std::string> ids;
  for (const auto & c : result.ordered_candidates) {
    EXPECT_TRUE(ids.insert(c.rawpart_selected_candidate).second);
    EXPECT_TRUE(c.confidence_grade == "HIGH" || c.confidence_grade == "MEDIUM");

  }

}



TEST(CuboidDisturbanceRegions, CompetingFootprintsUseGeometricIntersectionCentroid)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  DetectionResult result;
  result.cuboid_hypotheses = {outline_hypothesis(1, 0, 0), outline_hypothesis(2, 0.02, 0.01)};
  append_disturbance_regions(result, {outline_region()}, {}, obs, profile());
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  const auto & point = result.disturbance_candidates[0].center.pose.position;
  EXPECT_NEAR(point.x, 0.01, 1e-8); EXPECT_NEAR(point.y, 0.005, 1e-8);
  EXPECT_EQ(result.disturbance_regions[0].representative_method, "footprint_intersection");
  EXPECT_EQ(result.disturbance_regions[0].hypothesis_ids, (std::vector<uint32_t>{1, 2}));
}

TEST(CuboidDisturbanceRegions, SingleHypothesisUsesItsFullInferredFootprintCenter)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  DetectionResult result; result.cuboid_hypotheses = {outline_hypothesis(1, 0.017, -0.021)};
  append_disturbance_regions(result, {outline_region()}, {}, obs, profile());
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  EXPECT_NEAR(result.disturbance_candidates[0].center.pose.position.x, 0.017, 1e-8);
  EXPECT_NEAR(result.disturbance_candidates[0].center.pose.position.y, -0.021, 1e-8);
  EXPECT_EQ(result.disturbance_regions[0].representative_method, "single_hypothesis");
}

TEST(CuboidDisturbanceRegions, DisjointOrInvalidFootprintsFallBackToOriginalOutlineCentroid)
{
  for (const bool invalid : {false, true}) {
    auto obs = scene(); obs.heights.setTo(0.03);
    DetectionResult result;
    result.cuboid_hypotheses = {outline_hypothesis(1, -0.1, 0), outline_hypothesis(2, 0.1, 0)};
    if (invalid) {result.cuboid_hypotheses[1].length_m = std::numeric_limits<double>::quiet_NaN();}
    auto outline = outline_region(); outline.observed_centroid = {0.003, -0.007};
    append_disturbance_regions(result, {outline}, {}, obs, profile());
    ASSERT_EQ(result.disturbance_candidates.size(), 1U);
    EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].center.pose.position.x, 0.003);
    EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].center.pose.position.y, -0.007);
    EXPECT_EQ(result.disturbance_regions[0].representative_method, "outline_centroid");
  }
}

TEST(CuboidDisturbanceRegions, IndependentOutlinesAreNotMergedEvenWithIdenticalFootprints)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  DetectionResult result;
  result.cuboid_hypotheses = {outline_hypothesis(1, 0, 0), outline_hypothesis(2, 0, 0)};
  result.cuboid_hypotheses[1].outline_id = 2;
  append_disturbance_regions(result, {outline_region(1), outline_region(2)}, {}, obs, profile());
  ASSERT_EQ(result.disturbance_candidates.size(), 2U);
  EXPECT_NE(result.disturbance_regions[0].outline_id, result.disturbance_regions[1].outline_id);
}

TEST(CuboidDisturbanceRegions, NoHypothesisUsesObservedCentroidAndLocalHeightOrMeanFallback)
{
  auto obs = scene(); obs.heights.setTo(std::numeric_limits<double>::quiet_NaN());
  auto outline = outline_region(); outline.observed_centroid = {0.01, 0.02};
  for (const bool observed : {false, true}) {
    if (observed) {obs.heights.at<double>(128, 124) = 0.071;}
    DetectionResult result;
    append_disturbance_regions(result, {outline}, {}, obs, profile());
    ASSERT_EQ(result.disturbance_candidates.size(), 1U);
    EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].center.pose.position.z, observed ? 0.071 : 0.04);
    EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].observed_height_m, observed ? 0.071 : 0.04);
  }
  DetectionResult sampled; outline.mean_observed_height.reset();
  append_disturbance_regions(sampled, {outline}, {}, obs, profile());
  ASSERT_EQ(sampled.disturbance_candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(sampled.disturbance_candidates[0].observed_height_m, 0.071);
  obs.heights.setTo(std::numeric_limits<double>::quiet_NaN());
  DetectionResult result;
  append_disturbance_regions(result, {outline}, {}, obs, profile());
  EXPECT_TRUE(result.disturbance_candidates.empty());
}

TEST(CuboidDisturbanceRegions, MixedParentRequiresIndependentUnresolvedPartition)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  auto parent = outline_region();
  auto normal = outline_hypothesis(1, -0.05, 0);
  normal.selection_state = "SELECTED"; normal.confidence_grade = "HIGH"; normal.rectangle_id = 1;
  auto low = outline_hypothesis(2, 0.05, 0); low.rectangle_id = 2;
  auto partition = parent; partition.rectangle_id = 2;
  for (const bool independent : {false, true}) {
    DetectionResult result; result.cuboid_hypotheses = {normal, low};
    partition.independent = independent;
    append_disturbance_regions(result, {parent}, {partition}, obs, profile());
    ASSERT_EQ(result.disturbance_candidates.size(), independent ? 1U : 0U);
    if (independent) {
      EXPECT_EQ(result.disturbance_regions[0].rectangle_id, 2U);
      EXPECT_NEAR(result.disturbance_candidates[0].center.pose.position.x, 0.05, 1e-8);
    }
  }
  DetectionResult result; result.cuboid_hypotheses = {normal, low};
  append_disturbance_regions(result, {parent}, {}, obs, profile());
  EXPECT_TRUE(result.disturbance_candidates.empty());
}

TEST(CuboidDisturbanceRegions, AlternativeAndRejectedDoNotIndependentlySeedCandidates)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  auto region = outline_region(); region.unresolved = false;
  DetectionResult result;
  result.cuboid_hypotheses = {outline_hypothesis(1, 0, 0), outline_hypothesis(2, 0, 0)};
  result.cuboid_hypotheses[0].selection_state = "ALTERNATIVE";
  result.cuboid_hypotheses[0].confidence_grade = "MEDIUM";
  result.cuboid_hypotheses[1].selection_state = "REJECTED";
  append_disturbance_regions(result, {region}, {}, obs, profile());
  EXPECT_TRUE(result.disturbance_candidates.empty());
}

TEST(CuboidDisturbanceRegions, NormalizedProposalsUseOneExistingHypothesis)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  DetectionResult result;
  result.cuboid_hypotheses = {outline_hypothesis(1, 0.02, 0), outline_hypothesis(2, -0.02, 0)};
  result.cuboid_hypotheses[1].deduplicated_to_candidate_id = 1;
  result.cuboid_hypotheses[1].disposition = "removed_by_deduplication";
  append_disturbance_regions(result, {outline_region()}, {}, obs, profile());
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  EXPECT_EQ(result.disturbance_regions[0].representative_method, "single_hypothesis");
  EXPECT_EQ(result.disturbance_regions[0].hypothesis_ids, (std::vector<uint32_t>{1}));
}




TEST(CuboidDisturbanceRegions, RotatedFootprintIntersectionUsesPolygonAreaCentroid)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  DetectionResult result;
  result.cuboid_hypotheses = {
    outline_hypothesis(1, 0, 0, 0.1, 0.1), outline_hypothesis(2, 0.1, 0, 0.2, 0.2)};
  result.cuboid_hypotheses[0].yaw_rad = std::acos(-1.0) / 4;
  append_disturbance_regions(result, {outline_region()}, {}, obs, profile());
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  // The intersection is half a diamond: a triangle with centroid at r/3.
  EXPECT_NEAR(result.disturbance_candidates[0].center.pose.position.x,
    0.1 / (3 * std::sqrt(2.0)), 1e-8);
  EXPECT_NEAR(result.disturbance_candidates[0].center.pose.position.y, 0, 1e-8);
}

TEST(CuboidDisturbanceRegions, ConfirmedNormalizedAliasKeepsItsOriginalPartitionCorrespondence)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  auto normal = outline_hypothesis(1, 0, 0);
  normal.selection_state = "SELECTED"; normal.confidence_grade = "HIGH"; normal.rectangle_id = 7;
  auto alias = normal; alias.candidate_id = 2; alias.rectangle_id = 2;
  alias.deduplicated_to_candidate_id = 1; alias.disposition = "removed_by_deduplication";
  auto partition = outline_region(); partition.rectangle_id = 2;
  DetectionResult result; result.cuboid_hypotheses = {normal, alias};
  append_disturbance_regions(result, {outline_region()}, {partition}, obs, profile());
  EXPECT_TRUE(result.disturbance_candidates.empty());
}

TEST(CuboidDisturbanceRegions, IndependentPartitionOnAnotherPlaneKeepsItsOwnLowHypothesis)
{
  auto obs = scene(); obs.heights.setTo(0.03);
  auto normal = outline_hypothesis(1, -0.05, 0);
  normal.selection_state = "SELECTED"; normal.confidence_grade = "MEDIUM"; normal.rectangle_id = 1;
  auto low = outline_hypothesis(2, 0.05, 0); low.rectangle_id = 2; low.patch_index = 1;
  auto partition = outline_region(); partition.rectangle_id = 2; partition.patch_index = 1;
  DetectionResult result; result.cuboid_hypotheses = {normal, low};
  append_disturbance_regions(result, {outline_region()}, {partition}, obs, profile());
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  EXPECT_EQ(result.disturbance_regions[0].patch_index, 1U);
  EXPECT_EQ(result.disturbance_regions[0].hypothesis_ids, (std::vector<uint32_t>{2}));
}

TEST(CuboidTopFace, ConcaveOutlineCentroidDoesNotBorrowForeignObjectHeight)
{
  auto obs = scene();
  cv::rectangle(obs.heights, {96, 96, 25, 4}, cv::Scalar(0.03), cv::FILLED);
  cv::rectangle(obs.heights, {96, 96, 4, 25}, cv::Scalar(0.03), cv::FILLED);
  // The L's area centroid lies in its empty notch. A separate, higher object
  // at that location must not supply this L-shaped target's height.
  cv::rectangle(obs.heights, {103, 103, 2, 2}, cv::Scalar(0.08), cv::FILLED);
  auto p = profile(); p.dimensions_m = {0.15, 0.12, 0.10};
  const auto result = detect_cuboid_top_face(obs, p);
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  EXPECT_EQ(result.disturbance_regions[0].representative_method, "outline_centroid");
  const auto & candidate = result.disturbance_candidates[0];
  const double expected_xy = obs.origin.x + (83809.0 / 813) * obs.spacing_m;
  EXPECT_NEAR(candidate.center.pose.position.x, expected_xy, 1e-12);
  EXPECT_NEAR(candidate.center.pose.position.y, expected_xy, 1e-12);
  EXPECT_NEAR(candidate.observed_height_m, 0.03, 1e-12);
}

TEST(CuboidDisturbanceRegions, OtherPartitionHeightOutsideTargetUsesMeanOrNoCandidate)
{
  auto obs = scene(); obs.heights.setTo(0.08);
  auto region = outline_region(); region.rectangle_id = 2;
  region.partition_boundary = {{-0.1F, -0.1F}, {0, -0.1F}, {0, 0}, {-0.1F, 0}};
  for (const bool have_mean : {true, false}) {
    if (!have_mean) {region.mean_observed_height.reset();}
    DetectionResult result;
    result.cuboid_hypotheses = {outline_hypothesis(1, 0.05, 0.05)};
    append_disturbance_regions(result, {region}, {}, obs, profile());
    ASSERT_EQ(result.disturbance_candidates.size(), have_mean ? 1U : 0U);
    if (have_mean) {
      EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].observed_height_m, 0.04);
      EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].center.pose.position.x, 0.05);
      EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].center.pose.position.y, 0.05);
    }
  }
}

TEST(CuboidDisturbanceRegions, RoundedCellOutsideObservationDoesNotSupplyHeight)
{
  auto obs = scene(); obs.heights.setTo(0.08);
  auto region = outline_region();
  region.observed_centroid = {0.0009, 0};
  region.partition_boundary = {{-0.001F, -0.01F}, {0.001F, -0.01F},
    {0.001F, 0.01F}, {-0.001F, 0.01F}};
  // At this offset the nearest sample is x=0.0015, outside the partition,
  // although the representative itself is inside it.
  obs.origin.x = -0.301;
  DetectionResult result;
  append_disturbance_regions(result, {region}, {}, obs, profile());
  ASSERT_EQ(result.disturbance_candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].observed_height_m, 0.04);
  EXPECT_DOUBLE_EQ(result.disturbance_candidates[0].center.pose.position.x, 0.0009);
}

TEST(CuboidTopFace, ObservedLowerPlaneResolvesClippedThicknessWithoutAssumingTraySupport)
{
  for (const double support_height : {0.0, 0.04}) {
    auto observation = scene();
    observation.heights.setTo(support_height);
    cuboid(observation, 0, 0, 0, support_height, 0.07, 0.07, 0.08);
    for (int y = 0; y < observation.heights.rows; ++y) {
      for (int x = 0; x < observation.heights.cols; ++x) {
        if (observation.origin.x + x * observation.spacing_m < -0.005) {
          observation.heights.at<double>(y, x) = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }
    auto rawpart = profile();
    rawpart.dimensions_m = {0.07, 0.07, 0.08};
    const auto result = detect_cuboid_top_face(observation, rawpart, true);
    ASSERT_FALSE(result.ordered_candidates.empty()) << result.diagnostic_detail;
    ASSERT_EQ(result.ordered_candidates.size(), 1U);
    EXPECT_EQ(result.ordered_candidates.front().confidence_grade, "MEDIUM");
    EXPECT_NEAR(result.ordered_candidates.at(0).estimated_height_m, 0.08, 1e-6);
  }
}

TEST(CuboidTopFace, MissingOrMixedLowerEvidenceDoesNotResolveClippedThickness)
{
  for (const bool mixed : {false, true}) {
    auto observation = scene();
    cuboid(observation, 0, 0, 0, 0, 0.07, 0.07, 0.08);
    for (int y = 0; y < observation.heights.rows; ++y) {
      for (int x = 0; x < observation.heights.cols; ++x) {
        auto & z = observation.heights.at<double>(y, x);
        if (observation.origin.x + x * observation.spacing_m < -0.005 || (!mixed && z == 0)) {
          z = std::numeric_limits<double>::quiet_NaN();
        } else if (mixed && z == 0 && y < 120) {
          z = 0.01;
        }
      }
    }
    auto rawpart = profile();
    rawpart.dimensions_m = {0.07, 0.07, 0.08};
    const auto result = detect_cuboid_top_face(observation, rawpart, true);
    EXPECT_FALSE(result.has_target_pose) << result.diagnostic_detail;
    EXPECT_EQ(result.result_code, Code::DETECT_RESULT_GEOMETRY_ERROR);
  }
}

TEST(CuboidTopFace, AdjacentRawPartTopIsNotSupportForClippedCandidate)
{
  for (const bool opposed_neighbours : {false, true}) {
    SCOPED_TRACE(opposed_neighbours ? "two disconnected neighbours" : "one adjacent RawPart");
    auto observation = scene();
    // Candidate: 70x70 top, 80mm thick, at top 150mm. The alternative
    // 70mm thickness has bottom 80mm, exactly the neighbouring RawPart top.
    cuboid(observation, 0, 0, 0, 0.07, 0.07, 0.07, 0.08);
    cuboid(observation, 0, 0.0725, 0, 0, 0.07, 0.07, 0.08);
    if (opposed_neighbours) {
      cuboid(observation, 0, -0.0725, 0, 0, 0.07, 0.07, 0.08);
    }
    for (int y = 0; y < observation.heights.rows; ++y) {
      for (int x = 0; x < observation.heights.cols; ++x) {
        auto & z = observation.heights.at<double>(y, x);
        if (observation.origin.x + x * observation.spacing_m < -0.005 || z == 0) {
          z = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }
    auto rawpart = profile();
    rawpart.dimensions_m = {0.07, 0.07, 0.08};
    const auto result = detect_cuboid_top_face(observation, rawpart, true);
    // Check the physical target's square-top interpretation, not unrelated
    // shifted proposals that earlier comparison criteria may discard.
    const auto square = std::find_if(result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
      [](const auto & h) {
        return std::abs(h.height_m - 0.15) < 1e-6 && std::abs(h.thickness_m - 0.08) < 1e-6 &&
               std::hypot(h.center_x_m, h.center_y_m) < 1e-6 &&
               h.disposition != "removed_by_deduplication";
      });
    ASSERT_NE(square, result.cuboid_hypotheses.end());
    EXPECT_EQ(square->selection_state, "UNRESOLVED");

    // Isolate the two otherwise tied thickness interpretations at the existing
    // reconstruction/selection seam, without additional shifted-pose competitors.
    size_t operations = 20000;
    auto map = arm_cell_vision::cuboid_geometry::build_plane_map(observation, rawpart, operations, true);
    auto upper = map.regions.front();
    upper.cells = cv::Mat::zeros(observation.heights.size(), CV_8U);
    for (const auto & pixel : upper.pixels) {upper.cells.at<uint8_t>(pixel) = 255;}
    uint32_t next_id = 1;
    const auto reconstructed = arm_cell_vision::cuboid_geometry::reconstruct_plane(
      upper, 0, next_id, operations, observation, rawpart, true);
    std::vector<arm_cell_vision::cuboid_geometry::Fit> pair;
    for (const auto & fit : reconstructed.fits) {
      const double expected_x = fit.face.thickness == 0.08 ? 0 : -0.005;
      if (std::hypot(fit.center.x - expected_x, fit.center.y) < 1e-6) {pair.push_back(fit);}
    }
    ASSERT_EQ(pair.size(), 2U);
    const auto selections = arm_cell_vision::cuboid_geometry::select_hypotheses(
      pair, observation, rawpart, operations);
    for (const auto & fit : pair) {
      EXPECT_EQ(selections.at(fit.candidate_id).state, "UNRESOLVED") <<
        "thickness=" << fit.face.thickness;
    }
  }
}

TEST(CuboidTopFace, PartialOrMixedLowerObservationsCannotCertifyLocalSupport)
{
  for (const bool mixed : {false, true}) {
    SCOPED_TRACE(mixed ? "two lower levels inside support tolerance" : "only one lower side observed");
    auto observation = scene();
    cuboid(observation, 0, 0, 0, 0, 0.07, 0.07, 0.08);
    for (int y = 0; y < observation.heights.rows; ++y) {
      for (int x = 0; x < observation.heights.cols; ++x) {
        auto & z = observation.heights.at<double>(y, x);
        const double px = observation.origin.x + x * observation.spacing_m;
        const double py = observation.origin.y + y * observation.spacing_m;
        if (px < -0.005 || (!mixed && z == 0 && py < 0.035)) {
          z = std::numeric_limits<double>::quiet_NaN();
        } else if (mixed && z == 0 && py > 0) {
          z = 0.006;
        }
      }
    }
    auto rawpart = profile();
    rawpart.dimensions_m = {0.07, 0.07, 0.08};
    // Two distinct levels can fit inside the wider contact tolerance without
    // constituting one observed plane under the geometry height tolerance.
    rawpart.support_contact_tolerance_m = 0.01;
    size_t operations = 20000;
    auto map = arm_cell_vision::cuboid_geometry::build_plane_map(observation, rawpart, operations, true);
    auto patch = map.regions.front();
    patch.cells = cv::Mat::zeros(observation.heights.size(), CV_8U);
    for (const auto & pixel : patch.pixels) {patch.cells.at<uint8_t>(pixel) = 255;}
    uint32_t next_id = 1;
    const auto reconstructed = arm_cell_vision::cuboid_geometry::reconstruct_plane(
      patch, 0, next_id, operations, observation, rawpart, true);
    ASSERT_FALSE(reconstructed.fits.empty());
    for (const auto & fit : reconstructed.fits) {EXPECT_FALSE(fit.support_height_m);}
    const auto result = detect_cuboid_top_face(observation, rawpart, true);
    EXPECT_FALSE(result.has_target_pose) << result.diagnostic_detail;
  }
}

















TEST(CuboidTopFace, InconsistentSteepFragmentsDoNotBecomeAnInclinedPlane)
{
  auto obs = scene();
  for (int col = 140; col < 160; ++col) {
    const double height = 0.03 + ((col - 140) % 4) * 0.008;
    obs.heights(cv::Rect(col, 100, 1, 20)).setTo(height);
  }
  size_t operations = 20000;
  const auto map = arm_cell_vision::cuboid_geometry::build_plane_map(obs, profile(), operations, false);
  EXPECT_TRUE(std::none_of(map.recovered_inclined_regions.begin(), map.recovered_inclined_regions.end(),
    [](const auto & region) {return region.status == "mixed_or_inclined" && region.plane_fitted;}));
}
