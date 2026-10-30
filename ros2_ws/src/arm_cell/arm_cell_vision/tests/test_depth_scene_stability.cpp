#include <limits>
#include <opencv2/core.hpp>
#include <gtest/gtest.h>
#include "arm_cell_vision/depth_scene_stability.hpp"

namespace
{
using arm_cell_vision::DepthSceneFrame;
using arm_cell_vision::DepthSceneStabilityConfig;
using arm_cell_vision::DepthSceneStabilityMonitor;
using arm_cell_vision::DepthSceneStatus;

DepthSceneFrame frame(int index, const cv::Mat & depth)
{
  DepthSceneFrame result;
  result.stamp_ns = index * 33'000'000LL;
  result.receipt_monotonic_ns = index * 33'000'000LL;
  result.frame_id = "camera_depth_optical_frame";
  result.depth_m = depth;
  return result;
}

cv::Mat stationary_depth()
{
  return cv::Mat(20, 20, CV_32F, cv::Scalar(1.0F));
}
}

TEST(DepthSceneStability, IdenticalDepthFramesNeedFiveObservations)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  const auto depth = stationary_depth();
  for (int i = 1; i <= 4; ++i) {
    EXPECT_NE(monitor.observe(frame(i, depth)).status, DepthSceneStatus::kStable);
  }
  EXPECT_EQ(monitor.observe(frame(5, depth)).status, DepthSceneStatus::kStable);
}

TEST(DepthSceneStability, HorizontalObjectMotionChangesOriginalDepthPixels)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto first = stationary_depth();
  first(cv::Rect(4, 4, 3, 3)).setTo(0.8F);
  monitor.observe(frame(1, first));
  monitor.observe(frame(2, first));
  auto moved = stationary_depth();
  moved(cv::Rect(8, 4, 3, 3)).setTo(0.8F);
  const auto state = monitor.observe(frame(3, moved));
  EXPECT_EQ(state.status, DepthSceneStatus::kUnstable);
  EXPECT_EQ(state.qualifying_observations, 1U);
  EXPECT_EQ(state.previous_comparison.changed_pixels, 18U);
}

TEST(DepthSceneStability, VerticalObjectMotionChangesDepthAtTheSamePixels)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto near = stationary_depth();
  near(cv::Rect(5, 5, 4, 4)).setTo(0.8F);
  monitor.observe(frame(1, near));
  monitor.observe(frame(2, near));
  auto raised = near.clone();
  raised(cv::Rect(5, 5, 4, 4)).setTo(0.75F);
  const auto state = monitor.observe(frame(3, raised));
  EXPECT_EQ(state.status, DepthSceneStatus::kUnstable);
  EXPECT_EQ(state.previous_comparison.changed_pixels, 16U);
}

TEST(DepthSceneStability, DifferenceWithinToleranceKeepsTheCount)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto depth = stationary_depth();
  monitor.observe(frame(1, depth));
  auto within_tolerance = depth.clone();
  within_tolerance(cv::Rect(5, 5, 5, 5)).setTo(0.998F);
  const auto state = monitor.observe(frame(2, within_tolerance));
  EXPECT_EQ(state.qualifying_observations, 2U);
  EXPECT_EQ(state.previous_comparison.changed_pixels, 0U);
}

TEST(DepthSceneStability, DifferenceBeyondToleranceResetsTheCount)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto depth = stationary_depth();
  monitor.observe(frame(1, depth));
  auto changed = depth.clone();
  changed(cv::Rect(5, 5, 5, 5)).setTo(0.99F);
  const auto state = monitor.observe(frame(2, changed));
  EXPECT_EQ(state.status, DepthSceneStatus::kUnstable);
  EXPECT_EQ(state.qualifying_observations, 1U);
  for (const auto count : state.previous_comparison.absolute_difference_histogram) {
    EXPECT_EQ(count, 0U);
  }
}

TEST(DepthSceneStability, ExistingSinglePixelNoiseFixtureStaysWithinDefaultRatio)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto still = stationary_depth();
  monitor.observe(frame(1, still));
  auto one_pixel_changed = still.clone();
  one_pixel_changed.at<float>(1, 1) = 0.95F;
  const auto state = monitor.observe(frame(2, one_pixel_changed));
  EXPECT_EQ(state.qualifying_observations, 2U);
  EXPECT_DOUBLE_EQ(state.previous_comparison.changed_pixel_ratio, 1.0 / 400.0);
}

TEST(DepthSceneStability, InvalidIntermediateFrameBreaksTheRun)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  const auto depth = stationary_depth();
  monitor.observe(frame(1, depth));
  monitor.observe(frame(2, depth));
  auto invalid = depth.clone();
  invalid.setTo(std::numeric_limits<float>::quiet_NaN());
  const auto missing = monitor.observe(frame(3, invalid));
  EXPECT_EQ(missing.status, DepthSceneStatus::kUnobservable);
  EXPECT_EQ(missing.qualifying_observations, 0U);
  EXPECT_EQ(missing.reason, "no_valid_depth_pixels");
  EXPECT_EQ(monitor.observe(frame(4, depth)).qualifying_observations, 1U);
}

TEST(DepthSceneStability, EmptyDepthCannotBecomeStable)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto empty = stationary_depth();
  empty.setTo(0.0F);
  EXPECT_EQ(monitor.observe(frame(1, empty)).status, DepthSceneStatus::kUnobservable);
  EXPECT_EQ(monitor.observe(frame(2, empty)).status, DepthSceneStatus::kUnobservable);
}

TEST(DepthSceneStability, PartialTrayVisibilityCanBecomeStable)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto partial = stationary_depth();
  partial.setTo(std::numeric_limits<float>::quiet_NaN());
  partial(cv::Rect(8, 8, 4, 4)).setTo(1.0F);
  EXPECT_EQ(cv::countNonZero(partial == partial), 16);
  EXPECT_LT(cv::countNonZero(partial == partial) / static_cast<double>(partial.total()), 0.55);
  for (int i = 1; i <= 4; ++i) {
    EXPECT_NE(monitor.observe(frame(i, partial)).status, DepthSceneStatus::kStable);
  }
  const auto state = monitor.observe(frame(5, partial));
  EXPECT_EQ(state.status, DepthSceneStatus::kStable);
  EXPECT_EQ(state.valid_depth_pixels, 16U);
  EXPECT_EQ(state.unobserved_depth_pixels, 384U);
}

TEST(DepthSceneStability, EachComparisonUsesItsOwnComparablePixelDenominator)
{
  DepthSceneStabilityConfig config;
  config.changed_pixel_ratio_tolerance = 1.0;
  DepthSceneStabilityMonitor monitor(config);
  auto baseline = stationary_depth();
  baseline.setTo(std::numeric_limits<float>::quiet_NaN());
  baseline(cv::Rect(0, 0, 4, 4)).setTo(1.0F);
  baseline(cv::Rect(2, 0, 2, 4)).setTo(0.98F);
  monitor.observe(frame(1, baseline));
  auto previous = stationary_depth();
  previous.setTo(std::numeric_limits<float>::quiet_NaN());
  previous(cv::Rect(2, 0, 4, 4)).setTo(1.0F);
  monitor.observe(frame(2, previous));
  auto current = stationary_depth();
  current.setTo(std::numeric_limits<float>::quiet_NaN());
  current(cv::Rect(2, 0, 4, 4)).setTo(1.0F);
  const auto state = monitor.observe(frame(3, current));
  EXPECT_EQ(state.reference_comparison.comparable_pixels, 8U);
  EXPECT_EQ(state.previous_comparison.comparable_pixels, 16U);
  EXPECT_DOUBLE_EQ(state.reference_comparison.changed_pixel_ratio, 1.0);
  EXPECT_DOUBLE_EQ(state.previous_comparison.changed_pixel_ratio, 0.0);
  EXPECT_TRUE(state.comparison_evidence_limited);
  EXPECT_EQ(state.status, DepthSceneStatus::kUnobservable);
}

TEST(DepthSceneStability, NoOverlapBreaksTheRunWithoutInventingEmptyPixels)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  auto left = stationary_depth();
  left.setTo(std::numeric_limits<float>::quiet_NaN());
  left(cv::Rect(0, 0, 4, 4)).setTo(1.0F);
  monitor.observe(frame(1, left));
  auto right = stationary_depth();
  right.setTo(std::numeric_limits<float>::quiet_NaN());
  right(cv::Rect(10, 10, 4, 4)).setTo(1.0F);
  const auto state = monitor.observe(frame(2, right));
  EXPECT_EQ(state.status, DepthSceneStatus::kUnobservable);
  EXPECT_EQ(state.qualifying_observations, 0U);
  EXPECT_EQ(state.reason, "no_comparable_depth_pixels");
}

TEST(DepthSceneStability, DetailedModeRetainsHistogramWithMeterBins)
{
  DepthSceneStabilityConfig config;
  config.detailed_diagnostics_enabled = true;
  DepthSceneStabilityMonitor monitor(config);
  auto still = stationary_depth();
  monitor.observe(frame(1, still));
  auto changed = still.clone();
  changed.at<float>(0, 0) = 0.995F;
  const auto state = monitor.observe(frame(2, changed));
  EXPECT_EQ(state.previous_comparison.absolute_difference_histogram[4], 1U);
  EXPECT_STREQ(arm_cell_vision::depth_difference_histogram_bin_label(4), "[0.003,0.005)");
}

TEST(DepthSceneStability, NonIncreasingTimestampBreaksTheRun)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  const auto still = stationary_depth();
  monitor.observe(frame(1, still));
  auto out_of_order = frame(1, still);
  EXPECT_EQ(monitor.observe(out_of_order).reason, "invalid_or_out_of_order_timestamp");
  EXPECT_EQ(monitor.observe(frame(2, still)).qualifying_observations, 1U);
}

TEST(DepthSceneStability, CameraAndResolutionMustMatchTheReference)
{
  DepthSceneStabilityMonitor monitor(DepthSceneStabilityConfig{});
  const auto depth = stationary_depth();
  monitor.observe(frame(1, depth));
  auto other_camera = frame(2, depth);
  other_camera.frame_id = "other_depth_camera";
  EXPECT_EQ(monitor.observe(other_camera).reason, "invalid_or_incompatible_depth_frame");
  EXPECT_EQ(monitor.observe(frame(3, depth)).qualifying_observations, 1U);

  auto other_resolution = frame(4, cv::Mat(10, 20, CV_32F, cv::Scalar(1.0F)));
  EXPECT_EQ(monitor.observe(other_resolution).reason, "invalid_or_incompatible_depth_frame");
}
