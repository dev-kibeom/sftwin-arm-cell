#include "arm_cell_vision/depth_scene_stability.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arm_cell_vision
{
namespace
{

constexpr std::array<double, kDepthDifferenceHistogramBinCount - 1> kHistogramUpperBounds{
  0.0005, 0.001, 0.002, 0.003, 0.005, 0.01, 0.02, 0.05,
  0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0};

bool valid_depth(float value)
{
  return std::isfinite(value) && value > 0.0F;
}

DepthSceneComparison compare_depth(
  const cv::Mat & current, const cv::Mat & earlier, double tolerance_m, bool collect_histogram)
{
  DepthSceneComparison comparison;
  for (int y = 0; y < current.rows; ++y) {
    const float * now = current.ptr<float>(y);
    const float * before = earlier.ptr<float>(y);
    for (int x = 0; x < current.cols; ++x) {
      if (!valid_depth(now[x]) || !valid_depth(before[x])) {continue;}
      ++comparison.comparable_pixels;
      const double difference = std::abs(static_cast<double>(now[x]) - before[x]);
      if (difference > tolerance_m) {++comparison.changed_pixels;}
      if (collect_histogram) {
        size_t bin = 0;
        while (bin < kHistogramUpperBounds.size() && difference >= kHistogramUpperBounds[bin]) {
          ++bin;
        }
        ++comparison.absolute_difference_histogram[bin];
      }
    }
  }
  if (comparison.comparable_pixels != 0) {
    comparison.changed_pixel_ratio = static_cast<double>(comparison.changed_pixels) /
      static_cast<double>(comparison.comparable_pixels);
  }
  return comparison;
}

}  // namespace

DepthSceneStabilityMonitor::DepthSceneStabilityMonitor(DepthSceneStabilityConfig config)
: config_(config)
{
  if (config_.stable_observation_count == 0 ||
    !std::isfinite(config_.depth_difference_tolerance_m) ||
    config_.depth_difference_tolerance_m < 0.0 ||
    !std::isfinite(config_.changed_pixel_ratio_tolerance) ||
    config_.changed_pixel_ratio_tolerance < 0.0 ||
    config_.changed_pixel_ratio_tolerance > 1.0)
  {
    throw std::invalid_argument("invalid depth scene stability configuration");
  }
}

void DepthSceneStabilityMonitor::reset()
{
  state_ = DepthSceneStabilityState{};
  reference_depth_m_.release();
  previous_depth_m_.release();
  frame_id_.clear();
  previous_stamp_ns_ = previous_receipt_ns_ = 0;
}

DepthSceneStabilityState DepthSceneStabilityMonitor::observe(const DepthSceneFrame & frame)
{
  const auto last_reset_reason = state_.last_reset_reason;
  const auto qualifying_observations = state_.qualifying_observations;
  state_ = DepthSceneStabilityState{};
  state_.last_reset_reason = last_reset_reason;
  state_.qualifying_observations = qualifying_observations;
  state_.required_observations = config_.stable_observation_count;
  state_.depth_difference_tolerance_m = config_.depth_difference_tolerance_m;
  state_.changed_pixel_ratio_tolerance = config_.changed_pixel_ratio_tolerance;

  const bool has_compatible_depth = !frame.depth_m.empty() && frame.depth_m.type() == CV_32F &&
    (reference_depth_m_.empty() ||
    (frame.depth_m.size() == reference_depth_m_.size() && frame.frame_id == frame_id_));
  if (!has_compatible_depth) {
    reset();
    state_.required_observations = config_.stable_observation_count;
    state_.depth_difference_tolerance_m = config_.depth_difference_tolerance_m;
    state_.changed_pixel_ratio_tolerance = config_.changed_pixel_ratio_tolerance;
    state_.reason = "invalid_or_incompatible_depth_frame";
    state_.last_reset_reason = state_.reason;
    return state_;
  }
  if (frame.stamp_ns <= 0 || frame.receipt_monotonic_ns <= 0 ||
    (previous_stamp_ns_ != 0 && (frame.stamp_ns <= previous_stamp_ns_ ||
    frame.receipt_monotonic_ns <= previous_receipt_ns_)))
  {
    reset();
    state_.required_observations = config_.stable_observation_count;
    state_.depth_difference_tolerance_m = config_.depth_difference_tolerance_m;
    state_.changed_pixel_ratio_tolerance = config_.changed_pixel_ratio_tolerance;
    state_.reason = "invalid_or_out_of_order_timestamp";
    state_.last_reset_reason = state_.reason;
    return state_;
  }

  const auto current_valid = [&frame](int y, int x) {
      return valid_depth(frame.depth_m.ptr<float>(y)[x]);
    };
  const uint64_t total_pixels = static_cast<uint64_t>(frame.depth_m.total());
  for (int y = 0; y < frame.depth_m.rows; ++y) {
    for (int x = 0; x < frame.depth_m.cols; ++x) {
      if (current_valid(y, x)) {++state_.valid_depth_pixels;}
    }
  }
  state_.unobserved_depth_pixels = total_pixels - state_.valid_depth_pixels;
  if (state_.valid_depth_pixels == 0) {
    reset();
    state_.required_observations = config_.stable_observation_count;
    state_.depth_difference_tolerance_m = config_.depth_difference_tolerance_m;
    state_.changed_pixel_ratio_tolerance = config_.changed_pixel_ratio_tolerance;
    state_.unobserved_depth_pixels = total_pixels;
    state_.reason = "no_valid_depth_pixels";
    state_.last_reset_reason = state_.reason;
    return state_;
  }

  const bool collect_histogram = config_.detailed_diagnostics_enabled;
  if (reference_depth_m_.empty()) {
    reference_depth_m_ = frame.depth_m.clone();
    previous_depth_m_ = frame.depth_m.clone();
    frame_id_ = frame.frame_id;
    state_.qualifying_observations = 1;
    state_.reason = "awaiting_stable_observations";
  } else {
    state_.reference_comparison = compare_depth(
      frame.depth_m, reference_depth_m_, config_.depth_difference_tolerance_m,
      collect_histogram);
    state_.previous_comparison = compare_depth(
      frame.depth_m, previous_depth_m_, config_.depth_difference_tolerance_m,
      collect_histogram);
    const auto reference_count = state_.reference_comparison.comparable_pixels;
    const auto previous_count = state_.previous_comparison.comparable_pixels;
    state_.comparison_evidence_limited = reference_count < state_.valid_depth_pixels ||
      previous_count < state_.valid_depth_pixels;
    if (reference_count == 0 || previous_count == 0) {
      const auto valid_pixels = state_.valid_depth_pixels;
      const auto unobserved_pixels = state_.unobserved_depth_pixels;
      const auto reference_comparison = state_.reference_comparison;
      const auto previous_comparison = state_.previous_comparison;
      const auto evidence_limited = state_.comparison_evidence_limited;
      reset();
      state_.required_observations = config_.stable_observation_count;
      state_.depth_difference_tolerance_m = config_.depth_difference_tolerance_m;
      state_.changed_pixel_ratio_tolerance = config_.changed_pixel_ratio_tolerance;
      state_.valid_depth_pixels = valid_pixels;
      state_.unobserved_depth_pixels = unobserved_pixels;
      state_.reference_comparison = reference_comparison;
      state_.previous_comparison = previous_comparison;
      state_.comparison_evidence_limited = evidence_limited;
      state_.reason = "no_comparable_depth_pixels";
      state_.last_reset_reason = state_.reason;
      return state_;
    }
    const bool changed =
      state_.reference_comparison.changed_pixel_ratio > config_.changed_pixel_ratio_tolerance ||
      state_.previous_comparison.changed_pixel_ratio > config_.changed_pixel_ratio_tolerance;
    if (changed) {
      reference_depth_m_ = frame.depth_m.clone();
      state_.qualifying_observations = 1;
      state_.status = DepthSceneStatus::kUnstable;
      state_.reason = "depth_change_reset_consecutive_observations";
      state_.last_reset_reason = state_.reason;
    } else {
      state_.qualifying_observations = std::min(
        state_.qualifying_observations + 1, config_.stable_observation_count);
      state_.status = state_.qualifying_observations >= config_.stable_observation_count ?
        DepthSceneStatus::kStable : DepthSceneStatus::kUnobservable;
      state_.reason = state_.status == DepthSceneStatus::kStable ? "depth_scene_stable" :
        "awaiting_stable_observations";
    }
    previous_depth_m_ = frame.depth_m.clone();
  }
  previous_stamp_ns_ = frame.stamp_ns;
  previous_receipt_ns_ = frame.receipt_monotonic_ns;
  return state_;
}

const char * depth_scene_status_name(DepthSceneStatus status)
{
  switch (status) {
    case DepthSceneStatus::kStable: return "STABLE";
    case DepthSceneStatus::kUnstable: return "UNSTABLE";
    case DepthSceneStatus::kUnobservable: return "UNOBSERVABLE";
  }
  return "UNOBSERVABLE";
}

const char * depth_difference_histogram_bin_label(size_t index)
{
  static constexpr std::array<const char *, kDepthDifferenceHistogramBinCount> labels{
    "[0,0.0005)", "[0.0005,0.001)", "[0.001,0.002)", "[0.002,0.003)",
    "[0.003,0.005)", "[0.005,0.01)", "[0.01,0.02)", "[0.02,0.05)",
    "[0.05,0.1)", "[0.1,0.2)", "[0.2,0.5)", "[0.5,1)",
    "[1,2)", "[2,5)", "[5,10)", "[10,inf)"};
  return index < labels.size() ? labels[index] : "invalid";
}

}  // namespace arm_cell_vision
