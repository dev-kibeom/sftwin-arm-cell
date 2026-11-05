#ifndef ARM_CELL_VISION__DEPTH_SCENE_STABILITY_HPP_
#define ARM_CELL_VISION__DEPTH_SCENE_STABILITY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <opencv2/core.hpp>

namespace arm_cell_vision
{

enum class DepthSceneStatus : uint8_t {kUnobservable, kUnstable, kStable};

constexpr size_t kDepthDifferenceHistogramBinCount = 16;

struct DepthSceneFrame
{
  int64_t stamp_ns = 0;
  int64_t receipt_monotonic_ns = 0;
  std::string frame_id;
  cv::Mat depth_m;  // CV_32F original camera pixel grid, metres; NaN/Inf/zero are unobserved.
};

struct DepthSceneStabilityConfig
{
  uint32_t stable_observation_count = 5;
  double depth_difference_tolerance_m = 0.003;
  double changed_pixel_ratio_tolerance = 0.01;
  bool detailed_diagnostics_enabled = false;
};

struct DepthSceneComparison
{
  uint64_t comparable_pixels = 0;
  uint64_t changed_pixels = 0;
  double changed_pixel_ratio = 0.0;
  std::array<uint64_t, kDepthDifferenceHistogramBinCount> absolute_difference_histogram{};
};

struct DepthSceneStabilityState
{
  DepthSceneStatus status = DepthSceneStatus::kUnobservable;
  uint32_t qualifying_observations = 0;
  uint32_t required_observations = 5;
  uint64_t valid_depth_pixels = 0;
  uint64_t unobserved_depth_pixels = 0;
  bool comparison_evidence_limited = false;
  DepthSceneComparison reference_comparison;
  DepthSceneComparison previous_comparison;
  double depth_difference_tolerance_m = 0.0;
  double changed_pixel_ratio_tolerance = 0.0;
  std::string reason = "awaiting_observation";
  std::string last_reset_reason;
};

class DepthSceneStabilityMonitor
{
public:
  explicit DepthSceneStabilityMonitor(DepthSceneStabilityConfig config);
  DepthSceneStabilityState observe(const DepthSceneFrame & frame);
  void reset();

private:
  DepthSceneStabilityConfig config_;
  DepthSceneStabilityState state_;
  cv::Mat reference_depth_m_;
  cv::Mat previous_depth_m_;
  std::string frame_id_;
  int64_t previous_stamp_ns_ = 0;
  int64_t previous_receipt_ns_ = 0;
};

const char * depth_scene_status_name(DepthSceneStatus status);
const char * depth_difference_histogram_bin_label(size_t index);

}  // namespace arm_cell_vision

#endif
