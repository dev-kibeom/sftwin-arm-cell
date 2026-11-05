#include "vision_snapshot_internal.hpp"

#include <algorithm>
#include <array>
#include <iomanip>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace arm_cell_vision
{
namespace snapshot_detail
{
void render_scene_stability(
  const VisionDetectorSnapshotJob & job, std::vector<uint8_t> & png)
{
  if (job.depth_scene_diagnostics_enabled && !job.depth_scene_samples.empty()) {
    const cv::Scalar baseline_color(230, 120, 30), previous_color(40, 170, 245);
    const cv::Scalar small_difference_color(80, 205, 100), large_difference_color(60, 155, 230);
    const int panel_width = 430;
    const int panel_height = 118;
    const int header_height = 126;
    cv::Mat panorama(header_height + 5 * panel_height, 2 * panel_width, CV_8UC3,
      cv::Scalar(24, 24, 24));
    cv::putText(panorama,
      std::string("Final ") + depth_scene_status_name(job.depth_scene_stability.status),
      {6, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    cv::putText(panorama,
      std::string("Recent observations: ") + std::to_string(job.depth_scene_samples.size()) +
      " / 5 | consecutive valid: " + std::to_string(job.depth_scene_stability.qualifying_observations) +
      "/" + std::to_string(job.depth_scene_stability.required_observations),
      {6, 39}, cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);
    const size_t count = job.depth_scene_samples.size();
    const int chart_left = 34, chart_right = 2 * panel_width - 12;
    const int chart_top = 54, chart_bottom = 112;
    cv::line(panorama, {chart_left, chart_top}, {chart_left, chart_bottom}, cv::Scalar(100, 100, 100));
    cv::line(panorama, {chart_left, chart_bottom}, {chart_right, chart_bottom}, cv::Scalar(100, 100, 100));
    cv::putText(panorama, "Changed pixel ratio: baseline", {chart_left + 5, chart_top + 10},
      cv::FONT_HERSHEY_SIMPLEX, 0.28, baseline_color, 1, cv::LINE_AA);
    cv::putText(panorama, "previous", {chart_left + 195, chart_top + 10},
      cv::FONT_HERSHEY_SIMPLEX, 0.28, previous_color, 1, cv::LINE_AA);
    const auto chart_point = [&](size_t observation, double ratio) {
        const int x = count <= 1 ? (chart_left + chart_right) / 2 : chart_left +
          static_cast<int>((chart_right - chart_left) * observation / (count - 1));
        const int y = chart_bottom - static_cast<int>(std::clamp(ratio, 0.0, 1.0) * (chart_bottom - chart_top - 4));
        return cv::Point(x, y);
      };
    std::vector<std::vector<cv::Point>> baseline_segments, previous_segments;
    std::vector<cv::Point> baseline_segment, previous_segment;
    for (size_t observation = 0; observation < count; ++observation) {
      const auto & state = job.depth_scene_samples[observation].state;
      if (state.reference_comparison.comparable_pixels > 0) {
        baseline_segment.push_back(chart_point(observation, state.reference_comparison.changed_pixel_ratio));
      } else if (!baseline_segment.empty()) {
        baseline_segments.push_back(std::move(baseline_segment));
        baseline_segment.clear();
      }
      if (state.previous_comparison.comparable_pixels > 0) {
        previous_segment.push_back(chart_point(observation, state.previous_comparison.changed_pixel_ratio));
      } else if (!previous_segment.empty()) {
        previous_segments.push_back(std::move(previous_segment));
        previous_segment.clear();
      }
    }
    if (!baseline_segment.empty()) {baseline_segments.push_back(std::move(baseline_segment));}
    if (!previous_segment.empty()) {previous_segments.push_back(std::move(previous_segment));}
    const auto draw_series = [&](const std::vector<std::vector<cv::Point>> & segments,
        const cv::Scalar & color) {
        for (const auto & segment : segments) {
          if (segment.size() > 1) {cv::polylines(panorama, segment, false, color, 2, cv::LINE_AA);}
          for (const auto & point : segment) {cv::circle(panorama, point, 2, color, cv::FILLED);}
        }
      };
    draw_series(baseline_segments, baseline_color);
    draw_series(previous_segments, previous_color);
    size_t panel_index = 0;
    bool small_histogram_drawn = false, large_histogram_drawn = false;
    for (size_t observation = 0; observation < count && panel_index < 10; ++observation) {
      const auto & sample = job.depth_scene_samples[observation];
      const std::array<std::pair<const char *, const DepthSceneComparison *>, 2> comparisons{{
        {"baseline", &sample.state.reference_comparison},
        {"previous", &sample.state.previous_comparison}}};
      for (const auto & entry : comparisons) {
        const auto & comparison = *entry.second;
        if (comparison.comparable_pixels == 0) {continue;}
        const int x = static_cast<int>(panel_index % 2) * panel_width;
        const int y = header_height + static_cast<int>(panel_index / 2) * panel_height;
        const int plot_x = x + 12;
        const int plot_y = y + 25;
        const int plot_width = panel_width - 24;
        const int plot_height = 66;
        cv::line(panorama, {plot_x, plot_y + plot_height},
          {plot_x + plot_width, plot_y + plot_height}, cv::Scalar(150, 150, 150));
        const auto maximum = std::max<uint64_t>(1, *std::max_element(
          comparison.absolute_difference_histogram.begin(),
          comparison.absolute_difference_histogram.end()));
        for (size_t bin = 0; bin < comparison.absolute_difference_histogram.size(); ++bin) {
          const int bar_width = plot_width / static_cast<int>(kDepthDifferenceHistogramBinCount);
          const int bar_height = static_cast<int>(plot_height *
            static_cast<double>(comparison.absolute_difference_histogram[bin]) / maximum);
          if (bar_height > 0) {
            if (bin >= 4) {large_histogram_drawn = true;} else {small_histogram_drawn = true;}
          }
          cv::rectangle(panorama,
            cv::Rect(plot_x + static_cast<int>(bin) * bar_width,
              plot_y + plot_height - bar_height, std::max(1, bar_width - 2), bar_height),
            bin >= 4 ? large_difference_color : small_difference_color, cv::FILLED);
        }
        std::ostringstream title;
        title << "O" << observation + 1 << " " << std::fixed << std::setprecision(3) <<
          sample.stamp_ns / 1.0e9 << "s " << entry.first << " | n=" <<
          comparison.comparable_pixels << " changed=" << comparison.changed_pixels <<
          " ratio=" << std::setprecision(3) << comparison.changed_pixel_ratio <<
          " peak=" << maximum << " px/bin";
        cv::putText(panorama, title.str(), {x + 8, y + 16}, cv::FONT_HERSHEY_SIMPLEX,
          0.31, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
        cv::putText(panorama, "0", {plot_x, plot_y + plot_height + 14},
          cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::putText(panorama, "3 mm", {plot_x + 4 * plot_width / 16, plot_y + plot_height + 14},
          cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::putText(panorama, "50 mm", {plot_x + 8 * plot_width / 16, plot_y + plot_height + 14},
          cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        cv::putText(panorama, ">=10 m", {plot_x + 14 * plot_width / 16,
          plot_y + plot_height + 14}, cv::FONT_HERSHEY_SIMPLEX, 0.25,
          cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        ++panel_index;
      }
    }
    // Display the exact retained comparison inputs without repeating any
    // stability calculation. All frames share one depth color scale.
    double minimum = std::numeric_limits<double>::infinity(), maximum = 0;
    for (const auto & sample : job.depth_scene_samples) {
      const auto & depth = sample.observed_depth_m;
      if (depth.type() != CV_32FC1) {continue;}
      for (int y = 0; y < depth.rows; ++y) for (int x = 0; x < depth.cols; ++x) {
        const double value = depth.at<float>(y, x);
        if (std::isfinite(value) && value > 0) {minimum = std::min(minimum, value); maximum = std::max(maximum, value);}
      }
    }
    const int scene_top = panorama.rows;
    cv::copyMakeBorder(panorama, panorama, 0, 170, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(24, 24, 24));
    const int scene_width = panorama.cols / static_cast<int>(count);
    for (size_t i = 0; i < count; ++i) {
      const auto & depth = job.depth_scene_samples[i].observed_depth_m;
      const int left = static_cast<int>(i) * scene_width;
      cv::putText(panorama, "O" + std::to_string(i + 1), {left + 6, scene_top + 20},
        cv::FONT_HERSHEY_SIMPLEX, 0.45, observed_color, 1, cv::LINE_AA);
      if (depth.empty() || depth.type() != CV_32FC1) {
        cv::putText(panorama, "Input not retained", {left + 6, scene_top + 70},
          cv::FONT_HERSHEY_SIMPLEX, 0.35, observed_color, 1, cv::LINE_AA);
        continue;
      }
      cv::Mat normalized(depth.size(), CV_8UC1, cv::Scalar(0));
      cv::Mat valid(depth.size(), CV_8UC1, cv::Scalar(0));
      for (int y = 0; y < depth.rows; ++y) for (int x = 0; x < depth.cols; ++x) {
        const double value = depth.at<float>(y, x);
        if (!std::isfinite(value) || value <= 0) {continue;}
        valid.at<uint8_t>(y, x) = 255;
        normalized.at<uint8_t>(y, x) = maximum > minimum ?
          cv::saturate_cast<uint8_t>(255 * (value - minimum) / (maximum - minimum)) : 128;
      }
      cv::Mat scene;
      cv::applyColorMap(normalized, scene, cv::COLORMAP_TURBO);
      scene.setTo(cv::Scalar(55, 55, 55), valid == 0);
      const double scale = std::min(static_cast<double>(scene_width - 12) / scene.cols, 135.0 / scene.rows);
      cv::resize(scene, scene, {}, scale, scale, cv::INTER_NEAREST);
      scene.copyTo(panorama(cv::Rect(left + 6, scene_top + 27, scene.cols, scene.rows)));
    }
    append_legend(panorama, {
      {"O=실제 Depth 관측 순서; 장면 색=공통 Depth 범위, 회색=무효/미관측; 입력 미수집 시 장면 생성 안 함",
        "O=actual depth order; scene colors=shared depth range, gray=invalid/unobserved; missing inputs stay missing"}});
    std::vector<std::pair<std::string, cv::Scalar>> swatches;
    if (!baseline_segments.empty()) {swatches.push_back({"Baseline: 실선/점=비교 가능한 관측", baseline_color});}
    if (!previous_segments.empty()) {swatches.push_back({"Previous: 실선/점=비교 가능한 관측", previous_color});}
    if (small_histogram_drawn) {swatches.push_back({"Histogram: 막대=Depth 차이 <3 mm", small_difference_color});}
    if (large_histogram_drawn) {swatches.push_back({"Histogram: 막대=Depth 차이 >=3 mm", large_difference_color});}
    for (const auto & entry : swatches) {
      const int top = panorama.rows;
      const auto fallback = entry.first.substr(0, entry.first.find(':')) +
        (entry.first.find("Histogram") == 0 ?
        (entry.first.find("<3") != std::string::npos ? " depth difference <3 mm" : " depth difference >=3 mm") :
        " comparable observations: line/dot");
      append_legend(panorama, {{entry.first, fallback}});
      cv::line(panorama, {panorama.cols - 90, top + 15}, {panorama.cols - 40, top + 15}, entry.second, 3);
    }
    cv::imencode(".png", panorama, png);
  }
}

}  // namespace snapshot_detail

}  // namespace arm_cell_vision
