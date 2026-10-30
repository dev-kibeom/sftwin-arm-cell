#ifndef ARM_CELL_VISION__VISION_SNAPSHOT_INTERNAL_HPP_
#define ARM_CELL_VISION__VISION_SNAPSHOT_INTERNAL_HPP_

#include <algorithm>
#include <sstream>
#include <filesystem>
#include <opencv2/imgproc.hpp>
#include <opencv2/freetype.hpp>

#include "arm_cell_vision/vision_detector_snapshot.hpp"

namespace arm_cell_vision::snapshot_detail
{
// BGR styles are shared by the marks and their legend swatches.
inline const cv::Scalar observed_color{245, 245, 245};
inline const cv::Scalar inferred_color{220, 180, 90};
inline const cv::Scalar anchor_color{255, 220, 0};
inline const cv::Scalar unresolved_color{0, 165, 255};
inline const cv::Scalar rejected_color{40, 40, 220};
inline cv::Scalar confidence_color(const std::string & grade)
{
  return grade == "HIGH" ? cv::Scalar(50, 200, 50) :
         grade == "MEDIUM" ? cv::Scalar(230, 120, 30) :
         grade == "LOW" ? cv::Scalar(0, 165, 255) : cv::Scalar(210, 210, 210);
}
inline void evidence_line(
  cv::Mat & image, cv::Point a, cv::Point b,
  const cv::Scalar & color, bool inferred, int thickness = 1)
{
  if (!inferred) {cv::line(image, a, b, color, thickness, cv::LINE_AA); return;}
  const int steps = std::max(1, static_cast<int>(cv::norm(b - a) / 6));
  for (int k = 0; k < steps; k += 2) {
    cv::line(
      image, a + (b - a) * (static_cast<double>(k) / steps),
      a + (b - a) * (static_cast<double>(std::min(k + 1, steps)) / steps),
      color, thickness, cv::LINE_AA);
  }
}
inline cv::Ptr<cv::freetype::FreeType2> legend_font()
{
  // OpenCV Hershey fonts cannot render Hangul. Reuse the installed OpenCV
  // FreeType module; a readable English fallback keeps font absence non-fatal.
  static thread_local const auto font = []() {
      cv::Ptr<cv::freetype::FreeType2> value;
      for (const auto * path : {"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
          "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"})
      {
        if (!std::filesystem::exists(path)) {continue;}
        try {
          value = cv::freetype::createFreeType2(); value->loadFontData(path, 0); break;
        } catch (const cv::Exception &) {
          value.release();
        }
      }
      return value;
    }();
  return font;
}
inline void legend_text(
  cv::Mat & image, const std::string & korean,
  const std::string & fallback, cv::Point origin)
{
  const auto font = legend_font();
  if (font) {
    font->putText(image, korean, origin, 16, observed_color, -1, cv::LINE_AA, true);
  } else {
    cv::putText(
      image, fallback, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4,
      observed_color, 1, cv::LINE_AA);
  }
}
inline void append_legend(
  cv::Mat & image,
  const std::vector<std::pair<std::string, std::string>> & lines)
{
  if (lines.empty()) {return;}
  const int top = image.rows;
  int width = image.cols;
  const auto font = legend_font();
  for (const auto & line : lines) {
    int baseline = 0;
    const auto size = font ? font->getTextSize(line.first, 16, -1, &baseline) :
      cv::getTextSize(line.second, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    width = std::max(width, size.width + 24);
  }
  cv::copyMakeBorder(
    image, image, 0, static_cast<int>(lines.size()) * 25 + 10,
    0, width - image.cols, cv::BORDER_CONSTANT, cv::Scalar(20, 20, 20));
  for (size_t i = 0; i < lines.size(); ++i) {
    legend_text(image, lines[i].first, lines[i].second, {12, top + 22 + static_cast<int>(i) * 25});
  }
}
const char * event_name(uint8_t kind);
const char * result_code_name(uint8_t code);
std::string json_escape(const std::string & value);
bool is_renderable_presentation_rgb(const sensor_msgs::msg::Image & image);
void append_depth_validation_json(
  std::ostringstream & json, const SupportCalibrationSnapshot & calibration);
void append_observation_identity_json(
  std::ostringstream & json, const DetectionInput & input, const char * depth_key);
cv::Mat rotate_tray_image_to_rgb_orientation(
  const cv::Mat & image, const VisionDetectorSnapshotJob & job,
  const cv::Scalar & border_color);
std::string geometry_plane_annotation(size_t id, const std::string & status);
// Empty means insufficient space or collision; reservations never alter diagnostic evidence.
cv::Rect reserve_geometry_label(
  const std::string & text, cv::Point center, const cv::Rect & available,
  std::vector<cv::Rect> & occupied);
// Outputs use the existing Job containers so partial render/encoding failure
// retains the same evidence for the output boundary's failure handling.
void render_geometry(
  const VisionDetectorSnapshotJob & job,
  std::vector<VisionDetectorSnapshotJob::CuboidStageImage> & images);
void render_scene_stability(
  const VisionDetectorSnapshotJob & job, std::vector<uint8_t> & png);
}  // namespace arm_cell_vision::snapshot_detail

#endif  // ARM_CELL_VISION__VISION_SNAPSHOT_INTERNAL_HPP_
