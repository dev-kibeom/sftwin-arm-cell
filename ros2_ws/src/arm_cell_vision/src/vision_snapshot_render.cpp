#include "vision_snapshot_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <optional>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace arm_cell_vision
{
using snapshot_detail::event_name;
using snapshot_detail::is_renderable_presentation_rgb;
using snapshot_detail::rotate_tray_image_to_rgb_orientation;

namespace
{
cv::Mat rgb_mat(const sensor_msgs::msg::Image & image)
{
  if (!is_renderable_presentation_rgb(image)) {return {};}
  cv::Mat view(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3,
    const_cast<uint8_t *>(image.data.data()), image.step);
  cv::Mat bgr;
  cv::cvtColor(view, bgr, cv::COLOR_RGB2BGR);
  return bgr;
}

cv::Scalar rgb_color(const std::string & value, const cv::Scalar & fallback)
{
  if (value.size() != 7 || value[0] != '#') {return fallback;}
  try {
    const auto red = std::stoi(value.substr(1, 2), nullptr, 16);
    const auto green = std::stoi(value.substr(3, 2), nullptr, 16);
    const auto blue = std::stoi(value.substr(5, 2), nullptr, 16);
    return cv::Scalar(blue, green, red);
  } catch (const std::exception &) {
    return fallback;
  }
}

std::optional<cv::Point> disturbance_center_pixel(
  const VisionDetectorSnapshotJob & job,
  const arm_cell_interfaces::msg::DisturbanceCandidate & candidate,
  const cv::Mat & image)
{
  const auto * input = job.observation_input();
  if (!input || image.empty() || candidate.center.header.frame_id != "base_link" ||
    candidate.center.header.stamp.sec != input->depth.header.stamp.sec ||
    candidate.center.header.stamp.nanosec != input->depth.header.stamp.nanosec ||
    input->optical_to_base.header.frame_id != "base_link" ||
    input->optical_to_base.child_frame_id != input->depth.header.frame_id ||
    input->depth.header.frame_id != input->rgb.header.frame_id ||
    input->camera_info.header.frame_id != input->rgb.header.frame_id ||
    input->camera_info.width != input->rgb.width || input->camera_info.height != input->rgb.height)
  {return std::nullopt;}

  const auto & point = candidate.center.pose.position;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return std::nullopt;
  }
  const auto & camera_info = input->camera_info;
  if (std::any_of(camera_info.k.begin(), camera_info.k.end(),
      [](double value) {return !std::isfinite(value);}) ||
    camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 ||
    std::abs(camera_info.k[1]) > 1e-12 || std::abs(camera_info.k[3]) > 1e-12 ||
    std::abs(camera_info.k[6]) > 1e-12 || std::abs(camera_info.k[7]) > 1e-12 ||
    std::abs(camera_info.k[8] - 1.0) > 1e-12 ||
    std::any_of(camera_info.d.begin(), camera_info.d.end(),
      [](double coefficient) {return !std::isfinite(coefficient);}))
  {return std::nullopt;}
  const auto model = camera_info.distortion_model;
  const bool pinhole_model = model == "plumb_bob" || model == "rational_polynomial";
  const bool fisheye_model = model == "equidistant";
  if ((!pinhole_model && !fisheye_model) ||
    (fisheye_model && camera_info.d.size() != 4) ||
    (model == "plumb_bob" && camera_info.d.size() != 4 && camera_info.d.size() != 5) ||
    (model == "rational_polynomial" && camera_info.d.size() != 8))
  {return std::nullopt;}

  const auto & transform = input->optical_to_base;
  const auto & rotation = transform.transform.rotation;
  const auto & translation = transform.transform.translation;
  const double quaternion_norm = std::sqrt(rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w);
  if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-3 ||
    !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
    !std::isfinite(translation.z))
  {return std::nullopt;}

  const tf2::Quaternion base_from_camera_rotation(
    rotation.x, rotation.y, rotation.z, rotation.w);
  const tf2::Transform base_from_camera(base_from_camera_rotation,
    tf2::Vector3(translation.x, translation.y, translation.z));
  const auto camera_point = base_from_camera.inverse() *
    tf2::Vector3(point.x, point.y, point.z);
  if (!std::isfinite(camera_point.x()) || !std::isfinite(camera_point.y()) ||
    !std::isfinite(camera_point.z()) || camera_point.z() <= 0.0 ||
    std::abs(camera_point.x() / camera_point.z()) > 1000.0 ||
    std::abs(camera_point.y() / camera_point.z()) > 1000.0)
  {return std::nullopt;}

  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    camera_info.k[0], 0.0, camera_info.k[2],
    0.0, camera_info.k[4], camera_info.k[5], 0.0, 0.0, 1.0);
  const cv::Mat distortion(camera_info.d, true);
  const std::vector<cv::Point3d> camera_points{{camera_point.x(), camera_point.y(), camera_point.z()}};
  std::vector<cv::Point2d> projected;
  try {
    if (fisheye_model) {
      cv::fisheye::projectPoints(camera_points, projected, cv::Vec3d(0.0, 0.0, 0.0),
        cv::Vec3d(0.0, 0.0, 0.0), camera_matrix, distortion);
    } else {
      cv::projectPoints(camera_points, cv::Vec3d(0.0, 0.0, 0.0),
        cv::Vec3d(0.0, 0.0, 0.0), camera_matrix, distortion, projected);
    }
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
  if (projected.size() != 1 || !std::isfinite(projected.front().x) ||
    !std::isfinite(projected.front().y) || projected.front().x < 0.0 ||
    projected.front().y < 0.0 || projected.front().x >= image.cols ||
    projected.front().y >= image.rows)
  {return std::nullopt;}
  const cv::Point pixel(static_cast<int>(std::lround(projected.front().x)),
    static_cast<int>(std::lround(projected.front().y)));
  if (pixel.x < 0 || pixel.y < 0 || pixel.x >= image.cols || pixel.y >= image.rows) {
    return std::nullopt;
  }
  return pixel;
}

std::vector<cv::Point> cuboid_outline_pixels(
  const VisionDetectorSnapshotJob & job, const CuboidHypothesisVisualization & candidate)
{
  const auto * input = job.observation_input();
  if (!input || !job.calibration_data().ready || candidate.length_m <= 0.0 ||
    candidate.width_m <= 0.0 || !std::isfinite(candidate.center_x_m) ||
    !std::isfinite(candidate.center_y_m) || !std::isfinite(candidate.height_m) ||
    !std::isfinite(candidate.yaw_rad) ||
    job.calibration_data().tray_from_camera.header.frame_id != "Tray" ||
    input->rgb.header.frame_id != input->camera_info.header.frame_id ||
    job.calibration_data().tray_from_camera.child_frame_id != input->rgb.header.frame_id)
  {return {};}

  const auto & camera_info = input->camera_info;
  if (camera_info.width != input->rgb.width || camera_info.height != input->rgb.height ||
    std::any_of(camera_info.k.begin(), camera_info.k.end(),
      [](double value) {return !std::isfinite(value);}) ||
    camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 ||
    std::abs(camera_info.k[1]) > 1e-12 || std::abs(camera_info.k[3]) > 1e-12 ||
    std::abs(camera_info.k[6]) > 1e-12 || std::abs(camera_info.k[7]) > 1e-12 ||
    std::abs(camera_info.k[8] - 1.0) > 1e-12 ||
    std::any_of(camera_info.d.begin(), camera_info.d.end(),
      [](double coefficient) {return !std::isfinite(coefficient);}))
  {return {};}
  const auto model = camera_info.distortion_model;
  const bool pinhole_model = model == "plumb_bob" || model == "rational_polynomial";
  const bool fisheye_model = model == "equidistant";
  if ((!pinhole_model && !fisheye_model) ||
    (fisheye_model && camera_info.d.size() != 4) ||
    (model == "plumb_bob" && camera_info.d.size() != 4 && camera_info.d.size() != 5) ||
    (model == "rational_polynomial" && camera_info.d.size() != 8))
  {return {};}
  const auto & message = job.calibration_data().tray_from_camera.transform;
  const double quaternion_norm = std::sqrt(message.rotation.x * message.rotation.x +
    message.rotation.y * message.rotation.y + message.rotation.z * message.rotation.z +
    message.rotation.w * message.rotation.w);
  if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-3 ||
    !std::isfinite(message.translation.x) || !std::isfinite(message.translation.y) ||
    !std::isfinite(message.translation.z)) {return {};}
  const tf2::Quaternion tray_from_camera_rotation(
    message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w);
  const tf2::Transform tray_from_camera(
    tray_from_camera_rotation,
    tf2::Vector3(message.translation.x, message.translation.y, message.translation.z));
  const tf2::Transform camera_from_tray = tray_from_camera.inverse();
  const double cosine = std::cos(candidate.yaw_rad);
  const double sine = std::sin(candidate.yaw_rad);
  std::vector<cv::Point3f> corners;
  corners.reserve(4);
  for (const auto & sign : std::array<cv::Point2d, 4>{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}}) {
    const double tray_x = candidate.center_x_m +
      cosine * sign.x * candidate.length_m / 2.0 - sine * sign.y * candidate.width_m / 2.0;
    const double tray_y = candidate.center_y_m +
      sine * sign.x * candidate.length_m / 2.0 + cosine * sign.y * candidate.width_m / 2.0;
    const auto camera_point = camera_from_tray *
      tf2::Vector3(tray_x, tray_y, candidate.height_m);
    if (!std::isfinite(camera_point.x()) || !std::isfinite(camera_point.y()) ||
      !std::isfinite(camera_point.z()) || camera_point.z() <= 0.0 ||
      std::abs(camera_point.x() / camera_point.z()) > 1000.0 ||
      std::abs(camera_point.y() / camera_point.z()) > 1000.0)
    {return {};}
    corners.emplace_back(camera_point.x(), camera_point.y(), camera_point.z());
  }

  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    camera_info.k[0], 0.0, camera_info.k[2],
    0.0, camera_info.k[4], camera_info.k[5], 0.0, 0.0, 1.0);
  const cv::Mat distortion(camera_info.d, true);
  std::vector<cv::Point2f> projected;
  try {
    if (fisheye_model) {
      cv::fisheye::projectPoints(corners, projected, cv::Vec3d(0.0, 0.0, 0.0),
        cv::Vec3d(0.0, 0.0, 0.0), camera_matrix, distortion);
    } else {
      cv::projectPoints(corners, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0),
        camera_matrix, distortion, projected);
    }
  } catch (const cv::Exception &) {
    return {};
  }
  if (projected.size() != 4) {return {};}
  std::vector<cv::Point> outline;
  outline.reserve(projected.size());
  for (const auto & point : projected) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      std::abs(point.x) > 1000000.0F || std::abs(point.y) > 1000000.0F)
    {return {};}
    outline.emplace_back(static_cast<int>(std::lround(point.x)),
      static_cast<int>(std::lround(point.y)));
  }
  return outline;
}

}  // namespace

namespace snapshot_detail
{
cv::Mat rotate_tray_image_to_rgb_orientation(
  const cv::Mat & image, const VisionDetectorSnapshotJob & job,
  const cv::Scalar & border_color)
{
  const auto * input = job.observation_input();
  if (image.empty() || !input || !job.calibration_data().ready ||
    job.calibration_data().tray_from_camera.header.frame_id != "Tray" ||
    input->rgb.header.frame_id != input->camera_info.header.frame_id ||
    job.calibration_data().tray_from_camera.child_frame_id != input->rgb.header.frame_id)
  {return image;}
  const auto & camera_info = input->camera_info;
  if (camera_info.width != input->rgb.width || camera_info.height != input->rgb.height ||
    std::any_of(camera_info.k.begin(), camera_info.k.end(),
      [](double value) {return !std::isfinite(value);}) ||
    camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 ||
    std::abs(camera_info.k[1]) > 1e-12 || std::abs(camera_info.k[3]) > 1e-12 ||
    std::abs(camera_info.k[6]) > 1e-12 || std::abs(camera_info.k[7]) > 1e-12 ||
    std::abs(camera_info.k[8] - 1.0) > 1e-12 ||
    std::any_of(camera_info.d.begin(), camera_info.d.end(),
      [](double value) {return !std::isfinite(value);}))
  {return image;}
  const auto model = camera_info.distortion_model;
  const bool pinhole_model = model == "plumb_bob" || model == "rational_polynomial";
  const bool fisheye_model = model == "equidistant";
  if ((!pinhole_model && !fisheye_model) ||
    (fisheye_model && camera_info.d.size() != 4) ||
    (model == "plumb_bob" && camera_info.d.size() != 4 && camera_info.d.size() != 5) ||
    (model == "rational_polynomial" && camera_info.d.size() != 8))
  {return image;}
  const auto & message = job.calibration_data().tray_from_camera.transform;
  const double quaternion_norm = std::sqrt(message.rotation.x * message.rotation.x +
    message.rotation.y * message.rotation.y + message.rotation.z * message.rotation.z +
    message.rotation.w * message.rotation.w);
  if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-3 ||
    !std::isfinite(message.translation.x) || !std::isfinite(message.translation.y) ||
    !std::isfinite(message.translation.z))
  {return image;}

  // tray_from_camera is Camera -> Tray. Invert it to project Tray +X/+Y into
  // the same raw image plane as RGB, using ROS optical axes (X right, Y down).
  const tf2::Quaternion tray_from_camera_rotation(
    message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w);
  const tf2::Transform tray_from_camera(tray_from_camera_rotation,
    tf2::Vector3(message.translation.x, message.translation.y, message.translation.z));
  const tf2::Transform camera_from_tray = tray_from_camera.inverse();
  constexpr double basis_step_m = 0.01;
  const std::array<tf2::Vector3, 3> tray_points{{
    tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(basis_step_m, 0.0, 0.0),
    tf2::Vector3(0.0, basis_step_m, 0.0)}};
  std::vector<cv::Point3d> camera_points;
  camera_points.reserve(tray_points.size());
  for (const auto & point : tray_points) {
    const auto camera_point = camera_from_tray * point;
    if (!std::isfinite(camera_point.x()) || !std::isfinite(camera_point.y()) ||
      !std::isfinite(camera_point.z()) || camera_point.z() <= 1e-9)
    {return image;}
    camera_points.emplace_back(camera_point.x(), camera_point.y(), camera_point.z());
  }
  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    camera_info.k[0], 0.0, camera_info.k[2],
    0.0, camera_info.k[4], camera_info.k[5], 0.0, 0.0, 1.0);
  const cv::Mat distortion(camera_info.d, true);
  std::vector<cv::Point2d> projected;
  try {
    if (fisheye_model) {
      cv::fisheye::projectPoints(camera_points, projected, cv::Vec3d(0.0, 0.0, 0.0),
        cv::Vec3d(0.0, 0.0, 0.0), camera_matrix, distortion);
    } else {
      cv::projectPoints(camera_points, cv::Vec3d(0.0, 0.0, 0.0),
        cv::Vec3d(0.0, 0.0, 0.0), camera_matrix, distortion, projected);
    }
  } catch (const cv::Exception &) {
    return image;
  }
  if (projected.size() != 3) {return image;}
  const auto projected_x = projected[1] - projected[0];
  const auto projected_y = projected[2] - projected[0];
  const double x_length = std::hypot(projected_x.x, projected_x.y);
  const double y_length = std::hypot(projected_y.x, projected_y.y);
  if (!std::isfinite(x_length) || !std::isfinite(y_length) ||
    x_length < 1e-9 || y_length < 1e-9)
  {return image;}
  const cv::Point2d x_axis(projected_x.x / x_length, projected_x.y / x_length);
  // Tray rasters show +Y upward, so their positive image-down direction is -Y.
  const cv::Point2d y_down(-projected_y.x / y_length, -projected_y.y / y_length);
  const double determinant = x_axis.x * y_down.y - y_down.x * x_axis.y;
  double angle = 0.0;
  cv::Matx22d orientation;
  if (determinant >= 0.0) {
    angle = std::atan2(x_axis.y - y_down.x, x_axis.x + y_down.y);
    orientation = {std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle)};
  } else {
    angle = std::atan2(x_axis.y + y_down.x, x_axis.x - y_down.y);
    orientation = {std::cos(angle), std::sin(angle), std::sin(angle), -std::cos(angle)};
  }
  if (std::abs(orientation(0, 0) - 1.0) < 1e-9 &&
    std::abs(orientation(0, 1)) < 1e-9 && std::abs(orientation(1, 0)) < 1e-9 &&
    std::abs(orientation(1, 1) - 1.0) < 1e-9)
  {return image;}

  const double span_x = std::abs(orientation(0, 0)) * (image.cols - 1) +
    std::abs(orientation(0, 1)) * (image.rows - 1);
  const double span_y = std::abs(orientation(1, 0)) * (image.cols - 1) +
    std::abs(orientation(1, 1)) * (image.rows - 1);
  const int output_width = static_cast<int>(std::ceil(span_x - 1e-9)) + 1;
  const int output_height = static_cast<int>(std::ceil(span_y - 1e-9)) + 1;
  const cv::Point2d source_center((image.cols - 1) / 2.0, (image.rows - 1) / 2.0);
  const cv::Point2d output_center((output_width - 1) / 2.0, (output_height - 1) / 2.0);
  cv::Mat affine = (cv::Mat_<double>(2, 3) <<
    orientation(0, 0), orientation(0, 1), output_center.x -
    orientation(0, 0) * source_center.x - orientation(0, 1) * source_center.y,
    orientation(1, 0), orientation(1, 1), output_center.y -
    orientation(1, 0) * source_center.x - orientation(1, 1) * source_center.y);
  cv::Mat rotated;
  cv::warpAffine(image, rotated, affine, cv::Size(output_width, output_height),
    cv::INTER_NEAREST, cv::BORDER_CONSTANT, border_color);
  return rotated;
}

}  // namespace snapshot_detail

cv::Mat VisionDetectorSnapshotOutput::render_annotation(
  const VisionDetectorSnapshotJob & job, bool & source_image_rendered,
  std::vector<std::optional<cv::Point>> & disturbance_projections) const
{
  bool observed_candidate_drawn = false, inferred_candidate_drawn = false;

  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  cv::Mat image = job.has_observation && job.observation_input() ? rgb_mat(job.observation_input()->rgb) : cv::Mat();
  source_image_rendered = !image.empty();
  if (image.empty()) {
    image = cv::Mat(240, 800, CV_8UC3, cv::Scalar(28, 28, 28));
    if (style_.show_annotation_text) {
      cv::putText(image, "Source image unavailable", {24, 70},
        cv::FONT_HERSHEY_SIMPLEX, 0.8 * style_.annotation_text_scale,
        rgb_color(style_.detail_text_color_rgb, cv::Scalar(230, 230, 230)), 2, cv::LINE_AA);
    }
  }
  const bool detect_target = job.event_kind != Message::CALIBRATION_COMPLETE &&
    job.event_kind != Message::CALIBRATION_PROGRESS;
  const auto high_color = rgb_color(style_.candidate_color_rgb, snapshot_detail::confidence_color("HIGH"));
  const auto medium_color = snapshot_detail::confidence_color("MEDIUM");
  const std::string title = std::string(event_name(job.event_kind)) + "  rev=" +
    std::to_string(job.revision);
  if (style_.show_annotation_text && !detect_target) {
    cv::putText(image, title, {15, 25}, cv::FONT_HERSHEY_SIMPLEX,
      0.65 * style_.annotation_text_scale,
      rgb_color(style_.primary_text_color_rgb, cv::Scalar(0, 220, 255)), 2, cv::LINE_AA);
  }
  if (job.event_kind == Message::CALIBRATION_COMPLETE) {
    if (source_image_rendered && !job.calibration_data().support_region.empty()) {
      std::vector<cv::Point> points;
      for (const auto & point : job.calibration_data().support_region) {
        points.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
      }
      if (points.size() >= 3) {
        cv::polylines(image, points, true, cv::Scalar(0, 220, 0), 2);
      }
    }
    if (style_.show_annotation_text) {
      cv::putText(image, "support calibration READY; multi-frame basis shown in metadata",
        {15, 52}, cv::FONT_HERSHEY_SIMPLEX, 0.5 * style_.annotation_text_scale,
        cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
  } else if (job.event_kind == Message::CALIBRATION_PROGRESS) {
    if (style_.show_annotation_text) {
      cv::putText(image, "support calibration in progress; recent depth validation shown in metadata",
        {15, 52}, cv::FONT_HERSHEY_SIMPLEX, 0.5 * style_.annotation_text_scale,
        cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
    }
  } else {
    if (source_image_rendered && job.detection_result().object_region_valid) {
      cv::rectangle(image, cv::Rect(
          static_cast<int>(job.detection_result().object_region_x),
          static_cast<int>(job.detection_result().object_region_y),
          static_cast<int>(job.detection_result().object_region_width),
          static_cast<int>(job.detection_result().object_region_height)),
        rgb_color(style_.target_color_rgb, cv::Scalar(0, 255, 0)), style_.region_line_width_px);
    }
    const bool qualified_success = job.event_kind == Message::DETECT_TARGET_SUCCESS &&
      job.has_detection_result &&
      job.detection_result().result_code == arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    if (source_image_rendered && qualified_success) {
      if (job.algorithm != "cuboid_top_face") {
        for (const auto & candidate : job.detection_result().rawpart_candidate_diagnostics) {
          if (!candidate.profile_match || !candidate.selected_for_target ||
            (candidate.confidence_grade != "HIGH" && candidate.confidence_grade != "MEDIUM") ||
            candidate.fitted_outline_pixels.size() != 4) {continue;}
          std::vector<cv::Point> outline;
          for (const auto & point : candidate.fitted_outline_pixels) {
            outline.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
          }
          cv::polylines(image, outline, true,
            candidate.confidence_grade == "HIGH" ? high_color : medium_color,
            style_.candidate_line_width_px, cv::LINE_AA);
          observed_candidate_drawn = true;
        }
      }
      if (job.algorithm == "cuboid_top_face") {
        const auto & candidates = job.detection_result().ordered_candidates;
        const auto draw_candidate = [&](const DetectionResult & candidate) {
            if (candidate.confidence_grade != "HIGH" && candidate.confidence_grade != "MEDIUM") {return;}
            const auto fit = std::find_if(job.detection_result().cuboid_hypotheses.begin(),
              job.detection_result().cuboid_hypotheses.end(), [&](const auto & h) {
                return std::to_string(h.candidate_id) == candidate.rawpart_selected_candidate;
              });
            if (fit != job.detection_result().cuboid_hypotheses.end() &&
              !fit->selection_state.empty() && fit->selection_state != "SELECTED") {return;}
            const auto color = candidate.confidence_grade == "HIGH" ? high_color : medium_color;
            const auto diagnostic = std::find_if(
              job.detection_result().rawpart_candidate_diagnostics.begin(),
              job.detection_result().rawpart_candidate_diagnostics.end(),
              [&](const auto & item) {
                return std::to_string(item.candidate_id) == candidate.rawpart_selected_candidate &&
                  !item.fitted_outline_pixels.empty();
              });
            if (diagnostic != job.detection_result().rawpart_candidate_diagnostics.end()) {
              std::vector<cv::Point> outline;
              for (const auto & point : diagnostic->fitted_outline_pixels) {
                outline.emplace_back(static_cast<int>(std::lround(point.x)),
                  static_cast<int>(std::lround(point.y)));
              }
              if (!outline.empty()) {
                observed_candidate_drawn = observed_candidate_drawn || candidate.has_observed_rectangle;
                inferred_candidate_drawn = inferred_candidate_drawn || !candidate.has_observed_rectangle;
                cv::putText(image, "#" + candidate.rawpart_selected_candidate, outline.front(),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
              }
              if (outline.size() > 1) {
                if (candidate.has_observed_rectangle) {
                  cv::polylines(image, outline, true, color, style_.candidate_line_width_px, cv::LINE_AA);
                } else {
                  for (size_t edge = 0; edge < outline.size(); ++edge) {
                    const auto a = outline[edge], b = outline[(edge + 1) % outline.size()];
                    snapshot_detail::evidence_line(image, a, b, color, true,
                      style_.candidate_line_width_px);
                  }
                }
              }
            } else if (candidate.confidence_grade != "LOW") {
              const auto is_candidate_geometry = [&](const auto & item) {
                  return item.confidence_grade == candidate.confidence_grade &&
                    (item.selection_state == "SELECTED" ||
                    (item.selection_state.empty() && (item.disposition == "qualified_geometry" ||
                    item.disposition == "selected_partial_reconstruction")));
                };
              auto hypothesis = job.detection_result().cuboid_hypotheses.end();
              if (!candidate.rawpart_selected_candidate.empty()) {
                hypothesis = std::find_if(
                  job.detection_result().cuboid_hypotheses.begin(),
                  job.detection_result().cuboid_hypotheses.end(), [&](const auto & item) {
                    return is_candidate_geometry(item) &&
                      std::to_string(item.candidate_id) == candidate.rawpart_selected_candidate;
                  });
              } else if (candidates.empty()) {
                const auto first = std::find_if(
                  job.detection_result().cuboid_hypotheses.begin(),
                  job.detection_result().cuboid_hypotheses.end(), is_candidate_geometry);
                if (first != job.detection_result().cuboid_hypotheses.end() &&
                  std::find_if(std::next(first), job.detection_result().cuboid_hypotheses.end(),
                    is_candidate_geometry) == job.detection_result().cuboid_hypotheses.end())
                {hypothesis = first;}
              }
              if (hypothesis != job.detection_result().cuboid_hypotheses.end()) {
                const auto outline = cuboid_outline_pixels(job, *hypothesis);
                if (outline.size() == 4) {
                  inferred_candidate_drawn = true;
                  for (size_t edge = 0; edge < outline.size(); ++edge) {
                    snapshot_detail::evidence_line(image, outline[edge], outline[(edge + 1) % outline.size()],
                      color, true, style_.candidate_line_width_px);
                  }
                  cv::putText(image, "#" + std::to_string(hypothesis->candidate_id), outline.front(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
                }
              }
            }
          };
        if (!candidates.empty()) {
          for (const auto & candidate : candidates) {draw_candidate(candidate);}
        } else {
          draw_candidate(job.detection_result());
        }
      }
    }
    if (source_image_rendered && job.detection_result().support_region_valid &&
      job.detection_result().support_region.size() >= 3)
    {
      std::vector<cv::Point> support;
      for (const auto & point : job.detection_result().support_region) {
        support.emplace_back(static_cast<int>(point.x), static_cast<int>(point.y));
      }
      cv::polylines(image, support, true,
        rgb_color(style_.support_color_rgb, cv::Scalar(255, 180, 0)), style_.region_line_width_px);
    }
    if (source_image_rendered && job.detection_result().centroid_pixel_valid) {
      cv::drawMarker(image, {static_cast<int>(job.detection_result().centroid_pixel_x),
        static_cast<int>(job.detection_result().centroid_pixel_y)},
        rgb_color(style_.measurement_color_rgb, cv::Scalar(0, 0, 255)),
        cv::MARKER_CROSS, style_.marker_size_px, style_.measurement_line_width_px);
    }
    if (job.has_detection_result) {
      const auto & disturbances = job.detection_result().disturbance_candidates;
      disturbance_projections.reserve(disturbances.size());
      for (const auto & disturbance : disturbances) {
        const auto pixel = source_image_rendered ?
          disturbance_center_pixel(job, disturbance, image) : std::nullopt;
        disturbance_projections.push_back(pixel);
        if (!pixel) {continue;}
        const auto color = rgb_color(style_.unresolved_color_rgb, cv::Scalar(0, 165, 255));
        cv::drawMarker(
          image, *pixel, color, cv::MARKER_CROSS,
          style_.marker_size_px, style_.measurement_line_width_px, cv::LINE_AA);
        cv::circle(image, *pixel, std::max(4, style_.marker_size_px / 2), color, 1, cv::LINE_AA);
      }
    }
  }
  if (detect_target && style_.show_annotation_text) {
    // Fixed rows inside the source raster; no canvas expansion or object-dependent layout.
    const double scale = std::min(0.5 * style_.annotation_text_scale,
      std::max(0.2, image.cols / 1000.0));
    const auto text_width = [&](const std::string & text, double text_scale) {
        int baseline = 0;
        return cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, text_scale, 1, &baseline).width;
      };
    const auto overlay_text = [&](const std::string & text, cv::Point at,
        const cv::Scalar & color, double text_scale) {
        int baseline = 0;
        const auto size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, text_scale, 1, &baseline);
        const auto box = cv::Rect(at.x - 3, at.y - size.height - 3,
          size.width + 6, size.height + baseline + 6) & cv::Rect(0, 0, image.cols, image.rows);
        if (box.empty()) {return;}
        image(box) *= 0.35;
        cv::putText(image, text, at, cv::FONT_HERSHEY_SIMPLEX, text_scale, color, 1, cv::LINE_AA);
      };
    const bool terminal = job.event_kind == Message::DETECT_TARGET_SUCCESS ||
      job.event_kind == Message::DETECT_TARGET_FAILURE;
    const bool result_available = terminal || job.has_detection_result;
    // Selection availability does not indicate result availability: failed terminal
    // requests can retain a successful single-observation result for diagnostics.
    const auto result_code = terminal ? job.terminal_result_code : job.detection_result().result_code;
    std::string result = "DetectTarget unavailable";
    if (result_available) {
      std::string name = snapshot_detail::result_code_name(result_code);
      const std::string prefix = "DETECT_RESULT_";
      if (name.rfind(prefix, 0) == 0) {name.erase(0, prefix.size());}
      result = "DetectTarget " + name;
    }
    const auto title_color = rgb_color(style_.primary_text_color_rgb, cv::Scalar(0, 220, 255));
    int result_width = image.cols - 16;
    const bool success = job.event_kind == Message::DETECT_TARGET_SUCCESS &&
      result_available && job.has_detection_result &&
      result_code == arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS &&
      job.detection_result().result_code == arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    if (source_image_rendered && success) {
      size_t high = 0, medium = 0;
      const auto & candidates = job.detection_result().ordered_candidates;
      for (const auto & candidate : candidates) {
        high += candidate.confidence_grade == "HIGH";
        medium += candidate.confidence_grade == "MEDIUM";
      }
      const bool counts_available = !candidates.empty() && std::all_of(
        candidates.begin(), candidates.end(), [](const auto & candidate) {
          return candidate.confidence_grade == "HIGH" || candidate.confidence_grade == "MEDIUM" ||
                 candidate.confidence_grade == "LOW";
        });
      if (counts_available) {
        const auto high_text = "HIGH " + std::to_string(high);
        const auto medium_text = "MEDIUM " + std::to_string(medium);
        const int medium_width = text_width(medium_text, scale);
        const int high_width = text_width(high_text, scale);
        const int medium_x = std::max(8, image.cols - medium_width - 8);
        const int high_x = std::max(8, medium_x - high_width - 18);
        result_width = high_x - 16;
        overlay_text(high_text, {high_x, 20}, high_color, scale);
        overlay_text(medium_text, {medium_x, 20}, medium_color, scale);
      } else {
        const std::string unavailable = "Returned counts unavailable";
        const int counts_x = std::max(8, image.cols - text_width(unavailable, scale) - 8);
        result_width = counts_x - 16;
        overlay_text(unavailable, {counts_x, 20}, title_color, scale);
      }
    }
    overlay_text(result, {8, 20}, title_color,
      std::min(scale, scale * std::max(1, result_width) / std::max(1, text_width(result, scale))));
    if (source_image_rendered) {
      std::string legend = "Color=confidence";
      if (observed_candidate_drawn) {legend += " | solid=observed";}
      if (inferred_candidate_drawn) {legend += " | dashed=inferred";}
      if (std::any_of(disturbance_projections.begin(), disturbance_projections.end(),
        [](const auto & point) {return point.has_value();})) {
        legend += " | Circle/cross=disturbance";
      }
      overlay_text(legend, {8, 42},
        rgb_color(style_.detail_text_color_rgb, snapshot_detail::observed_color),
        std::min(scale, scale * std::max(1, image.cols - 16) / std::max(1, text_width(legend, scale))));
    }
  }
  return image;
}

}  // namespace arm_cell_vision
