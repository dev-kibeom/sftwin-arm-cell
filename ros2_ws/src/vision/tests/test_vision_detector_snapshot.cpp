#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include <sys/stat.h>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/msg/vision_detector_snapshot.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "arm_cell_vision/vision_detector_snapshot.hpp"
#include "../src/vision_snapshot_internal.hpp"
#include "arm_cell_vision/detect_target_service.hpp"
#include "arm_cell_vision/cuboid_top_face.hpp"

namespace
{
std::shared_ptr<const arm_cell_vision::AcquisitionResult> make_acquisition_result(
  arm_cell_vision::DetectionResult detection, uint64_t generation)
{
  auto result = std::make_shared<arm_cell_vision::AcquisitionResult>();
  result->generation = generation;
  result->target_id = "RawPart";
  result->acquisition_id = arm_cell_vision::make_acquisition_id("vision-run-snapshot-test", generation);
  detection.scene_generation = generation;
  result->detection = std::move(detection);
  return result;
}
}

namespace arm_cell_vision::detail
{
void set_cuboid_stage_image_metadata(
  arm_cell_interfaces::msg::VisionDetectorSnapshot & message,
  const std::string & stage, const std::string & status, bool saved);
}

namespace
{

std::filesystem::path temporary_snapshot_root(const std::string & name);
arm_cell_vision::VisionDetectorSnapshotJob calibration_job();

class VisionDetectorSnapshotTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {rclcpp::shutdown();}
  }

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("snapshot_test");
};

std::string utc_date_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto timestamp = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  if (gmtime_r(&timestamp, &utc) == nullptr) {return {};}
  char date[9]{};
  if (std::strftime(date, sizeof(date), "%Y%m%d", &utc) == 0) {return {};}
  return date;
}

std::filesystem::path temporary_snapshot_root(const std::string & name)
{
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("vision-snapshot-" + name + "-" + std::to_string(nonce));
}

cv::Point2d colored_pixel_centroid(
  const cv::Mat & image, const cv::Rect & region, bool observed, cv::Rect * bounds = nullptr)
{
  double sum_x = 0.0;
  double sum_y = 0.0;
  size_t count = 0;
  int min_x = region.x + region.width;
  int min_y = region.y + region.height;
  int max_x = region.x - 1;
  int max_y = region.y - 1;
  for (int y = region.y; y < region.y + region.height; ++y) {
    for (int x = region.x; x < region.x + region.width; ++x) {
      const auto pixel = image.at<cv::Vec3b>(y, x);
      const bool match = observed ?
        pixel[0] > 75 && pixel[0] < 155 && pixel[1] > 145 && pixel[2] > 220 :
        pixel[0] > 185 && pixel[0] < 235 && pixel[1] > 115 && pixel[1] < 185 &&
        pixel[2] > 185 && pixel[2] < 235;
      if (match) {
        sum_x += x; sum_y += y; ++count;
        min_x = std::min(min_x, x); min_y = std::min(min_y, y);
        max_x = std::max(max_x, x); max_y = std::max(max_y, y);
      }
    }
  }
  if (count == 0) {return {-1.0, -1.0};}
  if (bounds) {*bounds = cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);}
  return {sum_x / count, sum_y / count};
}

cv::Rect candidate_boundary_bounds(const cv::Mat & image, const cv::Rect & region, size_t & count)
{
  int min_x = region.x + region.width;
  int min_y = region.y + region.height;
  int max_x = region.x - 1;
  int max_y = region.y - 1;
  count = 0;
  for (int y = region.y; y < region.y + region.height; ++y) {
    for (int x = region.x; x < region.x + region.width; ++x) {
      const auto pixel = image.at<cv::Vec3b>(y, x);
      if (pixel[0] < 80 && pixel[1] > 180 && pixel[2] < 80) {
        min_x = std::min(min_x, x); min_y = std::min(min_y, y);
        max_x = std::max(max_x, x); max_y = std::max(max_y, y);
        ++count;
      }
    }
  }
  if (count == 0) {return {};}
  return {min_x, min_y, max_x - min_x + 1, max_y - min_y + 1};
}

arm_cell_vision::VisionDetectorSnapshotJob calibration_job()
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  arm_cell_vision::VisionDetectorSnapshotJob job;
  job.event_kind = Message::CALIBRATION_COMPLETE;
  job.calibration_id = "cal-1";
  job.save = true;
  job.has_observation = true;
  job.calibration.ready = true;
  job.calibration.accepted_frames = 5;
  job.calibration.required_frames = 5;
  job.calibration.median_depth_residual_m = 0.002;
  arm_cell_vision::SupportCalibrationSnapshot::DepthValidation validation;
  validation.valid_sample_count = 9;
  validation.residual_available = true;
  validation.median_signed_residual_m = 0.002;
  validation.minimum_signed_residual_m = -0.001;
  validation.maximum_signed_residual_m = 0.004;
  validation.support_contact_tolerance_m = 0.01;
  validation.configured_marker_height_m = 0.0002;
  validation.accepted = true;
  validation.reason = "accepted";
  job.calibration.recent_depth_validations.push_back(validation);
  auto input = std::make_shared<arm_cell_vision::DetectionInput>();
  input->rgb.width = 2;
  input->rgb.height = 2;
  input->rgb.encoding = "rgb8";
  input->rgb.step = 6;
  input->rgb.data.assign(12, 32);
  input->rgb.header.stamp.sec = 10;
  input->rgb.header.frame_id = "rgb_camera";
  input->depth.width = 2;
  input->depth.height = 2;
  input->depth.encoding = "32FC1";
  input->depth.step = 8;
  input->depth.data.resize(16);
  input->depth.header.stamp.sec = 10;
  input->depth.header.frame_id = "depth_camera";
  input->camera_info.k[0] = 1.0;
  input->camera_info.k[4] = 1.0;
  input->optical_to_base.transform.rotation.w = 1.0;
  job.input = std::move(input);
  job.tray_roi_min_m = {-0.1, -0.1};
  job.tray_roi_max_m = {0.1, 0.1};
  return job;
}

arm_cell_vision::VisionDetectorSnapshotJob cuboid_projection_job(const std::string & model)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  auto job = calibration_job();
  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
  input->rgb.width = 320;
  input->rgb.height = 240;
  input->rgb.step = 320 * 3;
  input->rgb.data.assign(input->rgb.step * input->rgb.height, 0);
  input->rgb.header.frame_id = "camera_optical";
  input->camera_info.width = 320;
  input->camera_info.height = 240;
  input->camera_info.header.frame_id = "camera_optical";
  input->camera_info.distortion_model = model;
  input->camera_info.k = {100.0, 0.0, 160.0, 0.0, 100.0, 120.0, 0.0, 0.0, 1.0};
  input->camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  job.input = input;
  job.event_kind = Message::DETECT_TARGET_SUCCESS;
  job.algorithm = "cuboid_top_face";
  job.has_detection_result = true;
  job.result.algorithm = "cuboid_top_face";
  job.result.result_code = arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  job.result.confidence_grade = "HIGH";
  job.result.cuboid_hypotheses = {{0, 0, 1.0, 0.0, 0.0, 0.0, 0.4, 0.2, 0.0, 1.0, 1.0,
    "qualified_geometry", "", 0}};
  job.result.cuboid_hypotheses.front().confidence_grade = "HIGH";
  job.calibration.ready = true;
  job.calibration.tray_from_camera.header.frame_id = "Tray";
  job.calibration.tray_from_camera.child_frame_id = "camera_optical";
  job.calibration.tray_from_camera.transform.translation.x = 0.1;
  job.calibration.tray_from_camera.transform.rotation.w = 1.0;
  return job;
}

arm_cell_vision::VisionDetectorSnapshotJob cuboid_stage_job(const std::string & acquisition_id)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  auto job = calibration_job();
  job.event_kind = Message::DETECT_TARGET_SUCCESS;
  job.vision_run_id = "stage-run";
  job.acquisition_id = acquisition_id;
  job.algorithm = "cuboid_top_face";
  job.target_id = "RawPart";
  job.has_detection_result = true;
  job.terminal_result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  job.terminal_selection_available = true;
  job.processing_stage = "cuboid_top_face_geometry";
  job.calibration.ready = true;
  job.tray_roi_min_m = {-0.2, -0.2};
  job.tray_roi_max_m = {0.2, 0.2};
  job.tray_depth_resolution_m = 0.005;
  auto input = std::make_shared<arm_cell_vision::DetectionInput>();
  input->rgb.width = 160;
  input->rgb.height = 120;
  input->rgb.step = 160 * 3;
  input->rgb.encoding = "rgb8";
  input->rgb.data.assign(input->rgb.step * input->rgb.height, 50);
  input->rgb.header.stamp.sec = 42;
  input->rgb.header.frame_id = "camera";
  input->depth.width = 160;
  input->depth.height = 120;
  input->depth.step = 160 * sizeof(float);
  input->depth.encoding = "32FC1";
  input->depth.data.resize(input->depth.step * input->depth.height);
  for (size_t offset = 0; offset < input->depth.data.size(); offset += sizeof(float)) {
    const float depth = 1.0F;
    std::memcpy(input->depth.data.data() + offset, &depth, sizeof(depth));
  }
  input->depth.header = input->rgb.header;
  input->camera_info.width = 160;
  input->camera_info.height = 120;
  input->camera_info.header.frame_id = "camera";
  input->camera_info.k = {500, 0, 80, 0, 500, 60, 0, 0, 1};
  input->optical_to_base.transform.rotation.w = 1.0;
  job.input = input;
  job.selected_profile = arm_cell_vision::DetectorProfile{};
  job.selected_profile->algorithm = "cuboid_top_face";
  job.selected_profile->cuboid.height_tolerance_m = 0.003;
  job.result.algorithm = "cuboid_top_face";
  job.result.source_rgb_header = input->rgb.header;
  job.result.result_code = job.terminal_result_code;
  job.result.has_target_pose = true;
  job.result.target_pose.header.frame_id = "base_link";
  job.result.target_pose.pose.position.x = 0.075;
  job.result.target_pose.pose.position.y = 0.075;
  job.result.has_rawpart_diagnostics = true;
  job.result.rawpart_candidate_generation = "candidate_local_eligibility";
  job.result.rawpart_candidate_count = 1;
  job.result.rawpart_profile_match_count = 1;
  job.result.rawpart_selected_candidate = "86";
  arm_cell_vision::RawPartCandidateDiagnostic final_candidate;
  final_candidate.candidate_id = 86;
  final_candidate.confidence_grade = "HIGH";
  final_candidate.profile_match = true;
  final_candidate.greedy_rank = 1;
  for (const auto & point : std::vector<std::array<float, 2>>{
      {-0.05F, -0.05F}, {0.05F, -0.05F}, {0.05F, 0.05F}, {-0.05F, 0.05F}})
  {
    geometry_msgs::msg::Point32 support_point;
    support_point.x = point[0];
    support_point.y = point[1];
    final_candidate.fitted_outline_support.push_back(support_point);
  }
  for (const auto & pixel : std::vector<std::array<float, 2>>{
      {80.0F, 40.0F}, {100.0F, 40.0F}, {100.0F, 60.0F}, {80.0F, 60.0F}})
  {
    geometry_msgs::msg::Point32 point;
    point.x = pixel[0];
    point.y = pixel[1];
    final_candidate.fitted_outline_pixels.push_back(point);
  }
  job.result.rawpart_candidate_diagnostics.push_back(final_candidate);
  arm_cell_vision::DetectionResult ordered_candidate;
  ordered_candidate.rawpart_selected_candidate = "86";
  ordered_candidate.confidence_grade = "HIGH";
  ordered_candidate.has_observed_rectangle = true;
  job.result.ordered_candidates.push_back(ordered_candidate);
  job.result.cuboid_height_patches.push_back({});
  job.result.cuboid_height_patches.back().height_m = 0.03;
  job.result.cuboid_height_patches.back().cell_count = 400;
  job.result.cuboid_height_patches.back().contour_count = 1;
  job.result.cuboid_height_patches.back().fitted_edge_count = 1;
  for (int y = 30; y < 50; ++y) {
    for (int x = 30; x < 50; ++x) {
      geometry_msgs::msg::Point32 cell;
      cell.x = static_cast<float>(-0.2 + x * 0.005);
      cell.y = static_cast<float>(-0.2 + y * 0.005);
      job.result.cuboid_height_patches.back().observed_cells_xy.push_back(cell);
    }
  }
  job.result.cuboid_height_patches.back().observed_contours_xy.push_back({
      geometry_msgs::msg::Point32(), geometry_msgs::msg::Point32(),
      geometry_msgs::msg::Point32(), geometry_msgs::msg::Point32()});
  auto & test_contour = job.result.cuboid_height_patches.back().observed_contours_xy.back();
  test_contour[0].x = -0.05F; test_contour[0].y = -0.05F;
  test_contour[1].x = 0.05F; test_contour[1].y = -0.05F;
  test_contour[2].x = 0.05F; test_contour[2].y = 0.05F;
  test_contour[3].x = -0.05F; test_contour[3].y = 0.05F;
  job.result.cuboid_height_patches.back().fitted_edges_xy_support.push_back(
    {-0.05, -0.05, 0.05, -0.05, 1.0});
  job.result.cuboid_height_patches.back().candidate_evaluation_attempt_count = 3;
  job.result.cuboid_height_patches.back().geometry_valid_before_deduplication_count = 2;
  job.result.cuboid_height_patches.back().generated_hypothesis_count = 1;
  job.result.cuboid_height_patches.back().deduplicated_count = 1;
  job.result.cuboid_height_patches.back().geometry_rejected_by_reason[
    "insufficient_observed_top_area"] = 1;
  auto replay = std::make_shared<arm_cell_vision::CuboidReplayEvidence>();
  replay->rows = 80;
  replay->columns = 80;
  replay->resolution_m = 0.005;
  replay->origin_x_m = -0.2;
  replay->origin_y_m = -0.2;
  replay->height_grid_m = cv::Mat(80, 80, CV_64FC1,
    std::numeric_limits<double>::quiet_NaN());
  replay->height_grid_m(cv::Rect(30, 30, 20, 20)).setTo(0.03);
  replay->height_grid_m(cv::Rect(52, 52, 12, 12)).setTo(0.06);
  job.result.cuboid_replay_evidence = replay;
  arm_cell_vision::CuboidHypothesisVisualization geometry_rejected;
  geometry_rejected.candidate_id = 6;
  geometry_rejected.height_m = 0.03;
  geometry_rejected.center_x_m = -0.025;
  geometry_rejected.center_y_m = -0.025;
  geometry_rejected.length_m = 0.10;
  geometry_rejected.width_m = 0.10;
  geometry_rejected.selection_state = "REJECTED";
  geometry_rejected.disposition = "rejected_geometry_validation";
  geometry_rejected.reason = "insufficient_observed_top_area";
  job.result.cuboid_hypotheses.push_back(geometry_rejected);
  arm_cell_vision::CuboidHypothesisVisualization hypothesis;
  hypothesis.candidate_id = 7;
  hypothesis.height_m = 0.03;
  hypothesis.center_x_m = -0.025;
  hypothesis.center_y_m = -0.025;
  hypothesis.length_m = 0.10;
  hypothesis.width_m = 0.10;
  hypothesis.thickness_m = 0.03;
  hypothesis.selection_state = "REJECTED";
  hypothesis.disposition = "rejected_candidate_eligibility";
  hypothesis.reason = "insufficient_observed_boundary_support";
  hypothesis.anchor_x_m = -0.125;
  hypothesis.anchor_y_m = -0.125;
  hypothesis.observed_anchor_edge_lengths_m = {0.125, 0.0625};
  hypothesis.observed_boundary_sides = 3;
  hypothesis.tray_support_evidence = "direct_tray_support_not_confirmed";
  hypothesis.parallel_edge_distances_m = {0.125};
  job.result.cuboid_hypotheses.push_back(hypothesis);
  auto removed = hypothesis;
  removed.candidate_id = 8;
  removed.selection_state = "ALTERNATIVE";
  removed.disposition = "removed_by_deduplication";
  removed.deduplicated_to_candidate_id = 7;
  job.result.cuboid_hypotheses.push_back(removed);
  job.result.cuboid_height_patches.push_back({});
  job.result.cuboid_height_patches.back().height_m = 0.06;
  job.result.cuboid_height_patches.back().cell_count = 144;
  job.result.cuboid_height_patches.back().contour_count = 1;
  job.result.cuboid_height_patches.back().fitted_edge_count = 4;
  job.result.cuboid_height_patches.back().observed_cells_xy.reserve(144);
  for (int y = 52; y < 64; ++y) {
    for (int x = 52; x < 64; ++x) {
      geometry_msgs::msg::Point32 cell;
      cell.x = static_cast<float>(-0.2 + x * 0.005);
      cell.y = static_cast<float>(-0.2 + y * 0.005);
      job.result.cuboid_height_patches.back().observed_cells_xy.push_back(cell);
    }
  }
  job.result.cuboid_height_patches.back().observed_contours_xy.push_back({
      geometry_msgs::msg::Point32(), geometry_msgs::msg::Point32(),
      geometry_msgs::msg::Point32(), geometry_msgs::msg::Point32()});
  auto & upper_contour = job.result.cuboid_height_patches.back().observed_contours_xy.back();
  upper_contour[0].x = 0.06F; upper_contour[0].y = 0.06F;
  upper_contour[1].x = 0.115F; upper_contour[1].y = 0.06F;
  upper_contour[2].x = 0.115F; upper_contour[2].y = 0.115F;
  upper_contour[3].x = 0.06F; upper_contour[3].y = 0.115F;
  const std::array<std::array<double, 4>, 4> upper_edges{{
    {{0.06, 0.06, 0.115, 0.06}}, {{0.115, 0.06, 0.115, 0.115}},
    {{0.115, 0.115, 0.06, 0.115}}, {{0.06, 0.115, 0.06, 0.06}}}};
  for (const auto & edge : upper_edges) {
    job.result.cuboid_height_patches.back().fitted_edges_xy_support.push_back(
      {edge[0], edge[1], edge[2], edge[3], 1.0});
  }
  job.result.cuboid_height_patches.back().candidate_evaluation_attempt_count = 6;
  job.result.cuboid_height_patches.back().geometry_valid_before_deduplication_count = 6;
  job.result.cuboid_height_patches.back().generated_hypothesis_count = 2;
  job.result.cuboid_height_patches.back().deduplicated_count = 4;
  auto ranked = hypothesis;
  ranked.candidate_id = 86;
  ranked.height_m = 0.06;
  ranked.center_x_m = 0.075;
  ranked.center_y_m = 0.075;
  ranked.length_m = 0.06;
  ranked.width_m = 0.06;
  ranked.thickness_m = 0.03;
  ranked.patch_index = 1;
  ranked.selection_state = "SELECTED";
  ranked.confidence_grade = "HIGH";
  ranked.disposition = "qualified_geometry";
  ranked.reason = "candidate_geometry_and_observation_checks_passed";
  job.result.cuboid_hypotheses.push_back(ranked);
  auto ambiguous = ranked;
  ambiguous.candidate_id = 10;
  ambiguous.center_x_m = -0.025;
  ambiguous.center_y_m = -0.025;
  ambiguous.selection_state = "UNRESOLVED";
  ambiguous.disposition = "rejected_identity_ambiguous";
  ambiguous.reason = "identity_ambiguous";
  job.result.cuboid_hypotheses.push_back(ambiguous);
  for (uint32_t id = 11; id <= 14; ++id) {
    auto duplicate = ranked;
    duplicate.candidate_id = id;
    duplicate.selection_state = "ALTERNATIVE";
    duplicate.disposition = "removed_by_deduplication";
    duplicate.deduplicated_to_candidate_id = 86;
    job.result.cuboid_hypotheses.push_back(duplicate);
  }
  auto diagnostic_low = hypothesis;
  diagnostic_low.candidate_id = 15;
  diagnostic_low.center_x_m = -0.11;
  diagnostic_low.center_y_m = -0.11;
  diagnostic_low.length_m = 0.05;
  diagnostic_low.width_m = 0.04;
  diagnostic_low.patch_index = 0;
  diagnostic_low.selection_state = "UNRESOLVED";
  diagnostic_low.confidence_grade = "LOW";
  diagnostic_low.confidence_reason = "distinct_hypotheses_remain_ambiguous";
  diagnostic_low.disposition = "rejected_identity_ambiguous";
  job.result.cuboid_hypotheses.push_back(diagnostic_low);
  job.result.cuboid_height_patches[0].candidate_evaluation_attempt_count = 4;
  job.result.cuboid_height_patches[0].geometry_rejected_by_reason[
    "insufficient_observed_boundary_support"] = 2;
  job.result.cuboid_boundaries.push_back(job.result.cuboid_height_patches.front());
  auto & plane = job.result.cuboid_height_patches.front();
  plane.plane_fitted = true;
  plane.plane_status = "horizontal";
  plane.partition_status = "alternatives";
  plane.rectangle_regions = {{-0.025, -0.025, 0, 0.10, 0.10}, {-0.025, 0, 0, 0.10, 0.15}};
  plane.inferred_partition_edges = {{-0.075, 0, 0.025, 0}};
  plane.rectangle_alternatives = {{1, 2}};
  plane.adjacent_planes = {{1, 0.03, 1}};
  plane.fitted_edge_source_ids = {4};
  job.result.cuboid_height_patches[1].plane_status = "mixed_or_inclined";
  job.result.cuboid_height_patches[1].plane_normal = {0.2, 0, 0.98};
  return job;
}

struct SnapshotOrientationCase
{
  std::string name;
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  bool camera_below_tray = false;
  bool calibration_valid = true;
  double fx = 500.0;
  double fy = 500.0;
};

arm_cell_vision::VisionDetectorSnapshotJob orientation_stage_job(
  const SnapshotOrientationCase & test_case, const std::string & acquisition_id)
{
  auto job = cuboid_stage_job(acquisition_id);
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  job.terminal_result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
  job.terminal_selection_available = false;
  job.result.result_code = job.terminal_result_code;
  job.result.has_target_pose = false;
  job.result.cuboid_hypotheses.clear();
  job.result.cuboid_height_patches.clear();
  job.result.cuboid_replay_evidence.reset();
  job.selected_profile->algorithm = "cuboid_top_face";
  job.selected_profile->target_id = "RawPart";
  job.calibration.ready = test_case.calibration_valid;
  auto & stamped = job.calibration.tray_from_camera;
  stamped.header.frame_id = "Tray";
  stamped.child_frame_id = "camera_optical";

  tf2::Quaternion camera_to_tray;
  if (test_case.camera_below_tray) {
    camera_to_tray.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    tf2::Quaternion yaw;
    yaw.setRPY(0.0, 0.0, test_case.yaw_rad);
    tf2::Quaternion pitch;
    pitch.setRPY(0.0, test_case.pitch_rad, 0.0);
    tf2::Quaternion optical_to_body;
    optical_to_body.setRPY(CV_PI, 0.0, 0.0);
    camera_to_tray = yaw * pitch * optical_to_body;
    camera_to_tray.normalize();
  }
  stamped.transform.rotation.x = camera_to_tray.x();
  stamped.transform.rotation.y = camera_to_tray.y();
  stamped.transform.rotation.z = camera_to_tray.z();
  stamped.transform.rotation.w = camera_to_tray.w();
  const auto camera_z_in_tray = tf2::quatRotate(camera_to_tray, tf2::Vector3(0.0, 0.0, 1.0));
  constexpr double camera_distance = 1.0;
  stamped.transform.translation.x = -camera_distance * camera_z_in_tray.x();
  stamped.transform.translation.y = -camera_distance * camera_z_in_tray.y();
  stamped.transform.translation.z = -camera_distance * camera_z_in_tray.z();

  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
  input->rgb.header.frame_id = "camera_optical";
  input->camera_info.header.frame_id = "camera_optical";
  input->camera_info.distortion_model = "plumb_bob";
  input->camera_info.k = {test_case.fx, 0.0, 80.0, 0.0, test_case.fy, 60.0, 0.0, 0.0, 1.0};
  input->camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::fill(input->depth.data.begin(), input->depth.data.end(), 0U);
  job.input = std::move(input);
  job.tray_roi_min_m = {-0.2, -0.2};
  job.tray_roi_max_m = {0.2, 0.2};
  job.tray_depth_resolution_m = 0.005;

  auto replay = std::make_shared<arm_cell_vision::CuboidReplayEvidence>();
  replay->rows = 80;
  replay->columns = 80;
  replay->resolution_m = 0.005;
  replay->origin_x_m = -0.2;
  replay->origin_y_m = -0.2;
  replay->frame_id = "Tray";
  replay->height_grid_m = cv::Mat(80, 80, CV_64FC1,
    std::numeric_limits<double>::quiet_NaN());
  replay->height_grid_m(cv::Rect(20, 35, 35, 1)).setTo(0.03);
  replay->height_grid_m.at<double>(79, 0) = 0.02;
  job.result.cuboid_replay_evidence = replay;

  job.result.cuboid_height_patches.resize(3);
  auto & strip = job.result.cuboid_height_patches[1];
  strip.height_m = 0.03;
  strip.cell_count = 35;
  strip.contour_count = 1;
  strip.fitted_edge_count = 1;
  for (int col = 20; col < 55; ++col) {
    geometry_msgs::msg::Point32 cell;
    cell.x = static_cast<float>(-0.2 + col * replay->resolution_m);
    cell.y = static_cast<float>(-0.2 + 35 * replay->resolution_m);
    strip.observed_cells_xy.push_back(cell);
  }
  strip.observed_contours_xy.push_back({
      geometry_msgs::msg::Point32(), geometry_msgs::msg::Point32(),
      geometry_msgs::msg::Point32(), geometry_msgs::msg::Point32()});
  auto & contour = strip.observed_contours_xy.back();
  contour[0].x = -0.1F; contour[0].y = -0.03F;
  contour[1].x = 0.075F; contour[1].y = -0.03F;
  contour[2].x = 0.075F; contour[2].y = -0.02F;
  contour[3].x = -0.1F; contour[3].y = -0.02F;
  strip.fitted_edges_xy_support.push_back({-0.1, -0.025, 0.075, -0.025, 1.0});
  strip.candidate_evaluation_attempt_count = 1;
  strip.geometry_valid_before_deduplication_count = 1;
  strip.generated_hypothesis_count = 1;

  auto & corner = job.result.cuboid_height_patches[2];
  corner.height_m = 0.02;
  corner.cell_count = 1;
  geometry_msgs::msg::Point32 corner_cell;
  corner_cell.x = -0.2F;
  corner_cell.y = 0.195F;
  corner.observed_cells_xy.push_back(corner_cell);

  arm_cell_vision::CuboidHypothesisVisualization candidate;
  candidate.candidate_id = 1;
  candidate.patch_index = 1;
  candidate.center_x_m = -0.0125;
  candidate.center_y_m = -0.025;
  candidate.height_m = 0.03;
  candidate.length_m = 0.175;
  candidate.width_m = 0.01;
  candidate.thickness_m = 0.03;
  candidate.confidence_grade = "HIGH";
  candidate.anchor_x_m = -0.1;
  candidate.anchor_y_m = -0.03;
  candidate.disposition = "qualified_geometry";
  job.result.cuboid_hypotheses.push_back(candidate);
  return job;
}

struct ProjectedTrayAxes
{
  cv::Matx22d image_transform;
  cv::Point2d tray_x_pixel_direction;
  cv::Point2d tray_y_pixel_direction;
};

ProjectedTrayAxes expected_tray_image_transform(
  const arm_cell_vision::VisionDetectorSnapshotJob & job)
{
  const auto & calibration = job.calibration.tray_from_camera;
  const auto & transform = calibration.transform;
  tf2::Quaternion tray_from_camera(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
  tray_from_camera.normalize();
  const tf2::Quaternion camera_from_tray = tray_from_camera.inverse();
  const tf2::Vector3 translation(
    transform.translation.x, transform.translation.y, transform.translation.z);
  constexpr double reference_step_m = 0.01;
  const std::array<tf2::Vector3, 3> tray_points{{
    tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(reference_step_m, 0.0, 0.0),
    tf2::Vector3(0.0, reference_step_m, 0.0)}};
  std::vector<cv::Point3d> camera_points;
  camera_points.reserve(tray_points.size());
  for (const auto & tray_point : tray_points) {
    const auto point = tf2::quatRotate(camera_from_tray, tray_point - translation);
    camera_points.emplace_back(point.x(), point.y(), point.z());
  }
  const auto & info = job.input->camera_info;
  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    info.k[0], info.k[1], info.k[2], info.k[3], info.k[4], info.k[5],
    info.k[6], info.k[7], info.k[8]);
  const cv::Mat distortion(info.d, true);
  std::vector<cv::Point2d> pixels;
  cv::projectPoints(camera_points, cv::Vec3d(0.0, 0.0, 0.0),
    cv::Vec3d(0.0, 0.0, 0.0), camera_matrix, distortion, pixels);
  const auto tray_x = pixels[1] - pixels[0];
  const auto tray_y = pixels[2] - pixels[0];
  const auto length_x = std::hypot(tray_x.x, tray_x.y);
  const auto length_y = std::hypot(tray_y.x, tray_y.y);
  const cv::Point2d x_axis(tray_x.x / length_x, tray_x.y / length_x);
  // The source Tray XY raster has +Y up, so its positive image-down basis is -Tray Y.
  const cv::Point2d y_down(-tray_y.x / length_y, -tray_y.y / length_y);
  const double determinant = x_axis.x * y_down.y - y_down.x * x_axis.y;
  double angle = 0.0;
  cv::Matx22d image_transform;
  if (determinant >= 0.0) {
    angle = std::atan2(x_axis.y - y_down.x, x_axis.x + y_down.y);
    image_transform = {std::cos(angle), -std::sin(angle),
      std::sin(angle), std::cos(angle)};
  } else {
    angle = std::atan2(x_axis.y + y_down.x, x_axis.x - y_down.y);
    image_transform = {std::cos(angle), std::sin(angle),
      std::sin(angle), -std::cos(angle)};
  }
  return {image_transform, x_axis, cv::Point2d(tray_y.x / length_y, tray_y.y / length_y)};
}

bool metadata_is_valid_json(const std::string & metadata)
{
  try {
    // OpenCV's JSON reader rejects empty arrays although they are valid JSON.
    auto parser_input = metadata;
    const std::string empty_ids = "\"candidate_ids\":[]";
    const auto empty_ids_offset = parser_input.find(empty_ids);
    if (empty_ids_offset != std::string::npos) {
      parser_input.replace(empty_ids_offset, empty_ids.size(), "\"candidate_ids\":[0]");
    }
    const cv::FileStorage parsed(parser_input,
      cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    return parsed.isOpened() && !parsed.root().empty();
  } catch (const cv::Exception &) {
    return false;
  }
}

TEST_F(VisionDetectorSnapshotTest, CalibrationSnapshotPairsTheProcessedObservationAndBootstrapSummary)
{
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  std::mutex mutex;
  bool saved_same = false;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-1",
    [&](const auto &, const auto & message) {saved_same = message.metadata_json.find("accepted_observations\":5") != std::string::npos; return true;},
    [&](const auto & message) {std::lock_guard<std::mutex> lock(mutex); published = message;});
  output.submit(calibration_job());
  output.shutdown();
  EXPECT_TRUE(saved_same);
  EXPECT_EQ(published.event_kind,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_COMPLETE);
  EXPECT_EQ(published.calibration_id, "cal-1");
  EXPECT_NE(published.metadata_json.find("rgb_camera"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("depth_camera"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("accepted_observations"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("valid_depth_sample_count\":9"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("median_signed_residual_m\":0.002"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("support_contact_tolerance_m\":0.01"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("configured_marker_height_m\":0.0002"), std::string::npos);
  EXPECT_FALSE(published.image_png.data.empty());
}

TEST_F(VisionDetectorSnapshotTest, MissingRgbKeepsDepthIdentityAndMarksImageUnavailable)
{
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "depth-only-run",
    [](const auto &, const auto &) {return true;},
    [&](const auto & message) {published = message;});
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  job.vision_run_id = "depth-only-run";
  job.acquisition_id = "depth-only-acq-1";
  job.target_id = "RawPart";
  job.processing_stage = "depth_observation";
  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
  input->rgb = sensor_msgs::msg::Image{};
  input->depth.header.frame_id = "camera_depth_optical_frame";
  input->depth.header.stamp.sec = 12;
  job.input = input;
  job.has_observation = true;
  job.has_detection_result = true;
  job.result.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SENSOR_ERROR;
  output.submit(std::move(job));
  output.shutdown();

  EXPECT_FALSE(published.image_png.data.empty());
  EXPECT_NE(published.metadata_json.find("\"rgb_available\":false"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"rgb\":null"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"aligned_depth\":{\"stamp\":{\"sec\":12"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"rgb_presentation_status\":\"not_received\""), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("rgb_depth_delta_ns"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"rgb_available\":false"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find(",,"), std::string::npos);
  EXPECT_EQ(published.diagnostic_record_json.find(",,"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"source_image_rendered\":false"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, DelayedSnapshotWorkerKeepsRgbSelectedBeforeEnqueue)
{
  using Snapshot = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  std::mutex worker_gate_mutex;
  std::condition_variable worker_gate_changed;
  bool save_entered = false;
  bool release_save = false;
  Snapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "pinned-rgb-run",
    [&](const auto & job, const auto &) {
      if (job.acquisition_id == "rgb-worker-blocker") {
        std::unique_lock<std::mutex> lock(worker_gate_mutex);
        save_entered = true;
        worker_gate_changed.notify_all();
        worker_gate_changed.wait(lock, [&] {return release_save;});
      }
      return true;
    },
    [&](const auto & message) {published = message;});

  arm_cell_vision::VisionDetectorSnapshotJob blocker;
  blocker.event_kind = Snapshot::DETECT_TARGET_FAILURE;
  blocker.vision_run_id = "pinned-rgb-run";
  blocker.acquisition_id = "rgb-worker-blocker";
  blocker.save = true;
  output.submit(std::move(blocker));
  {
    std::unique_lock<std::mutex> lock(worker_gate_mutex);
    ASSERT_TRUE(worker_gate_changed.wait_for(
        lock, std::chrono::seconds(2), [&] {return save_entered;}));
  }

  auto job = calibration_job();
  job.event_kind = Snapshot::DETECT_TARGET_SUCCESS;
  job.vision_run_id = "pinned-rgb-run";
  job.acquisition_id = "rgb-frozen-acq";
  job.target_id = "RawPart";
  job.save = true;
  auto selected_input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
  selected_input->depth.header.stamp.sec = 10;
  selected_input->depth.header.frame_id = "camera_color_optical_frame";
  selected_input->rgb.header.stamp.sec = 42;
  selected_input->rgb.header.frame_id = "camera_color_optical_frame";
  selected_input->rgb_presentation_status = "available";
  job.input = selected_input;
  job.has_observation = true;
  job.has_detection_result = true;
  job.result.result_code = arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  output.submit(std::move(job));

  // The worker receives only the already selected input image; it has no ingress resolver.
  {
    std::lock_guard<std::mutex> lock(worker_gate_mutex);
    release_save = true;
  }
  worker_gate_changed.notify_all();
  output.shutdown();

  EXPECT_NE(published.metadata_json.find("\"rgb\":{\"stamp\":{\"sec\":42"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"aligned_depth\":{\"stamp\":{\"sec\":10"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("snapshot_rgb_match"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, InvalidRgbIsReportedSeparatelyFromSuccessfulDepthDetection)
{
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "invalid-rgb-run",
    [](const auto &, const auto &) {return true;},
    [&](const auto & message) {published = message;});
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
  job.vision_run_id = "invalid-rgb-run";
  job.acquisition_id = "invalid-rgb-acq";
  job.target_id = "RawPart";
  job.processing_stage = "target_pose_estimation";
  job.terminal_result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  job.result.result_code = job.terminal_result_code;
  job.has_detection_result = true;
  job.depth_scene_stability.status = arm_cell_vision::DepthSceneStatus::kStable;
  job.depth_scene_stability.qualifying_observations = 5;
  job.depth_scene_stability.required_observations = 5;
  job.stable_depth_stamp_ns = 22'000'000'000LL;
  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
  input->rgb.encoding = "bgr8";
  input->rgb_presentation_status = "encoding_unsupported";
  input->depth.header.stamp.sec = 22;
  job.input = input;
  output.submit(std::move(job));
  output.shutdown();

  ASSERT_FALSE(published.image_png.data.empty());
  EXPECT_NE(published.metadata_json.find("\"rgb_available\":false"), std::string::npos);
  EXPECT_NE(published.metadata_json.find(
      "\"rgb_presentation_status\":\"encoding_unsupported\""), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"source_image_rendered\":false"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"event_kind\":\"DETECT_TARGET_SUCCESS\""),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"aligned_depth\":{\"stamp\":{\"sec\":22"),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"stable_depth_stamp_ns\":22000000000"),
    std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, CalibrationProgressSnapshotPublishesRejectedDepthEvidenceWithoutSaving)
{
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  bool saved = false;
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_PROGRESS;
  job.save = false;
  job.calibration.ready = false;
  job.calibration.accepted_frames = 0;
  auto & validation = job.calibration.recent_depth_validations.front();
  validation.median_signed_residual_m = 0.012;
  validation.minimum_signed_residual_m = 0.011;
  validation.maximum_signed_residual_m = 0.013;
  validation.accepted = false;
  validation.reason = "support depth conflicts with configured support plane";
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-1",
    [&](const auto &, const auto &) {saved = true; return true;},
    [&](const auto & message) {published = message;});
  output.submit(std::move(job));
  output.shutdown();
  EXPECT_FALSE(saved);
  EXPECT_EQ(published.event_kind,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_PROGRESS);
  EXPECT_NE(published.metadata_json.find("ready\":false"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("median_signed_residual_m\":0.012"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("accepted\":false"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("support depth conflicts with configured support plane"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, TerminalSnapshotSaveFailureDoesNotSuppressPublication)
{
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  bool save_attempted = false;
  arm_cell_vision::VisionDetectorSnapshotJob terminal = calibration_job();
  terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "acq-7";
  terminal.terminal_result_code = 2;
  terminal.terminal_diagnostic = "deadline expired";
  arm_cell_vision::VisionRequestDiagnosticRecord request_diagnostics;
  request_diagnostics.ingress.detailed_capture_enabled = true;
  request_diagnostics.ingress.sample_detail_dropped = 2;
  arm_cell_vision::VisionIngressPairDiagnostic pair;
  pair.rgb_frame_id = "rgb-optical";
  pair.depth_frame_id = "depth-optical";
  pair.rgb_stamp_ns = 10;
  pair.depth_stamp_ns = 11;
  pair.rgb_receipt_monotonic_ns = 20;
  pair.depth_receipt_monotonic_ns = 21;
  arm_cell_vision::VisionIngressPairFreshnessDiagnostic freshness;
  freshness.evaluation_steady_ns = 30;
  freshness.evaluation_clock_ns = 20;
  freshness.rgb_source_stamp_ns = 10;
  freshness.depth_source_stamp_ns = 11;
  freshness.rgb_source_minus_clock_ns = -10;
  freshness.depth_source_minus_clock_ns = -9;
  freshness.clock_received_steady_ns = 24;
  freshness.clock_receipt_age_ns = 6;
  freshness.maximum_absolute_delta_ns = 5;
  freshness.accepted = false;
  freshness.reason = "rgb_source_clock_delta_exceeded";
  freshness.clock_observation_status = arm_cell_vision::IngressStatus::kWaiting;
  freshness.clock_sample_relation = arm_cell_vision::VisionClockSampleRelation::kUnchanged;
  freshness.clock_sample_count = 8;
  freshness.clock_epoch = 2;
  freshness.clock_rollback_observed = true;
  freshness.epoch_invalidated = true;
  pair.freshness_evaluation = freshness;
  request_diagnostics.ingress.pair_details.push_back(pair);
  request_diagnostics.ingress.source_stamp_freshness_evaluated = 1;
  request_diagnostics.ingress.source_stamp_freshness_rejected = 1;
  request_diagnostics.ingress.source_stamp_freshness_clock_delta_rejected = 1;
  terminal.request_diagnostics = request_diagnostics;
  terminal.processing_stage = "RawPart profile evaluation";
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-2",
    [&](const auto &, const auto &) {save_attempted = true; return false;},
    [&](const auto & message) {published = message;});
  output.submit(std::move(terminal));
  output.shutdown();
  EXPECT_TRUE(save_attempted);
  EXPECT_EQ(published.event_kind,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE);
  EXPECT_EQ(published.acquisition_id, "acq-7");
  EXPECT_NE(published.diagnostic_record_json.find("deadline expired"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("RawPart profile evaluation"), std::string::npos);
  EXPECT_NE(
    published.diagnostic_record_json.find(
      "\"request_diagnostics\":{\"request_id\":"), std::string::npos);
  EXPECT_NE(
    published.diagnostic_record_json.find(
      "\"pairing\":{\"scope\":\"optional_rgbd_calibration_association\""), std::string::npos);
  EXPECT_NE(
    published.diagnostic_record_json.find(
      "\"freshness\":{\"scope\":\"calibration_rgbd_association\""), std::string::npos);
  EXPECT_NE(
    published.diagnostic_record_json.find(
      "\"depth_source_freshness\":{\"scope\":\"detect_target_depth_source\""), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"sample_details_dropped\":2"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"rgb_frame_id\":\"rgb-optical\""), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"depth_receipt_steady_ns\":21"), std::string::npos);
  EXPECT_NE(
    published.diagnostic_record_json.find("\"signed_delta_definition\":\"source_stamp_ns - evaluation_clock_ns\""),
    std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"rgb_source_minus_clock_ns\":-10"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"depth_source_minus_clock_ns\":-9"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"clock_receipt_age_ns\":6"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"maximum_absolute_delta_ns\":5"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"reason\":\"rgb_source_clock_delta_exceeded\""), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"clock_sample_relation\":\"unchanged\""), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"clock_observation_status\":\"waiting\""), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("request_diagnostics"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("terminal_result"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"clock_rollback_observed\":true"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"clock_delta_rejected\":1"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, TerminalAcquisitionWritesDiagnosticAndReplayByAcquisitionId)
{
  const auto root = temporary_snapshot_root("diagnostic-record");
  const auto diagnostic_root = root / "20261020" / "120000" / "vision";
  std::filesystem::remove_all(root);
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
  job.acquisition_id = "acq-replay-1";
  job.vision_run_id = "run-replay-1";
  job.algorithm = "cuboid_top_face";
  job.target_id = "RawPart";
  job.terminal_result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  job.has_detection_result = true;
  job.result.result_code = arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  job.result.has_target_pose = true;
  job.result.target_pose.header.frame_id = "base_link";
  job.result.target_pose.pose.position.x = -0.12;
  job.terminal_selection_available = true;
  job.result.algorithm = "cuboid_top_face";
  job.result.ordered_candidates = {job.result};
  job.result.has_target_pose = false;
  arm_cell_vision::DetectorProfile applied_profile;
  applied_profile.target_id = "RawPart";
  applied_profile.algorithm = "cuboid_top_face";
  applied_profile.cuboid.grid_resolution_m = 0.0031;
  applied_profile.tray_roi_min_m = {-0.17, -0.23};
  applied_profile.tray_roi_max_m = {0.19, 0.21};
  applied_profile.top_surface_geometry_tolerance_m = 0.0042;
  applied_profile.minimum_observation_coverage = 0.61;
  job.selected_profile = applied_profile;
  job.depth_scene_stability.reason = "awaiting_stable_observations";
  job.result.algorithm = "cuboid_top_face";
  arm_cell_vision::CuboidHeightPatchDiagnostic patch_diagnostic;
  patch_diagnostic.height_m = 0.03;
  patch_diagnostic.pre_object_rejected_count = 0;
  job.result.cuboid_height_patches.push_back(patch_diagnostic);
  arm_cell_vision::CuboidStageDiagnostic patch_stage;
  patch_stage.stage = "height_patch_formation";
  patch_stage.status = "computed";
  patch_stage.reason = "no_height_levels_in_workspace_roi";
  patch_stage.auxiliary_counts["workspace_roi_cells"] = 4;
  patch_stage.auxiliary_counts["formed_height_patches"] = 0;
  job.result.cuboid_stages.push_back(patch_stage);
  arm_cell_vision::CuboidStageDiagnostic geometry_stage;
  geometry_stage.stage = "dimension_and_geometry_validation";
  geometry_stage.status = "computed";
  geometry_stage.count_unit = "multiple_explicit_units";
  geometry_stage.candidate_evaluation_attempt_count = 3;
  geometry_stage.rejected_candidate_evaluation_count = 2;
  geometry_stage.generated_count = 1;
  geometry_stage.rejected_by_reason["measured_dimension_mismatch"] = 2;
  job.result.cuboid_stages.push_back(geometry_stage);
  arm_cell_vision::CuboidStageDiagnostic partition_stage;
  partition_stage.stage = "partitioned_or_isolated_classification";
  partition_stage.status = "computed";
  partition_stage.count_unit = "candidate";
  partition_stage.input_count = 1;
  partition_stage.output_count = 1;
  partition_stage.rejected_count = 0;
  partition_stage.outcome_by_status["unique_complete_partition"] = 1;
  job.result.cuboid_stages.push_back(partition_stage);
  job.result.cuboid_replay_evidence = std::make_shared<arm_cell_vision::CuboidReplayEvidence>();
  job.result.cuboid_replay_evidence->rows = 1;
  job.result.cuboid_replay_evidence->columns = 2;
  job.result.cuboid_replay_evidence->resolution_m = 0.0025;
  job.result.cuboid_replay_evidence->origin_x_m = -0.1;
  job.result.cuboid_replay_evidence->origin_y_m = -0.2;
  job.result.cuboid_replay_evidence->frame_id = "Tray";
  job.result.cuboid_replay_evidence->height_grid_m = cv::Mat(1, 2, CV_64FC1);
  job.result.cuboid_replay_evidence->height_grid_m.at<double>(0, 0) = 1.0;
  job.result.cuboid_replay_evidence->height_grid_m.at<double>(0, 1) =
    std::numeric_limits<double>::quiet_NaN();
  job.result.cuboid_replay_evidence->image_boundary_mask = cv::Mat(1, 2, CV_8UC1);
  job.result.cuboid_replay_evidence->image_boundary_mask.at<uint8_t>(0, 0) = 0;
  job.result.cuboid_replay_evidence->image_boundary_mask.at<uint8_t>(0, 1) = 255;
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root / "snapshots", "run-replay-1", {},
    [&](const auto & message) {published.push_back(message);}, "", diagnostic_root, "live-session-1");
  output.submit(std::move(job));
  output.shutdown();

  const auto record_path = diagnostic_root / "acq-replay-1.json";
  const auto replay_path = diagnostic_root / "acq-replay-1.height_grid.f64le";
  const auto boundary_path = diagnostic_root / "acq-replay-1.image_boundary.u8";
  ASSERT_EQ(published.size(), 1U);
  ASSERT_TRUE(std::filesystem::exists(record_path));
  ASSERT_TRUE(std::filesystem::exists(replay_path));
  ASSERT_TRUE(std::filesystem::exists(boundary_path));
  std::ifstream record_stream(record_path);
  const std::string record(
    (std::istreambuf_iterator<char>(record_stream)), std::istreambuf_iterator<char>());
  EXPECT_NE(record.find("\"acquisition_id\":\"acq-replay-1\""), std::string::npos);
  EXPECT_NE(record.find("\"record_storage_status\":\"saved\""), std::string::npos);
  EXPECT_NE(record.find("\"live_test_session_id\":\"live-session-1\""), std::string::npos);
  EXPECT_NE(record.find("\"stage\":\"height_patch_formation\""), std::string::npos);
  EXPECT_NE(record.find("\"generated_before_deduplication_count\":1"), std::string::npos);
  EXPECT_NE(record.find("\"candidate_evaluation_attempt_count\":3"), std::string::npos);
  EXPECT_NE(record.find("\"pre_object_rejected_count\":0"), std::string::npos);
  EXPECT_NE(record.find("\"outcome_status_count_unit\":\"height_patch\",\"outcome_by_status\":{\"unique_complete_partition\":1}"),
    std::string::npos);
  const auto partition_start = record.find("\"stage\":\"partitioned_or_isolated_classification\"");
  ASSERT_NE(partition_start, std::string::npos);
  const auto partition_end = record.find('}', partition_start);
  ASSERT_NE(partition_end, std::string::npos);
  EXPECT_EQ(record.substr(partition_start, partition_end - partition_start).find("rejected_by_reason"),
    std::string::npos);
  EXPECT_NE(record.find("\"grid_format\":\"float64_little_endian_row_major_nan_invalid\""), std::string::npos);
  EXPECT_NE(record.find("\"status\":\"saved\""), std::string::npos);
  std::smatch grid_resolution_match;
  ASSERT_TRUE(std::regex_search(
      record, grid_resolution_match, std::regex("\\\"grid_resolution_m\\\":([^,}]+)")));
  EXPECT_DOUBLE_EQ(std::stod(grid_resolution_match[1].str()), 0.0031);
  EXPECT_NE(record.find("\"selection_source\":\"acquisition_target_id\""), std::string::npos);
  EXPECT_NE(record.find("\"algorithm\":\"cuboid_top_face\""), std::string::npos);
  std::smatch profile_values;
  ASSERT_TRUE(std::regex_search(
      record, profile_values, std::regex("\\\"tray_roi_min_m\\\":\\[([^,]+),([^\\]]+)\\]")));
  EXPECT_DOUBLE_EQ(std::stod(profile_values[1].str()), -0.17);
  EXPECT_DOUBLE_EQ(std::stod(profile_values[2].str()), -0.23);
  ASSERT_TRUE(std::regex_search(
      record, profile_values,
      std::regex("\\\"top_surface_geometry_tolerance_m\\\":([^,}]+)")));
  EXPECT_DOUBLE_EQ(std::stod(profile_values[1].str()), 0.0042);
  ASSERT_TRUE(std::regex_search(
      record, profile_values, std::regex("\\\"minimum_observation_coverage\\\":([^,}]+)")));
  EXPECT_DOUBLE_EQ(std::stod(profile_values[1].str()), 0.61);
  EXPECT_NE(record.find("\"single_observation\":\"representative_processed_depth_observation\""),
    std::string::npos);
  EXPECT_NE(record.find("\"acquisition_selection\":{\"status\":\"available\""),
    std::string::npos);
  EXPECT_EQ(record.find("\"position_m\":[0.12,0,0]"), std::string::npos);
  EXPECT_NE(record.find("\"position_m\":[-0.12,0,0]"),
    std::string::npos);
  EXPECT_EQ(published.front().metadata_json.find("\"target_pose_position_m\":[-0.12,0,0]"),
    std::string::npos);
  EXPECT_NE(record.find("\"stability\":\"depth_scene_acquisition_aggregate\""), std::string::npos);
  EXPECT_EQ(record, published.front().diagnostic_record_json + "\n");
  EXPECT_EQ(published.front().metadata_json.find("cuboid_stages"), std::string::npos);
  EXPECT_EQ(published.front().metadata_json.find("request_diagnostics"), std::string::npos);
  EXPECT_EQ(std::filesystem::file_size(replay_path), 2U * sizeof(double));
  EXPECT_EQ(std::filesystem::file_size(boundary_path), 2U);
  std::ifstream replay_stream(replay_path, std::ios::binary);
  std::vector<uint8_t> bytes(
    (std::istreambuf_iterator<char>(replay_stream)), std::istreambuf_iterator<char>());
  ASSERT_EQ(bytes.size(), 16U);
  EXPECT_EQ(bytes[6], 0xf0U);
  EXPECT_EQ(bytes[7], 0x3fU);
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, ReplayDimensionMismatchIsUnavailableAndWritesNoBinary)
{
  const auto root = temporary_snapshot_root("diagnostic-invalid-grid");
  const auto diagnostic_root = root / "diagnostic_records";
  std::filesystem::remove_all(root);
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  job.acquisition_id = "acq-invalid-grid";
  job.algorithm = "cuboid_top_face";
  job.result.cuboid_replay_evidence = std::make_shared<arm_cell_vision::CuboidReplayEvidence>();
  job.result.cuboid_replay_evidence->rows = 2;
  job.result.cuboid_replay_evidence->columns = 2;
  job.result.cuboid_replay_evidence->height_grid_m = cv::Mat(1, 2, CV_64FC1, cv::Scalar(0.03));
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root / "snapshots", "run-invalid-grid", {},
    [&](const auto & message) {published.push_back(message);}, "", diagnostic_root);
  output.submit(std::move(job));
  output.shutdown();
  ASSERT_EQ(published.size(), 1U);
  EXPECT_NE(published.front().diagnostic_record_json.find("\"status\":\"invalid\""),
    std::string::npos);
  EXPECT_NE(published.front().diagnostic_record_json.find("grid_or_mask_dimensions_or_type_mismatch"),
    std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(diagnostic_root / "acq-invalid-grid.height_grid.f64le"));
  EXPECT_TRUE(std::filesystem::exists(diagnostic_root / "acq-invalid-grid.json"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, PartialReplayWriteListsOnlySuccessfullyPublishedFiles)
{
  const auto root = temporary_snapshot_root("diagnostic-partial-grid");
  const auto diagnostic_root = root / "diagnostic_records";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(diagnostic_root / "acq-partial.height_grid.f64le");
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  job.acquisition_id = "acq-partial";
  job.algorithm = "cuboid_top_face";
  job.result.cuboid_replay_evidence = std::make_shared<arm_cell_vision::CuboidReplayEvidence>();
  job.result.cuboid_replay_evidence->rows = 1;
  job.result.cuboid_replay_evidence->columns = 2;
  job.result.cuboid_replay_evidence->height_grid_m = cv::Mat(1, 2, CV_64FC1, cv::Scalar(0.03));
  job.result.cuboid_replay_evidence->image_boundary_mask = cv::Mat(1, 2, CV_8UC1, cv::Scalar(1));
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root / "snapshots", "run-partial-grid", {},
    [&](const auto & message) {published.push_back(message);}, "", diagnostic_root);
  output.submit(std::move(job));
  output.shutdown();
  ASSERT_EQ(published.size(), 1U);
  const auto & json = published.front().diagnostic_record_json;
  EXPECT_NE(json.find("\"status\":\"partial\""), std::string::npos);
  EXPECT_NE(json.find("\"record_storage_status\":\"saved\""), std::string::npos);
  EXPECT_NE(json.find("\"grid_status\":\"save_failed\""), std::string::npos);
  EXPECT_NE(json.find("\"image_boundary_mask_status\":\"saved\""), std::string::npos);
  EXPECT_EQ(json.find("acq-partial.height_grid.f64le\""), std::string::npos);
  EXPECT_TRUE(std::filesystem::exists(diagnostic_root / "acq-partial.image_boundary.u8"));
  EXPECT_TRUE(std::filesystem::exists(diagnostic_root / "acq-partial.json"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, DiagnosticStorageFailureStillPublishesRecordAndSnapshot)
{
  const auto root = temporary_snapshot_root("diagnostic-storage-failure");
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root);
  const auto blocked_root = root / "file-is-not-a-directory";
  {std::ofstream blocker(blocked_root); blocker << "reserved";}
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  job.acquisition_id = "acq-storage-failure";
  job.algorithm = "cuboid_top_face";
  job.result.algorithm = "cuboid_top_face";
  job.result.cuboid_replay_evidence =
    std::make_shared<arm_cell_vision::CuboidReplayEvidence>();
  job.result.cuboid_replay_evidence->rows = 1;
  job.result.cuboid_replay_evidence->columns = 1;
  job.result.cuboid_replay_evidence->height_grid_m = cv::Mat(1, 1, CV_64FC1, cv::Scalar(0.03));
  job.result.cuboid_replay_evidence->frame_id = "Tray";
  job.terminal_result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
  std::optional<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root / "snapshots", "run-storage-failure", {},
    [&](const auto & message) {published = message;}, "", blocked_root / "vision", "session-1");
  output.submit(std::move(job));
  output.shutdown();
  ASSERT_TRUE(published);
  EXPECT_EQ(published->acquisition_id, "acq-storage-failure");
  EXPECT_NE(published->metadata_json.find("image_available"), std::string::npos);
  EXPECT_NE(published->diagnostic_record_json.find("\"acquisition_id\":\"acq-storage-failure\""),
    std::string::npos);
  EXPECT_NE(published->diagnostic_record_json.find("\"record_storage_status\":\"unavailable\""),
    std::string::npos);
  EXPECT_NE(published->diagnostic_record_json.find("create diagnostic directory"),
    std::string::npos);
  EXPECT_NE(published->diagnostic_record_json.find("\"status\":\"save_failed\""),
    std::string::npos);
  EXPECT_NE(published->diagnostic_record_json.find("\"grid_status\":\"save_failed\""),
    std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(blocked_root / "vision" / "acq-storage-failure.json"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, TerminalFailureKeepsRejectedCalibrationDepthEvidence)
{
  arm_cell_vision::VisionDetectorSnapshotJob terminal = calibration_job();
  terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "acq-calibration-failure";
  terminal.calibration.ready = false;
  terminal.calibration.recent_depth_validations.back().median_signed_residual_m = -0.012;
  terminal.calibration.recent_depth_validations.back().minimum_signed_residual_m = -0.014;
  terminal.calibration.recent_depth_validations.back().maximum_signed_residual_m = -0.010;
  terminal.calibration.recent_depth_validations.back().accepted = false;
  terminal.calibration.recent_depth_validations.back().reason =
    "support_depth_conflicts_with_configured_support_plane";
  terminal.terminal_result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_GEOMETRY_ERROR;
  terminal.terminal_diagnostic = "support depth conflicts with configured support plane";
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-calibration-failure",
    [](const auto &, const auto &) {return true;},
    [&](const auto & message) {published = message;});

  output.submit(std::move(terminal));
  output.shutdown();

  EXPECT_NE(published.metadata_json.find("calibration_summary"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("median_signed_residual_m\":-0.012"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("support_contact_tolerance_m\":0.01"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("configured_marker_height_m\":0.0002"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("support_depth_conflicts_with_configured_support_plane"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, TerminalRejectionCarriesItsAnnotationAndDiagnostics)
{
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  auto terminal = calibration_job();
  terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "acq-9";
  terminal.target_id = "RawPart";
  terminal.has_detection_result = true;
  terminal.terminal_result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND;
  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*terminal.input);
  input->rgb.width = 160;
  input->rgb.height = 160;
  input->rgb.step = 480;
  input->rgb.data.assign(160 * 160 * 3, 32);
  input->rgb.header.stamp.sec = 21;
  input->depth.header.stamp.sec = 21;
  terminal.input = std::move(input);
  terminal.result.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND;
  terminal.result.has_rawpart_diagnostics = true;
  terminal.result.rawpart_workspace_xyz_points = 42;
  terminal.result.rawpart_candidate_count = 2;
  terminal.result.rawpart_profile_match_count = 0;
  terminal.result.rawpart_rejected_by_reason["dimension_mismatch"] = 2;
  arm_cell_vision::RawPartCandidateDiagnostic candidate;
  candidate.candidate_id = 3;
  candidate.position_x_m = 0.12;
  candidate.position_y_m = -0.03;
  candidate.measured_width_m = 0.055;
  candidate.measured_height_m = 0.055;
  candidate.expected_width_m = 0.07;
  candidate.expected_height_m = 0.07;
  candidate.width_tolerance_m = 0.01;
  candidate.height_tolerance_m = 0.01;
  candidate.measured_vertical_height_m = 0.08;
  candidate.expected_vertical_height_m = 0.08;
  candidate.vertical_height_tolerance_m = 0.02;
  candidate.orientation_symmetry_order = 0;
  candidate.observed_yaw_valid = false;
  candidate.inlier_fraction = 0.98;
  candidate.coverage = 0.91;
  candidate.edge_support = 0.83;
  candidate.max_normalized_residual = 1.5;
  candidate.profile_match = false;
  candidate.rejection_reason = "dimension_mismatch";
  candidate.greedy_rank = 0;
  candidate.selected_for_target = false;
  candidate.fitted_outline_support.resize(4);
  candidate.fitted_outline_pixels.resize(4);
  candidate.fitted_outline_pixels[0].x = 40.0F;
  candidate.fitted_outline_pixels[0].y = 100.0F;
  candidate.fitted_outline_pixels[1].x = 80.0F;
  candidate.fitted_outline_pixels[1].y = 100.0F;
  candidate.fitted_outline_pixels[2].x = 80.0F;
  candidate.fitted_outline_pixels[2].y = 140.0F;
  candidate.fitted_outline_pixels[3].x = 40.0F;
  candidate.fitted_outline_pixels[3].y = 140.0F;
  candidate.orientation_axis_pixels.resize(2);
  candidate.orientation_axis_pixels[0].x = 50.0F;
  candidate.orientation_axis_pixels[0].y = 120.0F;
  candidate.orientation_axis_pixels[1].x = 70.0F;
  candidate.orientation_axis_pixels[1].y = 120.0F;
  terminal.result.rawpart_candidate_diagnostics.push_back(candidate);
  terminal.result.rawpart_absence_confirmation_available = true;
  terminal.result.rawpart_absence_confirmed = true;
  terminal.depth_scene_stability.status = arm_cell_vision::DepthSceneStatus::kStable;
  terminal.depth_scene_stability.qualifying_observations = 5;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-5", {},
    [&](const auto & message) {published = message;});
  output.submit(std::move(terminal));
  output.shutdown();
  const auto snapshot = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(snapshot.empty());
  const auto candidate_pixel = snapshot.at<cv::Vec3b>(100, 60);
  EXPECT_TRUE(candidate_pixel[0] != 0 || candidate_pixel[1] != 0 || candidate_pixel[2] != 255);
  const auto axis_pixel = snapshot.at<cv::Vec3b>(120, 60);
  EXPECT_TRUE(axis_pixel[0] != 255 || axis_pixel[1] != 255 || axis_pixel[2] != 0);
  EXPECT_NE(published.metadata_json.find("\"sec\":21"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"schema_version\":19"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"workspace_xyz_points\":42"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"dimension_mismatch\":2"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"absence_confirmed\":true"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"candidate_id\":3"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"measured_dimensions_m\":[0.055,0.055,0.08]"),
    std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"orientation_symmetry_order\":0"), std::string::npos);
  EXPECT_EQ(published.diagnostic_record_json.find("orientation_uncertainty_rad"), std::string::npos);
  EXPECT_EQ(published.diagnostic_record_json.find("fit_residual_m"), std::string::npos);
  EXPECT_EQ(published.diagnostic_record_json.find("observed_yaw_rad"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"rejection_reason\":\"dimension_mismatch\""),
    std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"max_normalized_residual\":1.5"), std::string::npos);
  EXPECT_EQ(published.diagnostic_record_json.find("\"greedy_rank\":"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"fitted_outline_pixels\":["), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"qualifying_observations\":5"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("rawpart"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, TerminalRgbSnapshotKeepsVisionAnnotationWithoutOccupancyOverlay)
{
  auto terminal = calibration_job();
  terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "occupancy-overlay";
  terminal.has_detection_result = true;
  terminal.result.has_target_pose = true;
  terminal.result.target_pose.header.frame_id = "base_link";
  terminal.result.target_pose.pose.orientation.w = 1.0;
  terminal.result.object_region_valid = true;
  terminal.result.object_region_x = 30;
  terminal.result.object_region_y = 30;
  terminal.result.object_region_width = 40;
  terminal.result.object_region_height = 40;
  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*terminal.input);
  input->rgb.width = 100;
  input->rgb.height = 100;
  input->rgb.step = 300;
  input->rgb.data.assign(30000, 0);
  input->rgb.header.frame_id = "camera_optical";
  input->camera_info.k[0] = 80.0;
  input->camera_info.k[2] = 50.0;
  input->camera_info.k[4] = 80.0;
  input->camera_info.k[5] = 50.0;
  input->optical_to_base.header.frame_id = "base_link";
  input->optical_to_base.child_frame_id = "camera_optical";
  input->optical_to_base.transform.rotation.w = 1.0;
  terminal.input = std::move(input);
  auto first_candidate = terminal.result;
  first_candidate.ordered_candidates.clear();
  auto second_candidate = first_candidate;
  terminal.result.ordered_candidates = {first_candidate, second_candidate};

  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-occupancy",
    {}, [&](const auto & message) {published = message;});
  output.submit(std::move(terminal));
  output.shutdown();

  const auto snapshot = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(snapshot.empty());
  const auto edge = snapshot.at<cv::Vec3b>(50, 30);
  EXPECT_EQ(edge, cv::Vec3b(0, 255, 0));
  EXPECT_NE(published.metadata_json.find("\"schema_version\":19"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("occupancy_overlay"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("projected_count"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("non_target_occupancy"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, QualifiedCuboidUsesHighConfidenceGreenAndRejectedIsOmitted)
{
  auto terminal = cuboid_projection_job("plumb_bob");
  terminal.acquisition_id = "qualified-outlines";
  terminal.result.rawpart_selected_candidate = "0";
  auto second = terminal.result.cuboid_hypotheses.front();
  second.candidate_id = 2;
  second.center_x_m = 0.4;
  auto rejected = second;
  rejected.candidate_id = 3;
  rejected.center_x_m = 0.8;
  rejected.disposition = "rejected_geometry";
  terminal.result.cuboid_hypotheses.push_back(second);
  terminal.result.cuboid_hypotheses.push_back(rejected);

  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-qualified-outlines", {},
    [&](const auto & message) {published = message;});
  output.submit(std::move(terminal));
  output.shutdown();

  const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_GE(image.cols, 320);
  EXPECT_EQ(image.rows, 240);
  EXPECT_NE(published.metadata_json.find("\"disturbance_candidates\":[]"), std::string::npos);
  EXPECT_EQ(image.at<cv::Vec3b>(110, 130), cv::Vec3b(0, 200, 0));
  EXPECT_EQ(image.at<cv::Vec3b>(110, 170), cv::Vec3b(0, 200, 0));
  EXPECT_NE(image.at<cv::Vec3b>(110, 230), cv::Vec3b(0, 200, 0));
  EXPECT_NE(published.metadata_json.find("\"schema_version\":19"), std::string::npos);
  EXPECT_EQ(published.diagnostic_record_json.find("priority"), std::string::npos);
}

TEST_F(
  VisionDetectorSnapshotTest,
  DisturbanceCandidatesShareRgbSnapshotWithProjectedCategoryMarkers)
{
  using Candidate = arm_cell_interfaces::msg::DisturbanceCandidate;
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  using ResultCode = arm_cell_interfaces::msg::DetectTargetResultCode;
  auto terminal = calibration_job();
  terminal.event_kind = Message::DETECT_TARGET_SUCCESS;
  terminal.acquisition_id = "disturbance-visualization";
  terminal.terminal_scene_generation = 42;
  terminal.has_detection_result = true;
  terminal.algorithm = "cuboid_top_face";
  terminal.result.result_code = ResultCode::DETECT_RESULT_SUCCESS;
  terminal.result.object_region_valid = true;
  terminal.result.object_region_x = 20;
  terminal.result.object_region_y = 20;
  terminal.result.object_region_width = 20;
  terminal.result.object_region_height = 20;

  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*terminal.input);
  input->rgb.width = 200;
  input->rgb.height = 160;
  input->rgb.step = 600;
  input->rgb.data.assign(input->rgb.step * input->rgb.height, 0);
  input->rgb.header.frame_id = "camera_optical";
  input->camera_info.width = input->rgb.width;
  input->camera_info.height = input->rgb.height;
  input->camera_info.header.frame_id = "camera_optical";
  input->camera_info.distortion_model = "plumb_bob";
  input->camera_info.k = {100.0, 0.0, 100.0, 0.0, 100.0, 80.0, 0.0, 0.0, 1.0};
  input->camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  input->depth.header.frame_id = "camera_optical";
  input->optical_to_base.header.frame_id = "base_link";
  input->optical_to_base.child_frame_id = "camera_optical";
  input->optical_to_base.transform.translation.x = 0.1;
  input->optical_to_base.transform.translation.z = 1.0;
  input->optical_to_base.transform.rotation.w = 1.0;
  terminal.input = input;

  const auto make_candidate = [&](
    const std::string & id, uint8_t category, double x, double height) {
      Candidate candidate;
      candidate.candidate_id = id;
      candidate.category = category;
      candidate.center.header = input->depth.header;
      candidate.center.header.frame_id = "base_link";
      candidate.center.pose.position.x = x;
      candidate.center.pose.position.z = 3.0;
      candidate.center.pose.orientation.w = 1.0;
      candidate.observed_height_m = height;
      return candidate;
    };
  terminal.result.disturbance_candidates = {
    make_candidate("disturbance-1", Candidate::CATEGORY_INCLINED_SURFACE, -0.3, 0.03),
    make_candidate("disturbance-2", Candidate::CATEGORY_ELEVATED_UNRESOLVED, 0.1, 0.05),
    make_candidate("disturbance-3", Candidate::CATEGORY_OTHER_UNRESOLVED, 0.5, 0.07)};

  terminal.result.disturbance_regions.push_back(
    {"disturbance-1", 3, 1, 2, 4, {7, 9}, "footprint_intersection", "existing_region_low_or_unresolved_hypotheses"});

  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, temporary_snapshot_root("disturbance-visualization"), "disturbance-run", {},
    [&](const auto & message) {published = message;});
  output.submit(std::move(terminal));
  output.shutdown();

  const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.rows, 160);
  EXPECT_GE(image.cols, 200);
  EXPECT_EQ(image.at<cv::Vec3b>(80, 80), cv::Vec3b(0, 165, 255));
  EXPECT_EQ(image.at<cv::Vec3b>(80, 100), cv::Vec3b(0, 165, 255));
  EXPECT_EQ(image.at<cv::Vec3b>(80, 120), cv::Vec3b(0, 165, 255));
  EXPECT_EQ(image.at<cv::Vec3b>(30, 20), cv::Vec3b(0, 255, 0));
  for (const auto * json : {&published.diagnostic_record_json}) {
    EXPECT_NE(json->find("\"observation_provenance\":{\"reason\":\"existing_region_low_or_unresolved_hypotheses\""),
      std::string::npos);
    EXPECT_NE(json->find("\"source_boundary_id\":3,\"outline_id\":1,\"patch_index\":2,\"rectangle_id\":4,\"representative_method\":\"footprint_intersection\",\"hypothesis_ids\":[7,9]"),
      std::string::npos);
  }

  EXPECT_NE(
    published.metadata_json.find("\"acquisition_id\":\"disturbance-visualization\""),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"scene_generation\":42"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"schema_version\":19"), std::string::npos);
  EXPECT_NE(
    published.metadata_json.find("\"disturbance_inclined_color_rgb\":\"#FF00FF\""),
    std::string::npos);
  EXPECT_NE(
    published.metadata_json.find("\"disturbance_elevated_color_rgb\":\"#00FFFF\""),
    std::string::npos);
  EXPECT_NE(
    published.metadata_json.find("\"disturbance_other_color_rgb\":\"#FFFFFF\""),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find(
      "\"candidate_id\":\"disturbance-1\",\"category\":\"INCLINED_SURFACE\",\"observed_height_m\":0.03,\"center\":{\"frame_id\":\"base_link\",\"stamp_ns\":10000000000,\"position_m\":[-0.3,0,3]}"),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find(
      "\"candidate_id\":\"disturbance-2\",\"category\":\"ELEVATED_UNRESOLVED\",\"observed_height_m\":0.05,\"center\":{\"frame_id\":\"base_link\",\"stamp_ns\":10000000000,\"position_m\":[0.1,0,3]}"),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find(
      "\"candidate_id\":\"disturbance-3\",\"category\":\"OTHER_UNRESOLVED\",\"observed_height_m\":0.07,\"center\":{\"frame_id\":\"base_link\",\"stamp_ns\":10000000000,\"position_m\":[0.5,0,3]}"),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"projection_status\":\"projected\",\"rgb_pixel_xy\":[80,80]"),
    std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, InvalidProjectionInputsKeepDisturbanceListedWithoutMarker)
{
  using Candidate = arm_cell_interfaces::msg::DisturbanceCandidate;
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  for (int invalid_input = 0; invalid_input < 3; ++invalid_input) {
    auto terminal = calibration_job();
    terminal.event_kind = Message::DETECT_TARGET_FAILURE;
    terminal.acquisition_id = "unprojectable-disturbance-" + std::to_string(invalid_input);
    terminal.has_detection_result = true;
    terminal.result.disturbance_candidates.resize(1);
    auto & disturbance = terminal.result.disturbance_candidates.front();
    disturbance.candidate_id = "disturbance-1";
    disturbance.category = Candidate::CATEGORY_ELEVATED_UNRESOLVED;
    disturbance.center.header.frame_id = "base_link";
    disturbance.center.header.stamp = terminal.input->depth.header.stamp;
    disturbance.center.pose.position.z = 3.0;
    disturbance.center.pose.orientation.w = 1.0;
    disturbance.observed_height_m = 0.04;

    auto input = std::make_shared<arm_cell_vision::DetectionInput>(*terminal.input);
    input->rgb.width = 100;
    input->rgb.height = 80;
    input->rgb.step = 300;
    input->rgb.data.assign(input->rgb.step * input->rgb.height, 0);
    input->rgb.header.frame_id = "camera_optical";
    input->camera_info.width = input->rgb.width;
    input->camera_info.height = input->rgb.height;
    input->camera_info.header.frame_id = "camera_optical";
    input->camera_info.distortion_model = "plumb_bob";
    input->camera_info.k = {80.0, 0.0, 50.0, 0.0, 80.0, 40.0, 0.0, 0.0, 1.0};
    input->camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    input->depth.header.frame_id = "camera_optical";
    input->optical_to_base.header.frame_id = "base_link";
    input->optical_to_base.child_frame_id = "camera_optical";
    input->optical_to_base.transform.rotation.w = 1.0;
    if (invalid_input == 0) {
      disturbance.center.header.frame_id = "invalid_frame";
    } else if (invalid_input == 1) {
      input->optical_to_base.header.frame_id = "wrong_frame";
    } else {
      input->camera_info.k[0] = 0.0;
    }
    terminal.input = std::move(input);

    arm_cell_interfaces::msg::VisionDetectorSnapshot published;
    arm_cell_vision::VisionDetectorSnapshotOutput output(
      *node, temporary_snapshot_root(
        "unprojectable-disturbance-" + std::to_string(invalid_input)), "unprojectable-run", {},
      [&](const auto & message) {published = message;});
    output.submit(std::move(terminal));
    output.shutdown();

    const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
    ASSERT_FALSE(image.empty());
    EXPECT_EQ(image.rows, 80);
    cv::Mat category_color;
    cv::inRange(
      image(cv::Rect(0, 0, 100, 80)), cv::Scalar(255, 255, 0),
      cv::Scalar(255, 255, 0), category_color);
    EXPECT_EQ(cv::countNonZero(category_color), 0);
    EXPECT_NE(published.metadata_json.find(
        "\"candidate_id\":\"disturbance-1\",\"category\":\"ELEVATED_UNRESOLVED\",\"observed_height_m\":0.04"),
      std::string::npos);
    EXPECT_NE(published.metadata_json.find("\"projection_status\":\"unavailable\",\"rgb_pixel_xy\":null"),
      std::string::npos);
  }
}

TEST_F(VisionDetectorSnapshotTest, DisturbanceProjectsWhenRgbAndDepthTimestampsDiffer)
{
  using Candidate = arm_cell_interfaces::msg::DisturbanceCandidate;
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  auto terminal = calibration_job();
  terminal.event_kind = Message::DETECT_TARGET_FAILURE;
  terminal.acquisition_id = "disturbance-asynchronous-rgb";
  terminal.has_detection_result = true;

  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*terminal.input);
  input->rgb.width = 200;
  input->rgb.height = 160;
  input->rgb.step = 600;
  input->rgb.data.assign(input->rgb.step * input->rgb.height, 0);
  input->rgb.header.frame_id = "camera_optical";
  input->rgb.header.stamp.sec += 1;
  input->camera_info.width = input->rgb.width;
  input->camera_info.height = input->rgb.height;
  input->camera_info.header.frame_id = "camera_optical";
  input->camera_info.distortion_model = "plumb_bob";
  input->camera_info.k = {100.0, 0.0, 100.0, 0.0, 100.0, 80.0, 0.0, 0.0, 1.0};
  input->camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  input->depth.header.frame_id = "camera_optical";
  input->optical_to_base.header.frame_id = "base_link";
  input->optical_to_base.child_frame_id = "camera_optical";
  input->optical_to_base.transform.translation.x = 0.1;
  input->optical_to_base.transform.translation.z = 1.0;
  input->optical_to_base.transform.rotation.w = 1.0;
  terminal.input = input;

  Candidate candidate;
  candidate.candidate_id = "async-disturbance-1";
  candidate.category = Candidate::CATEGORY_ELEVATED_UNRESOLVED;
  candidate.center.header = input->depth.header;
  candidate.center.header.frame_id = "base_link";
  candidate.center.pose.position.x = 0.1;
  candidate.center.pose.position.z = 3.0;
  candidate.center.pose.orientation.w = 1.0;
  candidate.observed_height_m = 0.04;
  terminal.result.disturbance_candidates = {candidate};

  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, temporary_snapshot_root("disturbance-asynchronous-rgb"), "disturbance-run", {},
    [&](const auto & message) {published = message;});
  output.submit(std::move(terminal));
  output.shutdown();

  const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.at<cv::Vec3b>(80, 100), cv::Vec3b(0, 165, 255));
  EXPECT_NE(
    published.metadata_json.find(
      "\"candidate_id\":\"async-disturbance-1\",\"category\":\"ELEVATED_UNRESOLVED\",\"observed_height_m\":0.04"),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"projection_status\":\"projected\",\"rgb_pixel_xy\":[100,80]"),
    std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, SelectedPartialReconstructionUsesHighConfidenceColorWithoutText)
{
  auto terminal = cuboid_projection_job("plumb_bob");
  terminal.acquisition_id = "partial-reconstruction-outline";
  auto candidate = terminal.result.cuboid_hypotheses.front();
  candidate.disposition = "selected_partial_reconstruction";
  terminal.result.cuboid_hypotheses = {candidate};

  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-partial-outline",
    {}, [&](const auto & message) {published = message;});
  output.submit(std::move(terminal));
  output.shutdown();

  const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.at<cv::Vec3b>(110, 130), cv::Vec3b(0, 200, 0));
  bool has_estimated_target_label = false;
  for (int y = 60; y < 110; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      const auto & pixel = image.at<cv::Vec3b>(y, x);
      if (pixel[0] < 80 && pixel[1] > 150 && pixel[2] > 200) {
        has_estimated_target_label = true;
        break;
      }
    }
  }
  EXPECT_FALSE(has_estimated_target_label);
}

TEST_F(VisionDetectorSnapshotTest, TerminalAcceptanceSuppressesSameAcquisitionProgress)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool first_publish_started = false;
  bool release_publish = false;
  std::vector<std::pair<uint8_t, std::string>> events;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-terminal-progress",
    [](const auto &, const auto &) {return true;},
    [&](const auto & message) {
      std::unique_lock<std::mutex> lock(mutex);
      if (!first_publish_started) {
        first_publish_started = true;
        condition.notify_all();
        condition.wait(lock, [&]() {return release_publish;});
      }
      events.emplace_back(message.event_kind, message.acquisition_id);
      condition.notify_all();
    });
  auto first_progress = calibration_job();
  first_progress.event_kind =
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS;
  first_progress.calibration_id.clear();
  first_progress.acquisition_id = "acq-terminal";
  first_progress.save = false;
  output.submit(std::move(first_progress));
  bool first_publish_observed = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    first_publish_observed = condition.wait_for(lock, std::chrono::seconds(2), [&]() {
      return first_publish_started;
    });
    if (!first_publish_observed) {release_publish = true;}
  }
  condition.notify_all();
  ASSERT_TRUE(first_publish_observed);

  auto terminal = calibration_job();
  terminal.event_kind =
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "acq-terminal";
  output.submit(std::move(terminal));
  auto stale_progress = calibration_job();
  stale_progress.event_kind =
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS;
  stale_progress.calibration_id.clear();
  stale_progress.acquisition_id = "acq-terminal";
  stale_progress.save = false;
  output.submit(std::move(stale_progress));
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_publish = true;
  }
  condition.notify_all();
  output.shutdown();
  ASSERT_EQ(events.size(), 2U);
  EXPECT_EQ(events[0].first,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS);
  EXPECT_EQ(events[1].first,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE);
  EXPECT_EQ(events[1].second, "acq-terminal");
}

TEST_F(VisionDetectorSnapshotTest, NoSubscriberStillWritesThePublishedPngAndJson)
{
  const auto root = std::filesystem::temp_directory_path() / "vision-snapshot-output-test";
  const auto directory = root / "snapshots";
  std::filesystem::remove_all(root);
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  const auto date = utc_date_now();
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, directory, "run-3", {},
    [&](const auto & message) {published.push_back(message);}, date, root / "diagnostic_records", {}, "123456");
  output.submit(calibration_job());
  auto terminal = calibration_job();
  terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "acq-10";
  terminal.terminal_result_code = 2;
  terminal.terminal_diagnostic = "timeout";
  output.submit(std::move(terminal));
  auto second_terminal = calibration_job();
  second_terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
  second_terminal.calibration_id.clear();
  second_terminal.acquisition_id = "acq-11";
  second_terminal.terminal_result_code = 0;
  output.submit(std::move(second_terminal));
  auto progress = calibration_job();
  progress.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS;
  progress.calibration_id.clear();
  progress.acquisition_id = "acq-progress";
  progress.save = false;
  output.submit(std::move(progress));
  output.shutdown();
  ASSERT_EQ(published.size(), 4U);
  EXPECT_EQ(published[0].calibration_id, "cal-1");
  EXPECT_EQ(published[1].acquisition_id, "acq-10");
  EXPECT_EQ(published[2].acquisition_id, "acq-11");
  EXPECT_EQ(published[3].acquisition_id, "acq-progress");
  const auto run_directory = directory / date / "123456";
  const auto calibration_directory = run_directory / "calibration" / "01";
  const auto metadata_run = root / "diagnostic_records" / "snapshot_metadata" / date / "123456";
  const auto & calibration_message = published.front();
  ASSERT_TRUE(std::filesystem::exists(calibration_directory / "snapshot.png"));
  ASSERT_TRUE(std::filesystem::exists(metadata_run / "calibration" / "01" / "metadata.json"));
  std::ifstream metadata(metadata_run / "calibration" / "01" / "metadata.json");
  const std::string saved((std::istreambuf_iterator<char>(metadata)),
    std::istreambuf_iterator<char>());
  EXPECT_EQ(saved, calibration_message.metadata_json);
  EXPECT_NE(saved.find("\"vision_run_id\":\"run-3\""), std::string::npos);
  EXPECT_NE(saved.find("\"snapshot_id\":\"" + calibration_message.snapshot_id + "\""), std::string::npos);
  EXPECT_NE(saved.find("\"revision\":" + std::to_string(calibration_message.revision)), std::string::npos);
  EXPECT_NE(saved.find("\"calibration_id\":\"cal-1\""), std::string::npos);
  std::ifstream image(calibration_directory / "snapshot.png", std::ios::binary);
  const std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(image)),
    std::istreambuf_iterator<char>());
  EXPECT_EQ(bytes, calibration_message.image_png.data);
  EXPECT_TRUE(std::filesystem::is_directory(calibration_directory));
  const auto success_directory = run_directory / "acquisition" / "acq-11";
  const auto failure_directory = run_directory / "acquisition" / "acq-10";
  ASSERT_TRUE(std::filesystem::exists(metadata_run / "acquisition" / "acq-11" / "metadata.json"));
  ASSERT_TRUE(std::filesystem::exists(success_directory / "snapshot.png"));
  ASSERT_TRUE(std::filesystem::exists(metadata_run / "acquisition" / "acq-10" / "metadata.json"));
  ASSERT_TRUE(std::filesystem::exists(failure_directory / "snapshot.png"));
  std::ifstream failure_metadata(metadata_run / "acquisition" / "acq-10" / "metadata.json");
  const std::string failure_json((std::istreambuf_iterator<char>(failure_metadata)), {});
  EXPECT_NE(failure_json.find("\"acquisition_id\":\"acq-10\""), std::string::npos);
  EXPECT_NE(failure_json.find("\"event_kind\":\"DETECT_TARGET_FAILURE\""), std::string::npos);
  EXPECT_EQ(failure_json, published[1].metadata_json);
  for (const auto & entry : std::filesystem::recursive_directory_iterator(root / "diagnostic_records")) {
    EXPECT_NE(entry.path().extension(), ".png");
  }
  for (const auto & entry : std::filesystem::recursive_directory_iterator(directory)) {
    EXPECT_NE(entry.path().extension(), ".json");
  }
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, GeometryPanelsShareTheAcquisitionAndReferenceTwoSavedImages)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  const auto root = temporary_snapshot_root("geometry-panels");
  auto job = cuboid_stage_job("geometry-acq");
  auto acquisition = std::make_shared<arm_cell_vision::AcquisitionResult>();
  acquisition->generation = 1;
  acquisition->acquisition_id = job.acquisition_id;
  acquisition->target_id = job.target_id;
  acquisition->input = job.input;
  acquisition->detection = job.result;
  acquisition->calibration = job.calibration;
  if (job.selected_profile) {
    acquisition->detection.applied_profile =
      std::make_shared<const arm_cell_vision::DetectorProfile>(*job.selected_profile);
  }
  job.acquisition_result = acquisition;
  job.input.reset();
  job.result = arm_cell_vision::DetectionResult{};
  job.selected_profile.reset();

  std::vector<Message> published;
  const auto date = utc_date_now();
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root, "geometry-run", {}, [&](const auto & message) {published.push_back(message);},
    date, {}, {}, "123456");
  output.submit(std::move(job));
  output.shutdown();
  ASSERT_EQ(published.size(), 1U);
  const auto directory = root / date / "123456" / "acquisition" / "geometry-acq";
  const auto observation = cv::imread((directory / "geometry_observation.png").string(), cv::IMREAD_COLOR);
  const auto interpretation = cv::imread((directory / "geometry_interpretation.png").string(), cv::IMREAD_COLOR);
  ASSERT_FALSE(observation.empty());
  ASSERT_FALSE(interpretation.empty());
  EXPECT_GE(observation.cols, 2 * 320);
  EXPECT_EQ(observation.rows, interpretation.rows);
  EXPECT_EQ((observation.cols - 20) / 3, (interpretation.cols - 10) / 2);
  EXPECT_GE(interpretation.cols, 2 * 320);
  // Each stage has its own Header and map.
  const cv::Mat hypotheses_panel = interpretation(cv::Rect(0, 58,
      (interpretation.cols - 10) / 2, interpretation.rows - 70));
  const cv::Mat final_panel = interpretation(cv::Rect((interpretation.cols + 10) / 2, 58,
      (interpretation.cols - 10) / 2, interpretation.rows - 70));
  cv::Mat rejected_red;
  std::vector<cv::Mat> channels;
  cv::split(hypotheses_panel, channels);
  rejected_red = (channels[2] > channels[0] + 50) & (channels[2] > channels[1] + 50);
  EXPECT_GT(cv::countNonZero(rejected_red), 0)
    << "rejected Runtime hypotheses remain visible in red";
  // UNRESOLVED is a selection state, not an orange confidence grade.
  // All states/grades remain available in the serialized Runtime evidence.
  for (const auto * state : {"SELECTED", "ALTERNATIVE", "UNRESOLVED", "REJECTED"}) {
    EXPECT_NE(published.front().diagnostic_record_json.find(state), std::string::npos);
  }
  cv::Mat selected_green;
  cv::inRange(final_panel, cv::Scalar(45, 195, 45), cv::Scalar(55, 205, 55), selected_green);
  EXPECT_GT(cv::countNonZero(selected_green), 0);
  EXPECT_FALSE(std::filesystem::exists(directory / "height_patches.png"));
  EXPECT_FALSE(std::filesystem::exists(directory / "geometry_candidates.png"));
  EXPECT_FALSE(std::filesystem::exists(directory / "final_candidates.png"));

  if (const char * preview = std::getenv("SFTWIN_SNAPSHOT_PREVIEW")) {
    std::filesystem::create_directories(preview);
    std::ofstream(std::filesystem::path(preview) / "metadata.json") << published.front().metadata_json;
    std::ofstream(std::filesystem::path(preview) / "diagnostic_record.json") <<
      published.front().diagnostic_record_json;
    for (const auto * name : {"geometry_observation.png", "geometry_interpretation.png", "snapshot.png"}) {
      std::filesystem::copy_file(directory / name, std::filesystem::path(preview) / name,
        std::filesystem::copy_options::overwrite_existing);
    }
  }
  const auto & metadata = published.front().metadata_json;
  EXPECT_NE(metadata.find("\"acquisition_id\":\"geometry-acq\""), std::string::npos);
  EXPECT_NE(metadata.find("\"stage\":\"height_patches\",\"status\":"), std::string::npos);
  EXPECT_NE(metadata.find("\"stage\":\"geometry_candidates\",\"status\":\"rendered\",\"relative_path\":\"geometry_interpretation.png\",\"candidate_ids\":[6,7,8,86,10,11,12,13,14,15]"), std::string::npos);
  EXPECT_NE(metadata.find("\"stage\":\"final_candidates\",\"status\":\"rendered\",\"relative_path\":\"geometry_interpretation.png\",\"candidate_ids\":[86,15]"), std::string::npos);
  EXPECT_NE(metadata.find("\"stage\":\"final_candidates\",\"status\":"), std::string::npos);
  EXPECT_NE(metadata.find("\"relative_path\":\"geometry_observation.png\""), std::string::npos);
  EXPECT_NE(metadata.find("\"relative_path\":\"geometry_interpretation.png\""), std::string::npos);
  EXPECT_EQ(metadata.find("\"relative_path\":\"final_candidates.png\""), std::string::npos);
  EXPECT_NE(published.front().diagnostic_record_json.find("\"candidate_id\":86"), std::string::npos);
  EXPECT_NE(published.front().diagnostic_record_json.find("\"candidate_id\":10"), std::string::npos);
  EXPECT_NE(published.front().diagnostic_record_json.find(
      "\"candidate_id\":15,\"patch_index\":0,\"confidence_grade\":\"LOW\""),
    std::string::npos);
  EXPECT_NE(published.front().diagnostic_record_json.find("\"boundary_side_identity_available\":false"),
    std::string::npos);
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, HeightChangesOnlyAffectTheHeightmapStage)
{
  auto job = cuboid_stage_job("stage-backgrounds");
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> before, after;
  arm_cell_vision::snapshot_detail::render_geometry(job, before);
  auto & grid = job.result.cuboid_replay_evidence->height_grid_m;
  grid = grid.clone();
  for (int row = 0; row < grid.rows; ++row) {
    for (int col = 0; col < grid.cols; ++col) {
      if (std::isfinite(grid.at<double>(row, col))) {
        grid.at<double>(row, col) = (row + col) * 0.001;
      }
    }
  }
  arm_cell_vision::snapshot_detail::render_geometry(job, after);
  const auto first = cv::imdecode(before[0].png, cv::IMREAD_COLOR);
  const auto second = cv::imdecode(after[0].png, cv::IMREAD_COLOR);
  ASSERT_EQ(first.size(), second.size());
  const int panel_width = (first.cols - 20) / 3;
  const cv::Rect height(0, 0, panel_width, first.rows);
  const cv::Rect boundary(panel_width + 10, 0, panel_width, first.rows);
  EXPECT_GT(cv::norm(first(height), second(height), cv::NORM_INF), 0);
  EXPECT_EQ(cv::norm(first(boundary), second(boundary), cv::NORM_INF), 0)
    << "Boundary Extraction uses a neutral background";
  EXPECT_EQ(before[1].png, after[1].png)
    << "Plane Map and Hypothesis / Selection never inherit height colors";
}

TEST_F(VisionDetectorSnapshotTest, BoundaryExtractionExcludesEveryPostBoundaryOverlay)
{
  auto job = cuboid_stage_job("boundary-only-stage");
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> before, after;
  arm_cell_vision::snapshot_detail::render_geometry(job, before);
  ASSERT_EQ(before.size(), 3U);
  ASSERT_FALSE(job.result.cuboid_height_patches.front().rectangle_regions.empty());
  ASSERT_FALSE(job.result.cuboid_height_patches.front().inferred_partition_edges.empty());
  ASSERT_FALSE(job.result.cuboid_boundaries.empty());
  ASSERT_FALSE(job.result.cuboid_boundaries.front().observed_contours_xy.empty());
  ASSERT_FALSE(job.result.cuboid_boundaries.front().fitted_edges_xy_support.empty());

  auto & plane = job.result.cuboid_height_patches.front();
  plane.rectangle_regions.push_back({0.12, -0.12, 0.5, 0.08, 0.04});
  plane.inferred_partition_edges.push_back({0.08, -0.15, 0.16, -0.09});
  job.result.cuboid_hypotheses.front().anchor_x_m = 0.14;
  job.result.cuboid_hypotheses.front().anchor_y_m = -0.14;
  arm_cell_vision::snapshot_detail::render_geometry(job, after);
  ASSERT_EQ(after.size(), 3U);

  const auto before_observation = cv::imdecode(before[0].png, cv::IMREAD_COLOR);
  const auto after_observation = cv::imdecode(after[0].png, cv::IMREAD_COLOR);
  const int panel_width = (before_observation.cols - 20) / 3;
  const cv::Rect boundary(panel_width + 10, 58, panel_width,
    before_observation.rows - 70);
  EXPECT_EQ(cv::norm(before_observation(boundary), after_observation(boundary), cv::NORM_INF), 0)
    << "Boundary Extraction depends only on retained source boundaries";

  cv::Mat fitted_edge_color;
  cv::inRange(before_observation(boundary), cv::Scalar(235, 165, 55),
    cv::Scalar(255, 215, 115), fitted_edge_color);
  EXPECT_GT(cv::countNonZero(fitted_edge_color), 0) << "Fitted Edges have their own color";
  const auto before_interpretation = cv::imdecode(before[1].png, cv::IMREAD_COLOR);
  const auto after_interpretation = cv::imdecode(after[1].png, cv::IMREAD_COLOR);
  const cv::Rect selection(0, 58, panel_width,
    before_interpretation.rows - 70);
  EXPECT_GT(cv::norm(before_interpretation(selection), after_interpretation(selection),
    cv::NORM_INF), 0) << "Stage 4 displays changed Runtime partition and anchor evidence";

  auto without_partition = job;
  for (auto & partition_plane : without_partition.result.cuboid_height_patches) {
    partition_plane.rectangle_regions.clear();
    partition_plane.inferred_partition_edges.clear();
  }
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> no_partition;
  arm_cell_vision::snapshot_detail::render_geometry(without_partition, no_partition);
  const auto without_partition_observation = cv::imdecode(no_partition[0].png, cv::IMREAD_COLOR);
  const auto without_partition_interpretation = cv::imdecode(no_partition[1].png, cv::IMREAD_COLOR);
  EXPECT_EQ(cv::norm(after_observation(boundary), without_partition_observation(boundary),
    cv::NORM_INF), 0) << "Stage 2 never includes Rectangle Partition";
  EXPECT_GT(cv::norm(after_interpretation(selection),
    without_partition_interpretation(selection), cv::NORM_INF), 0)
    << "Stage 4 shows the recorded Rectangle Partition";
}

TEST_F(VisionDetectorSnapshotTest, BoundaryExtractionUsesOnlyOriginalBoundaryEdges)
{
  auto job = cuboid_stage_job("boundary-edge-source");
  ASSERT_FALSE(job.result.cuboid_boundaries.empty());
  ASSERT_FALSE(job.result.cuboid_boundaries.front().fitted_edges_xy_support.empty());
  ASSERT_FALSE(job.result.cuboid_height_patches.empty());
  ASSERT_FALSE(job.result.cuboid_height_patches.front().fitted_edges_xy_support.empty());
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> baseline;
  arm_cell_vision::snapshot_detail::render_geometry(job, baseline);
  ASSERT_EQ(baseline.size(), 3U);

  auto plane_edge_change = job;
  auto & plane_edges = plane_edge_change.result.cuboid_height_patches.front().fitted_edges_xy_support;
  plane_edges.front()[0] += 0.04;
  plane_edges.front()[2] += 0.04;
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> plane_changed;
  arm_cell_vision::snapshot_detail::render_geometry(plane_edge_change, plane_changed);
  ASSERT_EQ(plane_changed.size(), 3U);
  EXPECT_EQ(baseline[0].png, plane_changed[0].png)
    << "Stage 2 is independent of Plane fitted edges";
  EXPECT_EQ(baseline[1].png, plane_changed[1].png)
    << "Plane-edge changes do not alter the Plane Map or later interpretation";
  EXPECT_EQ(baseline[2].png, plane_changed[2].png)
    << "Plane-edge changes do not alter Rectangle Partition / Hypothesis / Selection";

  auto selection_change = job;
  selection_change.result.cuboid_hypotheses.front().selection_state = "SELECTED";
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> selection_changed;
  arm_cell_vision::snapshot_detail::render_geometry(selection_change, selection_changed);
  ASSERT_EQ(selection_changed.size(), 3U);
  EXPECT_EQ(baseline[0].png, selection_changed[0].png)
    << "Stage 2 is independent of Hypothesis Selection state";
  EXPECT_NE(baseline[1].png, selection_changed[1].png)
    << "Stage 4 retains the Runtime Hypothesis Selection state";

  auto boundary_edge_change = job;
  auto & boundary_edge = boundary_edge_change.result.cuboid_boundaries.front().fitted_edges_xy_support.front();
  boundary_edge[0] += 0.04;
  boundary_edge[2] += 0.04;
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> boundary_changed;
  arm_cell_vision::snapshot_detail::render_geometry(boundary_edge_change, boundary_changed);
  ASSERT_EQ(boundary_changed.size(), 3U);
  EXPECT_NE(baseline[0].png, boundary_changed[0].png)
    << "Stage 2 reflects changes to retained original Boundary fitted edges";
}

TEST_F(VisionDetectorSnapshotTest, SelectionUsesTheSamePlaneCellsAsThePlaneMap)
{
  auto job = cuboid_stage_job("plane-background");
  job.result.cuboid_height_patches.resize(1);
  job.result.cuboid_hypotheses.clear();
  job.result.cuboid_boundaries.clear();
  job.result.ordered_candidates.clear();
  job.result.rawpart_candidate_diagnostics.clear();
  for (auto & plane : job.result.cuboid_height_patches) {
    plane.candidate_evaluation_attempt_count = 0;
    plane.rectangle_regions.clear();
    plane.inferred_partition_edges.clear();
    // One cell retains its plane tint without enough room for an ID label.
    plane.observed_cells_xy.resize(1);
    plane.cell_count = 1;
  }
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> images;
  arm_cell_vision::snapshot_detail::render_geometry(job, images);
  ASSERT_EQ(images.size(), 3U);
  ASSERT_EQ(images[1].status, "rendered");
  const auto image = cv::imdecode(images[1].png, cv::IMREAD_COLOR);
  const int panel_width = (image.cols - 10) / 2;
  // Exclude the dedicated two-line Header and bottom padding, never map content.
  const cv::Rect plane(0, 58, panel_width, image.rows - 70);
  const cv::Rect selection(image.cols - panel_width, 58, panel_width, image.rows - 70);
  EXPECT_EQ(cv::norm(image(plane), image(selection), cv::NORM_INF), 0)
    << "with no overlays, Stage 4 is exactly the Stage 3 Plane Map";
  const auto observation = cv::imdecode(images[0].png, cv::IMREAD_COLOR);
  const cv::Rect plane_map(2 * (panel_width + 10), 58, panel_width, image.rows - 70);
  ASSERT_EQ(observation.cols, 3 * panel_width + 20);
  EXPECT_EQ(cv::norm(observation(plane_map), image(plane), cv::NORM_INF), 0);
  EXPECT_EQ(cv::norm(observation(plane_map), image(selection), cv::NORM_INF), 0);
  // With overlays removed, only plane fills can provide this contrast.
  std::vector<cv::Mat> channels;
  cv::split(observation(plane_map), channels);
  double brightest_fill = 0;
  for (const auto & channel : channels) {
    double maximum = 0;
    cv::minMaxLoc(channel, nullptr, &maximum);
    brightest_fill = std::max(brightest_fill, maximum);
  }
  EXPECT_GT(brightest_fill, 100) << "plane identity must remain visible on the dark background";
  job.result.cuboid_height_patches.front().plane_status = "unresolved_insufficient_support";
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> residual_images;
  arm_cell_vision::snapshot_detail::render_geometry(job, residual_images);
  ASSERT_EQ(residual_images.size(), 3U);
  const auto residual = cv::imdecode(residual_images[0].png, cv::IMREAD_COLOR);
  double residual_maximum = 0;
  cv::minMaxLoc(residual(plane_map).reshape(1), nullptr, &residual_maximum);
  EXPECT_GT(residual_maximum, 100) << "insufficient support is dimmer but still visible";
  job.result.cuboid_height_patches.front().plane_status = "horizontal";
  job.result.cuboid_height_patches.front().observed_cells_xy.front().x += 0.05;
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> changed;
  arm_cell_vision::snapshot_detail::render_geometry(job, changed);
  const auto changed_observation = cv::imdecode(changed[0].png, cv::IMREAD_COLOR);
  const auto changed_interpretation = cv::imdecode(changed[1].png, cv::IMREAD_COLOR);
  EXPECT_GT(cv::norm(observation(plane_map), changed_observation(plane_map), cv::NORM_INF), 0);
  EXPECT_EQ(cv::norm(changed_observation(plane_map), changed_interpretation(plane), cv::NORM_INF), 0);
  EXPECT_EQ(cv::norm(changed_observation(plane_map), changed_interpretation(selection), cv::NORM_INF), 0);
  const cv::Rect boundary(panel_width + 10, 0, panel_width, observation.rows);
  EXPECT_EQ(cv::norm(observation(boundary), changed_observation(boundary), cv::NORM_INF), 0);

}

TEST_F(VisionDetectorSnapshotTest, IncompleteHypothesisEvidenceKeepsThePlaneMapBackground)
{
  auto job = cuboid_stage_job("incomplete-hypothesis-background");
  job.result.cuboid_boundaries.clear();
  job.result.cuboid_height_patches.front().candidate_evaluation_attempt_count = 0;
  for (auto & plane : job.result.cuboid_height_patches) {
    plane.rectangle_regions.clear();
    plane.inferred_partition_edges.clear();
  }
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> images;
  arm_cell_vision::snapshot_detail::render_geometry(job, images);
  ASSERT_EQ(images.size(), 3U);
  EXPECT_EQ(images[1].status, "evidence_unavailable");

  const auto image = cv::imdecode(images[1].png, cv::IMREAD_COLOR);
  const int panel_width = (image.cols - 10) / 2;
  const cv::Rect plane(0, 58, panel_width, image.rows - 70);
  const cv::Rect selection(image.cols - panel_width, 58, panel_width, image.rows - 70);
  EXPECT_EQ(cv::norm(image(plane), image(selection), cv::NORM_INF), 0)
    << "missing Hypothesis evidence does not erase the Stage 3 Plane Map background";
}

TEST_F(VisionDetectorSnapshotTest, MissingReturnedOutlinesRemainUnavailableInTheSelectionHeader)
{
  auto job = cuboid_stage_job("missing-returned-outlines");
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> complete, partial;
  arm_cell_vision::snapshot_detail::render_geometry(job, complete);
  job.result.rawpart_candidate_diagnostics.clear();
  arm_cell_vision::snapshot_detail::render_geometry(job, partial);
  ASSERT_EQ(partial.size(), 3U);
  EXPECT_EQ(partial[1].status, "rendered") << "all hypotheses remain available";
  EXPECT_EQ(partial[2].status, "evidence_unavailable");
  EXPECT_EQ(partial[1].candidate_ids, complete[1].candidate_ids);
  const auto before = cv::imdecode(complete[1].png, cv::IMREAD_COLOR);
  const auto after = cv::imdecode(partial[1].png, cv::IMREAD_COLOR);
  const int panel_width = (after.cols - 10) / 2;
  const cv::Rect header(after.cols - panel_width, 0, panel_width, 58);
  EXPECT_GT(cv::norm(before(header), after(header), cv::NORM_INF), 0)
    << "the image must disclose unavailable returned contours, not silently omit them";
  const cv::Rect counts(0, 28, panel_width, 30);
  EXPECT_EQ(cv::norm(before(counts), after(counts), cv::NORM_INF), 0)
    << "existing hypothesis selection counts remain visible";
}

TEST_F(VisionDetectorSnapshotTest, ReturnedOutlinesRenderIndependentlyWhenSomeEvidenceIsMissing)
{
  auto job = cuboid_stage_job("partial-returned-outlines");
  auto medium = job.result.ordered_candidates.front();
  medium.rawpart_selected_candidate = "87";
  medium.confidence_grade = "MEDIUM";
  job.result.ordered_candidates.push_back(medium);
  auto diagnostic = job.result.rawpart_candidate_diagnostics.front();
  diagnostic.candidate_id = 87;
  diagnostic.confidence_grade = "MEDIUM";
  for (auto & point : diagnostic.fitted_outline_support) {
    point.x += 0.12F;
    point.y += 0.10F;
  }
  job.result.rawpart_candidate_diagnostics.push_back(diagnostic);
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> complete;
  arm_cell_vision::snapshot_detail::render_geometry(job, complete);
  ASSERT_EQ(complete[2].status, "rendered");
  const auto full = cv::imdecode(complete[1].png, cv::IMREAD_COLOR);
  const int panel_width = (full.cols - 10) / 2;
  const cv::Rect map(full.cols - panel_width, 58, panel_width, full.rows - 70);
  const cv::Rect header(full.cols - panel_width, 0, panel_width, 28);
  const cv::Rect counts(0, 28, panel_width, 30);
  const auto colored_pixels = [&](const cv::Mat & image, const std::string & grade) {
      cv::Mat mask;
      const auto color = arm_cell_vision::snapshot_detail::confidence_color(grade);
      cv::inRange(image(map), color, color, mask);
      return cv::countNonZero(mask);
    };
  ASSERT_GT(colored_pixels(full, "HIGH"), 0);
  ASSERT_GT(colored_pixels(full, "MEDIUM"), 0);
  const auto original_diagnostics = job.result.rawpart_candidate_diagnostics;
  cv::Mat partial_header;
  for (const size_t missing : {0U, 1U}) {
    SCOPED_TRACE(missing);
    job.result.rawpart_candidate_diagnostics = original_diagnostics;
    job.result.rawpart_candidate_diagnostics[missing].fitted_outline_support.clear();
    std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> partial;
    arm_cell_vision::snapshot_detail::render_geometry(job, partial);
    EXPECT_EQ(partial[1].status, "rendered");
    EXPECT_EQ(partial[2].status, "evidence_unavailable");
    EXPECT_EQ(partial[1].candidate_ids, complete[1].candidate_ids);
    EXPECT_EQ(partial[2].candidate_ids, complete[2].candidate_ids);
    EXPECT_EQ(partial[0].png, complete[0].png);
    const auto image = cv::imdecode(partial[1].png, cv::IMREAD_COLOR);
    partial_header = image(header).clone();
    const cv::Rect hypotheses(0, 0, panel_width, image.rows);
    EXPECT_EQ(cv::norm(full(hypotheses), image(hypotheses), cv::NORM_INF), 0)
      << "returned-outline availability affects only Final Selection";

    if (const char * preview = std::getenv("SFTWIN_SNAPSHOT_PREVIEW")) {
      cv::imwrite((std::filesystem::path(preview) /
        ("partial-returned-outlines-" + std::to_string(missing) + ".png")).string(), image);
    }
    EXPECT_GT(colored_pixels(image, missing == 0 ? "MEDIUM" : "HIGH"), 0);
    EXPECT_EQ(colored_pixels(image, missing == 0 ? "HIGH" : "MEDIUM"), 0)
      << "missing returned outlines are never reconstructed from hypotheses";
    EXPECT_GT(cv::norm(full(header), image(header), cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(full(counts), image(counts), cv::NORM_INF), 0);
  }
  job.result.rawpart_candidate_diagnostics.clear();
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> unavailable;
  arm_cell_vision::snapshot_detail::render_geometry(job, unavailable);
  const auto empty = cv::imdecode(unavailable[1].png, cv::IMREAD_COLOR);
  EXPECT_GT(cv::norm(partial_header, empty(header), cv::NORM_INF), 0)
    << "partial availability differs from no returned-outline evidence";
  EXPECT_EQ(colored_pixels(empty, "HIGH"), 0);
  EXPECT_EQ(colored_pixels(empty, "MEDIUM"), 0);
  EXPECT_EQ(unavailable[1].candidate_ids, complete[1].candidate_ids);
  EXPECT_EQ(cv::norm(full(counts), empty(counts), cv::NORM_INF), 0);
  job.result.rawpart_candidate_diagnostics = original_diagnostics;
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> restored;
  arm_cell_vision::snapshot_detail::render_geometry(job, restored);
  EXPECT_EQ(restored[1].png, complete[1].png);
  EXPECT_EQ(job.result.ordered_candidates.size(), 2U);
  EXPECT_EQ(job.result.result_code, job.terminal_result_code);
}

TEST(GeometrySnapshotLayoutTest, LabelsFitTheirRegionAndNeverOverlapEarlierReservations)
{
  using arm_cell_vision::snapshot_detail::reserve_geometry_label;
  const cv::Rect area(0, 0, 400, 300);
  std::vector<cv::Rect> occupied;
  const auto selected = reserve_geometry_label("#86", {200, 150}, area, occupied);
  ASSERT_FALSE(selected.empty());
  EXPECT_TRUE(reserve_geometry_label("#100", {200, 150}, area, occupied).empty());
  EXPECT_TRUE(reserve_geometry_label("P91", {3, 3}, area, occupied).empty());
  EXPECT_TRUE(reserve_geometry_label("P92", {80, 80}, {78, 78, 4, 4}, occupied).empty());
  const auto distant = reserve_geometry_label("P0", {60, 60}, area, occupied);
  ASSERT_FALSE(distant.empty());
  EXPECT_TRUE((selected & distant).empty());
  EXPECT_EQ(occupied.size(), 2U) << "omitted labels do not consume display space";
  for (const auto & box : occupied) {EXPECT_EQ(box & area, box);}
}

TEST(GeometrySnapshotLayoutTest, PlaneAnnotationRetainsRecordedStatusAndFitsAsOneBlock)
{
  using arm_cell_vision::snapshot_detail::geometry_plane_annotation;
  using arm_cell_vision::snapshot_detail::reserve_geometry_label;
  EXPECT_NE(geometry_plane_annotation(3, "horizontal").find("Horizontal"), std::string::npos);
  EXPECT_NE(geometry_plane_annotation(7, "mixed_or_inclined").find("Mixed/inclined"), std::string::npos);
  const auto insufficient = geometry_plane_annotation(8, "unresolved_insufficient_support");
  EXPECT_NE(insufficient.find("Insufficient support"), std::string::npos);
  EXPECT_EQ(insufficient.find("UNRESOLVED"), std::string::npos);
  EXPECT_NE(geometry_plane_annotation(9, "future_status").find("future_status"), std::string::npos);
  EXPECT_NE(geometry_plane_annotation(10, "").find("Not collected"), std::string::npos);
  std::vector<cv::Rect> occupied;
  const auto annotation = geometry_plane_annotation(3, "horizontal");
  const auto box = reserve_geometry_label(annotation, {50, 50}, {0, 0, 100, 100}, occupied);
  ASSERT_FALSE(box.empty());
  EXPECT_TRUE(reserve_geometry_label(annotation, {50, 50}, {0, 0, 100, 100}, occupied).empty());
  occupied.clear();
  EXPECT_TRUE(reserve_geometry_label(annotation, {10, 10}, {0, 0, 20, 20}, occupied).empty());
  EXPECT_TRUE(occupied.empty()) << "ID and status are omitted together";
}

TEST_F(VisionDetectorSnapshotTest, DenseGeometryKeepsBoundedHeadersAndEveryDiagnosticId)
{
  auto job = cuboid_stage_job("dense-geometry");
  job.result.cuboid_height_patches[0].plane_status = "horizontal";
  job.result.cuboid_height_patches[1].plane_status = "mixed_or_inclined";
  // Many one-cell residual planes and coincident hypotheses reproduce the live clutter.
  for (size_t i = 2; i < 92; ++i) {
    auto patch = job.result.cuboid_height_patches[0];
    patch.plane_status = "unresolved_insufficient_support";
    patch.observed_cells_xy.resize(1);
    patch.observed_cells_xy[0].x = -0.18F + static_cast<float>(i % 40) * 0.005F;
    patch.observed_cells_xy[0].y = 0.15F;
    patch.cell_count = 1;
    patch.candidate_evaluation_attempt_count = 0;
    job.result.cuboid_height_patches.push_back(patch);
  }
  for (uint32_t id = 100; job.result.cuboid_hypotheses.size() < 76; ++id) {
    auto h = job.result.cuboid_hypotheses.front();
    h.candidate_id = id;
    job.result.cuboid_hypotheses.push_back(h);
    ++job.result.cuboid_height_patches[h.patch_index].candidate_evaluation_attempt_count;
  }
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> dense, sparse;
  arm_cell_vision::snapshot_detail::render_geometry(job, dense);
  ASSERT_EQ(dense.size(), 3U);
  ASSERT_EQ(dense[1].status, "rendered");
  EXPECT_EQ(dense[1].candidate_ids.size(), 76U);
  for (size_t i = 0; i < 76; ++i) {
    EXPECT_EQ(dense[1].candidate_ids[i], job.result.cuboid_hypotheses[i].candidate_id);
  }
  auto original = cuboid_stage_job("sparse-geometry");
  arm_cell_vision::snapshot_detail::render_geometry(original, sparse);
  for (size_t i = 0; i < 2; ++i) {
    const auto dense_image = cv::imdecode(dense[i].png, cv::IMREAD_COLOR);
    const auto sparse_image = cv::imdecode(sparse[i].png, cv::IMREAD_COLOR);
    EXPECT_EQ(dense_image.size(), sparse_image.size())
      << "diagnostic list length must not grow the header or image";
  }
  EXPECT_EQ(job.result.cuboid_height_patches.size(), 92U);
  EXPECT_EQ(job.result.cuboid_hypotheses.size(), 76U);
  EXPECT_EQ(job.result.cuboid_height_patches.back().plane_status, "unresolved_insufficient_support");
  if (const char * preview = std::getenv("SFTWIN_SNAPSHOT_PREVIEW")) {
    const auto directory = std::filesystem::path(preview) / "dense-geometry";
    std::filesystem::create_directories(directory);
    for (const auto & image : dense) {
      if (image.png.empty()) {continue;}
      std::ofstream file(directory / image.filename, std::ios::binary);
      file.write(reinterpret_cast<const char *>(image.png.data()), image.png.size());
    }
  }
}

TEST_F(VisionDetectorSnapshotTest, RecordedHeightmapRendersTheCompletedDetectorEvidenceWithoutRecomputingIt)
{
  const auto prefix = std::filesystem::path(__FILE__).parent_path() / "fixtures" /
    "occupancy_replay" / "arm-cell-live-20261024-acq-11";
  arm_cell_vision::CuboidHeightObservation observation{
    cv::Mat(161, 241, CV_64F), {-0.3, -0.2}, 0.0025, cv::Mat(161, 241, CV_8U)};
  std::ifstream grid(prefix.string() + ".height_grid.f64le", std::ios::binary);
  ASSERT_TRUE(grid.read(reinterpret_cast<char *>(observation.heights.data),
    observation.heights.total() * sizeof(double)));
  std::ifstream mask(prefix.string() + ".image_boundary.u8", std::ios::binary);
  ASSERT_TRUE(mask.read(reinterpret_cast<char *>(observation.image_boundary.data),
    observation.image_boundary.total()));
  arm_cell_vision::DetectorProfile profile;
  profile.algorithm = "cuboid_top_face";
  profile.dimensions_m = {0.07, 0.07, 0.08};
  profile.dimension_tolerance_m = {0.01, 0.01, 0.01};
  profile.workspace_roi_min_m = {-0.3, -0.2, 0.01};
  profile.workspace_roi_max_m = {0.3, 0.2, 1.0};
  profile.support_contact_tolerance_m = 0.01;
  profile.top_surface_geometry_tolerance_m = 0.01;
  // Offline producer pass: the renderer below receives this completed result.
  auto job = cuboid_stage_job("838ba736-56a7047b-bd5e5f48-acq-11");
  job.input.reset();  // Original RGB and camera Depth were not captured.
  job.has_observation = false;
  job.result = arm_cell_vision::detect_cuboid_top_face(observation, profile, true);
  ASSERT_FALSE(job.result.cuboid_boundaries.empty());
  ASSERT_FALSE(job.result.cuboid_hypotheses.empty());
  const auto original_count = job.result.cuboid_hypotheses.size();
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> images;
  arm_cell_vision::snapshot_detail::render_geometry(job, images);
  ASSERT_EQ(images.size(), 3U);
  EXPECT_EQ(images[0].status, "rendered");
  EXPECT_EQ(images[1].status, "rendered");
  EXPECT_EQ(images[1].candidate_ids.size(), original_count);
  EXPECT_EQ(job.result.cuboid_hypotheses.size(), original_count);
  const auto rendered = cv::imdecode(images[1].png, cv::IMREAD_COLOR);
  ASSERT_FALSE(rendered.empty());
  if (const char * preview = std::getenv("SFTWIN_SNAPSHOT_PREVIEW")) {
    const auto directory = std::filesystem::path(preview) / "recorded-heightmap";
    std::filesystem::create_directories(directory);
    for (const auto & image : images) {
      if (!image.png.empty()) {
        std::ofstream file(directory / image.filename, std::ios::binary);
        file.write(reinterpret_cast<const char *>(image.png.data()), image.png.size());
      }
    }
  }
}

TEST_F(VisionDetectorSnapshotTest, RecordedLiveAcquisitionRendersOnlyRetainedGeometryStages)
{
  const auto prefix = std::filesystem::path(__FILE__).parent_path() / "fixtures" /
    "occupancy_replay" / "arm-cell-live-20261027-acq-1";
  arm_cell_vision::CuboidHeightObservation observation{
    cv::Mat(129, 193, CV_64F), {-0.24, -0.16}, 0.0025, cv::Mat(129, 193, CV_8U)};
  std::ifstream grid(prefix.string() + ".height_grid.f64le", std::ios::binary);
  ASSERT_TRUE(grid.read(reinterpret_cast<char *>(observation.heights.data),
    observation.heights.total() * sizeof(double)));
  std::ifstream mask(prefix.string() + ".image_boundary.u8", std::ios::binary);
  ASSERT_TRUE(mask.read(reinterpret_cast<char *>(observation.image_boundary.data),
    observation.image_boundary.total()));
  arm_cell_vision::DetectorProfile profile;
  profile.algorithm = "cuboid_top_face";
  profile.dimensions_m = {0.07, 0.07, 0.08};
  profile.dimension_tolerance_m = {0.01, 0.01, 0.01};
  profile.workspace_roi_min_m = {-0.24, -0.16, 0.01};
  profile.workspace_roi_max_m = {0.24, 0.16, 1.0};
  profile.support_contact_tolerance_m = 0.01;
  profile.top_surface_geometry_tolerance_m = 0.01;
  profile.cuboid.height_tolerance_m = 0.003;
  profile.cuboid.line_residual_limit_px = 1.25;

  auto job = cuboid_stage_job("f3ed6977-b8eb0f9d-ca09a153-acq-1");
  job.input.reset();
  job.has_observation = false;
  job.result = arm_cell_vision::detect_cuboid_top_face(observation, profile, true);
  ASSERT_EQ(job.result.result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS);
  ASSERT_FALSE(job.result.cuboid_boundaries.empty());
  ASSERT_FALSE(job.result.cuboid_hypotheses.empty());

  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> images;
  arm_cell_vision::snapshot_detail::render_geometry(job, images);
  ASSERT_EQ(images.size(), 3U);
  EXPECT_EQ(images[0].status, "rendered");
  EXPECT_EQ(images[1].status, "rendered");
  EXPECT_EQ(images[1].candidate_ids.size(), job.result.cuboid_hypotheses.size());
  if (const char * preview = std::getenv("SFTWIN_SNAPSHOT_PREVIEW")) {
    const auto directory = std::filesystem::path(preview) / "recorded-live-20261027";
    std::filesystem::create_directories(directory);
    for (const auto & image : images) {
      if (image.png.empty()) {continue;}
      std::ofstream file(directory / image.filename, std::ios::binary);
      file.write(reinterpret_cast<const char *>(image.png.data()), image.png.size());
    }
  }
}

TEST_F(VisionDetectorSnapshotTest, RgbOmitsHighConfidenceAlternativeUsingTheRecordedSelectionState)
{
  auto selected = cuboid_stage_job("rgb-selected");
  auto alternative = selected;
  alternative.acquisition_id = "rgb-alternative";
  alternative.result.cuboid_hypotheses[3].selection_state = "ALTERNATIVE";
  selected.save = false;
  alternative.save = false;
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  const auto root = temporary_snapshot_root("rgb-selection-state");
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root, "rgb-selection-run", {}, [&](const auto & message) {published.push_back(message);});
  output.submit(std::move(selected));
  output.submit(std::move(alternative));
  output.shutdown();
  ASSERT_EQ(published.size(), 2U);
  std::array<int, 2> candidate_pixels{};
  for (size_t i = 0; i < published.size(); ++i) {
    const auto image = cv::imdecode(published[i].image_png.data, cv::IMREAD_COLOR);
    ASSERT_FALSE(image.empty());
    cv::Mat green;
    cv::inRange(image(cv::Rect(70, 50, 40, 25)), cv::Scalar(0, 180, 0), cv::Scalar(70, 255, 70), green);
    candidate_pixels[i] = cv::countNonZero(green);
  }
  EXPECT_GT(candidate_pixels[0], 0);
  EXPECT_EQ(candidate_pixels[1], 0) << "HIGH is not a substitute for SELECTED";
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, GeometryUsesSelectionStateIndependentlyOfConfidenceAndLegacyDisposition)
{
  auto job = cuboid_stage_job("selection-state");
  auto & hypothesis = job.result.cuboid_hypotheses[3];
  ASSERT_EQ(hypothesis.selection_state, "SELECTED");
  ASSERT_EQ(hypothesis.confidence_grade, "HIGH");
  const auto source_grid = job.result.cuboid_replay_evidence->height_grid_m.clone();
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> selected, alternative;
  arm_cell_vision::snapshot_detail::render_geometry(job, selected);
  hypothesis.selection_state = "ALTERNATIVE";
  arm_cell_vision::snapshot_detail::render_geometry(job, alternative);
  ASSERT_EQ(selected.size(), 3U);
  ASSERT_EQ(alternative.size(), selected.size());
  EXPECT_EQ(selected[0].png, alternative[0].png) << "selection never changes observed geometry";
  EXPECT_NE(selected[1].png, alternative[1].png) << "Runtime selection state remains visible";
  EXPECT_EQ(selected[1].candidate_ids, alternative[1].candidate_ids);
  const auto selected_image = cv::imdecode(selected[1].png, cv::IMREAD_COLOR);
  const auto alternative_image = cv::imdecode(alternative[1].png, cv::IMREAD_COLOR);
  const int width = (selected_image.cols - 10) / 2;
  const cv::Rect final_panel(width + 10, 0, width, selected_image.rows);
  EXPECT_EQ(cv::norm(selected_image(final_panel), alternative_image(final_panel), cv::NORM_INF), 0)
    << "only ordered_candidates define the final returned panel";

  EXPECT_EQ(hypothesis.confidence_grade, "HIGH") << "ALTERNATIVE does not imply LOW";
  EXPECT_EQ(hypothesis.disposition, "qualified_geometry");
  EXPECT_EQ(std::memcmp(source_grid.data, job.result.cuboid_replay_evidence->height_grid_m.data,
    source_grid.total() * source_grid.elemSize()), 0) << "rendering preserves the source heightmap";
}

TEST_F(VisionDetectorSnapshotTest, BoundaryEvidenceSurvivesUnavailablePlaneAndHypothesisEvidence)
{
  auto job = cuboid_stage_job("boundary-only");
  job.result.cuboid_height_patches.clear();
  job.result.cuboid_hypotheses.clear();
  std::vector<arm_cell_vision::VisionDetectorSnapshotJob::CuboidStageImage> images;
  arm_cell_vision::snapshot_detail::render_geometry(job, images);
  ASSERT_EQ(images.size(), 3U);
  EXPECT_EQ(images[0].status, "intermediate_evidence_unavailable");
  const auto rendered = cv::imdecode(images[0].png, cv::IMREAD_COLOR);
  ASSERT_FALSE(rendered.empty());
  const auto boundary_map = rendered(cv::Rect((rendered.cols - 20) / 3 + 10, 58,
      (rendered.cols - 20) / 3, rendered.rows - 70));
  cv::Mat observed_lines;
  cv::inRange(boundary_map, cv::Scalar(180, 180, 180), cv::Scalar(255, 255, 255), observed_lines);
  EXPECT_GT(cv::countNonZero(observed_lines), 0) << "original observed contours remain visible";
}

TEST_F(VisionDetectorSnapshotTest, ObservedAndInferredEdgesUseDifferentStrokePatterns)
{
  cv::Mat observed(30, 200, CV_8UC3, cv::Scalar(0, 0, 0)), inferred = observed.clone();
  using arm_cell_vision::snapshot_detail::evidence_line;
  const auto color = arm_cell_vision::snapshot_detail::observed_color;
  evidence_line(observed, {10, 15}, {190, 15}, color, false, 2);
  evidence_line(inferred, {10, 15}, {190, 15}, color, true, 2);
  cv::Mat observed_mask, inferred_mask;
  cv::cvtColor(observed, observed_mask, cv::COLOR_BGR2GRAY);
  cv::cvtColor(inferred, inferred_mask, cv::COLOR_BGR2GRAY);
  EXPECT_GT(cv::countNonZero(observed_mask), cv::countNonZero(inferred_mask));
  EXPECT_GT(cv::countNonZero(inferred_mask), 0);
}

TEST_F(VisionDetectorSnapshotTest, CombinedGeometryImagesUseOneTrayOrientationForAllPanels)
{
  const std::array<SnapshotOrientationCase, 4> cases{{
    {"camera_x_tray_plus_x", 0.0, 0.0, false, true, 500.0, 500.0},
    {"camera_yaw_90", CV_PI / 2.0, 0.0, false, true, 500.0, 500.0},
    {"camera_roll_180", CV_PI, 0.0, false, true, 500.0, 500.0},
    {"invalid_calibration_keeps_tray_axes", CV_PI / 2.0, 0.0, false, false, 500.0, 500.0}}};
  const std::array<std::string, 2> filenames{{"geometry_observation.png", "geometry_interpretation.png"}};
  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.name);
    auto job = orientation_stage_job(test_case, "orientation-" + test_case.name);
    const auto projected_axes = expected_tray_image_transform(job);
    std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
    const auto root = temporary_snapshot_root("orientation-" + test_case.name);
    const auto date = utc_date_now();
    arm_cell_vision::VisionDetectorSnapshotOutput output(
      *node, root, "orientation-run", {}, [&](const auto & message) {published.push_back(message);},
      date, {}, {}, "123456");
    const auto acquisition_id = job.acquisition_id;
    output.submit(std::move(job));
    output.shutdown();
    ASSERT_EQ(published.size(), 1U);
    const auto directory = root / date / "123456" / "acquisition" / acquisition_id;
    const auto observation = cv::imread((directory / filenames[0]).string(), cv::IMREAD_COLOR);
    const auto interpretation = cv::imread((directory / filenames[1]).string(), cv::IMREAD_COLOR);
    ASSERT_FALSE(observation.empty());
    ASSERT_FALSE(interpretation.empty());
    EXPECT_GE(observation.cols, 2 * 320)
      << "three observation stages preserve map scale";
    EXPECT_GT(interpretation.cols, interpretation.rows)
      << "two aligned panels stay side by side regardless of Tray rotation";
    // Each map still uses the original 80 x 80 grid at scale 4.
    const int observation_panel_width = (observation.cols - 20) / 3;
    const int interpretation_panel_width = (interpretation.cols - 10) / 2;
    EXPECT_EQ(observation_panel_width, interpretation_panel_width)
      << "each Tray map has the same rotated bounds and pixel scale in both images";
    // Each stage has its own Header and map.
    const cv::Mat patch_panel = observation(cv::Rect(2 * (observation_panel_width + 10),
        58, observation_panel_width, 320));
    const cv::Mat raw_panel(patch_panel.size(), CV_8UC3, cv::Scalar(35, 35, 35));
    cv::Mat difference, patch_color_mask;
    cv::absdiff(patch_panel, raw_panel, difference);
    cv::cvtColor(difference, patch_color_mask, cv::COLOR_BGR2GRAY);
    cv::threshold(patch_color_mask, patch_color_mask, 20, 255, cv::THRESH_BINARY);
    std::vector<cv::Point> patch_pixels;
    patch_color_mask(cv::Rect(0, 0, patch_color_mask.cols, 30)).setTo(0);
    patch_color_mask(cv::Rect(0, patch_color_mask.rows - 30, patch_color_mask.cols, 30)).setTo(0);
    patch_color_mask(cv::Rect(0, 0, 30, patch_color_mask.rows)).setTo(0);
    patch_color_mask(cv::Rect(patch_color_mask.cols - 30, 0, 30, patch_color_mask.rows)).setTo(0);
    cv::findNonZero(patch_color_mask, patch_pixels);
    ASSERT_FALSE(patch_pixels.empty());
    const auto patch_bounds = cv::boundingRect(patch_pixels);
    const cv::Point2d tray_x_direction = test_case.calibration_valid ?
      projected_axes.tray_x_pixel_direction : cv::Point2d(1.0, 0.0);
    const bool x_axis_is_horizontal = std::abs(tray_x_direction.x) >=
      std::abs(tray_x_direction.y);
    EXPECT_EQ(patch_bounds.width > patch_bounds.height, x_axis_is_horizontal)
      << "the patch is rotated inside its panel using the existing Tray-to-RGB direction";
    EXPECT_NEAR(std::max(patch_bounds.width, patch_bounds.height), 140, 3)
      << "the same Tray map scale is retained after orientation alignment";
    EXPECT_NE(published.front().metadata_json.find("geometry_observation.png"), std::string::npos);
    EXPECT_NE(published.front().metadata_json.find("geometry_interpretation.png"), std::string::npos);
    std::filesystem::remove_all(root);
  }
}

TEST_F(VisionDetectorSnapshotTest, RgbSnapshotKeepsSourceSizeAndOrientationWithoutDepthInset)
{
  const std::array<SnapshotOrientationCase, 2> cases{{
    {"depth_camera_x_tray_x", 0.0, 0.0, false, true, 500.0, 500.0},
    {"depth_camera_yaw_90", CV_PI / 2.0, 0.0, false, true, 500.0, 500.0}}};
  const std::array<cv::Point, 3> rgb_probe_points{{{0, 0}, {159, 0}, {0, 119}}};
  const std::array<cv::Vec3b, 3> rgb_probe_colors{{
    cv::Vec3b(0, 0, 0), cv::Vec3b(159, 0, 159), cv::Vec3b(119, 119, 0)}};
  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.name);
    auto job = orientation_stage_job(test_case, "depth-orientation-" + test_case.name);
    auto input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
    for (uint32_t row = 0; row < input->rgb.height; ++row) {
      for (uint32_t col = 0; col < input->rgb.width; ++col) {
        const size_t offset = static_cast<size_t>(row) * input->rgb.step + col * 3;
        input->rgb.data[offset] = static_cast<uint8_t>(col);
        input->rgb.data[offset + 1] = static_cast<uint8_t>(row);
        input->rgb.data[offset + 2] = static_cast<uint8_t>((col + row) % 256);
      }
    }
    for (uint32_t row = 57; row <= 63; ++row) {
      for (uint32_t col = 30; col <= 130; ++col) {
        const size_t offset = static_cast<size_t>(row) * input->depth.step +
          static_cast<size_t>(col) * sizeof(float);
        const float depth = 1.0F;
        std::memcpy(input->depth.data.data() + offset, &depth, sizeof(depth));
      }
    }
    job.input = std::move(input);
    std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
    const auto root = temporary_snapshot_root("depth-orientation-" + test_case.name);
    const auto date = utc_date_now();
    const auto acquisition_id = job.acquisition_id;
    arm_cell_vision::VisionDetectorSnapshotOutput output(
      *node, root, "depth-orientation-run", {},
      [&](const auto & message) {published.push_back(message);}, date, root / "diagnostic_records", {}, "123456");
    output.submit(std::move(job));
    output.shutdown();
    ASSERT_EQ(published.size(), 1U);
    const auto snapshot_path = root / date / "123456" / "acquisition" / acquisition_id /
      "snapshot.png";
    const auto snapshot = cv::imread(snapshot_path.string(), cv::IMREAD_COLOR);
    ASSERT_FALSE(snapshot.empty());
    EXPECT_EQ(snapshot.cols, 160);
    EXPECT_EQ(snapshot.rows, 120);
    for (size_t index = 0; index < rgb_probe_points.size(); ++index) {
      EXPECT_EQ(snapshot.at<cv::Vec3b>(rgb_probe_points[index]), rgb_probe_colors[index])
        << "snapshot preserves source RGB pixel " << index << " without rotating RGB";
    }

    std::filesystem::remove_all(root);
  }
}

TEST_F(VisionDetectorSnapshotTest, CuboidStageMetadataRemainsValidForSaveAndEncodingOutcomes)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  const auto original = std::string(
    R"({"cuboid_stage_images":[{"stage":"geometry_candidates","status":"evidence_unavailable","relative_path":"geometry_interpretation.png","candidate_ids":[6,7]}]})");
  Message message;
  message.metadata_json = original;

  arm_cell_vision::detail::set_cuboid_stage_image_metadata(
    message, "geometry_candidates", "saved", true);
  EXPECT_TRUE(metadata_is_valid_json(message.metadata_json));
  EXPECT_NE(message.metadata_json.find("\"status\":\"evidence_unavailable\""), std::string::npos)
    << "successful file save does not replace the evidence availability status";
  EXPECT_NE(message.metadata_json.find("\"relative_path\":\"geometry_interpretation.png\""),
    std::string::npos);

  for (const auto * status : {"encoding_failed", "file_save_failed"}) {
    message.metadata_json = original;
    arm_cell_vision::detail::set_cuboid_stage_image_metadata(
      message, "geometry_candidates", status, false);
    EXPECT_TRUE(metadata_is_valid_json(message.metadata_json)) << status;
    EXPECT_NE(message.metadata_json.find("\"status\":\"" + std::string(status) + "\""),
      std::string::npos);
    EXPECT_EQ(message.metadata_json.find("\"relative_path\""), std::string::npos);
  }
}

TEST_F(VisionDetectorSnapshotTest, CuboidStageImagesRemainReadableWhenThereAreNoHypotheses)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  const auto root = temporary_snapshot_root("cuboid-stages-empty");
  auto job = cuboid_stage_job("stage-empty");
  job.event_kind = Message::DETECT_TARGET_FAILURE;
  job.terminal_result_code = arm_cell_interfaces::msg::DetectTargetResultCode::
    DETECT_RESULT_TEMPORARY_INVALID_TARGET;
  job.terminal_selection_available = false;
  job.result.result_code = job.terminal_result_code;
  job.result.has_target_pose = false;
  job.result.cuboid_hypotheses.clear();
  job.result.cuboid_height_patches.clear();
  job.result.ordered_candidates.clear();
  job.result.rawpart_candidate_diagnostics.clear();
  job.result.rawpart_candidate_count = 0;
  job.result.rawpart_profile_match_count = 0;
  job.result.rawpart_selected_candidate.clear();
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root, "stage-run", {}, [&](const auto & message) {published.push_back(message);},
    utc_date_now(), {}, {}, "123456");
  output.submit(std::move(job));
  output.shutdown();

  ASSERT_EQ(published.size(), 1U);
  EXPECT_FALSE(published.front().image_png.data.empty());
  EXPECT_TRUE(metadata_is_valid_json(published.front().metadata_json));
  EXPECT_NE(published.front().metadata_json.find("\"candidate_count_displayed\":0"),
    std::string::npos);
  const auto directory = root / utc_date_now() / "123456" / "acquisition" / "stage-empty";
  for (const auto * filename : {"geometry_observation.png", "geometry_interpretation.png"}) {
    const auto image = cv::imread((directory / filename).string(), cv::IMREAD_COLOR);
    ASSERT_FALSE(image.empty()) << filename;
  }
  EXPECT_NE(published.front().metadata_json.find("\"status\":\"evidence_unavailable\""), std::string::npos);
  EXPECT_NE(published.front().metadata_json.find("\"candidate_ids\":[]"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(directory / "height_patches.png"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, IncompleteHypothesesStayUnavailableAndRgbTimestampDoesNotGateAnnotation)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  const auto render = [&](const std::string & acquisition_id, bool mismatch_header) {
      const auto root = temporary_snapshot_root(acquisition_id);
      auto job = cuboid_stage_job(acquisition_id);
      job.result.cuboid_hypotheses.pop_back();
      if (mismatch_header) {++job.result.source_rgb_header.stamp.sec;}
      std::vector<Message> published;
      arm_cell_vision::VisionDetectorSnapshotOutput output(
        *node, root, "stage-run", {}, [&](const auto & message) {published.push_back(message);},
        utc_date_now(), {}, {}, "123456");
      output.submit(std::move(job));
      output.shutdown();
      EXPECT_EQ(published.size(), 1U);
      EXPECT_NE(published.front().metadata_json.find("evidence_unavailable"),
        std::string::npos);
      EXPECT_NE(published.front().metadata_json.find("\"stage\":\"final_candidates\",\"status\":\"evidence_unavailable\""),
        std::string::npos);
      const auto directory = root / utc_date_now() / "123456" / "acquisition" / acquisition_id;
      EXPECT_TRUE(std::filesystem::exists(directory / "geometry_interpretation.png"));
      const auto interpretation = cv::imread(
        (directory / "geometry_interpretation.png").string(), cv::IMREAD_COLOR);
      ASSERT_FALSE(interpretation.empty());
      size_t rejected_red_pixels = 0;
      for (int y = 0; y < interpretation.rows; ++y) {
        for (int x = 0; x < interpretation.cols / 2; ++x) {
          const auto pixel = interpretation.at<cv::Vec3b>(y, x);
          if (pixel[2] > 180 && pixel[1] < 100 && pixel[0] < 100) {++rejected_red_pixels;}
        }
      }
      EXPECT_EQ(rejected_red_pixels, 0U)
        << "the unavailable all-hypothesis panel does not draw a partial rejected hypothesis";
      std::filesystem::remove_all(root);
    };
  render("stage-incomplete-counts", false);

  const auto root = temporary_snapshot_root("stage-rgb-timestamp-difference");
  auto job = cuboid_stage_job("stage-rgb-timestamp-difference");
  ++job.result.source_rgb_header.stamp.sec;
  std::vector<Message> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root, "stage-run", {}, [&](const auto & message) {published.push_back(message);},
    utc_date_now(), {}, {}, "123456");
  output.submit(std::move(job));
  output.shutdown();
  ASSERT_EQ(published.size(), 1U);
  EXPECT_EQ(published.front().metadata_json.find("observation_identity_mismatch"),
    std::string::npos);
  EXPECT_NE(published.front().metadata_json.find("\"candidate_ids\":[]"), std::string::npos);
  const auto directory = root / utc_date_now() / "123456" / "acquisition" /
    "stage-rgb-timestamp-difference";
  EXPECT_TRUE(std::filesystem::exists(directory / "geometry_interpretation.png"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, HypothesisCompletenessUsesMatchingRuntimeEvaluationAttempts)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  const auto root = temporary_snapshot_root("stage-attempt-count-mismatch");
  auto job = cuboid_stage_job("stage-attempt-count-mismatch");
  ++job.result.cuboid_height_patches[0].candidate_evaluation_attempt_count;
  std::vector<Message> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root, "stage-run", {}, [&](const auto & message) {published.push_back(message);},
    utc_date_now(), {}, {}, "123456");
  output.submit(std::move(job));
  output.shutdown();

  ASSERT_EQ(published.size(), 1U);
  EXPECT_NE(published.front().metadata_json.find(
      "\"stage\":\"geometry_candidates\",\"status\":\"evidence_unavailable\""),
    std::string::npos);
  EXPECT_NE(published.front().metadata_json.find(
      "\"stage\":\"final_candidates\",\"status\":\"evidence_unavailable\""),
    std::string::npos);
  EXPECT_TRUE(std::filesystem::exists(root / utc_date_now() / "123456" / "acquisition" /
      "stage-attempt-count-mismatch" / "geometry_interpretation.png"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, CuboidStageFileSaveFailureKeepsMetadataValidAndOmitsPaths)
{
  const auto root = temporary_snapshot_root("cuboid-stage-save-failure");
  const auto run_directory = root / "run";
  std::filesystem::create_directories(run_directory / "acquisition");
  auto job = cuboid_stage_job("stage-save-failure");
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root, "stage-run", {}, [&](const auto & message) {published.push_back(message);},
    "20261020", root / "diagnostic_records", {}, {}, run_directory);

  const auto previous_umask = umask(0777);
  output.submit(std::move(job));
  output.shutdown();
  umask(previous_umask);

  ASSERT_EQ(published.size(), 1U);
  EXPECT_TRUE(metadata_is_valid_json(published.front().metadata_json));
  const auto & metadata = published.front().metadata_json;
  for (const auto * stage : {"height_patches", "geometry_candidates", "final_candidates"}) {
    const std::string stage_status = "\"stage\":\"" + std::string(stage) +
      "\",\"status\":\"file_save_failed\"";
    EXPECT_NE(metadata.find(stage_status), std::string::npos) << stage;
  }
  EXPECT_EQ(metadata.find("\"relative_path\""), std::string::npos);

  const auto snapshot_directory = run_directory / "acquisition" / "stage-save-failure";
  std::filesystem::permissions(snapshot_directory, std::filesystem::perms::owner_all,
    std::filesystem::perm_options::replace);
  std::filesystem::permissions(root / "diagnostic_records", std::filesystem::perms::owner_all,
    std::filesystem::perm_options::replace);
  EXPECT_FALSE(std::filesystem::exists(snapshot_directory / "geometry_observation.png"));
  EXPECT_FALSE(std::filesystem::exists(snapshot_directory / "geometry_interpretation.png"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, NewRunUsesTimestampFolderAfterRestartWithoutOverwriting)
{
  const auto root = std::filesystem::temp_directory_path() / "vision-snapshot-sequence-test";
  std::filesystem::remove_all(root);
  const auto date = utc_date_now();
  ASSERT_FALSE(date.empty());
  const auto date_directory = root / date;
  const auto first_directory = date_directory / "01";
  const auto ninety_nine_directory = date_directory / "99";
  std::filesystem::create_directories(first_directory);
  std::filesystem::create_directories(ninety_nine_directory);
  {
    std::ofstream(first_directory / "sentinel") << "first run";
    std::ofstream(ninety_nine_directory / "sentinel") << "ninety ninth run";
  }

  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root, "run-after-restart", {},
    [&](const auto & message) {published.push_back(message);}, date, root / "diagnostic_records", {}, "123456");
  output.submit(calibration_job());
  output.shutdown();

  ASSERT_EQ(published.size(), 1U);
  const auto timestamp_run = date_directory / "123456";
  ASSERT_TRUE(std::filesystem::exists(root / "diagnostic_records" / "snapshot_metadata" / date / "123456" /
      "calibration" / "01" / "metadata.json"));
  std::ifstream first_sentinel(first_directory / "sentinel");
  std::ifstream ninety_nine_sentinel(ninety_nine_directory / "sentinel");
  EXPECT_EQ(std::string((std::istreambuf_iterator<char>(first_sentinel)), {}), "first run");
  EXPECT_EQ(std::string((std::istreambuf_iterator<char>(ninety_nine_sentinel)), {}), "ninety ninth run");
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, SeparateRunsOnOneDateUseTimestampDirectories)
{
  const auto root = std::filesystem::temp_directory_path() / "vision-snapshot-run-sequence-test";
  std::filesystem::remove_all(root);
  const auto date = utc_date_now();
  ASSERT_FALSE(date.empty());

  for (unsigned sequence = 1; sequence <= 2; ++sequence) {
    std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
    auto run_id = "run-" + std::to_string(sequence);
    arm_cell_vision::VisionDetectorSnapshotOutput output(
      *node, root, run_id, {},
      [&](const auto & message) {published.push_back(message);}, date, root / "diagnostic_records", {}, "123456");
    output.submit(calibration_job());
    output.shutdown();

    const auto run_directory = root / date / (sequence == 1 ? "123456" : "123456-1");
    const auto snapshot_directory = run_directory / "calibration" / "01";
    ASSERT_EQ(published.size(), 1U);
    const auto metadata_path = root / "diagnostic_records" / "snapshot_metadata" / date /
      run_directory.filename() / "calibration" / "01" / "metadata.json";
    EXPECT_TRUE(std::filesystem::exists(metadata_path));
    EXPECT_FALSE(std::filesystem::exists(snapshot_directory / "metadata.json"));
    EXPECT_TRUE(std::filesystem::exists(snapshot_directory / "snapshot.png"));
    std::ifstream metadata(metadata_path);
    const std::string saved((std::istreambuf_iterator<char>(metadata)), {});
    EXPECT_NE(saved.find("\"vision_run_id\":\"" + run_id + "\""), std::string::npos);
  }

  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, SessionSnapshotDirectoryUsesSuppliedSessionTimestampPath)
{
  const auto root = std::filesystem::temp_directory_path() / "vision-session-snapshot-directory-test";
  std::filesystem::remove_all(root);
  const auto snapshot_run_directory = root / "snapshots" / "20261020" / "120000-01";
  const auto diagnostic_root = root / "diagnostic_records" / "20261020" / "120000-01" / "vision";
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root / "snapshots", "session-run", {},
    [&](const auto & message) {published.push_back(message);}, "", diagnostic_root,
    "session-id", "", snapshot_run_directory);
  output.submit(calibration_job());
  auto terminal = calibration_job();
  terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "session-acquisition";
  output.submit(terminal);
  output.shutdown();

  ASSERT_EQ(published.size(), 2U);
  const std::array<std::filesystem::path, 2> event_paths{
    std::filesystem::path("calibration") / "01",
    std::filesystem::path("acquisition") / terminal.acquisition_id};
  for (std::size_t index = 0; index < event_paths.size(); ++index) {
    const auto image_directory = snapshot_run_directory / event_paths[index];
    const auto metadata_path = index == 0U ?
      diagnostic_root / "snapshot_metadata" / event_paths[index] / "metadata.json" :
      image_directory / "metadata.json";
    ASSERT_TRUE(std::filesystem::is_regular_file(metadata_path));
    std::ifstream metadata(metadata_path);
    const std::string saved((std::istreambuf_iterator<char>(metadata)), {});
    EXPECT_EQ(saved, published[index].metadata_json);
    EXPECT_NE(saved.find("\"snapshot_directory\":\"" + image_directory.string()), std::string::npos);
    EXPECT_FALSE(cv::imread((image_directory / "snapshot.png").string()).empty());
    EXPECT_EQ(std::filesystem::exists(image_directory / "metadata.json"), index != 0U);
  }
  EXPECT_FALSE(std::filesystem::exists(
      diagnostic_root / "snapshot_metadata" / "acquisition" / "session-acquisition"));
  EXPECT_FALSE(std::filesystem::exists(snapshot_run_directory / "20261020" / "120000-01"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, MetadataDirectoryFailureKeepsPngAndPublicationForSubsequentJobs)
{
  const auto root = std::filesystem::temp_directory_path() / "vision-metadata-directory-failure-test";
  std::filesystem::remove_all(root);
  const auto diagnostic_root = root / "diagnostic_records" / "vision";
  const auto snapshot_run_directory = root / "snapshots" / "session";
  std::filesystem::create_directories(diagnostic_root);
  std::ofstream(diagnostic_root / "snapshot_metadata") << "not a directory";
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root / "snapshots", "failure-run", {},
    [&](const auto & message) {published.push_back(message);}, "", diagnostic_root,
    "session-id", "", snapshot_run_directory);
  output.submit(calibration_job());
  auto terminal = calibration_job();
  terminal.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "failed-acquisition";
  terminal.terminal_result_code = 2;
  terminal.terminal_diagnostic = "deadline expired";
  output.submit(terminal);
  output.shutdown();

  ASSERT_EQ(published.size(), 2U);
  EXPECT_EQ(published.front().event_kind,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_COMPLETE);
  EXPECT_EQ(published.back().event_kind, terminal.event_kind);
  EXPECT_EQ(published.back().acquisition_id, terminal.acquisition_id);
  EXPECT_NE(published.back().diagnostic_record_json.find("deadline expired"), std::string::npos);
  EXPECT_NE(published.back().diagnostic_record_json.find("\"result_code\":2"), std::string::npos);
  const std::array<std::filesystem::path, 2> event_paths{
    std::filesystem::path("calibration") / "01",
    std::filesystem::path("acquisition") / terminal.acquisition_id};
  for (std::size_t index = 0; index < event_paths.size(); ++index) {
    const auto image_path = snapshot_run_directory / event_paths[index] / "snapshot.png";
    EXPECT_FALSE(cv::imread(image_path.string()).empty());
    std::ifstream image(image_path, std::ios::binary);
    const std::vector<uint8_t> saved((std::istreambuf_iterator<char>(image)), {});
    EXPECT_EQ(saved, published[index].image_png.data);
    EXPECT_TRUE(metadata_is_valid_json(published[index].metadata_json));
  }
  EXPECT_TRUE(std::filesystem::is_regular_file(diagnostic_root / "snapshot_metadata"));
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, TerminalSnapshotIsDequeuedBeforePendingProgress)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool first_publish_started = false;
  bool release_publish = false;
  std::vector<std::pair<uint8_t, std::string>> events;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-priority",
    [](const auto &, const auto &) {return true;},
    [&](const auto & message) {
      std::unique_lock<std::mutex> lock(mutex);
      if (!first_publish_started) {
        first_publish_started = true;
        condition.notify_all();
        condition.wait(lock, [&]() {return release_publish;});
      }
      events.emplace_back(message.event_kind, message.acquisition_id);
      condition.notify_all();
    });
  auto first_progress = calibration_job();
  first_progress.event_kind =
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS;
  first_progress.calibration_id.clear();
  first_progress.acquisition_id = "acq-first";
  first_progress.save = false;
  output.submit(std::move(first_progress));
  bool first_publish_observed = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    first_publish_observed = condition.wait_for(lock, std::chrono::seconds(2), [&]() {
      return first_publish_started;
    });
    if (!first_publish_observed) {release_publish = true;}
  }
  condition.notify_all();
  ASSERT_TRUE(first_publish_observed);
  auto terminal = calibration_job();
  terminal.event_kind =
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  terminal.calibration_id.clear();
  terminal.acquisition_id = "acq-priority";
  output.submit(std::move(terminal));
  auto pending_progress = calibration_job();
  pending_progress.event_kind =
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS;
  pending_progress.calibration_id.clear();
  pending_progress.acquisition_id = "acq-next";
  pending_progress.save = false;
  output.submit(std::move(pending_progress));
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_publish = true;
  }
  condition.notify_all();
  output.shutdown();
  ASSERT_EQ(events.size(), 3U);
  EXPECT_EQ(events[0].first,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS);
  EXPECT_EQ(events[0].second, "acq-first");
  EXPECT_EQ(events[1].first,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE);
  EXPECT_EQ(events[1].second, "acq-priority");
  EXPECT_EQ(events[2].first,
    arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS);
  EXPECT_EQ(events[2].second, "acq-next");
}

TEST_F(VisionDetectorSnapshotTest, BlockedFileWriteDoesNotBlockSnapshotSubmission)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool save_started = false;
  bool release_save = false;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-4",
    [&](const auto &, const auto &) {
      std::unique_lock<std::mutex> lock(mutex);
      save_started = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_save;});
      return true;
    }, [](const auto &) {});
  auto first = calibration_job();
  output.submit(std::move(first));
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() {return save_started;}));
  }
  auto progress = calibration_job();
  progress.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS;
  progress.save = false;
  const auto start = std::chrono::steady_clock::now();
  output.submit(std::move(progress));
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_save = true;
  }
  condition.notify_all();
  output.shutdown();
}

TEST_F(VisionDetectorSnapshotTest, BlockedSnapshotWriteDoesNotDelayDetectTargetResult)
{
  arm_cell_vision::VisionRosIngress ingress;
  std::mutex mutex;
  std::condition_variable condition;
  bool save_started = false;
  bool release_save = false;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "run-6",
    [&](const auto &, const auto &) {
      std::unique_lock<std::mutex> lock(mutex);
      save_started = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return release_save;});
      return true;
    }, [](const auto &) {});
  output.submit(calibration_job());
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() {return save_started;}));
  }
  for (size_t index = 0; index < 16; ++index) {
    auto queued = calibration_job();
    queued.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
    queued.acquisition_id = "queued-" + std::to_string(index);
    output.submit(std::move(queued));
  }
  int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  arm_cell_vision::DetectionResult retained;
  retained.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  retained.has_target_pose = true;
  retained.target_pose.header.frame_id = "base_link";
  retained.target_pose.pose.orientation.w = 1.0;
  retained.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  retained.processing_stage = "cuboid_top_face";
  retained.ordered_candidates = {retained};
  retained.ordered_candidates.front().confidence_grade = "HIGH";
  arm_cell_vision::DetectTargetService service(
    ingress, [&now]() {return now++;},
    [](const std::string &, uint64_t, int64_t, int64_t, const std::string &) {
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    }, []() {},
    [&retained](const std::string &, uint64_t generation) {
      return make_acquisition_result(retained, generation);
    }, {},
    [&output](const std::string &, uint64_t generation,
      const arm_cell_interfaces::srv::DetectTarget::Response & response,
      const arm_cell_vision::SharedAcquisitionResult & acquisition_result,
      const arm_cell_vision::VisionRequestDiagnosticRecord & diagnostics) {
      arm_cell_vision::VisionDetectorSnapshotJob job;
      job.event_kind = response.result_code.value ==
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS ?
        arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS :
        arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
      job.acquisition_id = response.acquisition_id;
      job.acquisition_result = acquisition_result;
      job.has_observation = acquisition_result && acquisition_result->input;
      job.has_detection_result = static_cast<bool>(acquisition_result);
      job.processing_stage = acquisition_result ? acquisition_result->detection.processing_stage : "request_deadline";
      job.terminal_result_code = response.result_code.value;
      job.terminal_selection_available = !response.material_candidates.empty();
      job.terminal_scene_generation = response.scene_generation;
      if (diagnostics.ingress.detailed_capture_enabled) {job.request_diagnostics = diagnostics;}
      job.save = true;
      output.submit(std::move(job));
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response response;
  service.handle(request, response);
  EXPECT_EQ(response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS);
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() {return save_started;}));
    release_save = true;
  }
  condition.notify_all();
  output.shutdown();
}

TEST_F(VisionDetectorSnapshotTest, ServiceAndSnapshotConsumeTheSameCompletedAcquisition)
{
  arm_cell_vision::VisionRosIngress ingress;
  auto completed = std::make_shared<arm_cell_vision::AcquisitionResult>();
  completed->target_id = "RawPart";
  completed->detection.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  completed->detection.algorithm = "cuboid_top_face";
  completed->detection.processing_stage = "depth_observation";
  completed->detection.has_target_pose = true;
  completed->detection.target_pose.header.frame_id = "base_link";
  completed->detection.target_pose.pose.position.x = 0.123;
  completed->detection.target_pose.pose.orientation.w = 1.0;
  completed->detection.object_region_valid = true;
  completed->detection.object_region_x = 10;
  completed->detection.object_region_y = 10;
  completed->detection.object_region_width = 20;
  completed->detection.object_region_height = 20;
  completed->detection.orientation_symmetry_order = 1;
  completed->detection.rawpart_candidate_count = 1;
  auto candidate = completed->detection;
  candidate.ordered_candidates.clear();
  candidate.confidence_grade = "HIGH";
  candidate.target_pose.pose.position.x = 0.123;
  completed->detection.ordered_candidates.push_back(std::move(candidate));
  auto input = std::make_shared<arm_cell_vision::DetectionInput>();
  input->rgb.header.frame_id = "camera_color_optical_frame";
  input->rgb.height = 48;
  input->rgb.width = 64;
  input->rgb.encoding = "rgb8";
  input->rgb.step = input->rgb.width * 3;
  input->rgb.data.assign(input->rgb.step * input->rgb.height, 32);
  completed->input = input;

  std::mutex mutex;
  std::condition_variable condition;
  bool snapshot_published = false;
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, "/tmp/snapshot-test-unused", "vision-run-shared",
    [](const auto &, const auto &) {return true;},
    [&](const auto & message) {
      std::lock_guard<std::mutex> lock(mutex);
      published = message;
      snapshot_published = true;
      condition.notify_all();
    });

  int64_t now = 100;
  arm_cell_vision::DetectTargetService service(
    ingress, [&now]() {return now++;},
    [](const std::string &, uint64_t, int64_t, int64_t, const std::string &) {
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    },
    []() {},
    [&](const std::string &, uint64_t generation) {
      completed->generation = generation;
      completed->acquisition_id =
        arm_cell_vision::make_acquisition_id("vision-run-shared", generation);
      completed->detection.scene_generation = generation;
      return std::shared_ptr<const arm_cell_vision::AcquisitionResult>(completed);
    },
    {},
    [&](const std::string &, uint64_t generation,
      const arm_cell_interfaces::srv::DetectTarget::Response & response,
      const arm_cell_vision::SharedAcquisitionResult & acquisition_result,
      const arm_cell_vision::VisionRequestDiagnosticRecord &) {
      EXPECT_EQ(acquisition_result.get(), completed.get());
      EXPECT_EQ(acquisition_result->generation, generation);
      arm_cell_vision::VisionDetectorSnapshotJob job;
      job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
      job.acquisition_id = response.acquisition_id;
      job.target_id = "RawPart";
      job.acquisition_result = acquisition_result;
      job.has_observation = static_cast<bool>(acquisition_result->input);
      job.has_detection_result = true;
      job.processing_stage = acquisition_result->detection.processing_stage;
      job.terminal_result_code = response.result_code.value;
      job.terminal_selection_available = true;
      job.terminal_scene_generation = response.scene_generation;
      job.save = true;
      output.submit(std::move(job));
    },
    "vision-run-shared");

  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response response;
  service.handle(request, response);
  ASSERT_EQ(response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS);
  EXPECT_DOUBLE_EQ(response.material_candidates.at(0).target_pose.pose.position.x, 0.123);
  ASSERT_EQ(response.material_candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(response.material_candidates.front().target_pose.pose.position.x, 0.123);

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() {
      return snapshot_published;
    }));
  }
  output.shutdown();

  EXPECT_NE(published.metadata_json.find("\"source_image_rendered\":true"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"observation\":{\"rgb_available\":"),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"aligned_depth\":"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("\"has_target_pose\":true"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("target_pose_position_m\":[0.123"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"position_m\":[0.123"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"candidate_count_displayed\":1"), std::string::npos);
  EXPECT_EQ(published.image_png.header.frame_id, input->rgb.header.frame_id);
  const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.cols, 64);
  EXPECT_EQ(image.rows, 48);
  EXPECT_EQ(image.at<cv::Vec3b>(10, 10), cv::Vec3b(0, 255, 0));
}

TEST_F(VisionDetectorSnapshotTest, CuboidOutlineUsesCameraModelAndConfiguredSnapshotStyle)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("snapshot_style.candidate.color_rgb", "#00FF00"),
    rclcpp::Parameter("snapshot_style.candidate.line_width_px", 5)});
  auto style_node = std::make_shared<rclcpp::Node>("snapshot_projection_style_test", options);
  auto job = cuboid_projection_job("plumb_bob");
  arm_cell_vision::RawPartCandidateDiagnostic stale_pixel_outline;
  stale_pixel_outline.profile_match = true;
  stale_pixel_outline.selected_for_target = true;
  stale_pixel_outline.fitted_outline_pixels.resize(4);
  stale_pixel_outline.fitted_outline_pixels[0].x = 10;
  stale_pixel_outline.fitted_outline_pixels[0].y = 10;
  stale_pixel_outline.fitted_outline_pixels[1].x = 30;
  stale_pixel_outline.fitted_outline_pixels[1].y = 10;
  stale_pixel_outline.fitted_outline_pixels[2].x = 30;
  stale_pixel_outline.fitted_outline_pixels[2].y = 30;
  stale_pixel_outline.fitted_outline_pixels[3].x = 10;
  stale_pixel_outline.fitted_outline_pixels[3].y = 30;
  job.result.rawpart_candidate_diagnostics = {stale_pixel_outline};
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(*style_node,
    temporary_snapshot_root("projection-style"), "projection-run", {},
    [&](const auto & message) {published = message;});
  output.submit(std::move(job));
  output.shutdown();
  const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.at<cv::Vec3b>(110, 130), cv::Vec3b(0, 255, 0));
  EXPECT_EQ(image.at<cv::Vec3b>(10, 10), cv::Vec3b(0, 0, 0));
  EXPECT_NE(published.metadata_json.find("\"candidate_color_rgb\":\"#00FF00\""), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"candidate_line_width_px\":5"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"schema_version\":19"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, InvalidCameraModelOmitsOnlyCuboidOutline)
{
  auto job = cuboid_projection_job("unsupported_model");
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(*node,
    temporary_snapshot_root("invalid-projection"), "projection-run", {},
    [&](const auto & message) {published = message;});
  output.submit(std::move(job));
  output.shutdown();
  const auto image = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(image.empty());
  EXPECT_EQ(image.at<cv::Vec3b>(110, 130), cv::Vec3b(0, 0, 0));
  EXPECT_NE(published.diagnostic_record_json.find("cuboid_top_face"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, SnapshotReportsOrderedCandidatesWithoutOccupancyResidualOutlines)
{
  auto job = calibration_job();job.save = false;
  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
  input->rgb.width = 200;
  input->rgb.height = 200;
  input->rgb.step = 600;
  input->rgb.data.assign(200 * 200 * 3, 32);
  job.input = input;
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
  job.acquisition_id = "acq-version-ten";
  job.algorithm = "cuboid_top_face";
  job.has_detection_result = true;
  job.result.algorithm = "cuboid_top_face";
  job.result.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  job.result.has_rawpart_diagnostics = true;
  job.result.thickness_evidence = "material_face_hypothesis";
  job.result.has_estimated_height = true;job.result.estimated_height_m = 0.03F;
  job.result.unresolved_pixels = 7;
  job.result.rawpart_candidate_generation = "unique_complete_partition";
  arm_cell_vision::RawPartCandidateDiagnostic ranked;
  ranked.candidate_id = 0;
  ranked.profile_match = true;
  ranked.selected_for_target = true;
  ranked.fitted_outline_pixels.resize(4);
  ranked.fitted_outline_pixels[0].x = 100; ranked.fitted_outline_pixels[0].y = 100;
  ranked.fitted_outline_pixels[1].x = 140; ranked.fitted_outline_pixels[1].y = 100;
  ranked.fitted_outline_pixels[2].x = 140; ranked.fitted_outline_pixels[2].y = 140;
  ranked.fitted_outline_pixels[3].x = 100; ranked.fitted_outline_pixels[3].y = 140;
  job.result.rawpart_candidate_diagnostics = {ranked};
  job.result.ordered_candidates = {job.result};
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(*node, "/tmp/snapshot-test-unused", "v7-run",
    {},
    [&](const auto & message) {published = message;});
  output.submit(job);output.shutdown();
  const auto rendered = cv::imdecode(published.image_png.data, cv::IMREAD_COLOR);
  ASSERT_FALSE(rendered.empty());
  EXPECT_EQ(rendered.at<cv::Vec3b>(120, 100), cv::Vec3b(32, 32, 32));
  EXPECT_EQ(rendered.at<cv::Vec3b>(90, 20), cv::Vec3b(32, 32, 32));
  EXPECT_EQ(rendered.at<cv::Vec3b>(80, 170), cv::Vec3b(32, 32, 32));
  EXPECT_NE(published.metadata_json.find("\"schema_version\":19"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"algorithm\":\"cuboid_top_face\""), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"candidate_count_displayed\":"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"partition_status\":"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"partition_status\":\"unique_complete_partition\""),
    std::string::npos);
  EXPECT_EQ(published.diagnostic_record_json.find("unresolved_regions"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"unresolved_pixels\":7"), std::string::npos);
  EXPECT_EQ(published.metadata_json.find("algorithm"), std::string::npos);
}
namespace
{
arm_cell_vision::VisionDetectorSnapshotJob rgb_overlay_job()
{
  auto job = cuboid_stage_job("rgb-overlay");
  auto input = std::make_shared<arm_cell_vision::DetectionInput>(*job.input);
  input->rgb.width = 640;
  input->rgb.height = 480;
  input->rgb.step = 640 * 3;
  input->rgb.data.assign(input->rgb.step * input->rgb.height, 0);
  job.input = input;
  auto & high = job.result.rawpart_candidate_diagnostics.front();
  const auto set_outline = [](auto & diagnostic, float x) {
      diagnostic.fitted_outline_pixels.resize(4);
      const std::array<std::array<float, 2>, 4> corners{{
        {{x, 180}}, {{x + 120, 180}}, {{x + 120, 300}}, {{x, 300}}}};
      for (size_t i = 0; i < corners.size(); ++i) {
        diagnostic.fitted_outline_pixels[i].x = corners[i][0];
        diagnostic.fitted_outline_pixels[i].y = corners[i][1];
      }
    };
  set_outline(high, 150);
  auto medium = high;
  medium.candidate_id = 87;
  medium.confidence_grade = "MEDIUM";
  set_outline(medium, 400);
  job.result.rawpart_candidate_diagnostics.push_back(medium);
  auto medium_candidate = job.result.ordered_candidates.front();
  medium_candidate.rawpart_selected_candidate = "87";
  medium_candidate.confidence_grade = "MEDIUM";
  medium_candidate.has_observed_rectangle = false;
  job.result.ordered_candidates.push_back(medium_candidate);
  job.save = false;
  return job;
}

cv::Mat grade_mask(const cv::Mat & image, const cv::Scalar & color)
{
  cv::Mat mask;
  cv::inRange(image, color * 0.85, color + cv::Scalar(5, 5, 5), mask);
  return mask;
}

void save_rgb_preview(const std::string & name, const arm_cell_interfaces::msg::VisionDetectorSnapshot & message)
{
  if (const char * preview = std::getenv("SFTWIN_SNAPSHOT_PREVIEW")) {
    std::filesystem::create_directories(preview);
    const auto prefix = std::filesystem::path(preview) / name;
    std::ofstream png(prefix.string() + ".png", std::ios::binary);
    png.write(reinterpret_cast<const char *>(message.image_png.data.data()), message.image_png.data.size());
    std::ofstream(prefix.string() + ".metadata.json") << message.metadata_json;
    std::ofstream(prefix.string() + ".diagnostic_record.json") << message.diagnostic_record_json;
  }
}
}  // namespace

TEST_F(VisionDetectorSnapshotTest, RgbOverlayReusesConfidenceColorsWithoutInsetFooterOrPoseArrows)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("snapshot_style.candidate.color_rgb", "#22DD00")});
  auto style_node = std::make_shared<rclcpp::Node>("snapshot_overlay_style_test", options);
  auto job = rgb_overlay_job();
  std::vector<arm_cell_interfaces::msg::VisionDetectorSnapshot> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(*style_node,
    temporary_snapshot_root("rgb-overlay"), "overlay-run", {},
    [&](const arm_cell_interfaces::msg::VisionDetectorSnapshot & message) {published.push_back(message);});
  output.submit(job);
  auto low = job.result.ordered_candidates.front();
  low.confidence_grade = "LOW";
  low.rawpart_selected_candidate = "15";
  job.result.ordered_candidates.push_back(low);
  output.submit(job);
  job.result.ordered_candidates.back().confidence_grade.clear();
  output.submit(job);
  output.shutdown();
  ASSERT_EQ(published.size(), 3U);
  save_rgb_preview("rgb-overlay-success", published[0]);
  save_rgb_preview("rgb-overlay-unavailable-counts", published[2]);
  const auto image = cv::imdecode(published[0].image_png.data, cv::IMREAD_COLOR);
  ASSERT_EQ(image.size(), cv::Size(640, 480));
  EXPECT_EQ(cv::norm(image(cv::Rect(0, image.rows - 60, image.cols, 60)), cv::NORM_INF), 0)
    << "source bottom rows must not contain footer legends";
  const auto high = cv::Scalar(0, 221, 34);
  const auto medium = arm_cell_vision::snapshot_detail::confidence_color("MEDIUM");
  const cv::Rect summary(0, 0, image.cols, 60);
  EXPECT_GT(cv::countNonZero(grade_mask(image(summary), high)), 0);
  EXPECT_GT(cv::countNonZero(grade_mask(image(summary), medium)), 0);
  EXPECT_GT(cv::countNonZero(grade_mask(image(cv::Rect(140, 175, 140, 135)), high)), 0);
  EXPECT_GT(cv::countNonZero(grade_mask(image(cv::Rect(390, 175, 140, 135)), medium)), 0);
  // Interiors are unmodified source pixels, including where the old arrows pointed.
  for (const auto & interior : {cv::Rect(170, 200, 80, 80), cv::Rect(420, 200, 80, 80)}) {
    EXPECT_EQ(cv::norm(image(interior), cv::NORM_INF), 0);
  }
  // A fully observed side is continuous; reconstruction retains visible dash gaps.
  const auto solid = grade_mask(image(cv::Rect(150, 205, 1, 70)), high);
  const auto dashed = grade_mask(image(cv::Rect(400, 205, 1, 70)), medium);
  EXPECT_EQ(cv::countNonZero(solid), solid.rows);
  EXPECT_GT(cv::countNonZero(dashed), 0);
  EXPECT_LT(cv::countNonZero(dashed), dashed.rows);
  const auto with_low = cv::imdecode(published[1].image_png.data, cv::IMREAD_COLOR);
  EXPECT_EQ(cv::norm(image, with_low, cv::NORM_INF), 0)
    << "LOW neither adds a normal outline nor changes HIGH/MEDIUM summary counts";
  const auto missing_grade = cv::imdecode(published[2].image_png.data, cv::IMREAD_COLOR);
  EXPECT_EQ(cv::countNonZero(grade_mask(missing_grade(summary), high)), 0);
  EXPECT_EQ(cv::countNonZero(grade_mask(missing_grade(summary), medium)), 0)
    << "missing confidence is unavailable, not a fabricated zero";
  EXPECT_NE(published[1].diagnostic_record_json.find("LOW"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, RgbFailureOverlaysKeepObservedEvidenceWithoutNormalCandidateSummary)
{
  using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  std::vector<Message> published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(*node,
    temporary_snapshot_root("rgb-failure-overlay"), "failure-overlay-run", {},
    [&](const arm_cell_interfaces::msg::VisionDetectorSnapshot & message) {published.push_back(message);});
  for (const auto code : {Code::DETECT_RESULT_GEOMETRY_ERROR,
      Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET, Code::DETECT_RESULT_OBJECT_NOT_FOUND,
      Code::DETECT_RESULT_SENSOR_ERROR})
  {
    auto job = rgb_overlay_job();
    job.event_kind = Message::DETECT_TARGET_FAILURE;
    job.terminal_result_code = code;
    job.result.result_code = code;
    output.submit(job);
  }
  auto canonical_failure = rgb_overlay_job();
  canonical_failure.event_kind = Message::DETECT_TARGET_FAILURE;
  canonical_failure.terminal_result_code = Code::DETECT_RESULT_GEOMETRY_ERROR;
  canonical_failure.terminal_selection_available = false;
  canonical_failure.result.result_code = Code::DETECT_RESULT_SUCCESS;
  output.submit(canonical_failure);
  auto no_rgb = rgb_overlay_job();
  auto missing_input = std::make_shared<arm_cell_vision::DetectionInput>(*no_rgb.input);
  missing_input->rgb = sensor_msgs::msg::Image();
  no_rgb.input = missing_input;
  output.submit(no_rgb);
  output.shutdown();
  ASSERT_EQ(published.size(), 6U);
  const auto canonical_image = cv::imdecode(published[4].image_png.data, cv::IMREAD_COLOR);
  const auto geometry_error_image = cv::imdecode(published[0].image_png.data, cv::IMREAD_COLOR);
  EXPECT_EQ(cv::norm(canonical_image, geometry_error_image, cv::NORM_INF), 0)
    << "failure overlay uses the terminal result, not retained single-observation success";
  for (size_t i = 0; i < 4; ++i) {
    save_rgb_preview("rgb-overlay-error-" + std::to_string(i), published[i]);
  }
  save_rgb_preview("rgb-overlay-no-source", published.back());
  for (size_t i = 0; i < 4; ++i) {
    const auto image = cv::imdecode(published[i].image_png.data, cv::IMREAD_COLOR);
    ASSERT_EQ(image.size(), cv::Size(640, 480));
    const auto top = image(cv::Rect(0, 0, image.cols, 60));
    EXPECT_EQ(cv::countNonZero(grade_mask(top, cv::Scalar(0, 200, 0))), 0);
    EXPECT_EQ(cv::countNonZero(grade_mask(top,
        arm_cell_vision::snapshot_detail::confidence_color("MEDIUM"))), 0);
    EXPECT_EQ(image.at<cv::Vec3b>(150, 320), cv::Vec3b(0, 0, 0));
    EXPECT_EQ(cv::norm(image(cv::Rect(170, 200, 80, 80)), cv::NORM_INF), 0);
  }
  EXPECT_NE(published.back().metadata_json.find("\"source_image_rendered\":false"), std::string::npos);
  EXPECT_NE(published.back().metadata_json.find("\"rgb_available\":false"), std::string::npos);
}

TEST_F(VisionDetectorSnapshotTest, DetailedDepthScenePanoramaUsesSnapshotWorkerOnly)
{
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  job.acquisition_id = "depth-scene-panorama";
  job.depth_scene_diagnostics_enabled = true;
  arm_cell_vision::DepthSceneStabilityConfig stability_config;
  stability_config.detailed_diagnostics_enabled = true;
  arm_cell_vision::DepthSceneStabilityMonitor monitor(stability_config);
  for (int i = 0; i < 6; ++i) {
    auto depth = cv::Mat(20, 20, CV_32F, cv::Scalar(1.0F));
    if (i > 0) {depth(cv::Rect(5, 5, 4, 4)).setTo(0.95F);}
    arm_cell_vision::DepthSceneFrame frame;
    frame.stamp_ns = (i + 1) * 33'000'000LL;
    frame.receipt_monotonic_ns = (i + 1) * 33'000'000LL;
    frame.depth_m = depth;
    const auto state = monitor.observe(frame);
    arm_cell_vision::VisionDetectorSnapshotJob::DepthSceneSample sample;
    sample.stamp_ns = (i + 1) * 33'000'000LL;
    sample.acquisition_elapsed_ms = (i + 1) * 33.0;
    sample.processing_elapsed_ms = 8.0;
    sample.receipt_monotonic_ns = frame.receipt_monotonic_ns;
    sample.state = state;
    sample.observed_depth_m = depth.clone();
    job.depth_scene_samples.push_back(std::move(sample));
    if (job.depth_scene_samples.size() > 5) {
      job.depth_scene_samples.erase(job.depth_scene_samples.begin());
    }
  }
  job.depth_scene_stability = job.depth_scene_samples.back().state;
  EXPECT_EQ(job.depth_scene_samples.back().state.status, arm_cell_vision::DepthSceneStatus::kStable);
  EXPECT_EQ(job.depth_scene_samples.front().state.qualifying_observations, 1U);
  const auto root = temporary_snapshot_root("scene-panorama");
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, root / "artifacts", "scene-run", {},
    [&](const auto & message) {published = message;}, "", root / "diagnostics",
    "", "", root / "snapshot-run");
  auto missing_inputs = job;
  for (auto & sample : missing_inputs.depth_scene_samples) {sample.observed_depth_m.release();}
  std::vector<uint8_t> missing_png, present_png;
  arm_cell_vision::snapshot_detail::render_scene_stability(missing_inputs, missing_png);
  arm_cell_vision::snapshot_detail::render_scene_stability(job, present_png);
  EXPECT_NE(missing_png, present_png) << "actual inputs add scenes without inventing missing inputs";
  const auto missing_image = cv::imdecode(missing_png, cv::IMREAD_COLOR);
  const auto present_image = cv::imdecode(present_png, cv::IMREAD_COLOR);
  EXPECT_EQ(cv::norm(missing_image(cv::Rect(0, 0, 860, 716)),
    present_image(cv::Rect(0, 0, 860, 716)), cv::NORM_INF), 0)
    << "retaining scenes does not change stability statistics or comparison charts";
  if (const char * preview = std::getenv("SFTWIN_SNAPSHOT_PREVIEW")) {
    std::filesystem::create_directories(preview);
    cv::imwrite((std::filesystem::path(preview) / "scene_stability_panorama.png").string(), present_image);
  }
  output.submit(std::move(job));
  output.shutdown();
  const auto panorama = root / "snapshot-run" / "acquisition" / "depth-scene-panorama" /
    "scene_stability_panorama.png";
  ASSERT_TRUE(std::filesystem::exists(panorama));
  const auto rendered_panorama = cv::imread(panorama.string(), cv::IMREAD_COLOR);
  ASSERT_FALSE(rendered_panorama.empty());
  const cv::Mat ratio_chart = rendered_panorama(cv::Rect(34, 54, 814, 59));
  cv::Mat baseline_series, previous_series;
  cv::inRange(ratio_chart, cv::Scalar(225, 115, 25), cv::Scalar(235, 125, 35), baseline_series);
  cv::inRange(ratio_chart, cv::Scalar(35, 165, 240), cv::Scalar(45, 175, 250), previous_series);
  EXPECT_GT(cv::countNonZero(baseline_series), 0)
    << "the changed-pixel ratio chart includes recorded baseline comparisons";
  EXPECT_GT(cv::countNonZero(previous_series), 3)
    << "the changed-pixel ratio chart includes recorded previous-observation comparisons";
  EXPECT_TRUE(std::filesystem::exists(root / "diagnostics" / "depth-scene-panorama.json"));
  EXPECT_NE(published.metadata_json.find("scene_stability_panorama.png"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"qualifying_observations\":5"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"recent_observation_count\":5"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("depth_change_reset_consecutive_observations"),
    std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"changed_pixel_ratio\":"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"comparable_pixels\":"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"valid_depth_pixels\":"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"unobserved_depth_pixels\":"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("depth_difference_tolerance_m"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("changed_pixel_ratio_tolerance"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("absolute_difference_histogram_unit"), std::string::npos);
  EXPECT_NE(published.metadata_json.find("\"processing_elapsed_ms\":8"), std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"comparable_pixels\":"),
    std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("\"changed_pixel_ratio_tolerance\":"),
    std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("absolute_difference_histogram_unit"),
    std::string::npos);
  EXPECT_NE(published.diagnostic_record_json.find("scene_stability_panorama.png"),
    std::string::npos);
  std::filesystem::remove_all(root);
}

TEST_F(VisionDetectorSnapshotTest, DisabledDepthSceneCaptureDoesNotRenderPanorama)
{
  auto job = calibration_job();
  job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  job.acquisition_id = "depth-scene-no-panorama";
  arm_cell_vision::VisionDetectorSnapshotJob::DepthSceneSample sample;
  job.depth_scene_samples.push_back(std::move(sample));
  bool panorama_empty = false;
  arm_cell_interfaces::msg::VisionDetectorSnapshot published;
  arm_cell_vision::VisionDetectorSnapshotOutput output(
    *node, temporary_snapshot_root("scene-no-panorama"), "scene-run",
    [&](const auto & rendered_job, const auto & message) {
      panorama_empty = rendered_job.depth_scene_panorama_png.empty();
      published = message;
      return true;
    }, [&](const auto & message) {published = message;});
  output.submit(std::move(job));
  output.shutdown();
  EXPECT_TRUE(panorama_empty);
  EXPECT_EQ(published.metadata_json.find("scene_stability_panorama.png"), std::string::npos);
}
}  // namespace
