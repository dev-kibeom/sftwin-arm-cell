#ifndef ARM_CELL_VISION__VISION_DETECTOR_SNAPSHOT_HPP_
#define ARM_CELL_VISION__VISION_DETECTOR_SNAPSHOT_HPP_

#include <condition_variable>
#include <array>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <arm_cell_interfaces/msg/vision_detector_snapshot.hpp>
#include <rclcpp/rclcpp.hpp>
#include "arm_cell_vision/detect_target_processor.hpp"
#include "arm_cell_vision/detect_target_acquisition.hpp"
#include "arm_cell_vision/depth_scene_stability.hpp"
#include "arm_cell_vision/vision_ros_ingress.hpp"

namespace arm_cell_vision
{

struct VisionDetectorSnapshotJob
{
  struct DepthSceneSample
  {
    DepthSceneStabilityState state;
    cv::Mat observed_depth_m;  // Owned copy of this exact comparison input; detailed capture only.
    int64_t stamp_ns = 0;
    int64_t receipt_monotonic_ns = 0;
    double acquisition_elapsed_ms = 0.0;
    double processing_elapsed_ms = 0.0;
  };
  struct CuboidStageImage
  {
    std::string stage;
    std::string filename;
    std::vector<uint8_t> png;
    std::vector<uint32_t> candidate_ids;
    std::string status = "not_rendered";
  };
  uint8_t event_kind = 0;
  std::string vision_run_id;
  std::string calibration_id;
  std::string acquisition_id;
  std::string target_id;
  SharedAcquisitionResult acquisition_result;
  // Calibration and isolated renderer fixtures can carry input directly.
  std::string algorithm;
  std::shared_ptr<const DetectionInput> input;
  bool has_observation = false;
  bool has_detection_result = false;
  DetectionResult result;
  std::optional<DetectorProfile> selected_profile;
  SupportCalibrationSnapshot calibration;
  DepthSceneStabilityState depth_scene_stability;
  int64_t stable_depth_stamp_ns = 0;
  bool depth_scene_diagnostics_enabled = false;
  std::vector<DepthSceneSample> depth_scene_samples;
  std::vector<uint8_t> depth_scene_panorama_png;
  std::array<double, 2> tray_roi_min_m{};
  std::array<double, 2> tray_roi_max_m{};
  double tray_depth_resolution_m = 0.002;
  uint64_t revision = 0;
  std::string processing_stage;
  std::string last_observation_stage;
  uint8_t terminal_result_code = 0;
  std::string terminal_diagnostic;
  bool terminal_selection_available = false;
  uint64_t terminal_scene_generation = 0;
  // Bounded request evidence serialized only by the existing Snapshot worker.
  std::optional<VisionRequestDiagnosticRecord> request_diagnostics;
  // Debug-only sidecar generated on the Snapshot worker and stored beside the
  // terminal Snapshot files. It is never part of the ROS Snapshot message.
  std::vector<CuboidStageImage> cuboid_stage_images;
  bool save = false;

  const DetectionInput * observation_input() const
  {
    return acquisition_result ? acquisition_result->input.get() : input.get();
  }

  const DetectionResult & detection_result() const
  {
    return acquisition_result ? acquisition_result->detection : result;
  }

  const SupportCalibrationSnapshot & calibration_data() const
  {
    return acquisition_result ? acquisition_result->calibration : calibration;
  }

  const DetectorProfile * selected_detector_profile() const
  {
    if (acquisition_result && acquisition_result->detection.applied_profile) {
      return acquisition_result->detection.applied_profile.get();
    }
    return selected_profile ? &*selected_profile : nullptr;
  }

  const std::string & processing_stage_data() const
  {
    return acquisition_result ? acquisition_result->detection.processing_stage : processing_stage;
  }
};

class VisionDetectorSnapshotOutput
{
public:
  using Save = std::function<bool(
      const VisionDetectorSnapshotJob &, const arm_cell_interfaces::msg::VisionDetectorSnapshot &)>;
  using Publish = std::function<void(const arm_cell_interfaces::msg::VisionDetectorSnapshot &)>;

  VisionDetectorSnapshotOutput(
    rclcpp::Node & node, std::filesystem::path artifact_root,
    std::string vision_run_id, Save save = {}, Publish publish = {},
    std::string vision_run_date = {},
    std::filesystem::path diagnostic_root = {}, std::string live_test_session_id = {},
    std::string vision_run_time = {},
    std::filesystem::path snapshot_run_directory = {});
  ~VisionDetectorSnapshotOutput();
  VisionDetectorSnapshotOutput(const VisionDetectorSnapshotOutput &) = delete;
  VisionDetectorSnapshotOutput & operator=(const VisionDetectorSnapshotOutput &) = delete;

  void submit(VisionDetectorSnapshotJob job) noexcept;
  void shutdown();
private:
  struct SnapshotStyle
  {
    std::string candidate_color_rgb = "#00C800";
    int candidate_line_width_px = 3;
    std::string unresolved_color_rgb = "#FFA500";
    std::string target_color_rgb = "#00FF00";
    std::string support_color_rgb = "#00B4FF";
    std::string disturbance_inclined_color_rgb = "#FF00FF";
    std::string disturbance_elevated_color_rgb = "#00FFFF";
    std::string disturbance_other_color_rgb = "#FFFFFF";
    int region_line_width_px = 2;
    std::string measurement_color_rgb = "#FF0000";
    int marker_size_px = 24;
    int measurement_line_width_px = 3;
    bool show_annotation_text = true;
    double annotation_text_scale = 1.0;
    std::string primary_text_color_rgb = "#FFDC00";
    std::string detail_text_color_rgb = "#FFFFFF";
  };
  void run();
  arm_cell_interfaces::msg::VisionDetectorSnapshot render(VisionDetectorSnapshotJob & job);
  cv::Mat render_annotation(
    const VisionDetectorSnapshotJob & job, bool & source_image_rendered,
    std::vector<std::optional<cv::Point>> & disturbance_projections) const;
  std::string serialize_metadata(
    const VisionDetectorSnapshotJob & job,
    const arm_cell_interfaces::msg::VisionDetectorSnapshot & message,
    bool source_image_rendered,
    const std::vector<std::optional<cv::Point>> & disturbance_projections) const;
  bool save_job(
    const VisionDetectorSnapshotJob & job,
    arm_cell_interfaces::msg::VisionDetectorSnapshot & message,
    std::string & failure_cause);
  bool save_diagnostic_record(
    const VisionDetectorSnapshotJob & job, std::string & failure_cause,
    std::string & serialized_record);
  void publish_job(const arm_cell_interfaces::msg::VisionDetectorSnapshot & message);

  rclcpp::Logger logger_;
  rclcpp::Publisher<arm_cell_interfaces::msg::VisionDetectorSnapshot>::SharedPtr publisher_;
  std::filesystem::path artifact_root_;
  std::filesystem::path diagnostic_root_;
  std::filesystem::path snapshot_run_directory_;
  std::string live_test_session_id_;
  std::string run_date_;
  std::string run_time_;
  std::optional<std::filesystem::path> run_directory_;
  std::string vision_run_id_;
  SnapshotStyle style_;
  Save save_;
  Publish publish_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<VisionDetectorSnapshotJob> calibration_pending_;
  std::optional<VisionDetectorSnapshotJob> progress_pending_;
  std::deque<VisionDetectorSnapshotJob> terminal_pending_;
  uint64_t revision_ = 0;
  bool stopping_ = false;
  std::thread worker_;
};

std::string make_vision_run_id();

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__VISION_DETECTOR_SNAPSHOT_HPP_
