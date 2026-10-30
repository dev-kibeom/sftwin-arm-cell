#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <arm_cell_interfaces/msg/scene_stability.hpp>
#include <arm_cell_interfaces/msg/vision_calibration.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arm_cell_vision/detect_target_service.hpp"
#include "arm_cell_vision/calibration_file.hpp"
#include "arm_cell_vision/cuboid_top_face.hpp"
#include "arm_cell_vision/vision_detector_snapshot.hpp"
#include "arm_cell_vision/depth_scene_stability.hpp"
#include "detect_target_node_factory.hpp"

namespace arm_cell_vision
{
namespace
{


int64_t source_stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + stamp.nanosec;
}

int64_t scene_monotonic_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

cv::Mat depth_image_to_meters(const sensor_msgs::msg::Image & depth)
{
  if ((depth.encoding != "32FC1" && depth.encoding != "16UC1" && depth.encoding != "mono16") ||
    depth.width == 0 || depth.height == 0 || depth.step == 0 ||
    depth.width > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    depth.height > static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
    depth.data.size() < static_cast<uint64_t>(depth.step) * depth.height)
  {
    return {};
  }
  const uint32_t bytes_per_pixel = depth.encoding == "32FC1" ? 4U : 2U;
  if (depth.step < static_cast<uint64_t>(depth.width) * bytes_per_pixel) {return {};}
  cv::Mat depth_m(static_cast<int>(depth.height), static_cast<int>(depth.width), CV_32F,
    cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  for (uint32_t y = 0; y < depth.height; ++y) {
    auto * row = depth_m.ptr<float>(static_cast<int>(y));
    for (uint32_t x = 0; x < depth.width; ++x) {
      const auto * pixel = depth.data.data() + static_cast<size_t>(y) * depth.step +
        static_cast<size_t>(x) * bytes_per_pixel;
      if (depth.encoding == "32FC1") {
        std::memcpy(&row[x], pixel, sizeof(float));
      } else {
        uint16_t millimeters = 0;
        std::memcpy(&millimeters, pixel, sizeof(millimeters));
        row[x] = millimeters == 0 ? std::numeric_limits<float>::quiet_NaN() :
          static_cast<float>(millimeters) * 0.001F;
      }
    }
  }
  return depth_m;
}

std::filesystem::path project_snapshot_artifact_root()
{
  const auto * configured_root = std::getenv("SFTWIN_PROJECT_ROOT");
  if (configured_root && *configured_root) {
    return std::filesystem::path(configured_root) / ".local_artifacts" /
           "snapshots";
  }
  std::error_code error;
  auto directory = std::filesystem::current_path(error);
  if (error) {return std::filesystem::path(".local_artifacts") / "snapshots";}
  while (true) {
    if (std::filesystem::exists(directory / ".git", error) && !error) {
      return directory / ".local_artifacts" / "snapshots";
    }
    const auto parent = directory.parent_path();
    if (parent == directory || parent.empty()) {break;}
    directory = parent;
    error.clear();
  }
  return std::filesystem::path(".local_artifacts") / "snapshots";
}

}  // namespace

class DetectTargetNodeLifecycleTestAccess;

class DetectTargetNode : public VisionRosIngress
{
  friend class DetectTargetNodeLifecycleTestAccess;

public:
  explicit DetectTargetNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : VisionRosIngress(options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    detector_profiles_(load_detector_profiles()),
    processor_(),
    service_(
      *this,
      []() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      },
      [this](const std::string & target_id, uint64_t generation,
        int64_t start, int64_t watermark, const std::string & algorithm) {
        return begin_material_acquisition(target_id, generation, start, watermark, algorithm);
      },
      [this]() {end_material_acquisition();},
      [this](const std::string & target_id, uint64_t generation) {
        return acquisition_result(target_id, generation);
      },
      [this]() {return scene_acquisition_diagnostic();},
      [this](const std::string & target_id, uint64_t generation,
        const DetectTargetService::Response & response,
        const SharedAcquisitionResult & result,
        const VisionRequestDiagnosticRecord & request_diagnostics) {
        on_acquisition_terminal(target_id, generation, response, result, request_diagnostics);
      }, vision_run_id_, [this](const std::string & source_frame) {
        return support_outward_normal_in_base(source_frame);
      })
  {
    const auto stable_observation_count = declare_parameter<int>(
      "scene_stability.stable_observation_count", 5);
    tray_depth_resolution_m_ = declare_parameter<double>(
      "tray_aligned_depth_resolution_m", 0.002);
    const auto maximum_frame_age_ms = declare_parameter<double>(
      "observation.maximum_frame_age_ms", 1000.0);
    if (stable_observation_count <= 0 ||
      !std::isfinite(maximum_frame_age_ms) || maximum_frame_age_ms <= 0.0 ||
      !std::isfinite(tray_depth_resolution_m_) || tray_depth_resolution_m_ <= 0.0)
    {
      throw std::invalid_argument("scene stability maximum frame age must be positive");
    }
    scene_stability_required_observations_ = static_cast<uint32_t>(stable_observation_count);
    DepthSceneStabilityConfig depth_scene_config;
    depth_scene_config.stable_observation_count = scene_stability_required_observations_;
    depth_difference_tolerance_m_ = declare_parameter<double>(
      "diagnostics.depth_scene_stability.depth_difference_tolerance_m", 0.003);
    changed_pixel_ratio_tolerance_ = declare_parameter<double>(
      "diagnostics.depth_scene_stability.changed_pixel_ratio_tolerance", 0.01);
    depth_scene_config.depth_difference_tolerance_m = depth_difference_tolerance_m_;
    depth_scene_config.changed_pixel_ratio_tolerance = changed_pixel_ratio_tolerance_;
    depth_scene_config.detailed_diagnostics_enabled = detailed_capture_enabled();
    depth_scene_stability_monitor_ =
      std::make_unique<DepthSceneStabilityMonitor>(depth_scene_config);
    stale_frame_limit_ns_ = static_cast<int64_t>(maximum_frame_age_ms * 1'000'000.0);
    scene_stability_publisher_ = create_publisher<arm_cell_interfaces::msg::SceneStability>(
      "/vision/scene_stability", rclcpp::QoS(1).reliable());
    calibration_publisher_ = create_publisher<arm_cell_interfaces::msg::VisionCalibration>(
      "/vision/calibration_status", rclcpp::QoS(1).reliable().transient_local());
    calibration_file_path_ = declare_parameter<std::string>("calibration_file", "");
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    if (calibration_file_path_.empty()) {
      last_calibration_detail_ = "vision.calibration_file is required";
    } else if (CalibrationFile::load(
        calibration_file_path_, camera_info_topic_, selected_calibration_file_,
        last_calibration_detail_))
    {
      selected_calibration_file_loaded_ = true;
      calibration_validation_pending_ = true;
      last_calibration_detail_ = "selected Calibration file loaded; waiting for compatible CameraInfo";
    }
    tray_depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(
      "/vision/tray_aligned_depth/image_raw", rclcpp::QoS(1).best_effort());
    try {
      snapshot_output_ = std::make_unique<VisionDetectorSnapshotOutput>(
        *this, project_snapshot_artifact_root(), vision_run_id_,
        VisionDetectorSnapshotOutput::Save{}, VisionDetectorSnapshotOutput::Publish{},
        std::string{},
        declare_parameter<std::string>("diagnostics.output_directory", ""),
        declare_parameter<std::string>("diagnostics.live_test_session_id", ""),
        std::string{},
        declare_parameter<std::string>("diagnostics.snapshot_run_directory", ""));
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "detector snapshot output unavailable cause=%s",
                  error.what());
    }
    set_observation_observer(
      [this](const VisionObservation & observation) {
        on_depth_scene_observation(observation);
        on_depth_detector_observation(observation);
      });
    set_synchronized_observer(
      [this](const VisionObservation & observation) {
        on_synchronized_calibration_observation(observation);
      });
    set_observation_invalidator([this]() {on_depth_observation_invalidated();});
    publish_calibration_state(
      std_msgs::msg::Header(), processor_.support_calibration_snapshot(),
      last_calibration_detail_);
    calibration_status_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {
        std_msgs::msg::Header header;
        std::string detail;
        {
          std::lock_guard<std::mutex> lock(calibration_state_mutex_);
          header = last_calibration_header_;
          detail = last_calibration_detail_;
        }
        SupportCalibrationSnapshot snapshot;
        {
          std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
          snapshot = latest_calibration_snapshot_;
        }
        bool calibration_active = false;
        {
          std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
          calibration_active = calibration_request_active_;
        }
        publish_calibration_state(header, snapshot, detail, true, calibration_active);
      });
    calibration_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "/vision/calibrate",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_calibration_request(*response);
      }, rmw_qos_profile_services_default, calibration_callback_group_);
    service_.advertise(*this);
  }

  ~DetectTargetNode()
  {
    if (snapshot_output_) {snapshot_output_->shutdown();}
  }

private:
  void update_calibration_compatibility(const VisionObservation & observation)
  {
    if (!receipt_is_fresh(observation)) {
      return;
    }
    std::string diagnostic;
    const bool compatible = selected_calibration_file_loaded_ && CalibrationFile::compatible(
      selected_calibration_file_, observation.camera_info, camera_info_topic_,
      observation.depth.header.frame_id, diagnostic);
    bool compatibility_changed = false;
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      compatibility_changed = !calibration_compatibility_initialized_ ||
        compatible != calibration_camera_compatible_;
      calibration_compatibility_initialized_ = true;
      calibration_camera_compatible_ = compatible;
      if (compatibility_changed && compatible) {
        SupportCalibrationSnapshot snapshot;
        snapshot.ready = true;
        snapshot.tray_from_camera = selected_calibration_file_.tray_from_camera;
        processor_.set_loaded_support_calibration(snapshot);
        std::lock_guard<std::mutex> snapshot_lock(snapshot_state_mutex_);
        latest_calibration_snapshot_ = snapshot;
      } else if (compatibility_changed) {
        processor_.clear_loaded_support_calibration();
        std::lock_guard<std::mutex> snapshot_lock(snapshot_state_mutex_);
        latest_calibration_snapshot_ = SupportCalibrationSnapshot{};
      }
    }
    if (compatibility_changed) {
      if (compatible) {
        set_calibration_detail("selected Calibration file is READY");
      } else if (selected_calibration_file_loaded_) {
        set_calibration_detail(diagnostic);
      }
      {
        std::lock_guard<std::mutex> lock(calibration_state_mutex_);
        calibration_validation_pending_ = false;
      }
      publish_calibration_state(
        observation.depth.header, processor_.support_calibration_snapshot(),
        calibration_detail());
    }
  }

  bool receipt_is_fresh(const VisionObservation & observation) const
  {
    const auto now = scene_monotonic_now_ns();
    const auto receipt = observation.depth_receipt_monotonic_ns;
    return receipt >= 0 && now >= receipt && now - receipt <= stale_frame_limit_ns_;
  }

  void on_depth_scene_observation(const VisionObservation & observation)
  {
    const auto processing_start_ns = scene_monotonic_now_ns();
    const int64_t stamp = source_stamp_ns(observation.depth.header.stamp);
    const int64_t now = scene_monotonic_now_ns();
    std::string acquisition_id;
    int64_t acquisition_start_ns = 0;
    int64_t acquisition_watermark_ns = 0;
    bool active = false;
    bool request_fresh = false;
    {
      std::lock_guard<std::mutex> acquisition_lock(acquisition_callback_mutex_);
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      active = acquisition_active_ && !detector_invoked_;
      acquisition_id = acquisition_id_;
      acquisition_start_ns = acquisition_start_monotonic_ns_;
      acquisition_watermark_ns = acquisition_watermark_stamp_ns_;
      request_fresh = active &&
        observation.depth_receipt_monotonic_ns > acquisition_start_ns &&
        stamp > acquisition_watermark_ns;
    }
    if (!active) {return;}
    const auto finish_unobservable = [&](const std::string & reason) {
        DepthSceneFrame frame;
        frame.stamp_ns = stamp;
        frame.receipt_monotonic_ns = observation.depth_receipt_monotonic_ns;
        frame.frame_id = observation.depth.header.frame_id;
        finish_depth_scene_observation(
          acquisition_id, frame, reason, acquisition_start_ns,
          processing_start_ns);
      };
    if (!request_fresh) {
      return;
    }
    const auto clock = acceptance_snapshot();
    const auto pair_freshness = evaluate_depth_source_stamp_freshness(
      observation, stale_frame_limit_ns_);
    if (observation.depth_receipt_monotonic_ns < 0 || now < observation.depth_receipt_monotonic_ns ||
      now - observation.depth_receipt_monotonic_ns > stale_frame_limit_ns_ ||
      !pair_freshness.accepted ||
      (clock.latest_clock_stamp_ns && std::llabs(stamp - *clock.latest_clock_stamp_ns) >
      stale_frame_limit_ns_) || clock.epoch_invalidated)
    {
      finish_unobservable(clock.epoch_invalidated ?
        "sensor_clock_epoch_invalidated" : "depth_source_stamp_or_receipt_stale");
      return;
    }
    std::string calibration_diagnostic;
    const bool compatible = selected_calibration_file_loaded_ && CalibrationFile::compatible(
      selected_calibration_file_, observation.camera_info, camera_info_topic_,
      observation.depth.header.frame_id, calibration_diagnostic);
    if (!compatible) {
      finish_unobservable("camera_info_or_calibration_incompatible");
      return;
    }
    DepthSceneFrame frame;
    frame.stamp_ns = stamp;
    frame.receipt_monotonic_ns = observation.depth_receipt_monotonic_ns;
    frame.depth_m = depth_image_to_meters(observation.depth);
    frame.frame_id = observation.depth.header.frame_id;
    const std::string invalid_reason = frame.depth_m.empty() ?
      "invalid_depth_encoding_or_layout" : std::string();
    finish_depth_scene_observation(
      acquisition_id, frame, invalid_reason, acquisition_start_ns, processing_start_ns);
  }

  void finish_depth_scene_observation(
    const std::string & acquisition_id, const DepthSceneFrame & frame,
    const std::string & invalid_reason,
    int64_t acquisition_start_ns, int64_t processing_start_ns)
  {
    std::unique_lock<std::mutex> acquisition_lock(acquisition_callback_mutex_);
    std::unique_lock<std::mutex> lock(acquisition_state_mutex_);
    if (!acquisition_active_ || acquisition_id_ != acquisition_id ||
      acquisition_terminal_snapshot_taken_)
    {
      return;
    }
    DepthSceneStabilityState state;
    if (invalid_reason.empty()) {
      state = depth_scene_stability_monitor_->observe(frame);
    } else {
      depth_scene_stability_monitor_->reset();
      state.required_observations = scene_stability_required_observations_;
      state.depth_difference_tolerance_m = depth_difference_tolerance_m_;
      state.changed_pixel_ratio_tolerance = changed_pixel_ratio_tolerance_;
      state.reason = invalid_reason;
      state.last_reset_reason = invalid_reason;
    }
    depth_scene_stability_state_ = state;
    depth_scene_acquisition_id_ = acquisition_id;
    depth_scene_stamp_ns_ = frame.stamp_ns;
    if (detailed_capture_enabled()) {
      VisionDetectorSnapshotJob::DepthSceneSample sample;
      sample.state = state;
      sample.observed_depth_m = frame.depth_m.clone();
      sample.stamp_ns = frame.stamp_ns;
      sample.receipt_monotonic_ns = frame.receipt_monotonic_ns;
      sample.acquisition_elapsed_ms = static_cast<double>(
        frame.receipt_monotonic_ns - acquisition_start_ns) / 1.0e6;
      sample.processing_elapsed_ms = static_cast<double>(
        scene_monotonic_now_ns() - processing_start_ns) / 1.0e6;
      depth_scene_samples_.push_back(std::move(sample));
      while (depth_scene_samples_.size() > 5) {depth_scene_samples_.pop_front();}
    }
    const auto generation = acquisition_generation_;
    lock.unlock();
    acquisition_lock.unlock();
    std_msgs::msg::Header source_header;
    source_header.stamp.sec = static_cast<int32_t>(frame.stamp_ns / 1'000'000'000LL);
    source_header.stamp.nanosec = static_cast<uint32_t>(frame.stamp_ns % 1'000'000'000LL);
    source_header.frame_id = frame.frame_id;
    publish_scene_stability_state(
      source_header, state.status, generation);
  }

  void handle_calibration_request(std_srvs::srv::Trigger::Response & response)
  {
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      if (acquisition_active_ || calibration_request_active_) {
        response.success = false;
        response.message = acquisition_active_ ?
          "DetectTarget Acquisition is active" : "another Calibration request is active";
        return;
      }
      calibration_request_active_ = true;
    }
    std_msgs::msg::Header calibration_header;
    auto finish = [this, &calibration_header]() {
        {
          std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
          calibration_request_active_ = false;
        }
        publish_calibration_state(
          calibration_header, processor_.support_calibration_snapshot(),
          calibration_detail(), true);
      };
    publish_calibration_state(
      calibration_header, processor_.support_calibration_snapshot(),
      "explicit operator Calibration request is running", true, true);

    if (detector_profiles_.empty() || calibration_file_path_.empty()) {
      response.success = false;
      response.message = calibration_file_path_.empty() ?
        "vision.calibration_file is required" : "no configured detector profile is available";
      finish();
      return;
    }
    std::shared_ptr<const DetectionInput> input;
    int64_t input_receipt = 0;
    {
      std::lock_guard<std::mutex> lock(calibration_input_mutex_);
      input = latest_calibration_input_;
      input_receipt = latest_calibration_input_receipt_ns_;
    }
    const auto now = scene_monotonic_now_ns();
    if (!input || now < input_receipt || now - input_receipt > stale_frame_limit_ns_) {
      response.success = false;
      response.message = "no fresh synchronized RGB-D observation is available for Calibration";
      finish();
      return;
    }
    calibration_header = input->depth.header;
    publish_calibration_state(
      input->depth.header, processor_.support_calibration_snapshot(),
      "explicit operator Calibration request is running", true, true);

    try {
      auto calibration_input = *input;
      DetectTargetProcessor calibration_processor;
      const auto result = calibration_processor.process(
        calibration_input, detector_profiles_.front().target_id, detector_profiles_, true);
      auto calibration = calibration_processor.support_calibration_snapshot();
      if (!calibration.ready) {
        response.success = false;
        response.message = result.diagnostic_detail.empty() ?
          "Calibration observation did not establish a valid support reference" :
          result.diagnostic_detail;
        set_calibration_detail(response.message);
        finish();
        return;
      }
      CalibrationFileData output;
      output.tray_from_camera = calibration.tray_from_camera;
      output.camera_info = calibration_input.camera_info;
      output.camera_info_topic = camera_info_topic_;
      std::string diagnostic;
      std::string created_path;
      if (!CalibrationFile::save_new_sibling(
          calibration_file_path_, output, created_path, diagnostic))
      {
        response.success = false;
        response.message = diagnostic;
        set_calibration_detail(diagnostic);
        finish();
        return;
      }
      response.success = true;
      response.message = "Calibration file created: " + created_path;
      set_calibration_detail("selected Calibration file unchanged; generated file awaits operator selection");
      if (snapshot_output_) {
        VisionDetectorSnapshotJob job;
        job.event_kind = arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_COMPLETE;
        job.calibration_id = "cal-" + make_vision_run_id();
        job.input = input;
        job.has_observation = true;
        job.calibration = calibration;
        job.tray_roi_min_m = detector_profiles_.front().tray_roi_min_m;
        job.tray_roi_max_m = detector_profiles_.front().tray_roi_max_m;
        job.tray_depth_resolution_m = tray_depth_resolution_m_;
        job.save = true;
        snapshot_output_->submit(std::move(job));
      }
    } catch (const std::exception & error) {
      response.success = false;
      response.message = std::string("Calibration failed: ") + error.what();
      set_calibration_detail(response.message);
    } catch (...) {
      response.success = false;
      response.message = "Calibration failed: unknown error";
      set_calibration_detail(response.message);
    }
    finish();
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookup_observation_transform(
    const std::string & source_frame)
  {
    try {
      return tf_buffer_.lookupTransform("base_link", source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  std::optional<geometry_msgs::msg::Vector3> support_outward_normal_in_base(
    const std::string & camera_frame)
  {
    const auto calibration = processor_.support_calibration_snapshot();
    if (!calibration.ready || camera_frame.empty() ||
      calibration.tray_from_camera.header.frame_id != "Tray" ||
      calibration.tray_from_camera.child_frame_id != camera_frame)
    {
      return std::nullopt;
    }
    const auto base_from_camera = lookup_observation_transform(camera_frame);
    if (!base_from_camera) {return std::nullopt;}
    const auto & base_rotation = base_from_camera->transform.rotation;
    const auto & tray_from_camera = calibration.tray_from_camera.transform.rotation;
    tf2::Quaternion base_from_camera_rotation(
      base_rotation.x, base_rotation.y, base_rotation.z, base_rotation.w);
    tf2::Quaternion tray_from_camera_rotation(
      tray_from_camera.x, tray_from_camera.y, tray_from_camera.z, tray_from_camera.w);
    if (base_from_camera_rotation.length2() < 1.0e-12 ||
      tray_from_camera_rotation.length2() < 1.0e-12)
    {
      return std::nullopt;
    }
    base_from_camera_rotation.normalize();
    tray_from_camera_rotation.normalize();
    const auto camera_from_tray_rotation = tray_from_camera_rotation.inverse();
    auto normal = (base_from_camera_rotation * camera_from_tray_rotation) *
      tf2::Vector3(0.0, 0.0, 1.0);
    const auto length = normal.length();
    if (!std::isfinite(length) || length < 1.0e-6) {return std::nullopt;}
    normal /= length;
    geometry_msgs::msg::Vector3 result;
    result.x = normal.x();
    result.y = normal.y();
    result.z = normal.z();
    return result;
  }

  uint8_t begin_material_acquisition(
    const std::string & target_id, uint64_t generation, int64_t start, int64_t watermark,
    const std::string & algorithm)
  {
    std::lock_guard<std::mutex> acquisition_lock(acquisition_callback_mutex_);
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      if (acquisition_active_ || calibration_request_active_ || detector_profiles_.empty() ||
        detector_profiles_.front().target_id != target_id)
      {
        if (acquisition_active_ || calibration_request_active_) {
          return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_BUSY;
        }
        return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT;
      }
      if (!processor_.support_calibration_snapshot().ready) {
        return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
      }
      acquisition_active_ = true;
      acquisition_terminal_snapshot_taken_ = false;
      acquisition_target_id_ = target_id;
      acquisition_algorithm_ = algorithm;
      acquisition_start_monotonic_ns_ = start;
      acquisition_watermark_stamp_ns_ = watermark;
      depth_scene_stability_monitor_->reset();
      depth_scene_stability_state_ = DepthSceneStabilityState{};
      depth_scene_stability_state_.required_observations = scene_stability_required_observations_;
      depth_scene_stability_state_.depth_difference_tolerance_m = depth_difference_tolerance_m_;
      depth_scene_stability_state_.changed_pixel_ratio_tolerance = changed_pixel_ratio_tolerance_;
      depth_scene_acquisition_id_.clear();
      depth_scene_stamp_ns_ = 0;
      detector_invoked_ = false;
      detector_not_invoked_reason_.clear();
      detector_elapsed_ms_.reset();
      detector_result_code_.reset();
      depth_scene_samples_.clear();
      acquisition_generation_ = generation;
      acquisition_result_.reset();
      acquisition_id_ = make_acquisition_id(vision_run_id_, generation);
    }
    publish_scene_stability_state(
      std_msgs::msg::Header(), DepthSceneStatus::kUnobservable, generation);
    return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  }

  void end_material_acquisition()
  {
    std::lock_guard<std::mutex> acquisition_lock(acquisition_callback_mutex_);
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      acquisition_active_ = false;
      acquisition_terminal_snapshot_taken_ = false;
      acquisition_target_id_.clear();
      acquisition_start_monotonic_ns_ = 0;
      acquisition_watermark_stamp_ns_ = 0;
      acquisition_result_.reset();
      depth_scene_stability_monitor_->reset();
      depth_scene_stability_state_ = DepthSceneStabilityState{};
      depth_scene_acquisition_id_.clear();
      depth_scene_stamp_ns_ = 0;
      detector_invoked_ = false;
      detector_not_invoked_reason_.clear();
      detector_elapsed_ms_.reset();
      detector_result_code_.reset();
      depth_scene_samples_.clear();
      acquisition_generation_ = 0;
    }
    publish_scene_stability_state(
      std_msgs::msg::Header(), DepthSceneStatus::kUnobservable, 0);
  }

  SharedAcquisitionResult acquisition_result(
    const std::string & target_id, uint64_t acquisition_generation)
  {
    std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
    if (!acquisition_active_ || !acquisition_result_ ||
      acquisition_result_->target_id != target_id ||
      acquisition_result_->generation != acquisition_generation)
    {
      return {};
    }
    return acquisition_result_;
  }

  AcquisitionDiagnosticSummary scene_acquisition_diagnostic()
  {
    std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
    AcquisitionDiagnosticSummary summary;
    std::ostringstream detail;
    detail << "Depth Scene Stability " << depth_scene_status_name(
      depth_scene_stability_state_.status)
           << " reason=" << depth_scene_stability_state_.reason
           << " detector_invoked=" << (detector_invoked_ ? "true" : "false");
    if (depth_scene_stability_state_.status == DepthSceneStatus::kStable && !detector_invoked_) {
      detail << " detector_not_invoked_reason=" <<
        (detector_not_invoked_reason_.empty() ? "input_not_eligible" : detector_not_invoked_reason_);
    }
    detail << "; target_id=" << acquisition_target_id_
           << " qualifying_observations=" << depth_scene_stability_state_.qualifying_observations
           << '/' << depth_scene_stability_state_.required_observations;
    detail << " scene_stable=" <<
      (depth_scene_stability_state_.status == DepthSceneStatus::kStable ? "true" : "false");
    if (detector_elapsed_ms_) {
      detail << " detector_result_code=" << static_cast<unsigned int>(detector_result_code_.value_or(0))
             << " detector_elapsed_ms=" << std::fixed << std::setprecision(3) << *detector_elapsed_ms_;
    }
    if (acquisition_result_ && !acquisition_result_->detection.diagnostic_detail.empty()) {
      detail << " last_profile_processing={" <<
        acquisition_result_->detection.diagnostic_detail << '}';
    }
    summary.detail = detail.str();
    if (depth_scene_stability_state_.status != DepthSceneStatus::kStable) {
      summary.failure_cause = depth_scene_stability_state_.reason;
    } else if (!detector_invoked_) {
      summary.failure_cause = detector_not_invoked_reason_.empty() ?
        "detector input was not eligible" : detector_not_invoked_reason_;
    } else if (acquisition_result_ && !acquisition_result_->detection.failure_cause.empty()) {
      summary.failure_cause = acquisition_result_->detection.failure_cause;
    } else {
      summary.failure_cause = "detector result was not available before the Acquisition deadline";
    }
    return summary;
  }

  void on_synchronized_calibration_observation(const VisionObservation & observation)
  {
    const auto now = scene_monotonic_now_ns();
    const auto oldest_receipt = std::min(
      observation.rgb_receipt_monotonic_ns, observation.depth_receipt_monotonic_ns);
    const bool receipt_fresh = oldest_receipt >= 0 && now >= oldest_receipt &&
      now - oldest_receipt <= stale_frame_limit_ns_;
    const auto source_stamp_freshness = evaluate_pair_source_stamp_freshness(
      observation, stale_frame_limit_ns_);
    if (receipt_fresh && source_stamp_freshness.accepted) {
      update_calibration_compatibility(observation);
      auto calibration_input = std::make_shared<DetectionInput>();
      calibration_input->rgb = observation.rgb;
      calibration_input->rgb_presentation_status = "available";
      calibration_input->depth = observation.depth;
      calibration_input->camera_info = observation.camera_info;
      std::lock_guard<std::mutex> lock(calibration_input_mutex_);
      latest_calibration_input_ = std::move(calibration_input);
      latest_calibration_input_receipt_ns_ = std::max(
        observation.rgb_receipt_monotonic_ns, observation.depth_receipt_monotonic_ns);
    }
  }

  void on_depth_detector_observation(const VisionObservation & observation)
  {
    // Keep in-flight evaluation ordered with STABLE finalization and lifecycle changes.
    std::lock_guard<std::mutex> acquisition_lock(acquisition_callback_mutex_);
    const auto now = scene_monotonic_now_ns();
    const auto receipt = observation.depth_receipt_monotonic_ns;
    const auto source_stamp = source_stamp_ns(observation.depth.header.stamp);
    const auto depth_source_stamp = source_stamp;
    const bool receipt_fresh = receipt >= 0 && now >= receipt &&
      now - receipt <= stale_frame_limit_ns_;
    const auto source_stamp_freshness = evaluate_depth_source_stamp_freshness(
      observation, stale_frame_limit_ns_);
    const bool sensor_stamp_fresh = source_stamp_freshness.accepted;
    bool evaluate_scene = false;
    bool request_fresh = false;
    bool acquisition_active = false;
    bool detector_invoked = false;
    DepthSceneStatus scene_status = DepthSceneStatus::kUnobservable;
    std::string acquisition_target_id;
    std::string acquisition_algorithm;
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      acquisition_active = acquisition_active_;
      acquisition_target_id = acquisition_target_id_;
      acquisition_algorithm = acquisition_algorithm_;
      detector_invoked = detector_invoked_;
      scene_status = depth_scene_stability_state_.status;
      evaluate_scene = acquisition_active &&
        depth_scene_stability_state_.status == DepthSceneStatus::kStable &&
        depth_scene_acquisition_id_ == acquisition_id_ && depth_scene_stamp_ns_ ==
        depth_source_stamp && !detector_invoked_;
      request_fresh = acquisition_active &&
        observation.depth_receipt_monotonic_ns > acquisition_start_monotonic_ns_ &&
        source_stamp > acquisition_watermark_stamp_ns_;
    }
    const auto calibration = processor_.support_calibration_snapshot();
    // Tray Depth is a UI observation stream. Keep its original validity,
    // subscriber, and rate limits independent from the DetectTarget gate.
    if (receipt_fresh && sensor_stamp_fresh && calibration.ready &&
      tray_depth_publisher_->get_subscription_count() > 0)
    {
      const auto publish_now = scene_monotonic_now_ns();
      if (publish_now - last_tray_depth_publish_monotonic_ns_ >= 200'000'000LL) {
        publish_tray_aligned_depth(observation.depth, observation.camera_info, calibration);
        last_tray_depth_publish_monotonic_ns_ = publish_now;
      }
    }
    std::optional<DetectionResult> completed_detection;
    const bool should_process = calibration.ready && request_fresh && evaluate_scene &&
      receipt_fresh && sensor_stamp_fresh;
    if (!should_process && acquisition_active && !detector_invoked &&
      scene_status == DepthSceneStatus::kStable)
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      if (!calibration.ready) {
        detector_not_invoked_reason_ = "calibration_unavailable";
      } else if (!request_fresh) {
        detector_not_invoked_reason_ = "observation_not_after_request_start_or_watermark";
      } else if (!receipt_fresh) {
        detector_not_invoked_reason_ = "receipt_stale";
      } else if (!sensor_stamp_fresh) {
        detector_not_invoked_reason_ = "sensor_stamp_" + source_stamp_freshness.reason;
      } else {
        detector_not_invoked_reason_ = "stable_depth_observation_not_current";
      }
    }
    std::shared_ptr<const DetectionInput> processed_input;
    if (should_process) {
      const auto transform = lookup_observation_transform(
        observation.depth.header.frame_id);
      if (!transform) {
        // TF may arrive before the single Acquisition deadline. This sample
        // cannot enter the Detector until its required transform is available.
        std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
        if (acquisition_active_ && !detector_invoked_ &&
          depth_scene_stability_state_.status == DepthSceneStatus::kStable &&
          depth_scene_stamp_ns_ == depth_source_stamp)
        {
          detector_not_invoked_reason_ = "tf_unavailable";
        }
        return;
      }
      auto detector_input = std::make_shared<DetectionInput>(DetectionInput{
        observation.rgb, observation.depth, observation.camera_info, *transform,
        detailed_capture_enabled()});
      detector_input->rgb_presentation_status = observation.rgb_presentation_status;
      detector_input->depth_receipt_monotonic_ns = observation.depth_receipt_monotonic_ns;
      processed_input = std::move(detector_input);
      const auto target_id = acquisition_active ? acquisition_target_id :
        (detector_profiles_.empty() ? std::string() : detector_profiles_.front().target_id);
      // Bind dispatch to this Acquisition; operating settings remain shared and immutable.
      auto request_profiles = detector_profiles_;
      for (auto & profile : request_profiles) {
        profile.algorithm = acquisition_algorithm;
      }
      {
        std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
        if (!acquisition_active_ || detector_invoked_ ||
          depth_scene_acquisition_id_ != acquisition_id_ ||
          depth_scene_stamp_ns_ != depth_source_stamp)
        {
          return;
        }
        detector_invoked_ = true;
      }
      const auto detector_started = std::chrono::steady_clock::now();
      auto detection = processor_.process(*processed_input, target_id, request_profiles);
      const auto detector_elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - detector_started).count();
      {
        std::ostringstream detector_detail;
        detector_detail << "; detector_result_code=" <<
          static_cast<unsigned int>(detection.result_code) << "; detector_elapsed_ms=" <<
          std::fixed << std::setprecision(3) << detector_elapsed_ms;
        detection.diagnostic_detail += detector_detail.str();
      }
      {
        std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
        detector_elapsed_ms_ = detector_elapsed_ms;
        detector_result_code_ = detection.result_code;
        detector_not_invoked_reason_.clear();
      }
      if (acquisition_active && request_fresh && evaluate_scene) {
        completed_detection = std::move(detection);
      }
    } else {
      return;
    }
    if (!acquisition_active || !request_fresh || !evaluate_scene || !completed_detection) {
      return;
    }
    auto snapshot_input = std::make_shared<DetectionInput>(*processed_input);
    const auto rgb = latest_snapshot_rgb(snapshot_input->depth, snapshot_input->camera_info);
    if (rgb.image) {snapshot_input->rgb = std::move(*rgb.image);}
    snapshot_input->rgb_presentation_status = rgb.status;

    auto result = std::make_shared<AcquisitionResult>();
    result->target_id = acquisition_target_id;
    result->input = std::move(snapshot_input);
    result->detection = std::move(*completed_detection);
    result->calibration = calibration;
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      if (!acquisition_active_ || !acquisition_active ||
        depth_scene_acquisition_id_ != acquisition_id_ ||
        depth_scene_stamp_ns_ != depth_source_stamp)
      {
        return;
      }
      result->generation = acquisition_generation_;
      result->acquisition_id = acquisition_id_;
      result->detection.scene_generation = acquisition_generation_;
      // Publish only after detector output, source input and Acquisition identity are complete.
      acquisition_result_ = std::move(result);
    }
    {
      std::lock_guard<std::mutex> lock(snapshot_state_mutex_);
      latest_calibration_snapshot_ = calibration;
    }
  }

  void on_acquisition_terminal(
    const std::string & target_id, uint64_t acquisition_generation,
    const DetectTargetService::Response & response,
    const SharedAcquisitionResult & result,
    const VisionRequestDiagnosticRecord & request_diagnostics) noexcept
  {
    try {
      if (!snapshot_output_) {return;}
      VisionDetectorSnapshotJob job;
      job.event_kind = response.result_code.value ==
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS ?
        arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS :
        arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
      job.acquisition_id = response.acquisition_id;
      job.target_id = target_id;
      job.acquisition_result = result;
      job.save = true;
      job.terminal_result_code = response.result_code.value;
      job.terminal_diagnostic = response.diagnostic_detail;
      job.request_diagnostics = request_diagnostics;
      job.depth_scene_diagnostics_enabled = request_diagnostics.ingress.detailed_capture_enabled;
      {
        std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
        acquisition_terminal_snapshot_taken_ = true;
        job.algorithm = acquisition_algorithm_;
        job.depth_scene_stability = depth_scene_stability_state_;
        job.stable_depth_stamp_ns = depth_scene_stamp_ns_;
        job.depth_scene_samples.assign(depth_scene_samples_.begin(), depth_scene_samples_.end());
      }
      const bool result_matches_acquisition = result &&
        result->generation == acquisition_generation && result->target_id == target_id &&
        result->acquisition_id == response.acquisition_id &&
        result->detection.scene_generation == acquisition_generation &&
        (response.result_code.value !=
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS ||
        response.scene_generation == acquisition_generation);
      if (result_matches_acquisition) {
        job.last_observation_stage = result->detection.processing_stage;
        job.processing_stage = result->detection.processing_stage;
        job.has_observation = static_cast<bool>(result->input);
        job.has_detection_result = true;
        if (result->detection.applied_profile) {
          job.algorithm = result->detection.applied_profile->algorithm;
        }
      } else {
        job.acquisition_result.reset();
        job.processing_stage = response.result_code.value ==
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT ?
          "request_deadline" : "acquisition_result_unavailable";
      }
      if (response.result_code.value ==
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS)
      {
        job.terminal_selection_available = true;
        job.terminal_scene_generation = response.scene_generation;
      }
      if (const auto * profile = job.selected_detector_profile()) {
        job.tray_roi_min_m = profile->tray_roi_min_m;
        job.tray_roi_max_m = profile->tray_roi_max_m;
      }
      job.tray_depth_resolution_m = tray_depth_resolution_m_;
      if (!job.has_observation && response.result_code.value ==
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT)
      {
        job.processing_stage = job.depth_scene_stability.status == DepthSceneStatus::kStable ?
          "detector_preflight" : job.stable_depth_stamp_ns > 0 ?
          "depth_scene_stability" : "observation_ingress";
      }
      else if (response.result_code.value ==
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT &&
        job.depth_scene_stability.status != DepthSceneStatus::kStable)
      {job.processing_stage = "depth_scene_stability";}
      snapshot_output_->submit(std::move(job));
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(),
                  "detector snapshot terminal enqueue failed cause=%s",
                  error.what());
    } catch (...) {
      RCLCPP_WARN(
          get_logger(),
          "detector snapshot terminal enqueue failed cause=unknown_exception");
    }
  }

  void on_depth_observation_invalidated()
  {
    std::lock_guard<std::mutex> acquisition_lock(acquisition_callback_mutex_);
    uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      if (!acquisition_active_ || detector_invoked_) {return;}
      depth_scene_stability_monitor_->reset();
      depth_scene_stability_state_ = DepthSceneStabilityState{};
      depth_scene_stability_state_.required_observations = scene_stability_required_observations_;
      depth_scene_stability_state_.depth_difference_tolerance_m = depth_difference_tolerance_m_;
      depth_scene_stability_state_.changed_pixel_ratio_tolerance = changed_pixel_ratio_tolerance_;
      depth_scene_stability_state_.reason = "observation_invalidated";
      depth_scene_acquisition_id_.clear();
      depth_scene_stamp_ns_ = 0;
      generation = acquisition_generation_;
    }
    publish_scene_stability_state(
      std_msgs::msg::Header(), DepthSceneStatus::kUnobservable, generation);
  }

  void publish_scene_stability_state(
    const std_msgs::msg::Header & header, DepthSceneStatus status, uint64_t generation)
  {
    std_msgs::msg::Header published_header = header;
    {
      std::lock_guard<std::mutex> lock(acquisition_state_mutex_);
      if (!header.frame_id.empty()) {
        last_scene_stability_header_ = header;
      } else {
        published_header = last_scene_stability_header_;
      }
    }
    arm_cell_interfaces::msg::SceneStability message;
    message.header = published_header;
    message.target_id = detector_profiles_.front().target_id;
    message.scene_generation = generation;
    message.status = static_cast<uint8_t>(status);
    scene_stability_publisher_->publish(message);
  }

  void set_calibration_detail(const std::string & detail)
  {
    std::lock_guard<std::mutex> lock(calibration_state_mutex_);
    last_calibration_detail_ = detail;
  }

  std::string calibration_detail() const
  {
    std::lock_guard<std::mutex> lock(calibration_state_mutex_);
    return last_calibration_detail_;
  }

  void publish_calibration_state(
    const std_msgs::msg::Header & header, const SupportCalibrationSnapshot & snapshot,
    const std::string & detail, bool force = false, bool calibrating = false)
  {
    std::lock_guard<std::mutex> lock(calibration_state_mutex_);
    // A loaded file is not known to be usable or incompatible until it has
    // been checked against a fresh CameraInfo observation. Withhold the
    // transient-local status so consumers treat startup as pending instead
    // of presenting a false NEEDS_CALIBRATION result.
    if (calibration_validation_pending_ && !calibrating) {
      return;
    }
    arm_cell_interfaces::msg::VisionCalibration message;
    message.header = header;
    last_calibration_header_ = header;
    message.state = calibrating ?
      arm_cell_interfaces::msg::VisionCalibration::STATE_CALIBRATING :
      snapshot.ready ? arm_cell_interfaces::msg::VisionCalibration::STATE_READY :
      arm_cell_interfaces::msg::VisionCalibration::STATE_NEEDS_CALIBRATION;
    message.accepted_frames = calibrating ? snapshot.accepted_frames : 0;
    message.required_frames = calibrating ? 1 : 0;
    message.median_depth_residual_m = calibrating ? snapshot.median_depth_residual_m : 0.0;
    if (snapshot.ready) {message.tray_from_camera = snapshot.tray_from_camera;}
    std::ostringstream diagnostic;
    diagnostic << detail;
    for (size_t index = 0; index < snapshot.recent_depth_validations.size(); ++index) {
      const auto & evidence = snapshot.recent_depth_validations[index];
      diagnostic << (index == 0 ? "; depth_validation_samples=[" : ",")
                 << "{valid_depth_sample_count=" << evidence.valid_sample_count
                 << ",median_signed_residual_m=";
      if (evidence.residual_available) {
        diagnostic << std::setprecision(12) << evidence.median_signed_residual_m
                   << ",minimum_signed_residual_m=" << evidence.minimum_signed_residual_m
                   << ",maximum_signed_residual_m=" << evidence.maximum_signed_residual_m;
      } else {
        diagnostic << "unavailable,minimum_signed_residual_m=unavailable,maximum_signed_residual_m=unavailable";
      }
      diagnostic << ",support_contact_tolerance_m=" << evidence.support_contact_tolerance_m
                 << ",configured_marker_height_m=" << evidence.configured_marker_height_m
                 << ",decision=" << (evidence.accepted ? "accepted" : "rejected")
                 << ",reason=" << evidence.reason << '}';
    }
    if (!snapshot.recent_depth_validations.empty()) {diagnostic << ']';}
    message.diagnostic_detail = diagnostic.str();
    if (!force && calibration_status_published_ &&
      last_calibration_state_ == message.state &&
      last_calibration_published_detail_ == message.diagnostic_detail)
    {
      return;
    }
    calibration_publisher_->publish(message);
    calibration_status_published_ = true;
    last_calibration_state_ = message.state;
    last_calibration_detail_ = detail;
    last_calibration_published_detail_ = message.diagnostic_detail;
  }

  void publish_tray_aligned_depth(
    const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & camera,
    const SupportCalibrationSnapshot & calibration)
  {
    if (depth.encoding != "32FC1" && depth.encoding != "16UC1") {return;}
    const auto & profile = detector_profiles_.front();
    const double span_x = profile.tray_roi_max_m[0] - profile.tray_roi_min_m[0];
    const double span_y = profile.tray_roi_max_m[1] - profile.tray_roi_min_m[1];
    if (!(span_x > 0.0 && span_y > 0.0) || camera.k[0] <= 0.0 || camera.k[4] <= 0.0) {
      return;
    }
    sensor_msgs::msg::Image output;
    output.header.stamp = depth.header.stamp;
    output.header.frame_id = "Tray";
    output.width = static_cast<uint32_t>(std::ceil(span_x / tray_depth_resolution_m_));
    output.height = static_cast<uint32_t>(std::ceil(span_y / tray_depth_resolution_m_));
    output.encoding = "32FC1";
    output.is_bigendian = false;
    output.step = output.width * sizeof(float);
    output.data.resize(static_cast<size_t>(output.step) * output.height);
    auto * pixels = reinterpret_cast<float *>(output.data.data());
    std::fill(
      pixels, pixels + static_cast<size_t>(output.width) * output.height,
      std::numeric_limits<float>::quiet_NaN());
    tf2::Quaternion rotation(
      calibration.tray_from_camera.transform.rotation.x,
      calibration.tray_from_camera.transform.rotation.y,
      calibration.tray_from_camera.transform.rotation.z,
      calibration.tray_from_camera.transform.rotation.w);
    const tf2::Vector3 translation(
      calibration.tray_from_camera.transform.translation.x,
      calibration.tray_from_camera.transform.translation.y,
      calibration.tray_from_camera.transform.translation.z);
    for (uint32_t row = 0; row < depth.height; ++row) {
      for (uint32_t col = 0; col < depth.width; ++col) {
        float z = 0.0F;
        const auto offset = static_cast<size_t>(row) * depth.step +
          static_cast<size_t>(col) * (depth.encoding == "32FC1" ? sizeof(float) : sizeof(uint16_t));
        if (depth.encoding == "32FC1") {
          std::memcpy(&z, depth.data.data() + offset, sizeof(float));
        } else {
          uint16_t millimeters = 0;
          std::memcpy(&millimeters, depth.data.data() + offset, sizeof(uint16_t));
          z = static_cast<float>(millimeters) * 0.001F;
        }
        if (!std::isfinite(z) || z <= 0.0F) {continue;}
        const double x = (static_cast<double>(col) - camera.k[2]) * z / camera.k[0];
        const double y = (static_cast<double>(row) - camera.k[5]) * z / camera.k[4];
        const auto point = tf2::quatRotate(rotation, tf2::Vector3(x, y, z)) + translation;
        if (point.x() < profile.tray_roi_min_m[0] ||
          point.x() >= profile.tray_roi_max_m[0] ||
          point.y() < profile.tray_roi_min_m[1] ||
          point.y() >= profile.tray_roi_max_m[1]) {continue;}
        const auto px = static_cast<uint32_t>((point.x() - profile.tray_roi_min_m[0]) /
          tray_depth_resolution_m_);
        const auto py = static_cast<uint32_t>((profile.tray_roi_max_m[1] - point.y()) /
          tray_depth_resolution_m_);
        if (px >= output.width || py >= output.height) {continue;}
        auto & destination = pixels[static_cast<size_t>(py) * output.width + px];
        if (!std::isfinite(destination) || point.z() > destination) {
          destination = static_cast<float>(point.z());
        }
      }
    }
    tray_depth_publisher_->publish(output);
  }

  std::vector<DetectorProfile> load_detector_profiles()
  {
    const auto prefix = std::string("detector_profiles.RawPart.");
    DetectorProfile profile;
    auto & c = profile.cuboid;
    c.grid_resolution_m = declare_parameter<double>(
      prefix + "cuboid.grid_resolution_m",
      c.grid_resolution_m);
    c.height_tolerance_m = declare_parameter<double>(
      prefix + "cuboid.height_tolerance_m",
      c.height_tolerance_m);
    c.line_residual_limit_px = declare_parameter<double>(
      prefix + "cuboid.line_residual_limit_px",
      c.line_residual_limit_px);
    c.minimum_line_support = declare_parameter<double>(
      prefix + "cuboid.minimum_line_support",
      c.minimum_line_support);
    validate_cuboid_config(c);
    profile.target_id = declare_parameter<std::string>(prefix + "target_id", "");
    profile.material_id = declare_parameter<std::string>(prefix + "material_id", "");
    profile.shape = declare_parameter<std::string>(
      "material_definitions.RawPart.shape", "");
    const auto dimensions = declare_parameter<std::vector<double>>(
      "material_definitions.RawPart.nominal_dimensions_m", {});
    const auto dimension_tolerance = declare_parameter<std::vector<double>>(
      prefix + "dimension_tolerance_m", {});
    const auto roi_x = declare_parameter<std::vector<double>>(
      prefix + "workspace_roi_m.x", {});
    const auto roi_y = declare_parameter<std::vector<double>>(
      prefix + "workspace_roi_m.y", {});
    const auto roi_z = declare_parameter<std::vector<double>>(
      prefix + "workspace_roi_m.z", {});
    const auto tray_roi_x = declare_parameter<std::vector<double>>(
      prefix + "tray_roi_m.x", {});
    const auto tray_roi_y = declare_parameter<std::vector<double>>(
      prefix + "tray_roi_m.y", {});
    if (tray_roi_x.size() != 2 || tray_roi_y.size() != 2) {
      throw std::invalid_argument("RawPart tray_roi_m x and y must each contain two bounds");
    }
    const auto marker_position = declare_parameter<std::vector<double>>(
      prefix + "fiducial.marker_in_support.position_m", {});
    const auto marker_rpy = declare_parameter<std::vector<double>>(
      prefix + "fiducial.marker_in_support.rpy_rad", {});
    const auto copy_three = [](const std::vector<double> & source,
        std::array<double, 3> & target) {
        if (source.size() == target.size()) {
          std::copy(source.begin(), source.end(), target.begin());
        }
      };
    const auto copy_roi = [](const std::vector<double> & source,
        size_t axis, std::array<double, 3> & minimum, std::array<double, 3> & maximum) {
        if (source.size() == 2) {
          minimum[axis] = source[0];
          maximum[axis] = source[1];
        }
      };
    copy_three(dimensions, profile.dimensions_m);
    copy_three(dimension_tolerance, profile.dimension_tolerance_m);
    copy_three(marker_position, profile.marker_in_support_m);
    copy_three(marker_rpy, profile.marker_rpy_in_support_rad);
    copy_roi(roi_x, 0, profile.workspace_roi_min_m, profile.workspace_roi_max_m);
    copy_roi(roi_y, 1, profile.workspace_roi_min_m, profile.workspace_roi_max_m);
    copy_roi(roi_z, 2, profile.workspace_roi_min_m, profile.workspace_roi_max_m);
    std::array<double, 3> tray_roi_min{};
    std::array<double, 3> tray_roi_max{};
    copy_roi(tray_roi_x, 0, tray_roi_min, tray_roi_max);
    copy_roi(tray_roi_y, 1, tray_roi_min, tray_roi_max);
    profile.tray_roi_min_m = {tray_roi_min[0], tray_roi_min[1]};
    profile.tray_roi_max_m = {tray_roi_max[0], tray_roi_max[1]};
    profile.support_contact_tolerance_m = declare_parameter<double>(
      prefix + "support_contact_tolerance_m", 0.0);
    profile.top_surface_geometry_tolerance_m = declare_parameter<double>(
      prefix + "top_surface_geometry_tolerance_m", 0.0);
    profile.minimum_observation_coverage = declare_parameter<double>(
      prefix + "minimum_observation_coverage", 0.55);
    profile.fiducial_dictionary = declare_parameter<std::string>(
      prefix + "fiducial.dictionary", "");
    profile.fiducial_marker_id = static_cast<int>(declare_parameter<int64_t>(
        prefix + "fiducial.marker_id", -1));
    profile.fiducial_size_m = declare_parameter<double>(
      prefix + "fiducial.size_m", 0.0);
    profile.fiducial_pose_translation_scale = declare_parameter<double>(
      prefix + "fiducial.pose_translation_scale", 1.0);
    return {profile};
  }

  std::string vision_run_id_ = make_vision_run_id();
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<DetectorProfile> detector_profiles_;
  DetectTargetProcessor processor_;
  std::string calibration_file_path_;
  std::string camera_info_topic_;
  CalibrationFileData selected_calibration_file_;
  bool selected_calibration_file_loaded_{false};
  std::mutex calibration_input_mutex_;
  std::shared_ptr<const DetectionInput> latest_calibration_input_;
  int64_t latest_calibration_input_receipt_ns_{0};
  bool calibration_request_active_{false};
  bool calibration_compatibility_initialized_{false};
  bool calibration_camera_compatible_{false};
  std::unique_ptr<DepthSceneStabilityMonitor> depth_scene_stability_monitor_;
  DepthSceneStabilityState depth_scene_stability_state_;
  double depth_difference_tolerance_m_ = 0.003;
  double changed_pixel_ratio_tolerance_ = 0.01;
  std::string depth_scene_acquisition_id_;
  int64_t depth_scene_stamp_ns_{0};
  bool detector_invoked_{false};
  std::string detector_not_invoked_reason_;
  std::optional<double> detector_elapsed_ms_;
  std::optional<uint8_t> detector_result_code_;
  std::deque<VisionDetectorSnapshotJob::DepthSceneSample> depth_scene_samples_;
  // Serializes acquisition callbacks, expiry/invalidation, and acquisition lifecycle edges.
  std::mutex acquisition_callback_mutex_;
  std::mutex acquisition_state_mutex_;
  SharedAcquisitionResult acquisition_result_;
  int64_t stale_frame_limit_ns_ = 1'000'000'000;
  uint64_t acquisition_generation_{0};
  std::string acquisition_id_;
  std::mutex snapshot_state_mutex_;
  SupportCalibrationSnapshot latest_calibration_snapshot_;
  std::unique_ptr<VisionDetectorSnapshotOutput> snapshot_output_;
  std_msgs::msg::Header last_scene_stability_header_;
  rclcpp::Publisher<arm_cell_interfaces::msg::SceneStability>::SharedPtr
    scene_stability_publisher_;
  rclcpp::Publisher<arm_cell_interfaces::msg::VisionCalibration>::SharedPtr calibration_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tray_depth_publisher_;
  double tray_depth_resolution_m_ = 0.002;
  uint32_t scene_stability_required_observations_{5};
  bool calibration_status_published_ = false;
  bool calibration_validation_pending_{false};
  uint8_t last_calibration_state_ =
    arm_cell_interfaces::msg::VisionCalibration::STATE_NEEDS_CALIBRATION;
  mutable std::mutex calibration_state_mutex_;
  std::string last_calibration_detail_;
  std::string last_calibration_published_detail_;
  bool acquisition_active_{false};
  bool acquisition_terminal_snapshot_taken_{false};
  std::string acquisition_target_id_;
  std::string acquisition_algorithm_;
  int64_t acquisition_start_monotonic_ns_{0};
  int64_t acquisition_watermark_stamp_ns_{0};
  std_msgs::msg::Header last_calibration_header_;
  int64_t last_tray_depth_publish_monotonic_ns_ = 0;
  rclcpp::TimerBase::SharedPtr calibration_status_timer_;
  rclcpp::CallbackGroup::SharedPtr calibration_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  DetectTargetService service_;
};

std::shared_ptr<rclcpp::Node> make_detect_target_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<DetectTargetNode>(options);
}

}  // namespace arm_cell_vision
