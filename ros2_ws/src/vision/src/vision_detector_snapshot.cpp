#include "vision_snapshot_internal.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <exception>
#include <fstream>
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include <opencv2/imgcodecs.hpp>

namespace arm_cell_vision
{
using snapshot_detail::append_depth_validation_json;
using snapshot_detail::append_observation_identity_json;
using snapshot_detail::event_name;
using snapshot_detail::json_escape;

namespace
{

constexpr size_t kMaxPendingTerminalSnapshots = 16;

}  // namespace

namespace detail
{

void set_cuboid_stage_image_metadata(
  arm_cell_interfaces::msg::VisionDetectorSnapshot & message,
  const std::string & stage, const std::string & status, bool saved)
{
  const std::string marker = "\"stage\":\"" + json_escape(stage) + "\",\"status\":\"";
  const auto start = message.metadata_json.find(marker);
  if (start == std::string::npos) {return;}
  const auto status_begin = start + marker.size();
  const auto status_end = message.metadata_json.find('"', status_begin);
  if (status_end == std::string::npos) {return;}
  if (!saved) {message.metadata_json.replace(status_begin, status_end - status_begin, json_escape(status));}
  if (saved) {return;}
  const auto object_end = message.metadata_json.find('}', status_begin);
  const auto path_key = message.metadata_json.find(",\"relative_path\":\"", status_begin);
  if (path_key != std::string::npos && path_key < object_end) {
    const auto path_value_begin = path_key + std::string(",\"relative_path\":\"").size();
    const auto path_end = message.metadata_json.find('"', path_value_begin);
    if (path_end != std::string::npos) {
      message.metadata_json.erase(path_key, path_end - path_key + 1);
    }
  }
}

}  // namespace detail

namespace
{

std::pair<std::string, std::string> snapshot_utc_run_date_time()
{
  const auto now = std::chrono::system_clock::now();
  const auto timestamp = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  if (gmtime_r(&timestamp, &utc) == nullptr) {
    throw std::runtime_error("failed to convert snapshot time to UTC");
  }

  std::ostringstream date;
  date << std::put_time(&utc, "%Y%m%d");
  std::ostringstream time;
  time << std::put_time(&utc, "%H%M%S");
  return {date.str(), time.str()};
}

void increment_decimal(std::string & value)
{
  for (auto digit = value.rbegin(); digit != value.rend(); ++digit) {
    if (*digit != '9') {
      ++(*digit);
      return;
    }
    *digit = '0';
  }
  value.insert(value.begin(), '1');
}

std::string normalized_sequence(std::string value)
{
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char character) {
      return character >= '0' && character <= '9';
    }))
  {return {};}
  const auto first_nonzero = value.find_first_not_of('0');
  return first_nonzero == std::string::npos ? "0" : value.substr(first_nonzero);
}

bool reserve_numbered_directory(
  const std::filesystem::path & parent, std::filesystem::path & reserved,
  std::string & failure_cause)
{
  std::error_code error;
  auto entry = std::filesystem::directory_iterator(parent, error);
  if (error) {
    failure_cause = "scan snapshot directory: " + error.message();
    return false;
  }
  const auto end = std::filesystem::directory_iterator();
  std::string maximum_sequence = "0";
  while (entry != end) {
    const auto name = entry->path().filename().string();
    error.clear();
    const bool is_directory = entry->is_directory(error);
    if (error) {
      failure_cause = "inspect snapshot directory entry: " + error.message();
      return false;
    }
    const auto sequence = is_directory ? normalized_sequence(name) : std::string();
    if (!sequence.empty() && (sequence.size() > maximum_sequence.size() ||
      (sequence.size() == maximum_sequence.size() && sequence > maximum_sequence)))
    {
      maximum_sequence = sequence;
    }
    entry.increment(error);
    if (error) {
      failure_cause = "scan snapshot directory: " + error.message();
      return false;
    }
  }

  increment_decimal(maximum_sequence);
  while (true) {
    const auto sequence = maximum_sequence.size() < 2 ?
      "0" + maximum_sequence : maximum_sequence;
    const auto candidate = parent / sequence;
    error.clear();
    if (std::filesystem::create_directory(candidate, error)) {
      reserved = candidate;
      return true;
    }
    if (error && error != std::errc::file_exists) {
      failure_cause = "reserve snapshot directory: " + error.message();
      return false;
    }
    increment_decimal(maximum_sequence);
  }
}

bool reserve_timestamped_directory(
  const std::filesystem::path & parent, const std::string & timestamp,
  std::filesystem::path & reserved, std::string & failure_cause)
{
  std::error_code error;
  for (uint64_t suffix = 0; ; ++suffix) {
    auto name = timestamp;
    if (suffix != 0) {name += "-" + std::to_string(suffix);}
    const auto candidate = parent / name;
    if (std::filesystem::create_directory(candidate, error)) {
      reserved = candidate;
      return true;
    }
    if (error && error != std::errc::file_exists) {
      failure_cause = "reserve snapshot run directory: " + error.message();
      return false;
    }
    error.clear();
  }
}

bool write_snapshot_files(
  const std::filesystem::path & directory,
  const arm_cell_interfaces::msg::VisionDetectorSnapshot & message,
  const std::filesystem::path & metadata_directory, std::string & failure_cause)
{
  bool success = true;
  {
    std::ofstream png(directory / "snapshot.png", std::ios::binary);
    if (png && !message.image_png.data.empty()) {
      png.write(reinterpret_cast<const char *>(message.image_png.data.data()),
        static_cast<std::streamsize>(message.image_png.data.size()));
      if (!png) {
        success = false;
        failure_cause = "write snapshot.png failed: " +
          std::error_code(errno, std::generic_category()).message();
      }
    } else if (!png) {
      success = false;
      failure_cause = "open snapshot.png failed: " +
        std::error_code(errno, std::generic_category()).message();
    }
  }
  std::error_code metadata_error;
  std::filesystem::create_directories(metadata_directory, metadata_error);
  if (metadata_error) {
    if (!failure_cause.empty()) {failure_cause += "; ";}
    failure_cause += "create metadata directory failed: " + metadata_error.message();
    return false;
  }
  std::ofstream metadata(metadata_directory / "metadata.json", std::ios::binary);
  if (metadata) {
    metadata << message.metadata_json;
    if (!metadata) {
      success = false;
      if (!failure_cause.empty()) {failure_cause += "; ";}
      failure_cause += "write metadata.json failed: " +
        std::error_code(errno, std::generic_category()).message();
    }
  } else {
    success = false;
    if (!failure_cause.empty()) {failure_cause += "; ";}
    failure_cause += "open metadata.json failed: " +
      std::error_code(errno, std::generic_category()).message();
  }
  return success;
}

}  // namespace

std::string make_vision_run_id()
{
  std::ostringstream id;
  try {
    std::random_device random;
    id << std::hex << std::setfill('0') << std::setw(8) << random() << '-' <<
      std::setw(8) << random() << '-' << std::setw(8) << random();
  } catch (const std::exception &) {
    static std::atomic<uint64_t> sequence{1};
    id << std::chrono::steady_clock::now().time_since_epoch().count() << '-' <<
      sequence.fetch_add(1);
  }
  return id.str();
}

VisionDetectorSnapshotOutput::VisionDetectorSnapshotOutput(
  rclcpp::Node & node, std::filesystem::path artifact_root, std::string vision_run_id,
  Save save, Publish publish, std::string vision_run_date,
  std::filesystem::path diagnostic_root, std::string live_test_session_id,
  std::string vision_run_time, std::filesystem::path snapshot_run_directory)
: logger_(node.get_logger()), artifact_root_(std::move(artifact_root)),
  diagnostic_root_(diagnostic_root.empty() ?
    artifact_root_.parent_path() / "diagnostic_records" / "unassociated" / "vision" :
    std::move(diagnostic_root)),
  snapshot_run_directory_(std::move(snapshot_run_directory)),
  live_test_session_id_(std::move(live_test_session_id)),
  vision_run_id_(std::move(vision_run_id)), save_(std::move(save)), publish_(std::move(publish))
{
  std::pair<std::string, std::string> generated_run_date_time;
  try {
    generated_run_date_time = snapshot_utc_run_date_time();
  } catch (const std::exception &) {
    generated_run_date_time = {};
  }
  run_date_ = vision_run_date.empty() ? std::move(generated_run_date_time.first) :
    std::move(vision_run_date);
  run_time_ = vision_run_time.empty() ? std::move(generated_run_date_time.second) :
    std::move(vision_run_time);
  const auto color_parameter = [&node](const std::string & name, const std::string & fallback) {
      const auto value = node.has_parameter(name) ? node.get_parameter(name).as_string() :
        node.declare_parameter<std::string>(name, fallback);
      if (value.size() != 7 || value[0] != '#' ||
        !std::all_of(value.begin() + 1, value.end(), [](unsigned char c) {return std::isxdigit(c);}))
      {return fallback;}
      return value;
    };
  const auto integer_parameter = [&node](const std::string & name, int fallback) -> int {
      return node.has_parameter(name) ? static_cast<int>(node.get_parameter(name).as_int()) :
        node.declare_parameter<int>(name, fallback);
    };
  const auto boolean_parameter = [&node](const std::string & name, bool fallback) {
      return node.has_parameter(name) ? node.get_parameter(name).as_bool() :
        node.declare_parameter<bool>(name, fallback);
    };
  const auto real_parameter = [&node](const std::string & name, double fallback) {
      return node.has_parameter(name) ? node.get_parameter(name).as_double() :
        node.declare_parameter<double>(name, fallback);
    };
  style_.candidate_color_rgb = color_parameter("snapshot_style.candidate.color_rgb", "#00C800");
  style_.candidate_line_width_px = std::clamp(
    integer_parameter("snapshot_style.candidate.line_width_px", 3), 1, 20);
  style_.unresolved_color_rgb = color_parameter("snapshot_style.regions.unresolved_color_rgb", "#FFA500");
  style_.target_color_rgb = color_parameter("snapshot_style.regions.target_color_rgb", "#00FF00");
  style_.support_color_rgb = color_parameter("snapshot_style.regions.support_color_rgb", "#00B4FF");
  style_.disturbance_inclined_color_rgb = color_parameter(
    "snapshot_style.disturbance.inclined_color_rgb", "#FF00FF");
  style_.disturbance_elevated_color_rgb = color_parameter(
    "snapshot_style.disturbance.elevated_color_rgb", "#00FFFF");
  style_.disturbance_other_color_rgb = color_parameter(
    "snapshot_style.disturbance.other_color_rgb", "#FFFFFF");
  style_.region_line_width_px = std::clamp(
    integer_parameter("snapshot_style.regions.line_width_px", 2), 1, 20);
  style_.measurement_color_rgb = color_parameter("snapshot_style.measurement.color_rgb", "#FF0000");
  style_.marker_size_px = std::clamp(
    integer_parameter("snapshot_style.measurement.marker_size_px", 24), 3, 100);
  style_.measurement_line_width_px = std::clamp(
    integer_parameter("snapshot_style.measurement.line_width_px", 3), 1, 20);
  style_.show_annotation_text = boolean_parameter("snapshot_style.annotation.show_text", true);
  const double configured_text_scale = real_parameter("snapshot_style.annotation.text_scale", 1.0);
  style_.annotation_text_scale = std::isfinite(configured_text_scale) ?
    std::clamp(configured_text_scale, 0.5, 2.0) : 1.0;
  style_.primary_text_color_rgb = color_parameter("snapshot_style.annotation.primary_color_rgb", "#FFDC00");
  style_.detail_text_color_rgb = color_parameter("snapshot_style.annotation.detail_color_rgb", "#FFFFFF");
  if (snapshot_run_directory_.empty() && (run_date_.empty() || run_time_.empty())) {
    RCLCPP_WARN(logger_, "detector snapshot run date unavailable cause=%s",
                "utc_date_unavailable");
  }
  try {
    publisher_ = node.create_publisher<arm_cell_interfaces::msg::VisionDetectorSnapshot>(
      "/vision/detector_snapshot", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger_, "detector snapshot publisher unavailable cause=%s",
                error.what());
  }
  try {
    worker_ = std::thread([this]() {run();});
  } catch (const std::exception & error) {
    stopping_ = true;
    RCLCPP_WARN(logger_, "detector snapshot worker unavailable cause=%s",
                error.what());
  }
}

VisionDetectorSnapshotOutput::~VisionDetectorSnapshotOutput()
{
  shutdown();
}

void VisionDetectorSnapshotOutput::submit(VisionDetectorSnapshotJob job) noexcept
{
  try {
    job.vision_run_id = vision_run_id_;
    const bool terminal_job =
      job.event_kind == arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS ||
      job.event_kind == arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || !worker_.joinable()) {
      if (terminal_job) {
        RCLCPP_WARN(logger_, "vision terminal evidence not enqueued cause=writer_unavailable");
      }
      return;
    }
    switch (job.event_kind) {
      case 0:
      case 4: calibration_pending_ = std::move(job); break;
      case 1:
        if (std::any_of(terminal_pending_.begin(), terminal_pending_.end(),
          [&job](const auto & terminal) {
            return terminal.acquisition_id == job.acquisition_id;
          }))
        {return;}
        progress_pending_ = std::move(job);
        break;
      default:
        if (terminal_pending_.size() == kMaxPendingTerminalSnapshots) {
          RCLCPP_WARN(logger_, "vision terminal evidence not enqueued "
                               "cause=bounded_output_capacity_exhausted");
          return;
        }
        if (progress_pending_ &&
          progress_pending_->acquisition_id == job.acquisition_id)
        {progress_pending_.reset();}
        terminal_pending_.push_back(std::move(job));
        break;
    }
    ready_.notify_one();
  } catch (const std::exception & error) {
    RCLCPP_WARN(logger_, "vision terminal evidence enqueue failed cause=%s", error.what());
  } catch (...) {
    RCLCPP_WARN(logger_, "vision terminal evidence enqueue failed cause=unknown_exception");
  }
}

void VisionDetectorSnapshotOutput::shutdown()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {return;}
    stopping_ = true;
  }
  ready_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

void VisionDetectorSnapshotOutput::run()
{
  while (true) {
    VisionDetectorSnapshotJob job;
    bool terminal_job = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock, [this]() {
        return stopping_ || calibration_pending_ || progress_pending_ || !terminal_pending_.empty();
      });
      if (!calibration_pending_ && !progress_pending_ && terminal_pending_.empty() && stopping_) {return;}
      if (calibration_pending_) {
        job = std::move(*calibration_pending_);
        calibration_pending_.reset();
      } else if (!terminal_pending_.empty()) {
        job = terminal_pending_.front();
        terminal_job = true;
      } else {
        job = std::move(*progress_pending_);
        progress_pending_.reset();
      }
    }
    job.revision = ++revision_;
    arm_cell_interfaces::msg::VisionDetectorSnapshot message;
    bool encoding_failure_logged = false;
    try {
      message = render(job);
    } catch (const std::exception & error) {
      message.snapshot_id = job.vision_run_id + ":" + std::to_string(job.revision);
      message.revision = job.revision;
      message.event_kind = job.event_kind;
      message.vision_run_id = job.vision_run_id;
      message.calibration_id = job.calibration_id;
      message.acquisition_id = job.acquisition_id;
      message.image_png.format = "png";
      std::ostringstream fallback;
      fallback << "{\"schema_version\":17,\"snapshot_id\":\"" <<
        json_escape(message.snapshot_id) << "\",\"revision\":" << message.revision <<
        ",\"event_kind\":\"" << event_name(message.event_kind) << "\",\"vision_run_id\":\"" <<
        json_escape(message.vision_run_id) << "\",\"calibration_id\":\"" <<
        json_escape(message.calibration_id) << "\",\"acquisition_id\":\"" <<
        json_escape(message.acquisition_id) << "\",\"image_available\":false,\"render_failure\":\"" <<
        json_escape(error.what()) << "\",\"observation\":";
      if (job.has_observation && job.observation_input()) {
        append_observation_identity_json(fallback, *job.observation_input(), "aligned_depth");
      } else {fallback << "null";}
      if (!job.target_id.empty()) {
        fallback << ",\"target_id\":\"" << json_escape(job.target_id) << "\"";
      }
      if (message.event_kind ==
          arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_COMPLETE ||
        !job.calibration_data().recent_depth_validations.empty())
      {
        fallback << ",\"calibration_summary\":{\"ready\":" <<
          (job.calibration_data().ready ? "true" : "false") << ",\"accepted_observations\":" <<
          job.calibration_data().accepted_frames << ",\"required_observations\":" <<
          job.calibration_data().required_frames << ",\"median_depth_residual_m\":" <<
          job.calibration_data().median_depth_residual_m;
        append_depth_validation_json(fallback, job.calibration_data());
        if (job.calibration_data().ready) {
          const auto & transform = job.calibration_data().tray_from_camera.transform;
          fallback << ",\"tray_from_camera\":{\"translation\":[" <<
            transform.translation.x << ',' << transform.translation.y << ',' <<
            transform.translation.z << "],\"rotation_xyzw\":[" << transform.rotation.x << ',' <<
            transform.rotation.y << ',' << transform.rotation.z << ',' << transform.rotation.w << "]}";
        }
        fallback << '}';
      }
      fallback << ",\"annotation\":{\"processing_label\":\"" <<
        json_escape(job.processing_stage_data()) << "\",\"target_id\":\"" <<
        json_escape(job.target_id) << "\"}";
      fallback << '}';
      message.metadata_json = fallback.str();
      encoding_failure_logged = true;
      RCLCPP_WARN(logger_, "detector snapshot encoding failed cause=%s", error.what());
    }
    const bool terminal_snapshot =
      job.event_kind == arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS ||
      job.event_kind == arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
    if (terminal_snapshot) {
      std::string diagnostic_failure;
      if (!save_diagnostic_record(job, diagnostic_failure, message.diagnostic_record_json)) {
        RCLCPP_WARN(logger_, "vision diagnostic record save failed cause=%s", diagnostic_failure.c_str());
      }
    }
    if (job.save && (job.event_kind ==
      arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_COMPLETE || terminal_snapshot))
    {
      try {
        std::string failure_cause;
        if (!save_job(job, message, failure_cause)) {
          RCLCPP_WARN(logger_, "detector snapshot local save failed cause=%s", failure_cause.c_str());
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(logger_, "detector snapshot local save failed cause=%s", error.what());
      }
    }
    if (message.image_png.data.empty() && !encoding_failure_logged) {
      RCLCPP_WARN(
          logger_,
          "detector snapshot PNG unavailable cause=png_encoder_returned_empty");
    }
    try {publish_job(message);} catch (const std::exception & error) {
      RCLCPP_WARN(logger_, "detector snapshot publication failed cause=%s", error.what());
    }
    if (terminal_job) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!terminal_pending_.empty() &&
        terminal_pending_.front().acquisition_id == job.acquisition_id)
      {terminal_pending_.pop_front();}
    }
  }
}

arm_cell_interfaces::msg::VisionDetectorSnapshot VisionDetectorSnapshotOutput::render(
  VisionDetectorSnapshotJob & job)
{
  if (job.depth_scene_diagnostics_enabled && !job.depth_scene_samples.empty()) {
    snapshot_detail::render_scene_stability(job, job.depth_scene_panorama_png);
  }
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  Message message;
  message.revision = job.revision;
  message.event_kind = job.event_kind;
  message.vision_run_id = job.vision_run_id;
  message.calibration_id = job.calibration_id;
  message.acquisition_id = job.acquisition_id;
  message.snapshot_id = job.vision_run_id + ":" + std::to_string(job.revision);
  message.image_png.format = "png";
  bool source_image_rendered = false;
  std::vector<std::optional<cv::Point>> disturbance_projections;
  const auto image = render_annotation(job, source_image_rendered, disturbance_projections);
  snapshot_detail::render_geometry(job, job.cuboid_stage_images);
  std::vector<uint8_t> png;
  try {
    if (cv::imencode(".png", image, png)) {message.image_png.data = std::move(png);}
  } catch (const cv::Exception &) {
    message.image_png.data.clear();
  }
  message.metadata_json = serialize_metadata(
    job, message, source_image_rendered, disturbance_projections);
  message.image_png.header = job.has_observation && job.observation_input() ?
    job.observation_input()->rgb.header : std_msgs::msg::Header();
  return message;
}

bool VisionDetectorSnapshotOutput::save_job(
  const VisionDetectorSnapshotJob & job,
  arm_cell_interfaces::msg::VisionDetectorSnapshot & message,
  std::string & failure_cause)
{
  const bool calibration_completion = job.event_kind ==
    arm_cell_interfaces::msg::VisionDetectorSnapshot::CALIBRATION_COMPLETE;
  const bool terminal_acquisition =
    job.event_kind == arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS ||
    job.event_kind == arm_cell_interfaces::msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE;
  if (!calibration_completion && !terminal_acquisition) {
    failure_cause = "only Calibration completion and DetectTarget terminal Snapshots are persisted";
    return false;
  }
  if (save_) {
    const bool saved = save_(job, message);
    if (!saved) {
      failure_cause = "save_callback_returned_failure";
    }
    return saved;
  }
  std::error_code error;
  if (snapshot_run_directory_.empty() && (run_date_.empty() || run_time_.empty())) {
    failure_cause = "vision run UTC date unavailable";
    return false;
  }
  if (!run_directory_) {
    if (!snapshot_run_directory_.empty()) {
      run_directory_ = snapshot_run_directory_;
      std::filesystem::create_directories(*run_directory_, error);
      if (error) {
        failure_cause = "create session snapshot directory: " + error.message();
        run_directory_.reset();
        return false;
      }
    } else {
      const auto date_directory = artifact_root_ / run_date_;
      std::filesystem::create_directories(date_directory, error);
      if (error) {
        failure_cause = "create snapshot date directory: " + error.message();
        return false;
      }
      std::filesystem::path reserved;
      if (!reserve_timestamped_directory(
          date_directory, run_time_, reserved, failure_cause)) {
        return false;
      }
      run_directory_ = std::move(reserved);
    }
  }
  std::filesystem::path directory;
  if (calibration_completion) {
    const auto calibration_root = *run_directory_ / "calibration";
    std::filesystem::create_directories(calibration_root, error);
    if (error) {
      failure_cause = "create calibration snapshot directory: " + error.message();
      return false;
    }
    if (!reserve_numbered_directory(calibration_root, directory, failure_cause)) {
      return false;
    }
  } else {
    if (job.acquisition_id.empty() || job.acquisition_id.find_first_not_of(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
    {
      failure_cause = "acquisition_id is empty or not a safe snapshot directory name";
      return false;
    }
    const auto acquisition_root = *run_directory_ / "acquisition";
    std::filesystem::create_directories(acquisition_root, error);
    if (error) {
      failure_cause = "create acquisition snapshot directory: " + error.message();
      return false;
    }
    directory = acquisition_root / job.acquisition_id;
    error.clear();
    if (!std::filesystem::create_directory(directory, error)) {
      failure_cause = error ? "reserve acquisition snapshot directory: " + error.message() :
        "snapshot directory already exists for this Acquisition";
      return false;
    }
  }
  bool stage_images_saved = true;
  std::map<std::string, bool> saved_stage_files;
  for (const auto & stage : job.cuboid_stage_images) {
    const auto prior = saved_stage_files.find(stage.filename);
    if (prior != saved_stage_files.end()) {
      const bool stage_saved = prior->second;
      detail::set_cuboid_stage_image_metadata(message, stage.stage,
        stage_saved ? stage.status : "file_save_failed", stage_saved);
      if (!stage_saved) {
        stage_images_saved = false;
        if (!failure_cause.empty()) {failure_cause += "; ";}
        failure_cause += "write " + stage.filename + " failed";
      }
      continue;
    }
    if (stage.png.empty()) {
      detail::set_cuboid_stage_image_metadata(message, stage.stage, stage.status, false);
      continue;
    }
    bool stage_saved = false;
    std::ofstream stream(directory / stage.filename, std::ios::binary);
    if (stream) {
      stream.write(reinterpret_cast<const char *>(stage.png.data()),
        static_cast<std::streamsize>(stage.png.size()));
      stream.flush();
      stage_saved = static_cast<bool>(stream);
      stream.close();
      stage_saved = stage_saved && !stream.fail();
    }
    saved_stage_files[stage.filename] = stage_saved;
    detail::set_cuboid_stage_image_metadata(message, stage.stage,
      stage_saved ? stage.status : "file_save_failed", stage_saved);
    if (!stage_saved) {
      stage_images_saved = false;
      std::filesystem::remove(directory / stage.filename, error);
      if (!failure_cause.empty()) {failure_cause += "; ";}
      failure_cause += "write " + stage.filename + " failed";
    }
  }
  bool scene_panorama_saved = true;
  if (job.depth_scene_diagnostics_enabled && !job.depth_scene_panorama_png.empty()) {
    std::ofstream stream(directory / "scene_stability_panorama.png", std::ios::binary);
    if (stream) {
      stream.write(
        reinterpret_cast<const char *>(job.depth_scene_panorama_png.data()),
        static_cast<std::streamsize>(job.depth_scene_panorama_png.size()));
      stream.flush();
      scene_panorama_saved = static_cast<bool>(stream);
    } else {
      scene_panorama_saved = false;
    }
    if (!scene_panorama_saved) {
      if (!failure_cause.empty()) {failure_cause += "; ";}
      failure_cause += "write scene_stability_panorama.png failed";
    }
  }
  auto metadata_directory = directory;
  if (job.acquisition_id.empty()) {
    metadata_directory = diagnostic_root_ / "snapshot_metadata";
    if (live_test_session_id_.empty()) {
      // Unassociated diagnostics share a root across runs; retain their run identity.
      metadata_directory /= run_directory_->parent_path().filename();
      metadata_directory /= run_directory_->filename();
    }
    metadata_directory /= directory.lexically_relative(*run_directory_);
  }
  message.metadata_json.insert(message.metadata_json.find_last_of('}'),
    ",\"snapshot_directory\":\"" + json_escape(std::filesystem::absolute(directory).string()) + "\"");
  const bool snapshot_saved = write_snapshot_files(
    directory, message, metadata_directory, failure_cause);
  const bool saved = snapshot_saved && stage_images_saved && scene_panorama_saved;
  return saved;
}

void VisionDetectorSnapshotOutput::publish_job(
  const arm_cell_interfaces::msg::VisionDetectorSnapshot & message)
{
  if (publish_) {publish_(message); return;}
  if (publisher_) {publisher_->publish(message);}
  else {throw std::runtime_error("ROS publisher unavailable");}
}

}  // namespace arm_cell_vision
