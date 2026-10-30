#include "arm_cell_vision/calibration_file.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <opencv2/core.hpp>
#include <unistd.h>

namespace arm_cell_vision
{
namespace
{

bool finite(double value)
{
  return std::isfinite(value);
}

bool read_vector(
  const cv::FileNode & parent, const char * key, size_t expected_size,
  std::vector<double> & values)
{
  const auto node = parent[key];
  if (node.empty() || !node.isSeq() || node.size() != expected_size) {
    return false;
  }
  values.clear();
  values.reserve(expected_size);
  for (const auto & element : node) {
    double value = 0.0;
    element >> value;
    if (!finite(value)) {
      return false;
    }
    values.push_back(value);
  }
  return true;
}

template<size_t N>
bool assign_array(const std::vector<double> & values, std::array<double, N> & output)
{
  if (values.size() != N) {
    return false;
  }
  std::copy(values.begin(), values.end(), output.begin());
  return true;
}

bool valid_transform(const geometry_msgs::msg::Transform & transform)
{
  const auto & t = transform.translation;
  const auto & q = transform.rotation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return finite(t.x) && finite(t.y) && finite(t.z) && finite(q.x) && finite(q.y) &&
         finite(q.z) && finite(q.w) && norm > 1e-12 && std::abs(norm - 1.0) < 1e-3;
}

bool close(double a, double b)
{
  return finite(a) && finite(b) && std::abs(a - b) <= 1e-9 * std::max({1.0, std::abs(a), std::abs(b)});
}

std::string timestamp_suffix()
{
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()).count() % 1000;
  std::tm utc{};
  gmtime_r(&seconds, &utc);
  std::ostringstream suffix;
  suffix << std::put_time(&utc, "%Y%m%dT%H%M%S") << '.' << std::setw(3)
         << std::setfill('0') << millis << 'Z';
  return suffix.str();
}

std::string encode(const CalibrationFileData & calibration)
{
  cv::FileStorage storage("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
  const auto & transform = calibration.tray_from_camera.transform;
  const auto & camera = calibration.camera_info;
  storage << "tray_T_camera" << "{";
  storage << "translation" << "[" << transform.translation.x << transform.translation.y <<
    transform.translation.z << "]";
  storage << "quaternion_xyzw" << "[" << transform.rotation.x << transform.rotation.y <<
    transform.rotation.z << transform.rotation.w << "]";
  storage << "}";
  storage << "reference_frame" << calibration.tray_from_camera.header.frame_id;
  storage << "camera_optical_frame" << calibration.tray_from_camera.child_frame_id;
  storage << "camera_info_topic" << calibration.camera_info_topic;
  storage << "camera_info" << "{" << "width" << static_cast<int>(camera.width) <<
    "height" << static_cast<int>(camera.height) << "distortion_model" << camera.distortion_model;
  storage << "k" << "[";
  for (const auto value : camera.k) {storage << value;}
  storage << "]" << "d" << "[";
  for (const auto value : camera.d) {storage << value;}
  storage << "]" << "r" << "[";
  for (const auto value : camera.r) {storage << value;}
  storage << "]" << "p" << "[";
  for (const auto value : camera.p) {storage << value;}
  storage << "]" << "}";
  return storage.releaseAndGetString();
}

}  // namespace

bool CalibrationFile::load(
  const std::string & path, const std::string & configured_camera_info_topic,
  CalibrationFileData & output, std::string & diagnostic)
{
  try {
    if (path.empty()) {
      diagnostic = "vision.calibration_file is empty";
      return false;
    }
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
      diagnostic = "selected Calibration file is unavailable: " + path;
      return false;
    }
    CalibrationFileData candidate;
    const auto transform_node = storage["tray_T_camera"];
    std::vector<double> translation;
    std::vector<double> quaternion;
    if (!transform_node.isMap() || !read_vector(transform_node, "translation", 3, translation) ||
      !read_vector(transform_node, "quaternion_xyzw", 4, quaternion))
    {
      diagnostic = "selected Calibration file has an invalid tray_T_camera transform";
      return false;
    }
    candidate.tray_from_camera.transform.translation.x = translation[0];
    candidate.tray_from_camera.transform.translation.y = translation[1];
    candidate.tray_from_camera.transform.translation.z = translation[2];
    candidate.tray_from_camera.transform.rotation.x = quaternion[0];
    candidate.tray_from_camera.transform.rotation.y = quaternion[1];
    candidate.tray_from_camera.transform.rotation.z = quaternion[2];
    candidate.tray_from_camera.transform.rotation.w = quaternion[3];
    storage["reference_frame"] >> candidate.tray_from_camera.header.frame_id;
    storage["camera_optical_frame"] >> candidate.tray_from_camera.child_frame_id;
    storage["camera_info_topic"] >> candidate.camera_info_topic;
    const auto camera_node = storage["camera_info"];
    int width = 0;
    int height = 0;
    camera_node["width"] >> width;
    camera_node["height"] >> height;
    camera_node["distortion_model"] >> candidate.camera_info.distortion_model;
    candidate.camera_info.width = width > 0 ? static_cast<uint32_t>(width) : 0U;
    candidate.camera_info.height = height > 0 ? static_cast<uint32_t>(height) : 0U;
    std::vector<double> k, d, r, p;
    if (!camera_node.isMap() || !read_vector(camera_node, "k", 9, k) ||
      !read_vector(camera_node, "d", camera_node["d"].size(), d) ||
      !read_vector(camera_node, "r", 9, r) || !read_vector(camera_node, "p", 12, p) ||
      !assign_array(k, candidate.camera_info.k) || !assign_array(r, candidate.camera_info.r) ||
      !assign_array(p, candidate.camera_info.p) || !valid_transform(candidate.tray_from_camera.transform) ||
      candidate.camera_info.width == 0 || candidate.camera_info.height == 0 ||
      candidate.camera_info.distortion_model.empty() ||
      candidate.tray_from_camera.header.frame_id != "Tray" ||
      candidate.tray_from_camera.child_frame_id.empty() ||
      candidate.camera_info_topic.empty() || candidate.camera_info_topic != configured_camera_info_topic)
    {
      diagnostic = "selected Calibration file is invalid or incompatible with configured camera identity";
      return false;
    }
    candidate.camera_info.d = std::move(d);
    output = std::move(candidate);
    diagnostic.clear();
    return true;
  } catch (const cv::Exception & error) {
    diagnostic = std::string("selected Calibration file is malformed: ") + error.what();
  } catch (const std::exception & error) {
    diagnostic = std::string("selected Calibration file could not be read: ") + error.what();
  }
  return false;
}

bool CalibrationFile::compatible(
  const CalibrationFileData & calibration, const sensor_msgs::msg::CameraInfo & camera_info,
  const std::string & configured_camera_info_topic, const std::string & camera_frame,
  std::string & diagnostic)
{
  const auto & stored = calibration.camera_info;
  bool compatible = calibration.camera_info_topic == configured_camera_info_topic &&
    calibration.tray_from_camera.header.frame_id == "Tray" &&
    calibration.tray_from_camera.child_frame_id == camera_frame &&
    camera_info.header.frame_id == camera_frame && stored.width == camera_info.width &&
    stored.height == camera_info.height && stored.distortion_model == camera_info.distortion_model &&
    stored.d.size() == camera_info.d.size();
  for (size_t i = 0; compatible && i < stored.k.size(); ++i) {compatible = close(stored.k[i], camera_info.k[i]);}
  for (size_t i = 0; compatible && i < stored.r.size(); ++i) {compatible = close(stored.r[i], camera_info.r[i]);}
  for (size_t i = 0; compatible && i < stored.p.size(); ++i) {compatible = close(stored.p[i], camera_info.p[i]);}
  for (size_t i = 0; compatible && i < stored.d.size(); ++i) {compatible = close(stored.d[i], camera_info.d[i]);}
  if (!compatible) {
    diagnostic = "selected Calibration file is incompatible with active camera frame or CameraInfo";
    return false;
  }
  diagnostic.clear();
  return true;
}

bool CalibrationFile::save_new_sibling(
  const std::string & configured_path, const CalibrationFileData & calibration,
  std::string & created_path, std::string & diagnostic)
{
  try {
    if (configured_path.empty() || calibration.camera_info_topic.empty() ||
      calibration.tray_from_camera.header.frame_id != "Tray" ||
      calibration.tray_from_camera.child_frame_id.empty() ||
      !valid_transform(calibration.tray_from_camera.transform) ||
      calibration.camera_info.width == 0 || calibration.camera_info.height == 0 ||
      calibration.camera_info.distortion_model.empty())
    {
      diagnostic = "Calibration output data is incomplete or invalid";
      return false;
    }
    const std::filesystem::path selected(configured_path);
    const auto directory = selected.has_parent_path() ? selected.parent_path() : std::filesystem::path(".");
    const auto stem = selected.stem().string();
    const auto extension = selected.extension().empty() ? std::string(".yaml") : selected.extension().string();
    std::filesystem::create_directories(directory);
    const auto contents = encode(calibration);
    for (uint32_t attempt = 0; attempt < 100; ++attempt) {
      std::ostringstream filename;
      filename << stem << '_' << timestamp_suffix();
      if (attempt != 0) {filename << '_' << attempt;}
      filename << extension;
      const auto candidate = directory / filename.str();
      const int descriptor = ::open(candidate.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
      if (descriptor < 0) {
        if (errno == EEXIST) {continue;}
        diagnostic = "cannot create Calibration output file: " + std::string(std::strerror(errno));
        return false;
      }
      size_t written = 0;
      bool success = true;
      while (written < contents.size()) {
        const auto count = ::write(descriptor, contents.data() + written, contents.size() - written);
        if (count <= 0) {success = false; break;}
        written += static_cast<size_t>(count);
      }
      const bool close_success = ::close(descriptor) == 0;
      if (!success || !close_success) {
        std::filesystem::remove(candidate);
        diagnostic = "failed while writing Calibration output file";
        return false;
      }
      created_path = candidate.string();
      diagnostic.clear();
      return true;
    }
    diagnostic = "could not reserve a unique timestamped Calibration output path";
  } catch (const cv::Exception & error) {
    diagnostic = std::string("Calibration output encoding failed: ") + error.what();
  } catch (const std::exception & error) {
    diagnostic = std::string("Calibration output failed: ") + error.what();
  }
  return false;
}

}  // namespace arm_cell_vision
