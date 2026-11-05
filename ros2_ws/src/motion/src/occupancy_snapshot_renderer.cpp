#include "arm_cell_motion/occupancy_snapshot_renderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace arm_cell_motion
{
namespace
{
using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;

std::vector<cv::Point3d> box_vertices(const Primitive & primitive)
{
  if (primitive.shape_type != Primitive::SOLID_PRIMITIVE ||
    primitive.occupancy_class != Primitive::OBSERVED_OCCUPANCY ||
    primitive.shape.type != shape_msgs::msg::SolidPrimitive::BOX ||
    primitive.shape.dimensions.size() != 3U)
  {
    return {};
  }
  const double x = primitive.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] * 0.5;
  const double y = primitive.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] * 0.5;
  const double z = primitive.shape.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] * 0.5;
  if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) ||
    x <= 0.0 || y <= 0.0 || z <= 0.0)
  {
    return {};
  }
  tf2::Quaternion rotation(
    primitive.pose.orientation.x, primitive.pose.orientation.y,
    primitive.pose.orientation.z, primitive.pose.orientation.w);
  const double norm = rotation.length();
  if (!std::isfinite(norm) || norm < 1.0e-9) {return {};}
  rotation.normalize();
  const tf2::Vector3 center(
    primitive.pose.position.x, primitive.pose.position.y, primitive.pose.position.z);
  const std::array<tf2::Vector3, 8> local{{
      {-x, -y, -z}, {x, -y, -z}, {x, y, -z}, {-x, y, -z},
      {-x, -y, z}, {x, -y, z}, {x, y, z}, {-x, y, z}}};
  std::vector<cv::Point3d> points;
  points.reserve(local.size());
  for (const auto & vertex : local) {
    const auto point = tf2::quatRotate(rotation, vertex) + center;
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
      return {};
    }
    points.emplace_back(point.x(), point.y(), point.z());
  }
  return points;
}

std::vector<cv::Point> projected_hull(
  const std::vector<cv::Point3d> & points, bool xz, const cv::Rect & area,
  double minimum_x, double maximum_x, double minimum_y, double maximum_y)
{
  std::vector<cv::Point> pixels;
  pixels.reserve(points.size());
  for (const auto & point : points) {
    const double horizontal = point.x;
    const double vertical = xz ? point.z : point.y;
    const double scale = std::min(
      area.width / (maximum_x - minimum_x), area.height / (maximum_y - minimum_y));
    const double offset_x = (area.width - (maximum_x - minimum_x) * scale) * 0.5;
    const double offset_y = (area.height - (maximum_y - minimum_y) * scale) * 0.5;
    pixels.emplace_back(
      area.x + static_cast<int>(std::lround(offset_x + (horizontal - minimum_x) * scale)),
      area.y + area.height - static_cast<int>(std::lround(offset_y +
        (vertical - minimum_y) * scale)));
  }
  std::vector<cv::Point> hull;
  if (!pixels.empty()) {cv::convexHull(pixels, hull);}
  return hull;
}

double nice_tick(double range)
{
  const double raw = range / 5.0;
  const double power = std::pow(10.0, std::floor(std::log10(raw)));
  const double scaled = raw / power;
  return (scaled <= 1.0 ? 1.0 : scaled <= 2.0 ? 2.0 : scaled <= 5.0 ? 5.0 : 10.0) * power;
}

std::string tick_label(double value, double step)
{
  std::ostringstream label;
  label << std::fixed << std::setprecision(step < 0.1 ? 2 : step < 1.0 ? 1 : 0) << value;
  return label.str();
}

}  // namespace

OccupancySnapshot render_occupancy_snapshot(
  const std::vector<Primitive> & primitives,
  const std::string & acquisition_id, const std::string & selected_candidate_id)
{
  constexpr int width = 1470;
  constexpr int height = 470;
  const cv::Scalar background(250, 250, 250);
  cv::Mat image(height, width, CV_8UC3, background);
  std::ostringstream title;
  title << "Motion occupancy snapshot | acquisition " <<
    (acquisition_id.empty() ? "unavailable" : acquisition_id) << " | selected candidate " <<
    (selected_candidate_id.empty() ? "unavailable" : selected_candidate_id);
  cv::putText(image, title.str(), {24, 34}, cv::FONT_HERSHEY_SIMPLEX, 0.62,
    cv::Scalar(25, 25, 25), 2, cv::LINE_AA);
  cv::putText(image, "Planning frame: base_link | XY: +Y up | XZ: +Z up | dimensions in meters",
    {24, 62}, cv::FONT_HERSHEY_SIMPLEX, 0.43, cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
  cv::putText(image, "Observed occupancy: orange | each polygon is the projection of a planning BOX",
    {24, 88}, cv::FONT_HERSHEY_SIMPLEX, 0.41, cv::Scalar(45, 45, 45), 1, cv::LINE_AA);

  const std::array<cv::Rect, 2> panels{{{24, 110, 700, 330}, {746, 110, 700, 330}}};
  const std::array<std::string, 2> labels{{"XY projection (X,Y)", "XZ projection (X,Z)"}};
  std::vector<std::vector<cv::Point3d>> vertices;
  vertices.reserve(primitives.size());
  bool invalid = false;
  for (const auto & primitive : primitives) {
    auto points = box_vertices(primitive);
    if (points.empty()) {invalid = true; continue;}
    vertices.push_back(std::move(points));
  }
  std::array<double, 2> min_x{{std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()}};
  std::array<double, 2> max_x{{-std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()}};
  std::array<double, 2> min_y{{std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()}};
  std::array<double, 2> max_y{{-std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()}};
  for (const auto & shape : vertices) {
    for (const auto & point : shape) {
      const std::array<double, 2> vertical{{point.y, point.z}};
      for (std::size_t panel = 0; panel < 2U; ++panel) {
        min_x[panel] = std::min(min_x[panel], point.x);
        max_x[panel] = std::max(max_x[panel], point.x);
        min_y[panel] = std::min(min_y[panel], vertical[panel]);
        max_y[panel] = std::max(max_y[panel], vertical[panel]);
      }
    }
  }
  for (std::size_t panel = 0; panel < 2U; ++panel) {
    if (!std::isfinite(min_x[panel])) {min_x[panel] = min_y[panel] = -0.05;}
    if (!std::isfinite(max_x[panel])) {max_x[panel] = max_y[panel] = 0.05;}
    if (max_x[panel] - min_x[panel] < 0.05) {
      const double center = (max_x[panel] + min_x[panel]) * 0.5;
      min_x[panel] = center - 0.025;
      max_x[panel] = center + 0.025;
    }
    if (max_y[panel] - min_y[panel] < 0.05) {
      const double center = (max_y[panel] + min_y[panel]) * 0.5;
      min_y[panel] = center - 0.025;
      max_y[panel] = center + 0.025;
    }
    const double pad_x = (max_x[panel] - min_x[panel]) * 0.05;
    const double pad_y = (max_y[panel] - min_y[panel]) * 0.05;
    min_x[panel] -= pad_x;
    max_x[panel] += pad_x;
    min_y[panel] -= pad_y;
    max_y[panel] += pad_y;
  }
  if (primitives.empty()) {
    cv::putText(image, "EMPTY MAP: planning used zero candidate occupancy objects",
      {250, 280}, cv::FONT_HERSHEY_SIMPLEX, 0.66, cv::Scalar(45, 45, 45), 2, cv::LINE_AA);
  }
  for (size_t panel_index = 0; panel_index < panels.size(); ++panel_index) {
    const auto & panel = panels[panel_index];
    cv::putText(image, labels[panel_index], {panel.x, panel.y - 10},
      cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(30, 30, 30), 1, cv::LINE_AA);
    const cv::Rect area(panel.x + 42, panel.y + 8, panel.width - 56, panel.height - 34);
    cv::rectangle(image, area, cv::Scalar(190, 190, 190), 1);
    const double x_tick = nice_tick(max_x[panel_index] - min_x[panel_index]);
    const double y_tick = nice_tick(max_y[panel_index] - min_y[panel_index]);
    const double scale = std::min(
      area.width / (max_x[panel_index] - min_x[panel_index]),
      area.height / (max_y[panel_index] - min_y[panel_index]));
    const double offset_x =
      (area.width - (max_x[panel_index] - min_x[panel_index]) * scale) * 0.5;
    const double offset_y =
      (area.height - (max_y[panel_index] - min_y[panel_index]) * scale) * 0.5;
    const auto pixel_x = [&](double value) {
        return area.x + static_cast<int>(std::lround(
          offset_x + (value - min_x[panel_index]) * scale));
      };
    const auto pixel_y = [&](double value) {
        return area.y + area.height - static_cast<int>(std::lround(
          offset_y + (value - min_y[panel_index]) * scale));
      };
    for (double value = std::ceil(min_x[panel_index] / x_tick) * x_tick;
      value <= max_x[panel_index] + x_tick * 1.0e-6; value += x_tick)
    {
      const int x = pixel_x(value);
      cv::line(image, {x, area.y}, {x, area.y + area.height}, cv::Scalar(232, 232, 232), 1);
      cv::putText(image, tick_label(value, x_tick), {x - 18, area.y + area.height + 16},
        cv::FONT_HERSHEY_SIMPLEX, 0.31, cv::Scalar(55, 55, 55), 1, cv::LINE_AA);
    }
    for (double value = std::ceil(min_y[panel_index] / y_tick) * y_tick;
      value <= max_y[panel_index] + y_tick * 1.0e-6; value += y_tick)
    {
      const int y = pixel_y(value);
      cv::line(image, {area.x, y}, {area.x + area.width, y}, cv::Scalar(232, 232, 232), 1);
      cv::putText(image, tick_label(value, y_tick), {area.x - 38, y + 4},
        cv::FONT_HERSHEY_SIMPLEX, 0.31, cv::Scalar(55, 55, 55), 1, cv::LINE_AA);
    }
    cv::putText(image, "X (m), +X right", {area.x + area.width / 2 - 42,
      area.y + area.height + 30}, cv::FONT_HERSHEY_SIMPLEX, 0.32,
      cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
    cv::putText(image, panel_index == 0U ? "Y (m), +Y up" : "Z (m), +Z up",
      {area.x - 40, area.y - 3}, cv::FONT_HERSHEY_SIMPLEX, 0.31,
      cv::Scalar(45, 45, 45), 1, cv::LINE_AA);
    for (size_t index = 0; index < vertices.size(); ++index) {
      const auto hull = projected_hull(
        vertices[index], panel_index == 1U, area, min_x[panel_index], max_x[panel_index],
        min_y[panel_index], max_y[panel_index]);
      if (hull.size() < 3U) {continue;}
      cv::fillConvexPoly(image, hull, cv::Scalar(20, 135, 245), cv::LINE_AA);
      cv::polylines(image, hull, true, cv::Scalar(10, 80, 170), 2, cv::LINE_AA);
      cv::putText(image, "O" + std::to_string(index), hull.front(),
        cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
    }
  }
  if (invalid) {
    cv::putText(image, "RENDER WARNING: unsupported or invalid planning primitive",
      {24, 462}, cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(20, 20, 200), 1, cv::LINE_AA);
  }
  std::vector<std::uint8_t> png;
  if (!cv::imencode(".png", image, png) || png.empty()) {
    return {{}, "png_encoding_failed"};
  }
  return {std::move(png), invalid ? "rendered_with_unsupported_primitive" :
    primitives.empty() ? "empty_map_rendered" : "rendered"};
}

void save_occupancy_snapshot_png(
  const std::filesystem::path & destination, const OccupancySnapshot & snapshot)
{
  if (snapshot.png.empty()) {throw std::runtime_error("snapshot has no PNG data");}
  std::filesystem::create_directories(destination.parent_path());
  auto temporary = destination;
  temporary += ".tmp";
  {
    std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char *>(snapshot.png.data()),
      static_cast<std::streamsize>(snapshot.png.size()));
    output.flush();
    if (!output) {throw std::runtime_error("PNG write failed");}
  }
  std::filesystem::rename(temporary, destination);
}

}  // namespace arm_cell_motion
