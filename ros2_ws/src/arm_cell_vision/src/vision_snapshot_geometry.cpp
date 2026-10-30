#include "vision_snapshot_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <numeric>
#include <unordered_set>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace arm_cell_vision
{
namespace snapshot_detail
{
namespace
{
cv::Size geometry_label_size(const std::string & text)
{
  std::istringstream lines(text);
  std::string line;
  cv::Size size;
  while (std::getline(lines, line)) {
    int baseline = 0;
    const auto measured = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    size.width = std::max(size.width, measured.width);
    size.height += measured.height + baseline;
  }
  return size + cv::Size(6, 6);
}
}  // namespace

std::string geometry_plane_annotation(size_t id, const std::string & status)
{
  const auto display_status = status == "horizontal" ? "Horizontal" :
    status == "mixed_or_inclined" ? "Mixed/inclined" :
    status == "unresolved_insufficient_support" ? "Insufficient support" :
    status.empty() ? "Not collected" : status;
  return "P" + std::to_string(id) + "\n" + display_status;
}

cv::Rect reserve_geometry_label(
  const std::string & text, cv::Point center, const cv::Rect & available,
  std::vector<cv::Rect> & occupied)
{
  const auto size = geometry_label_size(text);
  const cv::Rect box(center.x - size.width / 2, center.y - size.height / 2,
    size.width, size.height);
  if ((box & available) != box || std::any_of(occupied.begin(), occupied.end(),
    [&](const auto & used) {return !(used & box).empty();})) {return {};}
  occupied.push_back(box);
  return box;
}

void render_geometry(
  const VisionDetectorSnapshotJob & job,
  std::vector<VisionDetectorSnapshotJob::CuboidStageImage> & images)
{
  using Message = arm_cell_interfaces::msg::VisionDetectorSnapshot;
  images.clear();
  const auto & detection = job.detection_result();
  const auto replay = detection.cuboid_replay_evidence;
  const bool cuboid_terminal = job.algorithm == "cuboid_top_face" &&
    (job.event_kind == Message::DETECT_TARGET_SUCCESS ||
    job.event_kind == Message::DETECT_TARGET_FAILURE);
  if (cuboid_terminal) {
    const auto add_stage = [&](const std::string & stage, const std::string & filename,
        const std::string & status, const std::vector<uint32_t> & ids = {}) {
        VisionDetectorSnapshotJob::CuboidStageImage output;
        output.stage = stage;
        output.filename = filename;
        output.status = status;
        output.candidate_ids = ids;
        images.push_back(std::move(output));
      };
    const bool grid_available = replay && !replay->height_grid_m.empty() &&
      replay->height_grid_m.type() == CV_64FC1 && replay->resolution_m > 0.0 &&
      replay->rows == static_cast<uint32_t>(replay->height_grid_m.rows) &&
      replay->columns == static_cast<uint32_t>(replay->height_grid_m.cols);
    const auto & patches = detection.cuboid_height_patches;
    const auto & hypotheses = detection.cuboid_hypotheses;
    const bool patch_evidence_complete = !patches.empty() && std::all_of(
      patches.begin(), patches.end(), [](const auto & patch) {
        return patch.observed_cells_xy.size() == patch.cell_count &&
          patch.observed_contours_xy.size() == patch.contour_count &&
          patch.fitted_edges_xy_support.size() == patch.fitted_edge_count;
      });
    // The producer increments this counter in the exact loop iteration that
    // appends one visualization record. Geometry-pass/rejection counters are
    // separate outcomes; they do not establish whether the record was retained.
    std::vector<uint64_t> retained_hypotheses_by_patch(patches.size(), 0);
    bool hypothesis_ids_unique = true;
    std::unordered_set<uint32_t> observed_hypothesis_ids;
    observed_hypothesis_ids.reserve(hypotheses.size());
    for (const auto & hypothesis : hypotheses) {
      if (hypothesis.patch_index >= retained_hypotheses_by_patch.size() ||
        hypothesis.candidate_id == 0)
      {
        hypothesis_ids_unique = false;
        continue;
      }
      ++retained_hypotheses_by_patch[hypothesis.patch_index];
      hypothesis_ids_unique = observed_hypothesis_ids.insert(hypothesis.candidate_id).second &&
        hypothesis_ids_unique;
    }
    bool hypothesis_attempts_match = !patches.empty();
    for (size_t patch_index = 0; patch_index < patches.size(); ++patch_index) {
      hypothesis_attempts_match = hypothesis_attempts_match &&
        patches[patch_index].candidate_evaluation_attempt_count ==
        retained_hypotheses_by_patch[patch_index];
    }
    const bool hypothesis_evidence_complete = hypothesis_attempts_match && hypothesis_ids_unique;
    std::vector<uint32_t> hypothesis_ids;
    for (const auto & hypothesis : hypotheses) {hypothesis_ids.push_back(hypothesis.candidate_id);}
    std::vector<uint32_t> final_ids;
    for (const auto & candidate : detection.ordered_candidates) {
      if (candidate.confidence_grade != "HIGH" && candidate.confidence_grade != "MEDIUM") {
        continue;
      }
      try {
        const auto id = std::stoul(candidate.rawpart_selected_candidate);
        if (id <= std::numeric_limits<uint32_t>::max()) {
          final_ids.push_back(static_cast<uint32_t>(id));
        }
      } catch (const std::exception &) {}
    }
    for (const auto & hypothesis : hypotheses) {
      if (hypothesis.confidence_grade == "LOW") {final_ids.push_back(hypothesis.candidate_id);}
    }

    if (!grid_available) {
      add_stage("height_patches", "geometry_observation.png", "evidence_unavailable");
      add_stage("geometry_candidates", "geometry_interpretation.png", "evidence_unavailable");
      add_stage("final_candidates", "geometry_interpretation.png", "evidence_unavailable");
    } else {
      const auto & grid = replay->height_grid_m;
      double min_h = std::numeric_limits<double>::infinity();
      double max_h = -std::numeric_limits<double>::infinity();
      for (int y = 0; y < grid.rows; ++y) for (int x = 0; x < grid.cols; ++x) {
        const double h = grid.at<double>(y, x);
        if (std::isfinite(h)) {min_h = std::min(min_h, h); max_h = std::max(max_h, h);}
      }
      const int scale = std::max(1, std::min(4, 960 / std::max(grid.cols, grid.rows)));
      int left_cells = 0, right_cells = grid.cols - 1, bottom_cells = 0, top_cells = grid.rows - 1;
      const auto include_point = [&](double x, double y) {
          if (!std::isfinite(x) || !std::isfinite(y)) {return;}
          const double col = (x - replay->origin_x_m) / replay->resolution_m;
          const double row = (y - replay->origin_y_m) / replay->resolution_m;
          // Invalid diagnostic coordinates must not cause unbounded allocation.
          if (std::abs(col) > 10000 || std::abs(row) > 10000) {return;}
          left_cells = std::min(left_cells, static_cast<int>(std::floor(col)) - 2);
          right_cells = std::max(right_cells, static_cast<int>(std::ceil(col)) + 2);
          bottom_cells = std::min(bottom_cells, static_cast<int>(std::floor(row)) - 2);
          top_cells = std::max(top_cells, static_cast<int>(std::ceil(row)) + 2);
        };
      for (const auto & h : hypotheses) {
        if (!std::isfinite(h.yaw_rad) || !std::isfinite(h.length_m) || !std::isfinite(h.width_m)) {continue;}
        for (const auto & sign : std::array<cv::Point2d, 4>{{{-1,-1},{1,-1},{1,1},{-1,1}}}) {
          include_point(h.center_x_m + std::cos(h.yaw_rad) * sign.x * h.length_m / 2 -
            std::sin(h.yaw_rad) * sign.y * h.width_m / 2,
            h.center_y_m + std::sin(h.yaw_rad) * sign.x * h.length_m / 2 +
            std::cos(h.yaw_rad) * sign.y * h.width_m / 2);
        }
      }
      const int map_width = (right_cells - left_cells + 1) * scale;
      const int map_height = (top_cells - bottom_cells + 1) * scale;
      const int panel_header = 34, gap = 10;
      const int panel_width = std::max(map_width + 20, 240);
      const int panel_height = map_height + panel_header + 12;
      const auto point_on_map = [&](double x, double y) {
          const int col = static_cast<int>(std::lround((x - replay->origin_x_m) / replay->resolution_m));
          const int row = top_cells - static_cast<int>(std::lround(
              (y - replay->origin_y_m) / replay->resolution_m));
          return cv::Point((panel_width - map_width) / 2 + (col - left_cells) * scale,
            panel_header + row * scale);
        };
      cv::Mat observation;
      std::array<cv::Mat, 3> panels{{
        cv::Mat(panel_height, panel_width, CV_8UC3, cv::Scalar(35, 35, 35)),
        cv::Mat(panel_height, panel_width, CV_8UC3, cv::Scalar(35, 35, 35)),
        cv::Mat(panel_height, panel_width, CV_8UC3, cv::Scalar(35, 35, 35))}};
      cv::Mat normalized(grid.size(), CV_8UC1, cv::Scalar(0));
      cv::Mat valid(grid.size(), CV_8UC1, cv::Scalar(0));
      for (int y = 0; y < grid.rows; ++y) for (int x = 0; x < grid.cols; ++x) {
        const double h = grid.at<double>(y, x);
        if (!std::isfinite(h)) {continue;}
        valid.at<uint8_t>(y, x) = 255;
        const double range = max_h - min_h;
        normalized.at<uint8_t>(y, x) = range > 1e-9 ?
          static_cast<uint8_t>(std::lround(255.0 * (h - min_h) / range)) : 128;
      }
      cv::Mat height;
      cv::applyColorMap(normalized, height, cv::COLORMAP_TURBO);
      height.setTo(cv::Scalar(55, 55, 55), valid == 0);
      cv::flip(height, height, 0);
      cv::resize(height, height, cv::Size(grid.cols * scale, grid.rows * scale), 0, 0, cv::INTER_NEAREST);
      cv::copyMakeBorder(height, height, (top_cells - grid.rows + 1) * scale, -bottom_cells * scale,
        -left_cells * scale, (right_cells - grid.cols + 1) * scale,
        cv::BORDER_CONSTANT, cv::Scalar(55, 55, 55));
      height.copyTo(panels[0](cv::Rect((panel_width - map_width) / 2, panel_header, map_width, map_height)));
      const auto patch_color = [](size_t i) {
          // Bright identity colors stay distinct from the dark map background.
          static const std::array<cv::Scalar, 8> palette{{
            {240, 180, 60}, {80, 210, 250}, {200, 100, 240}, {100, 235, 120},
            {250, 130, 130}, {100, 240, 230}, {240, 180, 220}, {170, 210, 250}}};
          return palette[i % palette.size()];
        };
      // Label placement affects presentation only; diagnostic IDs and array order stay intact.
      std::array<std::vector<cv::Rect>, 3> label_boxes;
      const cv::Rect label_area((panel_width - map_width) / 2 + 3, panel_header + 3,
        std::max(0, map_width - 6), std::max(0, map_height - 6));
      const auto map_label = [&](cv::Mat & panel, const std::string & text, cv::Point at,
          size_t layer, const cv::Rect & available) {
          const auto box = reserve_geometry_label(text, at, label_area & available, label_boxes[layer]);
          if (box.empty()) {return;}
          std::istringstream lines(text);
          std::string line;
          int y = box.y + 3;
          while (std::getline(lines, line)) {
            int baseline = 0;
            const auto size = cv::getTextSize(line, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
            const cv::Point origin(box.x + (box.width - size.width) / 2, y + size.height);
            cv::putText(panel, line, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
            cv::putText(panel, line, origin, cv::FONT_HERSHEY_SIMPLEX, 0.4,
              observed_color, 1, cv::LINE_AA);
            y += size.height + baseline;
          }
        };
      for (size_t i = 0; i < patches.size(); ++i) {
        if (patch_evidence_complete) {
          for (const auto & cell : patches[i].observed_cells_xy) {
            const auto at = point_on_map(cell.x, cell.y);
            const cv::Rect cell_rect(at, cv::Size(scale, scale));
            const auto clipped = cell_rect & cv::Rect(0, 0, panels[1].cols, panels[1].rows);
            if (clipped.empty()) {continue;}
            const bool residual = patches[i].plane_status == "unresolved_insufficient_support";
            // Retain every observed cell and its identity tint, with quiet residuals.
            cv::Mat tint(clipped.size(), CV_8UC3, patch_color(i));
            const double weight = residual ? 0.4 : 0.6;
            cv::addWeighted(panels[1](clipped), 1.0 - weight, tint, weight, 0, panels[1](clipped));
          }
        }
      }
      std::vector<size_t> plane_order(patches.size());
      std::iota(plane_order.begin(), plane_order.end(), 0);
      std::stable_sort(plane_order.begin(), plane_order.end(), [&](size_t a, size_t b) {
        const bool a_residual = patches[a].plane_status == "unresolved_insufficient_support";
        const bool b_residual = patches[b].plane_status == "unresolved_insufficient_support";
        if (a_residual != b_residual) {return !a_residual;}
        return patches[a].cell_count > patches[b].cell_count;
      });
      for (const auto i : plane_order) {
        if (!patch_evidence_complete || patches[i].observed_cells_xy.empty()) {continue;}
        std::vector<cv::Point> cells;
        cv::Point2d center;
        for (const auto & cell : patches[i].observed_cells_xy) {
          const auto at = point_on_map(cell.x, cell.y);
          cells.push_back(at); center += cv::Point2d(at);
        }
        center *= 1.0 / cells.size();
        const auto bounds = cv::boundingRect(cells);
        // Sparse slivers should not acquire a label merely through a large bounding box.
        const auto text = geometry_plane_annotation(i, patches[i].plane_status);
        const auto size = geometry_label_size(text);
        if (cells.size() * scale * scale < static_cast<size_t>(size.area())) {continue;}
        map_label(panels[1], text, center, 0, bounds);
      }
      const cv::Mat plane_background = panels[1].clone();
      // Boundary records precede plane splitting: their contours and fitted
      // segments are observed, whereas plane partition edges may be inferred.
      // Legacy records without boundary evidence remain explicitly unavailable.
      const auto & boundaries = detection.cuboid_boundaries;
      size_t observed_contour_count = 0;
      size_t observed_fitted_edge_count = 0;
      for (const auto & boundary : boundaries) {
        observed_contour_count += boundary.observed_contours_xy.size();
        observed_fitted_edge_count += boundary.fitted_edges_xy_support.size();
      }
      const bool boundary_evidence_available = !boundaries.empty() &&
        std::all_of(boundaries.begin(), boundaries.end(),
        [](const auto & boundary) {
          return boundary.observed_contours_xy.size() == boundary.contour_count &&
            boundary.fitted_edges_xy_support.size() == boundary.fitted_edge_count;
        });
      for (const auto & boundary : boundaries) {
        for (const auto & edge : boundary.fitted_edges_xy_support) {
          evidence_line(panels[2], point_on_map(edge[0], edge[1]), point_on_map(edge[2], edge[3]),
            cv::Scalar(255, 190, 80), false, 2);
        }
        for (size_t contour_index = 0; contour_index < boundary.observed_contours_xy.size();
          ++contour_index)
        {
          const auto & contour = boundary.observed_contours_xy[contour_index];
          std::vector<cv::Point> points;
          for (const auto & pt : contour) {points.push_back(point_on_map(pt.x, pt.y));}
          if (points.size() > 1) {
            cv::polylines(panels[2], points, true, observed_color, 1, cv::LINE_AA);
            map_label(panels[2], "B" + std::to_string(boundary.source_boundary_id) + "/C" +
              std::to_string(contour_index + 1), points.front(), 2, label_area);
          }
        }
        for (const auto & corner : boundary.corners_xy_turn) {
          cv::drawMarker(panels[2], point_on_map(corner[0], corner[1]), anchor_color,
            cv::MARKER_CROSS, 7, 1, cv::LINE_AA);
        }
      }
      if (!patch_evidence_complete) {
        panels[1].setTo(cv::Scalar(35, 35, 35));
      }
      const cv::Rect map_rect((panel_width - map_width) / 2, panel_header, map_width, map_height);
      const auto orient_panel = [&](const cv::Mat & panel, const std::string & title,
          const std::string & status = std::string{},
          const std::string & summary = std::string{}) {
          const auto oriented_map = rotate_tray_image_to_rgb_orientation(
            panel(map_rect), job, cv::Scalar(20, 20, 20));
          const int oriented_header = 58;
          cv::Mat output(oriented_map.rows + oriented_header + 12,
            std::max(oriented_map.cols + 20, 640), CV_8UC3, cv::Scalar(20, 20, 20));
          oriented_map.copyTo(output(cv::Rect((output.cols - oriented_map.cols) / 2, oriented_header,
              oriented_map.cols, oriented_map.rows)));
          const auto header_text = [&](const std::string & text, int y, double preferred) {
              int baseline = 0;
              const auto size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, preferred, 1, &baseline);
              const double text_scale = preferred * std::min(1.0,
                static_cast<double>(output.cols - 16) / std::max(1, size.width));
              cv::putText(output, text, {8, y}, cv::FONT_HERSHEY_SIMPLEX, text_scale,
                cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
            };
          header_text(title, 22, 0.55);
          header_text(status.empty() ? summary : status, 45, 0.42);
          return output;
        };
      const auto height_panel = orient_panel(panels[0], "1 Heightmap", "",
          "Color=relative height | Gray=unobserved");
      const auto boundary_panel = orient_panel(panels[2], "2 Boundary Extraction",
          !boundary_evidence_available ? "Unavailable: original boundary evidence missing" : "",
          "Contours " + std::to_string(observed_contour_count) + " | Fitted edges " +
          std::to_string(observed_fitted_edge_count) + " | White=contour Cyan=edge +=corner");
      const auto join_panels = [&](const cv::Mat & first, const cv::Mat & second) {
          cv::Mat joined(first.rows, first.cols + second.cols + gap, CV_8UC3, cv::Scalar(20, 20, 20));
          first.copyTo(joined(cv::Rect(0, 0, first.cols, first.rows)));
          second.copyTo(joined(cv::Rect(first.cols + gap, 0, second.cols, second.rows)));
          return joined;
        };
      std::map<std::string, size_t> plane_counts;
      for (const auto & plane : patches) {++plane_counts[plane.plane_status];}
      std::ostringstream plane_summary;
      plane_summary << "Horizontal " << plane_counts["horizontal"] << " | Mixed/inclined " <<
        plane_counts["mixed_or_inclined"] << " | Insufficient support " <<
        plane_counts["unresolved_insufficient_support"];
      const auto plane_panel = orient_panel(panels[1], "3 Plane Map",
          !patch_evidence_complete ? "Unavailable: incomplete plane evidence" : "", plane_summary.str());
      observation = join_panels(join_panels(height_panel, boundary_panel), plane_panel);
      std::vector<uint8_t> observation_png;
      try {cv::imencode(".png", observation, observation_png);} catch (const cv::Exception &) {}
      const std::string observation_status = observation_png.empty() ? "render_failed" : "rendered";
      add_stage("height_patches", "geometry_observation.png",
        observation_status == "rendered" && patch_evidence_complete && boundary_evidence_available ? "rendered" :
        observation_status == "rendered" ? "intermediate_evidence_unavailable" : observation_status);
      if (!images.empty()) {images.back().png = observation_png;}

      cv::Mat interpretation;
      cv::Mat all_hypotheses(panel_height, panel_width, CV_8UC3, cv::Scalar(32, 32, 32));
      plane_background.copyTo(all_hypotheses);
      label_boxes[1] = label_boxes[0];  // Keep Plane labels visible beneath overlays.
      size_t rectangle_count = 0;
      size_t partition_edge_count = 0;
      bool partition_evidence_available = false;
      for (const auto & plane : patches) {
        rectangle_count += plane.rectangle_regions.size();
        partition_edge_count += plane.inferred_partition_edges.size();
        partition_evidence_available = partition_evidence_available ||
          !plane.partition_status.empty() || !plane.rectangle_regions.empty() ||
          !plane.inferred_partition_edges.empty();
      }
      // Stage 4 reuses only retained Runtime partition geometry over the exact
      // Stage 3 plane image. No rectangles or split lines are reconstructed here.
      for (const auto & boundary : boundaries) {
        for (const auto & contour : boundary.observed_contours_xy) {
          std::vector<cv::Point> points;
          for (const auto & pt : contour) {points.push_back(point_on_map(pt.x, pt.y));}
          if (points.size() > 1) {cv::polylines(all_hypotheses, points, true, observed_color, 1, cv::LINE_AA);}
        }
        for (const auto & edge : boundary.fitted_edges_xy_support) {
          evidence_line(all_hypotheses, point_on_map(edge[0], edge[1]),
            point_on_map(edge[2], edge[3]), cv::Scalar(255, 190, 80), false, 2);
        }
      }
      for (size_t plane_index = 0; plane_index < patches.size(); ++plane_index) {
        const auto & plane = patches[plane_index];
        for (const auto & edge : plane.inferred_partition_edges) {
          evidence_line(all_hypotheses, point_on_map(edge[0], edge[1]),
            point_on_map(edge[2], edge[3]), inferred_color, true, 2);
        }
        for (size_t ri = 0; ri < plane.rectangle_regions.size(); ++ri) {
          const auto & rect = plane.rectangle_regions[ri];
          const cv::Point2d u(std::cos(rect[2]), std::sin(rect[2])), v(-u.y, u.x);
          std::array<cv::Point, 4> points;
          const std::array<cv::Point2d, 4> signs{{{-1,-1},{1,-1},{1,1},{-1,1}}};
          for (size_t side = 0; side < points.size(); ++side) {
            const auto pt = cv::Point2d(rect[0], rect[1]) + u * (signs[side].x * rect[3] / 2) +
              v * (signs[side].y * rect[4] / 2);
            points[side] = point_on_map(pt.x, pt.y);
          }
          for (size_t side = 0; side < points.size(); ++side) {
            evidence_line(all_hypotheses, points[side], points[(side + 1) % points.size()],
              inferred_color, true, 2);
          }
          map_label(all_hypotheses, "B" + std::to_string(plane.source_boundary_id) + "/P" +
            std::to_string(plane_index) + "/R" + std::to_string(ri + 1),
            point_on_map(rect[0], rect[1]), 1, label_area);
        }
      }
      const auto draw_rect = [&](cv::Mat & panel, double cx, double cy, double yaw,
          double length, double width, const cv::Scalar & color, int thickness) {
          if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(yaw) ||
            !std::isfinite(length) || !std::isfinite(width) || length <= 0 || width <= 0) {return;}
          const cv::Point2d u(std::cos(yaw), std::sin(yaw)), v(-u.y, u.x);
          std::array<cv::Point, 4> points;
          const std::array<cv::Point2d, 4> signs{{{-1,-1},{1,-1},{1,1},{-1,1}}};
          for (size_t j = 0; j < 4; ++j) {
            const auto pt = cv::Point2d(cx, cy) + u * (signs[j].x * length / 2) + v * (signs[j].y * width / 2);
            points[j] = point_on_map(pt.x, pt.y);
          }
          for (size_t j = 0; j < 4; ++j) {
            evidence_line(panel, points[j], points[(j + 1) % 4], color, true, thickness);
          }
          cv::arrowedLine(panel, point_on_map(cx, cy),
            point_on_map(cx + u.x * length / 3, cy + u.y * length / 3), color, 1, cv::LINE_AA);
        };
      if (hypothesis_evidence_complete) {
        for (const auto & boundary : detection.cuboid_boundaries) {
          for (const auto & edge : boundary.fitted_edges_xy_support) {
            evidence_line(all_hypotheses, point_on_map(edge[0], edge[1]),
              point_on_map(edge[2], edge[3]), observed_color, false, 2);
          }
        }
        // Presentation order keeps rejected edges visible over alternatives, then emphasizes SELECTED.
        for (const int pass : {0, 1, 2}) {
          for (const auto & h : hypotheses) {
            const bool selected = h.selection_state == "SELECTED";
            const int layer = selected ? 2 : h.selection_state == "REJECTED" ? 1 : 0;
            if (layer != pass) {continue;}
            const auto base = h.selection_state == "REJECTED" ? rejected_color :
              h.selection_state == "UNRESOLVED" ? cv::Scalar(220, 180, 90) :
              h.selection_state == "ALTERNATIVE" ? cv::Scalar(190, 130, 200) : patch_color(h.candidate_id);
            const auto color = selected ? base : base * 0.55;
            draw_rect(all_hypotheses, h.center_x_m, h.center_y_m, h.yaw_rad,
              h.length_m, h.width_m, color, selected ? 3 : 1);
            cv::drawMarker(all_hypotheses, point_on_map(h.anchor_x_m, h.anchor_y_m),
              selected ? anchor_color : anchor_color * 0.45, cv::MARKER_CROSS,
              selected ? 7 : 5, 1, cv::LINE_AA);
          }
        }
        label_boxes[2] = label_boxes[1];
        for (const bool selected_pass : {true, false}) {
          for (const auto & h : hypotheses) {
            if ((h.selection_state == "SELECTED") != selected_pass) {continue;}
            map_label(all_hypotheses, "#" + std::to_string(h.candidate_id) + " " + h.selection_state,
              point_on_map(h.center_x_m, h.center_y_m), 2, label_area);
          }
        }
      }
      const auto draw_outline = [&](cv::Mat & panel, const auto & points,
          const cv::Scalar & color, bool inferred) {
          std::vector<cv::Point> projected;
          for (const auto & point : points) {projected.push_back(point_on_map(point.x, point.y));}
          if (projected.size() < 2) {return;}
          for (size_t edge = 0; edge < projected.size(); ++edge) {
            const auto a = projected[edge], b = projected[(edge + 1) % projected.size()];
            evidence_line(panel, a, b, color, inferred, 3);
          }
        };
      bool final_candidates_complete = hypothesis_evidence_complete &&
        (detection.has_rawpart_diagnostics || !detection.ordered_candidates.empty());
      cv::Mat final_selection = plane_background.clone();
      label_boxes[2] = label_boxes[0];  // Reserve the shared Plane labels in this panel.
      size_t rendered_returned_outlines = 0;
      for (const auto & candidate : detection.ordered_candidates) {
        if (candidate.confidence_grade != "HIGH" && candidate.confidence_grade != "MEDIUM") {
          final_candidates_complete = false;
          continue;
        }
        const auto diagnostic = std::find_if(detection.rawpart_candidate_diagnostics.begin(),
          detection.rawpart_candidate_diagnostics.end(), [&](const auto & item) {
            return std::to_string(item.candidate_id) == candidate.rawpart_selected_candidate &&
              item.fitted_outline_support.size() > 2;
          });
        if (diagnostic == detection.rawpart_candidate_diagnostics.end()) {
          final_candidates_complete = false;
          continue;
        }
        if (!hypothesis_evidence_complete) {continue;}
        // Availability is per returned contour; it never changes Runtime eligibility.
        ++rendered_returned_outlines;
        const auto color = confidence_color(candidate.confidence_grade);
        draw_outline(final_selection, diagnostic->fitted_outline_support, color,
          !candidate.has_observed_rectangle);
        const auto fit = std::find_if(hypotheses.begin(), hypotheses.end(), [&](const auto & h) {
            return std::to_string(h.candidate_id) == candidate.rawpart_selected_candidate;
          });
        if (fit != hypotheses.end()) {
          cv::arrowedLine(final_selection, point_on_map(fit->center_x_m, fit->center_y_m),
            point_on_map(fit->center_x_m + std::cos(fit->yaw_rad) * fit->length_m / 3,
              fit->center_y_m + std::sin(fit->yaw_rad) * fit->length_m / 3), color, 1, cv::LINE_AA);
          map_label(final_selection, "#" + candidate.rawpart_selected_candidate,
            point_on_map(diagnostic->fitted_outline_support.front().x,
              diagnostic->fitted_outline_support.front().y), 2, label_area);
        }
      }
      std::map<std::string, size_t> counts;
      for (const auto & h : hypotheses) {++counts[h.selection_state];}
      std::ostringstream hypothesis_summary;
      hypothesis_summary << (partition_evidence_available ? "Rectangles " +
        std::to_string(rectangle_count) + " | Split edges " + std::to_string(partition_edge_count) :
        "Rectangle Partition unavailable") << " | Solid=observed | Dashed=inferred | All " << hypotheses.size() <<
        " | SELECTED " << counts["SELECTED"] <<
        " | ALTERNATIVE " << counts["ALTERNATIVE"] << " | UNRESOLVED " << counts["UNRESOLVED"] <<
        " | REJECTED " << counts["REJECTED"];
      const auto hypotheses_panel = orient_panel(all_hypotheses, "4 Runtime Hypotheses",
          !hypothesis_evidence_complete ? "Unavailable: incomplete Runtime evidence" : "",
          hypothesis_summary.str());
      const auto selection_title = std::string("5 Final Selection | ") +
        (final_candidates_complete ? "HIGH green / MEDIUM blue" :
        rendered_returned_outlines > 0 ? "Returned outlines partial " +
        std::to_string(rendered_returned_outlines) + "/" + std::to_string(detection.ordered_candidates.size()) :
        "Returned outlines unavailable");
      const auto selection_panel = orient_panel(final_selection, selection_title, "",
          "Returned " + std::to_string(detection.ordered_candidates.size()) +
          " | Outlines " + std::to_string(rendered_returned_outlines) +
          " | HIGH green / MEDIUM blue | Solid=observed Dashed=inferred");
      interpretation = join_panels(hypotheses_panel, selection_panel);
      std::vector<uint8_t> interpretation_png;
      try {cv::imencode(".png", interpretation, interpretation_png);} catch (const cv::Exception &) {}
      const std::string interpretation_status = interpretation_png.empty() ? "render_failed" : "rendered";
      add_stage("geometry_candidates", "geometry_interpretation.png",
        interpretation_status != "rendered" ? interpretation_status :
        hypothesis_evidence_complete ? "rendered" : "evidence_unavailable",
        hypothesis_ids);
      if (!images.empty()) {images.back().png = interpretation_png;}
      add_stage("final_candidates", "geometry_interpretation.png",
        interpretation_status != "rendered" ? interpretation_status :
        final_candidates_complete ? "rendered" : "evidence_unavailable", final_ids);
    }
  }
}
}  // namespace snapshot_detail

}  // namespace arm_cell_vision
