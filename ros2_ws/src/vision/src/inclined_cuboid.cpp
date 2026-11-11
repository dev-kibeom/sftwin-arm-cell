#include "inclined_cuboid.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <map>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/imgproc.hpp>

namespace arm_cell_vision::cuboid_geometry
{
namespace
{
using V = cv::Vec3d;
struct Box
{
  V center, x, y, z, dimensions;
  double coverage;
  std::set<size_t> planes;
  uint32_t id;
  bool ambiguous = false;
  std::set<std::pair<size_t, uint32_t>> sources{};
  std::pair<size_t, uint32_t> reference_source{};
};
V point(const PlaneRegion & plane, cv::Point2d xy)
{
  const auto & c = plane.center_height;
  return {xy.x, xy.y, c[2] - (plane.normal[0] * (xy.x - c[0]) +
    plane.normal[1] * (xy.y - c[1])) / plane.normal[2]};
}
std::vector<V> vertices(const Box & b)
{
  std::vector<V> result;
  for (int i : {-1, 1}) {
    for (int j : {-1, 1}) {
      for (int k : {-1, 1}) {
        result.push_back(
          b.center + b.x * (i * b.dimensions[0] / 2) +
          b.y * (j * b.dimensions[1] / 2) + b.z * (k * b.dimensions[2] / 2));
      }
    }
  }
  return result;
}
bool equivalent(const Box & a, const Box & b, double epsilon)
{
  const auto av = vertices(a), bv = vertices(b);
  return std::all_of(
    av.begin(), av.end(), [&](const V & p) {
      return std::any_of(
        bv.begin(), bv.end(), [&](const V & q) {
          return cv::norm(p - q) <= epsilon;
        });
    });
}
bool overlaps(const Box & a, const Box & b, double epsilon)
{
  const std::array<V, 3> aa{a.x, a.y, a.z}, ba{b.x, b.y, b.z};
  std::vector<V> axes(aa.begin(), aa.end());
  axes.insert(axes.end(), ba.begin(), ba.end());
  for (const auto & x : aa) {
    for (const auto & y : ba) {
      axes.push_back(x.cross(y));
    }
  }
  for (auto axis : axes) {
    const double norm = cv::norm(axis);
    if (norm < 1e-8) {continue;}
    axis /= norm;
    double radius = 0;
    for (int i = 0; i < 3; ++i) {
      radius += (std::abs(axis.dot(aa[i])) * a.dimensions[i] +
        std::abs(axis.dot(ba[i])) * b.dimensions[i]) / 2;
    }
    if (std::abs((a.center - b.center).dot(axis)) >= radius - epsilon) {return false;}
  }
  return true;
}
// Clip a retained observation polygon against the strict interior of a box.
// Positive area there contradicts the proposed solid; absence of an observation
// is never treated as free space.
bool observed_surface_inside(const Box & box, const std::vector<V> & polygon, double epsilon)
{
  auto clipped = polygon;
  const std::array<V, 3> axes{box.x, box.y, box.z};
  for (int axis = 0; axis < 3; ++axis) {
    for (int sign : {-1, 1}) {
      if (clipped.empty()) {return false;}
      const V normal = axes[axis] * sign;
      const double limit = box.dimensions[axis] / 2 - epsilon;
      if (limit <= 0) {return false;}
      std::vector<V> next;
      V previous = clipped.back();
      double before = (previous - box.center).dot(normal) - limit;
      for (const auto & current : clipped) {
        const double after = (current - box.center).dot(normal) - limit;
        if ((before <= 0) != (after <= 0)) {
          next.push_back(previous + (current - previous) * (before / (before - after)));
        }
        if (after <= 0) {next.push_back(current);}
        previous = current; before = after;
      }
      clipped = std::move(next);
    }
  }
  V area(0, 0, 0);
  for (size_t i = 0; i < clipped.size(); ++i) {
    area += (clipped[i] - box.center).cross(clipped[(i + 1) % clipped.size()] - box.center);
  }
  return cv::norm(area) / 2 > epsilon * epsilon;
}
Box candidate_box(const DetectionResult & c)
{
  const auto & p = c.target_pose.pose;
  tf2::Matrix3x3 r(tf2::Quaternion(
      p.orientation.x, p.orientation.y,
      p.orientation.z, p.orientation.w));
  const auto axis = [&](int i) {const auto v = r.getColumn(i); return V(v.x(), v.y(), v.z());};
  const V z = axis(2);
  return {V(p.position.x, p.position.y, p.position.z) - z * (c.estimated_height_m / 2),
    axis(0), axis(1), z,
    V(c.rectangle_long_side_m, c.rectangle_short_side_m, c.estimated_height_m),
    1, {}, 0, false};
}
}

void append_inclined_cuboids(
  DetectionResult & result, const PlaneMap & map, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, uint32_t & next_id, size_t & operations)
{
  const double epsilon = std::max(
    obs.spacing_m,
    profile.cuboid.line_residual_limit_px * obs.spacing_m);
  const double tolerance = profile.cuboid.height_tolerance_m;
  std::vector<Box> boxes;
  std::vector<const PlaneRegion *> planes;
  std::vector<const std::vector<ContourEvidence> *> contours;
  for (size_t i = 0; i < map.regions.size(); ++i) {
    planes.push_back(&map.regions[i]); contours.push_back(&map.region_contours[i]);
  }
  for (size_t i = 0; i < map.recovered_inclined_regions.size(); ++i) {
    planes.push_back(&map.recovered_inclined_regions[i]);
    contours.push_back(&map.recovered_inclined_contours[i]);
  }
  const bool retain_evidence = std::any_of(
    result.cuboid_planes.begin(), result.cuboid_planes.end(),
    [](const auto & p) {return !p.observed_cells_xy.empty();});
  bool exhausted = false;
  for (size_t pi = 0; pi < planes.size(); ++pi) {
    const auto & plane = *planes[pi];
    if (!plane.plane_fitted || plane.normal[2] < 1e-4 || plane.residual_m > tolerance ||
      plane.status != "mixed_or_inclined") {continue;}
    const V z = plane.normal;
    for (const auto & contour : *contours[pi]) {
      if (contour.points.size() < 3) {continue;}
      for (size_t li = 0; li < contour.lines.size(); ++li) {
        const auto & first = contour.lines[li];
        if (!first.observed) {continue;}
        for (size_t lj = li + 1; lj < contour.lines.size(); ++lj) {
          const auto & second = contour.lines[lj];
          if (!second.observed) {continue;}
          const V a = point(plane, first.first), b = point(plane, first.last);
          const V c = point(plane, second.first), d = point(plane, second.last);
          const double al = cv::norm(b - a), bl = cv::norm(d - c);
          if (al <= 2 * epsilon || bl <= 2 * epsilon) {continue;}
          V u = (b - a) / al, v = (d - c) / bl;
          if (std::abs(u.dot(v)) > std::sin(std::atan2(2 * epsilon, std::min(al, bl)))) {continue;}
          // Intersect fitted lines in their common plane; observed endpoints must
          // reach the corner. A partition boundary cannot create an anchor.
          const V perpendicular = z.cross(v);
          const double along = (c - a).dot(perpendicular) / u.dot(perpendicular);
          const V anchor = a + u * along;
          if (std::min(cv::norm(anchor - a), cv::norm(anchor - b)) > 2 * epsilon ||
            std::min(cv::norm(anchor - c), cv::norm(anchor - d)) > 2 * epsilon) {continue;}
          if ((a + b - anchor * 2).dot(u) < 0) {u = -u;}
          if ((c + d - anchor * 2).dot(v) < 0) {v = -v;}
          const V probe = anchor + (u + v) * (2 * epsilon);
          if (cv::pointPolygonTest(
              contour.points,
              cv::Point2f(
                (probe[0] - obs.origin.x) / obs.spacing_m,
                (probe[1] - obs.origin.y) / obs.spacing_m), false) < 0) {continue;}
          for (int thickness_axis = 0; thickness_axis < 3; ++thickness_axis) {
            int length_axis = (thickness_axis + 1) % 3, width_axis = (thickness_axis + 2) % 3;
            if (profile.dimensions_m[length_axis] < profile.dimensions_m[width_axis]) {
              std::swap(length_axis, width_axis);
            }
            for (int mapping = 0; mapping < 2; ++mapping) {
              if (!operations) {exhausted = true; break;}
              --operations;
              V x = mapping ? v : u;
              const V y = z.cross(x);
              const double sign = y.dot(mapping ? u : v) >= 0 ? 1 : -1;
              const double length = profile.dimensions_m[length_axis];
              const double width = profile.dimensions_m[width_axis];
              const double thickness = profile.dimensions_m[thickness_axis];
              const V face_center = anchor + x * (length / 2) + y * (sign * width / 2);
              Box box{face_center - z * (thickness / 2), x, y, z,
                V(length, width, thickness), 0, {pi}, next_id++, false};
              box.reference_source = {pi, contour.outline_id};
              box.sources.insert(box.reference_source);
              bool valid = true;
              // Retained boundary vertices are the observation; never fill or
              // reselect Depth cells to make a recipe fit.
              std::vector<cv::Point2f> local_boundary;
              for (const auto & pixel : contour.points) {
                const auto xy = obs.origin + cv::Point2d(pixel) * obs.spacing_m;
                const V offset = point(plane, xy) - face_center;
                const double lx = offset.dot(x), ly = offset.dot(y);
                valid &= std::abs(lx) <= length / 2 + epsilon &&
                  std::abs(ly) <= width / 2 + epsilon;
                local_boundary.emplace_back(lx, ly);
              }
              if (!valid) {continue;}
              if (contour.vertices.size() == 4 && cv::isContourConvex(contour.vertices) &&
                contour.lines.size() == 4 && std::all_of(
                  contour.lines.begin(), contour.lines.end(),
                  [](const Segment & line) {return line.observed;}))
              {
                const auto rectangle = cv::minAreaRect(local_boundary);
                const double measured_long = std::max(rectangle.size.width, rectangle.size.height);
                const double measured_short = std::min(rectangle.size.width, rectangle.size.height);
                box.center += x * rectangle.center.x + y * rectangle.center.y;
                if (std::abs(measured_long - length) >
                  profile.dimension_tolerance_m[length_axis] + epsilon ||
                  std::abs(measured_short - width) >
                  profile.dimension_tolerance_m[width_axis] + epsilon) {continue;}
              }
              // Require both observed arms to fit the hypothesized sides.
              if ((mapping ? bl : al) >
                length + profile.dimension_tolerance_m[length_axis] + epsilon ||
                (mapping ? al : bl) > width + profile.dimension_tolerance_m[width_axis] + epsilon)
              {
                continue;
              }
              for (const auto & vertex : vertices(box)) {
                if (vertex[2] < -profile.support_contact_tolerance_m) {valid = false;}
              }
              if (!valid) {continue;}
              // The count is retained Plane Map area, not a new cell traversal.
              const double area = std::min(
                std::abs(cv::contourArea(local_boundary)),
                plane.pixels.size() * obs.spacing_m * obs.spacing_m / z[2]);
              box.coverage = std::clamp(area / (length * width), 0.0, 1.0);
              const auto same = std::find_if(
                boxes.begin(), boxes.end(), [&](const Box & other) {
                  if (!operations) {exhausted = true; return false;}
                  --operations;
                  return equivalent(box, other, epsilon);
                });
              if (same == boxes.end()) {boxes.push_back(box);} else {
                same->planes.insert(pi);
                same->sources.insert(box.sources.begin(), box.sources.end());
                if (box.coverage > same->coverage) {
                  const auto planes = same->planes;
                  const auto sources = same->sources;
                  const auto id = same->id;
                  *same = box; same->planes = planes; same->sources = sources; same->id = id;
                }
              }
            }
            if (exhausted) {break;}
          }
          if (exhausted) {break;}
        }
        if (exhausted) {break;}
      }
      if (exhausted) {break;}
    }
    if (exhausted) {break;}
  }
  // Validate volumes against every other retained, coherent observed plane.
  boxes.erase(
    std::remove_if(
      boxes.begin(), boxes.end(), [&](const Box & box) {
        for (size_t pi = 0; pi < planes.size(); ++pi) {
          const auto & plane = *planes[pi];
          if (box.planes.count(pi) || !plane.plane_fitted || plane.normal[2] < 1e-4 ||
          plane.residual_m > tolerance) {continue;}
          for (const auto & contour : *contours[pi]) {
            if (!operations) {exhausted = true; return false;}
            --operations;
            std::vector<V> polygon;
            for (const auto & pixel : contour.points) {
              polygon.push_back(point(plane, obs.origin + cv::Point2d(pixel) * obs.spacing_m));
            }
            if (observed_surface_inside(box, polygon, epsilon)) {return true;}
          }
        }
        return false;
      }), boxes.end());
  // Distinct boxes consuming the same observed face remain unresolved. Plane
  // count and a larger coverage score cannot decide object identity.
  for (size_t i = 0; i < boxes.size(); ++i) {
    for (size_t j = i + 1; j < boxes.size(); ++j) {
      if (!operations) {exhausted = true; break;}
      --operations;
      bool shared = false;
      for (const auto pi : boxes[i].planes) {
        shared |= boxes[j].planes.count(pi) != 0;
      }
      if (shared || (boxes[i].coverage >= profile.medium_confidence_observed_area_ratio &&
        boxes[j].coverage >= profile.medium_confidence_observed_area_ratio &&
        overlaps(boxes[i], boxes[j], epsilon)))
      {
        boxes[i].ambiguous = true; boxes[j].ambiguous = true;
      }
    }
  }
  for (auto & box : boxes) {
    box.ambiguous |= exhausted;
    for (const auto & candidate : result.ordered_candidates) {
      if (overlaps(box, candidate_box(candidate), tolerance)) {box.ambiguous = true;}
    }
    const std::string grade = box.ambiguous ||
      box.coverage < profile.medium_confidence_observed_area_ratio ?
      "LOW" : box.coverage < profile.high_confidence_observed_area_ratio ? "MEDIUM" : "HIGH";
    for (const auto & source : box.sources) {
      const auto pi = source.first;
      const auto & plane = *planes[pi];
      CuboidHypothesisVisualization h;
      h.candidate_id = source == box.reference_source ? box.id : next_id++;
      if (h.candidate_id != box.id) {h.deduplicated_to_candidate_id = box.id;}
      h.source_boundary_id = plane.source_boundary_id;
      h.patch_index = pi;
      h.outline_id = source.second;
      const V reference_center = box.center + box.z * (box.dimensions[2] / 2);
      h.center_x_m = reference_center[0]; h.center_y_m = reference_center[1];
      h.height_m = (box.center + box.z * (box.dimensions[2] / 2))[2];
      h.observed_area_ratio = box.coverage;
      h.length_m = box.dimensions[0]; h.width_m = box.dimensions[1];
      h.thickness_m = box.dimensions[2];
      h.yaw_rad = std::atan2(box.x[1], box.x[0]);
      h.selection_state = box.ambiguous ? "UNRESOLVED" : "SELECTED";
      h.selection_reason =
        box.ambiguous ? "distinct_inclined_volume_hypotheses" : "plane_map_volume_consistent";
      h.confidence_grade = grade; h.confidence_reason = "reference_face_observed_area_ratio";
      const V face = box.center + box.z * (box.dimensions[2] / 2);
      for (const auto & signs : std::array<cv::Point2d, 4>{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}}) {
        const V corner = face + box.x * (signs.x * box.dimensions[0] / 2) +
          box.y * (signs.y * box.dimensions[1] / 2);
        geometry_msgs::msg::Point32 p; p.x = corner[0]; p.y = corner[1]; p.z = corner[2];
        h.reference_face_corners.push_back(p);
      }
      h.disposition = h.deduplicated_to_candidate_id ? "removed_by_deduplication" :
        box.ambiguous ? "unresolved_inclined_geometry" : "qualified_geometry";
      h.reason =
        h.deduplicated_to_candidate_id ? "equivalent_inclined_cuboid_volume" : h.selection_reason;
      if (result.cuboid_planes.size() <= pi) {result.cuboid_planes.resize(pi + 1);}
      auto & diagnostic = result.cuboid_planes[pi];
      if (diagnostic.cell_count == 0) {
        diagnostic.cell_count = plane.pixels.size();
        diagnostic.height_m = plane.height;
        diagnostic.plane_center_height_m = plane.center_height;
        diagnostic.partition_status = "inclined_face_reconstruction";
        if (retain_evidence) {
          for (const auto & pixel : plane.pixels) {
            geometry_msgs::msg::Point32 p;
            p.x = obs.origin.x + pixel.x * obs.spacing_m;
            p.y = obs.origin.y + pixel.y * obs.spacing_m;
            diagnostic.observed_cells_xy.push_back(p);
            diagnostic.observed_cell_ids.push_back(pixel.y * obs.heights.cols + pixel.x);
          }
          for (const auto & contour : *contours[pi]) {
            std::vector<geometry_msgs::msg::Point32> boundary;
            for (const auto & pixel : contour.points) {
              geometry_msgs::msg::Point32 p;
              p.x = obs.origin.x + pixel.x * obs.spacing_m;
              p.y = obs.origin.y + pixel.y * obs.spacing_m;
              boundary.push_back(p);
            }
            diagnostic.observed_contours_xy.push_back(std::move(boundary));
            for (const auto & line : contour.lines) {
              diagnostic.fitted_edges_xy_support.push_back(
                {line.first.x, line.first.y,
                  line.last.x, line.last.y, line.support});
            }
          }
          diagnostic.contour_count = diagnostic.observed_contours_xy.size();
          diagnostic.fitted_edge_count = diagnostic.fitted_edges_xy_support.size();
        }
      }
      diagnostic.candidate_evaluation_attempt_count++;
      diagnostic.geometry_valid_before_deduplication_count++;
      if (h.deduplicated_to_candidate_id) {
        diagnostic.deduplicated_count++;
        diagnostic.deduplicated_by_reason[h.reason]++;
      } else {
        diagnostic.generated_hypothesis_count++;
      }
      diagnostic.plane_normal = {plane.normal[0], plane.normal[1], plane.normal[2]};
      diagnostic.plane_fitted = plane.plane_fitted;
      diagnostic.plane_status = plane.status;
      diagnostic.plane_residual_m = plane.residual_m;
      result.cuboid_hypotheses.push_back(std::move(h));
    }
    if (grade == "LOW") {continue;}
    DetectionResult candidate;
    candidate.algorithm = "cuboid_top_face";
    candidate.processing_stage = "target_pose_estimation";
    candidate.result_code = arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    candidate.has_target_pose = true;
    const V face = box.center + box.z * (box.dimensions[2] / 2);
    auto & pose = candidate.target_pose.pose;
    pose.position.x = face[0]; pose.position.y = face[1]; pose.position.z = face[2];
    tf2::Matrix3x3 rotation(box.x[0], box.y[0], box.z[0], box.x[1], box.y[1], box.z[1],
      box.x[2], box.y[2], box.z[2]);
    tf2::Quaternion q; rotation.getRotation(q); q.normalize();
    pose.orientation.x = q.x(); pose.orientation.y = q.y();
    pose.orientation.z = q.z(); pose.orientation.w = q.w();
    candidate.has_estimated_height = true; candidate.estimated_height_m = box.dimensions[2];
    candidate.thickness_evidence = "material_face_hypothesis";
    candidate.rectangle_long_side_m = box.dimensions[0];
    candidate.rectangle_short_side_m = box.dimensions[1];
    candidate.has_observed_rectangle = false;
    candidate.orientation_symmetry_order = std::abs(box.dimensions[0] - box.dimensions[1]) <
      1e-9 ? 4 : 2;
    candidate.confidence_grade = grade;
    candidate.confidence_reason = "reference_face_observed_area_ratio";
    candidate.rawpart_selected_candidate = std::to_string(box.id);
    candidate.profile_candidate_valid = true;
    candidate.profile_candidate_features = {face[0], face[1], face[2],
      box.dimensions[0], box.dimensions[1], box.dimensions[2]};
    RawPartCandidateDiagnostic diagnostic;
    diagnostic.candidate_id = box.id;
    diagnostic.confidence_grade = grade;
    diagnostic.confidence_reason = candidate.confidence_reason;
    diagnostic.position_x_m = face[0]; diagnostic.position_y_m = face[1];
    diagnostic.position_z_m = face[2];
    diagnostic.expected_width_m = box.dimensions[0];
    diagnostic.expected_height_m = box.dimensions[1];
    diagnostic.expected_vertical_height_m = box.dimensions[2];
    diagnostic.coverage = box.coverage; diagnostic.profile_match = true;
    diagnostic.orientation_symmetry_order = candidate.orientation_symmetry_order;
    for (const auto & signs : std::array<cv::Point2d, 4>{{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}}) {
      const V corner = face + box.x * (signs.x * box.dimensions[0] / 2) +
        box.y * (signs.y * box.dimensions[1] / 2);
      geometry_msgs::msg::Point32 p; p.x = corner[0]; p.y = corner[1]; p.z = corner[2];
      diagnostic.fitted_outline_support.push_back(p);
    }
    result.rawpart_candidate_diagnostics.push_back(std::move(diagnostic));
    result.ordered_candidates.push_back(std::move(candidate));
  }
}
void normalize_cuboid_hypothesis_aliases(std::vector<CuboidHypothesisVisualization> & hypotheses)
{
  std::map<uint32_t, CuboidHypothesisVisualization *> by_id;
  for (auto & h : hypotheses) {
    by_id[h.candidate_id] = &h;
  }
  for (auto & h : hypotheses) {
    if (!h.deduplicated_to_candidate_id) {continue;}
    auto * root = &h;
    std::set<uint32_t> seen;
    while (root->deduplicated_to_candidate_id && seen.insert(root->candidate_id).second) {
      const auto parent = by_id.find(root->deduplicated_to_candidate_id);
      if (parent == by_id.end()) {break;}
      root = parent->second;
    }
    if (root == &h || root->deduplicated_to_candidate_id) {continue;}
    h.deduplicated_to_candidate_id = root->candidate_id;
    if (h.disposition != "removed_by_deduplication") {h.reason = "equivalent_cuboid_volume";}
    h.disposition = "removed_by_deduplication";
    h.selection_state = root->selection_state;
    h.selection_reason = "equivalent_normalized_hypothesis";
    h.confidence_grade = root->confidence_grade;
    h.confidence_reason = "normalized_to_candidate_" + std::to_string(root->candidate_id);
    h.competing_candidate_ids = root->competing_candidate_ids;
  }
  for (auto & h : hypotheses) {
    const auto self =
      h.deduplicated_to_candidate_id ? h.deduplicated_to_candidate_id : h.candidate_id;
    for (auto & id : h.competing_candidate_ids) {
      const auto found = by_id.find(id);
      if (found != by_id.end() && found->second->deduplicated_to_candidate_id) {
        id = found->second->deduplicated_to_candidate_id;
      }
    }
    auto & competitors = h.competing_candidate_ids;
    competitors.erase(std::remove(competitors.begin(), competitors.end(), self), competitors.end());
    std::sort(competitors.begin(), competitors.end());
    competitors.erase(std::unique(competitors.begin(), competitors.end()), competitors.end());
  }
}

void reconcile_cuboid_volumes(DetectionResult & result, double tolerance)
{
  auto & candidates = result.ordered_candidates;
  std::set<size_t> duplicates, conflicts;
  const auto coverage = [&](const DetectionResult & c) {
      const auto id = std::stoul(c.rawpart_selected_candidate);
      const auto diagnostic = std::find_if(
        result.rawpart_candidate_diagnostics.begin(),
        result.rawpart_candidate_diagnostics.end(), [&](const auto & d) {
          return d.candidate_id == id;
        });
      if (diagnostic != result.rawpart_candidate_diagnostics.end()) {return diagnostic->coverage;}
      const auto hypothesis = std::find_if(
        result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
        [&](const auto & h) {return h.candidate_id == id;});
      return hypothesis == result.cuboid_hypotheses.end() ? 0.0 : hypothesis->observed_area_ratio;
    };
  // Choose the complete representative payload before evaluating conflicts.
  // Changing only its grade would detach HIGH from the face that supports it.
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (duplicates.count(i)) {continue;}
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (duplicates.count(j) || !equivalent(
          candidate_box(candidates[i]),
          candidate_box(candidates[j]), tolerance)) {continue;}
      if (coverage(candidates[j]) > coverage(candidates[i])) {
        std::swap(candidates[i], candidates[j]);
      }
      duplicates.insert(j);
      const auto representative = std::stoul(candidates[i].rawpart_selected_candidate);
      const auto duplicate = std::stoul(candidates[j].rawpart_selected_candidate);
      for (auto & h : result.cuboid_hypotheses) {
        if (h.candidate_id == duplicate) {
          h.deduplicated_to_candidate_id = representative;
          h.disposition = "removed_by_deduplication";
          h.reason = "equivalent_cuboid_volume";
        }
      }
    }
  }
  normalize_cuboid_hypothesis_aliases(result.cuboid_hypotheses);
  for (size_t i = 0; i < candidates.size(); ++i) {
    if (duplicates.count(i)) {continue;}
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      if (duplicates.count(j) || !overlaps(
          candidate_box(candidates[i]),
          candidate_box(candidates[j]), tolerance)) {continue;}
      conflicts.insert(i); conflicts.insert(j);
      const uint32_t a = std::stoul(candidates[i].rawpart_selected_candidate);
      const uint32_t b = std::stoul(candidates[j].rawpart_selected_candidate);
      for (auto & h : result.cuboid_hypotheses) {
        if (h.candidate_id != a && h.candidate_id != b) {continue;}
        h.selection_state = "UNRESOLVED";
        h.selection_reason = "incompatible_observed_material_volumes";
        h.confidence_grade = "LOW";
        h.confidence_reason = h.selection_reason;
        h.disposition = "unresolved_volume_conflict";
        h.reason = h.selection_reason;
        h.competing_candidate_ids.push_back(h.candidate_id == a ? b : a);
      }
    }
  }
  size_t index = 0;
  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(), [&](const auto &) {
        const auto i = index++;
        return duplicates.count(i) || conflicts.count(i);
      }), candidates.end());
  for (auto & h : result.cuboid_hypotheses) {
    if (h.deduplicated_to_candidate_id) {continue;}
    const auto returned = std::find_if(
      candidates.begin(), candidates.end(), [&](const auto & c) {
        return c.rawpart_selected_candidate == std::to_string(h.candidate_id);
      });
    if (returned != candidates.end()) {
      h.confidence_grade = returned->confidence_grade;
      h.confidence_reason = returned->confidence_reason;
    }
  }
  normalize_cuboid_hypothesis_aliases(result.cuboid_hypotheses);
  for (auto & diagnostic : result.rawpart_candidate_diagnostics) {
    const auto h = std::find_if(
      result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
      [&](const auto & h) {return h.candidate_id == diagnostic.candidate_id;});
    if (h == result.cuboid_hypotheses.end()) {continue;}
    diagnostic.confidence_grade = h->confidence_grade;
    diagnostic.confidence_reason = h->confidence_reason;
    const auto root = h->deduplicated_to_candidate_id ? std::find_if(
      result.cuboid_hypotheses.begin(),
      result.cuboid_hypotheses.end(), [&](const auto & r) {
        return r.candidate_id == h->deduplicated_to_candidate_id;
      }) : h;
    if (root != result.cuboid_hypotheses.end() && root->selection_state == "UNRESOLVED") {
      diagnostic.rejection_reason = root->selection_reason;
    }
  }
}

void finalize_cuboid_diagnostics(DetectionResult & result)
{
  normalize_cuboid_hypothesis_aliases(result.cuboid_hypotheses);
  std::set<uint32_t> returned;
  for (const auto & c : result.ordered_candidates) {
    returned.insert(std::stoul(c.rawpart_selected_candidate));
  }
  std::vector<std::pair<std::string, std::string>> pre_object_confidence;
  for (auto & patch : result.cuboid_planes) {
    pre_object_confidence.emplace_back(patch.confidence_grade, patch.confidence_reason);
    patch.partitioned_candidate_count = 0;
    patch.deduplicated_count = 0;
    patch.deduplicated_by_reason.clear();
    patch.confidence_grade.clear();
    patch.confidence_reason = "no_independent_normalized_candidate";
  }
  std::vector<uint64_t> plane_roots(result.cuboid_planes.size(), 0);
  uint64_t roots = 0, aliases = 0, attempts = 0, valid = 0, generated = 0;
  std::map<std::string, uint64_t> deduplicated_by_reason, qualification_rejections;
  uint64_t conflict_count = 0;
  for (const auto & h : result.cuboid_hypotheses) {
    if (h.deduplicated_to_candidate_id) {
      ++aliases; ++deduplicated_by_reason[h.reason];
      if (h.patch_index < result.cuboid_planes.size()) {
        auto & patch = result.cuboid_planes[h.patch_index];
        ++patch.deduplicated_count; ++patch.deduplicated_by_reason[h.reason];
      }
    } else {
      if (h.selection_state != "REJECTED") {
        ++roots;
        if (!returned.count(h.candidate_id)) {
          const auto reason = h.selection_state ==
            "SELECTED" ? h.confidence_reason : h.selection_reason;
          ++qualification_rejections[reason];
          conflict_count += h.selection_state != "SELECTED";
        }
      }
      if (h.selection_state != "REJECTED" && h.patch_index < result.cuboid_planes.size()) {
        auto & patch = result.cuboid_planes[h.patch_index];
        const auto count = ++plane_roots[h.patch_index];
        patch.confidence_grade = count == 1 ? h.confidence_grade : "";
        patch.confidence_reason = count ==
          1 ? h.confidence_reason : "multiple_normalized_candidates";
        if (returned.count(h.candidate_id)) {++patch.partitioned_candidate_count;}
      }
    }
  }
  // A LOW observation with no object anchor is pre-object evidence, not a
  // stale candidate grade. Preserve it when normalization has no object there.
  for (size_t i = 0; i < result.cuboid_planes.size(); ++i) {
    auto & patch = result.cuboid_planes[i];
    if (plane_roots[i] == 0 && patch.deduplicated_count == 0 &&
      pre_object_confidence[i].first == "LOW")
    {
      patch.confidence_grade = pre_object_confidence[i].first;
      patch.confidence_reason = pre_object_confidence[i].second;
    }
  }
  for (auto & patch : result.cuboid_planes) {
    patch.generated_hypothesis_count = patch.geometry_valid_before_deduplication_count -
      std::min(patch.geometry_valid_before_deduplication_count, patch.deduplicated_count);
    attempts += patch.candidate_evaluation_attempt_count;
    valid += patch.geometry_valid_before_deduplication_count;
    generated += patch.generated_hypothesis_count;
  }
  result.rawpart_candidate_count = returned.size();
  result.rawpart_profile_match_count = returned.size();
  for (auto & stage : result.cuboid_stages) {
    if (stage.stage == "cuboid_hypothesis_generation" ||
      stage.stage == "dimension_and_geometry_validation")
    {
      stage.candidate_evaluation_attempt_count = attempts;
      stage.deduplicated_count = aliases;
      stage.deduplicated_by_reason = deduplicated_by_reason;
      stage.generated_count = stage.stage == "cuboid_hypothesis_generation" ? generated : valid;
      if (attempts) {
        stage.status = "computed"; stage.reason = "retained_plane_map_hypotheses_evaluated";
      }
      stage.auxiliary_counts["normalized_geometry_candidates"] = roots;
    } else if (stage.stage == "candidate_local_eligibility") {
      stage.input_count = roots;
      stage.output_count = returned.size();
      stage.rejected_count = roots - returned.size();
      stage.rejected_by_reason = qualification_rejections;
      stage.auxiliary_counts["candidates_rejected_for_conflict"] = conflict_count;
      stage.auxiliary_counts["evaluated_height_patches"] = result.cuboid_planes.size();
      if (roots) {stage.status = "computed"; stage.reason = "normalized_candidate_qualification";}
    } else if (stage.stage == "candidate_enumeration") {
      stage.input_count = returned.size(); stage.output_count = returned.size();
      stage.rejected_count = 0;
      stage.auxiliary_counts["enumerated_candidates"] = returned.size();
      stage.status = returned.empty() ? "not_computed" : "computed";
      stage.reason =
        returned.empty() ? "no_individually_eligible_candidates" :
        "deterministic_candidate_id_enumeration";
    }
  }
  std::stable_sort(
    result.cuboid_hypotheses.begin(), result.cuboid_hypotheses.end(),
    [](const auto & a, const auto & b) {return a.candidate_id < b.candidate_id;});
  std::stable_sort(
    result.ordered_candidates.begin(), result.ordered_candidates.end(),
    [](const auto & a, const auto & b) {
      return std::stoul(a.rawpart_selected_candidate) < std::stoul(b.rawpart_selected_candidate);
    });
}

}
