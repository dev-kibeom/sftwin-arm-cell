#include "arm_cell_motion/finger_clearance.hpp"
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_cell_motion
{
namespace
{
// Numerical geometry tolerance in metres, not an operational clearance margin.
constexpr double kGeometryTolerance = 1e-10;
struct Plane
{
  Eigen::Vector3d normal;
  double bound;
};

double cross(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return a.x() * b.y() - a.y() * b.x();
}
double point_segment_squared(
  const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d delta = b - a;
  const double t = delta.squaredNorm() == 0.0 ? 0.0 :
    std::clamp((p - a).dot(delta) / delta.squaredNorm(), 0.0, 1.0);
  return (p - a - t * delta).squaredNorm();
}
std::vector<Eigen::Vector2d> convex_hull(std::vector<Eigen::Vector2d> points)
{
  std::sort(
    points.begin(), points.end(), [](const auto & a, const auto & b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
  points.erase(
    std::unique(
      points.begin(), points.end(), [](const auto & a, const auto & b) {
        return (a - b).norm() < kGeometryTolerance;
      }), points.end());
  if (points.size() < 3U) {return points;}
  std::vector<Eigen::Vector2d> hull;
  for (const auto & p : points) {
    while (hull.size() >= 2U && cross(
        hull.back() - hull[hull.size() - 2U], p - hull.back()) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(p);
  }
  const auto lower_size = hull.size();
  for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
    while (hull.size() > lower_size && cross(
        hull.back() - hull[hull.size() - 2U], *it - hull.back()) <= 0.0)
    {
      hull.pop_back();
    }
    hull.push_back(*it);
  }
  hull.pop_back();
  return hull;
}
bool inside(const Eigen::Vector2d & point, const std::vector<Eigen::Vector2d> & hull)
{
  if (hull.size() < 3U) {return false;}
  for (std::size_t i = 0; i < hull.size(); ++i) {
    if (cross(hull[(i + 1U) % hull.size()] - hull[i], point - hull[i]) < 0.0) {
      return false;
    }
  }
  return true;
}
}  // namespace

namespace
{
bool intersects_planes(const FingerProtectionVolume & volume, std::vector<Plane> planes)
{
  planes.push_back({-Eigen::Vector3d::UnitX(), 0.0});
  planes.push_back({Eigen::Vector3d::UnitZ(), 0.0});
  planes.push_back({-Eigen::Vector3d::UnitZ(), volume.dimensions.depth_m});

  // Clip the OBB by the outward half-space and finite insertion slab. All
  // vertices of the clipped convex polytope are intersections of three planes.
  std::vector<Eigen::Vector3d> vertices;
  for (std::size_t i = 0; i < planes.size(); ++i) {
    for (std::size_t j = i + 1U; j < planes.size(); ++j) {
      for (std::size_t k = j + 1U; k < planes.size(); ++k) {
        Eigen::Matrix3d matrix;
        matrix.row(0) = planes[i].normal;
        matrix.row(1) = planes[j].normal;
        matrix.row(2) = planes[k].normal;
        if (std::abs(matrix.determinant()) < 1e-12) {continue;}
        const Eigen::Vector3d p = matrix.inverse() *
          Eigen::Vector3d(planes[i].bound, planes[j].bound, planes[k].bound);
        if (std::all_of(
            planes.begin(), planes.end(), [&](const auto & plane) {
              return plane.normal.dot(p) <= plane.bound + kGeometryTolerance;
            })) {vertices.push_back(p);}
      }
    }
  }
  if (vertices.empty()) {return false;}
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & p : vertices) {
    centroid += p;
  }
  centroid /= static_cast<double>(vertices.size());
  // A positive-volume polytope has a vertex centroid strictly inside every
  // bounding plane. This rejects tangency to the face or insertion end caps.
  for (const auto & plane : planes) {
    if (plane.bound - plane.normal.dot(centroid) <= kGeometryTolerance) {return false;}
  }
  std::vector<Eigen::Vector2d> projection;
  for (const auto & p : vertices) {
    projection.emplace_back(p.x(), p.y());
  }
  const auto hull = convex_hull(std::move(projection));
  const Eigen::Vector2d a(0.0, -volume.dimensions.contact_span_m / 2.0);
  const Eigen::Vector2d b(0.0, volume.dimensions.contact_span_m / 2.0);
  double distance_squared = std::numeric_limits<double>::infinity();
  if (inside(a, hull) || inside(b, hull)) {distance_squared = 0.0;}
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const auto & c = hull[i];
    const auto & d = hull[(i + 1U) % hull.size()];
    distance_squared = std::min(
      {distance_squared,
        point_segment_squared(a, c, d), point_segment_squared(b, c, d),
        point_segment_squared(c, a, b), point_segment_squared(d, a, b)});
  }
  // The projection lies in x >= 0; it cannot cross the centre segment except
  // on x == 0, which is covered by vertex/edge distances above.
  return std::sqrt(distance_squared) + kGeometryTolerance < volume.dimensions.radius_m;
}
}  // namespace

bool finger_protection_intersects_box(
  const FingerProtectionVolume & volume, const Eigen::Isometry3d & box_pose,
  const Eigen::Vector3d & box_dimensions)
{
  const Eigen::Isometry3d box = volume.frame.inverse() * box_pose;
  std::vector<Plane> planes;
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d normal = box.linear().col(axis);
    planes.push_back({normal, normal.dot(box.translation()) + box_dimensions[axis] / 2.0});
    planes.push_back({-normal, -normal.dot(box.translation()) + box_dimensions[axis] / 2.0});
  }
  return intersects_planes(volume, std::move(planes));
}

bool finger_protection_intersects_convex_top(
  const FingerProtectionVolume & volume, const Eigen::Isometry3d & pose,
  const std::vector<Eigen::Vector3d> & top, double bottom)
{
  if (top.size() < 3) {return false;}
  const auto transform = volume.frame.inverse() * pose;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (const auto & p : top) {center += p;}
  center /= static_cast<double>(top.size());
  center.z() = (center.z() + bottom) / 2;
  std::vector<Plane> planes;
  const auto add_plane = [&](Eigen::Vector3d normal, const Eigen::Vector3d & point) {
      if (normal.dot(center - point) > 0) {normal = -normal;}
      normal = transform.linear() * normal.normalized();
      planes.push_back({normal, normal.dot(transform * point)});
    };
  add_plane(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d(0, 0, bottom));
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  for (size_t i = 1; i + 1 < top.size() && normal.norm() < 1e-12; ++i) {
    normal = (top[i] - top[0]).cross(top[i + 1] - top[0]);
  }
  if (normal.norm() < 1e-12) {return false;}
  add_plane(normal, top[0]);
  for (size_t i = 0; i < top.size(); ++i) {
    const auto edge = top[(i + 1) % top.size()] - top[i];
    const Eigen::Vector3d side(edge.y(), -edge.x(), 0);
    if (side.norm() < 1e-12) {return false;}
    add_plane(side, top[i]);
  }
  return intersects_planes(volume, std::move(planes));
}
}  // namespace arm_cell_motion
