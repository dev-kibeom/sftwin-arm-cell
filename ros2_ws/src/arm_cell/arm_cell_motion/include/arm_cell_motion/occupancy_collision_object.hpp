#pragma once

#include <string>
#include <utility>
#include <arm_cell_interfaces/msg/collision_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

namespace arm_cell_motion
{
inline constexpr char kCandidateOccupancyIdPrefix[] = "vision_candidate_occupancy_";

inline moveit_msgs::msg::CollisionObject candidate_occupancy_object(
  const arm_cell_interfaces::msg::CollisionPrimitive & primitive,
  const std_msgs::msg::Header & header, size_t index)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = std::string(kCandidateOccupancyIdPrefix) + std::to_string(index);
  object.header = header;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (primitive.shape_type == arm_cell_interfaces::msg::CollisionPrimitive::CONVEX_TOP) {
    shape_msgs::msg::Mesh mesh;
    const auto n = primitive.top_vertices.size();
    mesh.vertices = primitive.top_vertices;
    for (const auto & top : primitive.top_vertices) {
      auto bottom = top; bottom.z = primitive.bottom_height_m;
      mesh.vertices.push_back(bottom);
    }
    double area = 0;
    for (size_t i = 0; i < n; ++i) {
      const auto & a = primitive.top_vertices[i];
      const auto & b = primitive.top_vertices[(i + 1) % n];
      area += a.x * b.y - a.y * b.x;
    }
    const auto triangle = [&](size_t a, size_t b, size_t c) {
        if (area < 0) {std::swap(b, c);}
        shape_msgs::msg::MeshTriangle face;
        face.vertex_indices = {static_cast<uint32_t>(a), static_cast<uint32_t>(b),
          static_cast<uint32_t>(c)};
        mesh.triangles.push_back(face);
      };
    for (size_t i = 1; i + 1 < n; ++i) {
      triangle(0, i, i + 1); triangle(n, n + i + 1, n + i);
    }
    for (size_t i = 0; i < n; ++i) {
      const auto j = (i + 1) % n;
      triangle(i, n + i, n + j); triangle(i, n + j, j);
    }
    object.meshes.push_back(std::move(mesh));
    object.mesh_poses.push_back(primitive.pose);
  } else {
    object.primitives.push_back(primitive.shape);
    object.primitive_poses.push_back(primitive.pose);
  }
  return object;
}

}  // namespace arm_cell_motion
