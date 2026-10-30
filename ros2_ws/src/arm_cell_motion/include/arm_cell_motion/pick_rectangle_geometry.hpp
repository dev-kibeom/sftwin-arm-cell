#pragma once

#include <cmath>

namespace arm_cell_motion
{

enum class RectangleGeometryStatus
{
  UNAVAILABLE,
  AVAILABLE,
  INVALID
};

inline RectangleGeometryStatus classify_rectangle_geometry(
  bool required, bool has_height, double long_side_m, double short_side_m, double height_m)
{
  if (!required) {
    return RectangleGeometryStatus::UNAVAILABLE;
  }
  if (!has_height || !std::isfinite(long_side_m) || !std::isfinite(short_side_m) ||
    !std::isfinite(height_m) || short_side_m <= 0.0 || long_side_m < short_side_m ||
    height_m <= 0.0)
  {
    return RectangleGeometryStatus::INVALID;
  }
  return RectangleGeometryStatus::AVAILABLE;
}

inline const char * rectangle_geometry_status_name(RectangleGeometryStatus status)
{
  switch (status) {
    case RectangleGeometryStatus::AVAILABLE: return "available";
    case RectangleGeometryStatus::INVALID: return "invalid";
    case RectangleGeometryStatus::UNAVAILABLE:
    default: return "unavailable";
  }
}

}  // namespace arm_cell_motion
