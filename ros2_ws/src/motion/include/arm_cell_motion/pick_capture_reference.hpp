#pragma once

#include <cmath>
#include <functional>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "arm_cell_motion/pick_rectangle_geometry.hpp"

namespace arm_cell_motion
{

struct CaptureReferenceContactWidth
{
  std::string source;
  std::string status;
  std::string failure_reason;
  double width_mm{0.0};
};

using FixtureContactWidthLookup = std::function<std::optional<double>(std::string &)>;

inline CaptureReferenceContactWidth resolve_capture_reference_contact_width(
  RectangleGeometryStatus geometry_status, double candidate_width_mm,
  bool validation_fixture_enabled, const FixtureContactWidthLookup & fixture_lookup)
{
  CaptureReferenceContactWidth resolution;
  if (geometry_status == RectangleGeometryStatus::AVAILABLE) {
    resolution.source = "vision_candidate_geometry";
    if (std::isfinite(candidate_width_mm) && candidate_width_mm > 0.0) {
      resolution.width_mm = candidate_width_mm;
      resolution.status = "ready";
    } else {
      resolution.status = "failed";
      resolution.failure_reason = "contact_width_policy_mismatch";
    }
    return resolution;
  }

  if (geometry_status == RectangleGeometryStatus::INVALID) {
    resolution.source = "vision_candidate_geometry";
    resolution.status = "failed";
    resolution.failure_reason = "invalid_rectangle_geometry";
    return resolution;
  }

  if (!validation_fixture_enabled) {
    resolution.source = "vision_candidate_geometry";
    resolution.status = "failed";
    resolution.failure_reason = "geometry_not_provided";
    return resolution;
  }

  resolution.source = "validation_fixture";
  const auto width = fixture_lookup ? fixture_lookup(resolution.failure_reason) : std::nullopt;
  if (width && std::isfinite(*width) && *width > 0.0) {
    resolution.width_mm = *width;
    resolution.status = "ready";
    resolution.failure_reason.clear();
  } else {
    resolution.status = "failed";
    if (resolution.failure_reason.empty()) {
      resolution.failure_reason = "fixture_geometry_or_width_policy_invalid";
    }
  }
  return resolution;
}

inline nlohmann::json capture_reference_diagnostic_json(
  const CaptureReferenceContactWidth & resolution,
  RectangleGeometryStatus geometry_status, const std::string & fixture_id,
  bool has_observed_rectangle, bool has_estimated_height,
  double long_side_m, double short_side_m, double height_m,
  double policy_min_mm, double policy_max_mm)
{
  return {
    {"source", resolution.source},
    {"status", resolution.status},
    {"failure_reason", resolution.failure_reason},
    {"fixture_id", fixture_id},
    {"rectangle_geometry_status", rectangle_geometry_status_name(geometry_status)},
    {"has_observed_rectangle", has_observed_rectangle},
    {"has_estimated_object_height", has_estimated_height},
    {"rectangle_dimensions_m", {long_side_m, short_side_m}},
    {"estimated_object_height_m", height_m},
    {"recipe_contact_width_policy_mm", {policy_min_mm, policy_max_mm}},
    {"contact_width_input_mm", resolution.width_mm}};
}

}  // namespace arm_cell_motion
