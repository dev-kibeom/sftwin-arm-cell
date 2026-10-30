#include "arm_cell_orchestration_bt/recipe_repository.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <vector>

namespace arm_cell_orchestration_bt
{
namespace
{

using Json = nlohmann::json;

class RecipeError : public std::runtime_error
{
public:
  RecipeError(RecipeDiagnosticReason reason, const std::string & detail)
  : std::runtime_error(detail), reason_(reason) {}
  RecipeDiagnosticReason reason() const {return reason_;}

private:
  RecipeDiagnosticReason reason_;
};

const Json & required(const Json & value, const char * key)
{
  if (!value.is_object() || !value.contains(key)) {
    throw RecipeError(RecipeDiagnosticReason::REQUIRED_FIELD_MISSING, key);
  }
  return value.at(key);
}

const Json & object(const Json & value, const char * field)
{
  if (!value.is_object()) {
    throw RecipeError(RecipeDiagnosticReason::TYPE_MISMATCH, field);
  }
  return value;
}

std::string string_value(const Json & value, const char * field)
{
  if (!value.is_string()) {
    throw RecipeError(RecipeDiagnosticReason::TYPE_MISMATCH, field);
  }
  return value.get<std::string>();
}

double number_value(const Json & value, const char * field)
{
  if (!value.is_number()) {
    throw RecipeError(RecipeDiagnosticReason::TYPE_MISMATCH, field);
  }
  return value.get<double>();
}

std::vector<double> array_value(const Json & value, size_t size, const char * field)
{
  if (!value.is_array()) {
    throw RecipeError(RecipeDiagnosticReason::TYPE_MISMATCH, field);
  }
  if (value.size() != size) {
    throw RecipeError(RecipeDiagnosticReason::ARRAY_SIZE_INVALID, field);
  }
  std::vector<double> result;
  result.reserve(size);
  for (const auto & item : value) {
    result.push_back(number_value(item, field));
  }
  return result;
}

bool finite(const std::vector<double> & values)
{
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

std::string trim_for_filename(const std::string & target_id)
{
  if (target_id.empty()) {
    return {};
  }
  for (const unsigned char ch : target_id) {
    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
      (ch >= '0' && ch <= '9') || ch == '_' || ch == '-'))
    {
      return {};
    }
  }
  return target_id;
}

RecipeLoadResult failed(
  const std::filesystem::path & source, RecipeDiagnosticReason reason)
{
  return RecipeLoadResult{std::nullopt, reason, source};
}

}  // namespace

const char * recipe_reason_token(RecipeDiagnosticReason reason)
{
  switch (reason) {
    case RecipeDiagnosticReason::NONE: return "none";
    case RecipeDiagnosticReason::FILE_NOT_FOUND: return "recipe_file_not_found";
    case RecipeDiagnosticReason::JSON_MALFORMED: return "recipe_json_malformed";
    case RecipeDiagnosticReason::SCHEMA_UNSUPPORTED: return "recipe_schema_unsupported";
    case RecipeDiagnosticReason::REQUIRED_FIELD_MISSING: return "recipe_required_field_missing";
    case RecipeDiagnosticReason::TYPE_MISMATCH: return "recipe_type_mismatch";
    case RecipeDiagnosticReason::ARRAY_SIZE_INVALID: return "recipe_array_size_invalid";
    case RecipeDiagnosticReason::INVALID_ID: return "recipe_invalid_id";
    case RecipeDiagnosticReason::TARGET_MISMATCH: return "recipe_target_mismatch";
    case RecipeDiagnosticReason::INVALID_FRAME: return "recipe_invalid_frame";
    case RecipeDiagnosticReason::INVALID_POSE_COMPONENT: return "recipe_invalid_pose_component";
    case RecipeDiagnosticReason::INVALID_QUATERNION: return "recipe_invalid_quaternion";
    case RecipeDiagnosticReason::INVALID_APPROACH_DIRECTION:
      return "recipe_invalid_approach_direction";
    case RecipeDiagnosticReason::INVALID_DISTANCE: return "recipe_invalid_distance";
    case RecipeDiagnosticReason::INVALID_TOLERANCE: return "recipe_invalid_tolerance";
    case RecipeDiagnosticReason::INVALID_ORIENTATION_CONSTRAINT:
      return "recipe_invalid_orientation_constraint";
    case RecipeDiagnosticReason::INVALID_GRASP_WIDTH: return "recipe_invalid_grasp_width";
    case RecipeDiagnosticReason::INVALID_GRASP_YAW: return "recipe_invalid_grasp_yaw";
    case RecipeDiagnosticReason::INVALID_TOOL_ORIENTATION:
      return "recipe_invalid_tool_orientation";
  }
  return "recipe_unknown_error";
}

RecipeRepository::RecipeRepository(std::filesystem::path directory)
: directory_(std::move(directory)) {}

RecipeLoadResult RecipeRepository::load(const std::string & target_id) const
{
  const auto safe_id = trim_for_filename(target_id);
  const auto source = safe_id.empty() ? directory_ / "<invalid-target-id>.json" :
    directory_ / (safe_id + ".json");
  if (safe_id.empty()) {
    return failed(source, RecipeDiagnosticReason::INVALID_ID);
  }
  std::ifstream input(source);
  if (!input) {
    return failed(source, RecipeDiagnosticReason::FILE_NOT_FOUND);
  }

  Json document;
  try {
    input >> document;
  } catch (const Json::parse_error &) {
    return failed(source, RecipeDiagnosticReason::JSON_MALFORMED);
  } catch (const Json::exception &) {
    return failed(source, RecipeDiagnosticReason::JSON_MALFORMED);
  }

  try {
    object(document, "root");
    const auto version = required(document, "schema_version");
    if (!version.is_number_unsigned() && !version.is_number_integer()) {
      throw RecipeError(RecipeDiagnosticReason::TYPE_MISMATCH, "schema_version");
    }
    if (version.get<int64_t>() != 1) {
      throw RecipeError(RecipeDiagnosticReason::SCHEMA_UNSUPPORTED, "schema_version");
    }

    const std::string recipe_id = string_value(required(document, "recipe_id"), "recipe_id");
    const std::string recipe_target = string_value(required(document, "target_id"), "target_id");
    if (recipe_id.empty() || recipe_target.empty()) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_ID, "recipe_id/target_id");
    }
    if (recipe_target != target_id) {
      throw RecipeError(RecipeDiagnosticReason::TARGET_MISMATCH, "target_id");
    }

    const auto & grasp = object(required(document, "grasp"), "grasp");
    const double width = number_value(required(grasp, "width_mm"), "grasp.width_mm");
    const double yaw = number_value(required(grasp, "yaw_rad"), "grasp.yaw_rad");
    if (!std::isfinite(width) || width < 0.0 || width > 85.0) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_GRASP_WIDTH, "grasp.width_mm");
    }
    if (!std::isfinite(yaw)) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_GRASP_YAW, "grasp.yaw_rad");
    }

    const auto & place = object(required(document, "place"), "place");
    const auto orientation_constraint_token = string_value(
      required(place, "orientation_constraint"), "orientation_constraint");
    PlaceOrientationConstraint orientation_constraint;
    if (orientation_constraint_token == "fixed") {
      orientation_constraint = PlaceOrientationConstraint::FIXED;
    } else if (orientation_constraint_token == "bounded") {
      orientation_constraint = PlaceOrientationConstraint::BOUNDED;
    } else if (orientation_constraint_token == "free") {
      orientation_constraint = PlaceOrientationConstraint::FREE;
    } else {
      throw RecipeError(
              RecipeDiagnosticReason::INVALID_ORIENTATION_CONSTRAINT,
              "orientation_constraint");
    }
    bool has_tool_orientation_preference = false;
    std::vector<double> tool_orientation;
    if (place.contains("tool_orientation")) {
      const auto & policy = object(place.at("tool_orientation"), "tool_orientation");
      const auto mode = string_value(required(policy, "mode"), "tool_orientation.mode");
      if (mode != "preferred") {
        throw RecipeError(
                RecipeDiagnosticReason::INVALID_TOOL_ORIENTATION, "tool_orientation.mode");
      }
      if (orientation_constraint == PlaceOrientationConstraint::FREE) {
        throw RecipeError(
                RecipeDiagnosticReason::INVALID_TOOL_ORIENTATION,
                "tool_orientation is not valid for free orientation");
      }
      tool_orientation = array_value(
        required(policy, "orientation_xyzw"), 4, "tool_orientation.orientation_xyzw");
      if (!finite(tool_orientation)) {
        throw RecipeError(
                RecipeDiagnosticReason::INVALID_TOOL_ORIENTATION,
                "tool_orientation.orientation_xyzw");
      }
      double tool_orientation_norm_squared = 0.0;
      for (const double value : tool_orientation) {
        tool_orientation_norm_squared += value * value;
      }
      if (!std::isfinite(tool_orientation_norm_squared) ||
        std::abs(tool_orientation_norm_squared - 1.0) > 1e-6)
      {
        throw RecipeError(
                RecipeDiagnosticReason::INVALID_TOOL_ORIENTATION,
                "tool_orientation.orientation_xyzw");
      }
      has_tool_orientation_preference = true;
    }
    const auto & desired = object(required(place, "desired_object_pose"), "desired_object_pose");
    const auto frame = string_value(required(desired, "frame"), "frame");
    if (frame != "base_link") {
      throw RecipeError(RecipeDiagnosticReason::INVALID_FRAME, "frame");
    }
    const auto position = array_value(required(desired, "position"), 3, "position");
    const auto orientation = array_value(
      required(desired, "orientation_xyzw"), 4, "orientation_xyzw");
    if (!finite(position)) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_POSE_COMPONENT, "position");
    }
    if (!finite(orientation)) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_QUATERNION, "orientation_xyzw");
    }
    double quaternion_norm = 0.0;
    for (const double value : orientation) {
      quaternion_norm += value * value;
    }
    if (!std::isfinite(quaternion_norm) || std::abs(quaternion_norm - 1.0) > 1e-6) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_QUATERNION, "orientation_xyzw");
    }

    const double position_tolerance = number_value(
      required(place, "position_tolerance_m"), "position_tolerance_m");
    const double orientation_tolerance = number_value(
      required(place, "orientation_tolerance_rad"), "orientation_tolerance_rad");
    if (!std::isfinite(position_tolerance) || position_tolerance < 0.0 ||
      !std::isfinite(orientation_tolerance) || orientation_tolerance < 0.0)
    {
      throw RecipeError(RecipeDiagnosticReason::INVALID_TOLERANCE, "place tolerance");
    }
    const auto & approach = object(required(place, "approach"), "approach");
    const auto direction = array_value(
      required(approach, "direction_object"), 3, "direction_object");
    if (!finite(direction)) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_APPROACH_DIRECTION, "direction_object");
    }
    double direction_norm = 0.0;
    for (const double value : direction) {
      direction_norm += value * value;
    }
    if (!std::isfinite(direction_norm) || std::abs(std::sqrt(direction_norm) - 1.0) > 1e-6) {
      throw RecipeError(RecipeDiagnosticReason::INVALID_APPROACH_DIRECTION, "direction_object");
    }
    const double approach_distance = number_value(
      required(approach, "distance_m"), "approach.distance_m");
    const double retract_distance = number_value(
      required(place, "retract_distance_m"), "retract_distance_m");
    if (!std::isfinite(approach_distance) || approach_distance <= 0.0 ||
      !std::isfinite(retract_distance) || retract_distance <= 0.0)
    {
      throw RecipeError(RecipeDiagnosticReason::INVALID_DISTANCE, "approach/retract distance");
    }

    CycleCoordinator::MissionRecipe recipe;
    recipe.recipe_id = recipe_id;
    recipe.target_id = recipe_target;
    recipe.schema_version = 1;
    recipe.grasp_width_mm = static_cast<float>(width);
    recipe.has_grasp_width = true;
    recipe.grasp_yaw_rad = yaw;
    recipe.has_grasp_yaw = true;
    recipe.place_pose.header.frame_id = frame;
    recipe.place_pose.pose.position.x = position[0];
    recipe.place_pose.pose.position.y = position[1];
    recipe.place_pose.pose.position.z = position[2];
    recipe.place_pose.pose.orientation.x = orientation[0];
    recipe.place_pose.pose.orientation.y = orientation[1];
    recipe.place_pose.pose.orientation.z = orientation[2];
    recipe.place_pose.pose.orientation.w = orientation[3];
    recipe.has_place_pose = true;
    recipe.has_place_tool_orientation_preference = has_tool_orientation_preference;
    if (has_tool_orientation_preference) {
      recipe.place_tool_orientation_preference.x = tool_orientation[0];
      recipe.place_tool_orientation_preference.y = tool_orientation[1];
      recipe.place_tool_orientation_preference.z = tool_orientation[2];
      recipe.place_tool_orientation_preference.w = tool_orientation[3];
    }
    recipe.place_position_tolerance_m = position_tolerance;
    recipe.place_orientation_tolerance_rad = orientation_tolerance;
    recipe.place_orientation_constraint = orientation_constraint;
    recipe.place_approach_direction_object.x = direction[0];
    recipe.place_approach_direction_object.y = direction[1];
    recipe.place_approach_direction_object.z = direction[2];
    recipe.place_approach_distance_m = approach_distance;
    recipe.retract_distance_m = retract_distance;
    return RecipeLoadResult{recipe, RecipeDiagnosticReason::NONE, source};
  } catch (const RecipeError & error) {
    return failed(source, error.reason());
  } catch (const Json::exception &) {
    return failed(source, RecipeDiagnosticReason::TYPE_MISMATCH);
  }
}

}  // namespace arm_cell_orchestration_bt
