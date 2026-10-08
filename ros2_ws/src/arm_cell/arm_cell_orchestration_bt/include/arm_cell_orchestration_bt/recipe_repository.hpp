#pragma once

#include "arm_cell_orchestration_bt/cycle_coordinator.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace arm_cell_orchestration_bt
{

enum class RecipeDiagnosticReason
{
  NONE,
  FILE_NOT_FOUND,
  JSON_MALFORMED,
  SCHEMA_UNSUPPORTED,
  REQUIRED_FIELD_MISSING,
  TYPE_MISMATCH,
  ARRAY_SIZE_INVALID,
  INVALID_ID,
  TARGET_MISMATCH,
  INVALID_FRAME,
  INVALID_POSE_COMPONENT,
  INVALID_QUATERNION,
  INVALID_APPROACH_DIRECTION,
  INVALID_DISTANCE,
  INVALID_TOLERANCE,
  INVALID_ORIENTATION_CONSTRAINT,
  INVALID_GRASP_WIDTH,
  INVALID_GRASP_YAW,
  INVALID_TOOL_ORIENTATION
};

const char * recipe_reason_token(RecipeDiagnosticReason reason);

struct RecipeLoadResult
{
  std::optional<CycleCoordinator::MissionRecipe> recipe;
  RecipeDiagnosticReason reason{RecipeDiagnosticReason::NONE};
  std::filesystem::path source;
};

class RecipeRepository
{
public:
  explicit RecipeRepository(std::filesystem::path directory);
  RecipeLoadResult load(const std::string & target_id) const;
  const std::filesystem::path & directory() const {return directory_;}

private:
  std::filesystem::path directory_;
};

}  // namespace arm_cell_orchestration_bt
