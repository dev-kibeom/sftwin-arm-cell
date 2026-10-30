#pragma once

#include <cstddef>
#include <vector>

namespace arm_cell_motion
{

enum class PickSearchEvaluation
{
  GRASP_IK_NO_SOLUTION,
  GRASP_IK_FAILED,
  GRASP_VALIDATION_FAILED,
  GRASP_CONTACT_MISMATCH,
  GRASP_COLLISION,
  APPROACH_IK_FAILED,
  APPROACH_VALIDATION_FAILED,
  BACKEND_ERROR,
  GLOBAL_PLANNING_FAILED,
  EXECUTABLE_APPROACH_SELECTED
};

inline const char * pick_search_evaluation_name(PickSearchEvaluation evaluation)
{
  switch (evaluation) {
    case PickSearchEvaluation::GRASP_IK_NO_SOLUTION: return "GRASP_IK_NO_SOLUTION";
    case PickSearchEvaluation::GRASP_IK_FAILED: return "GRASP_IK_FAILED";
    case PickSearchEvaluation::GRASP_VALIDATION_FAILED: return "GRASP_VALIDATION_FAILED";
    case PickSearchEvaluation::GRASP_CONTACT_MISMATCH: return "GRASP_CONTACT_MISMATCH";
    case PickSearchEvaluation::GRASP_COLLISION: return "GRASP_COLLISION";
    case PickSearchEvaluation::APPROACH_IK_FAILED: return "APPROACH_IK_FAILED";
    case PickSearchEvaluation::APPROACH_VALIDATION_FAILED: return "APPROACH_VALIDATION_FAILED";
    case PickSearchEvaluation::BACKEND_ERROR: return "BACKEND_ERROR";
    case PickSearchEvaluation::GLOBAL_PLANNING_FAILED: return "GLOBAL_PLANNING_FAILED";
    case PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED: return "EXECUTABLE_APPROACH_SELECTED";
  }
  return "UNKNOWN";
}

enum class PickSearchSeed
{
  MEASURED,
  DIVERSIFIED
};

enum class PickSearchStatus
{
  SEARCHING,
  SELECTED,
  ALL_CANDIDATES_EXHAUSTED,
  SEARCH_BUDGET_EXHAUSTED,
  INTERRUPTED,
  BACKEND_ERROR,
  SEEDS_EXHAUSTED
};

// Visits an already ordered pose list. Evaluators own geometry, solver calls,
// validation, trajectories, diagnostics and Motion result classification.
class PickSearchController
{
public:
  PickSearchController(
    std::size_t pose_count, std::size_t seed_limit, std::size_t first_pose = 0U)
  : seed_limit_(seed_limit), first_pose_(first_pose), pose_index_(first_pose),
    poses_(pose_count) {}

  // Called at the existing attempt boundary, after runtime interruption checks.
  // The caller supplies the remaining shared budget; advancing never resets it.
  PickSearchStatus next(double remaining_s, bool interrupted = false)
  {
    if (interrupted) {status_ = PickSearchStatus::INTERRUPTED;}
    if (status_ != PickSearchStatus::SEARCHING) {return status_;}
    if (remaining_s <= 0.0) {
      status_ = PickSearchStatus::SEARCH_BUDGET_EXHAUSTED;
    } else if (pose_index_ >= poses_.size()) {
      status_ = PickSearchStatus::ALL_CANDIDATES_EXHAUSTED;
    } else if (poses_[pose_index_].attempts >= seed_limit_) {
      status_ = PickSearchStatus::SEEDS_EXHAUSTED;
    } else {
      seed_attempt_ = ++poses_[pose_index_].attempts;
      ++attempt_count_;
    }
    return status_;
  }

  void evaluate(PickSearchEvaluation evaluation)
  {
    if (status_ != PickSearchStatus::SEARCHING) {return;}
    if (evaluation == PickSearchEvaluation::EXECUTABLE_APPROACH_SELECTED) {
      status_ = PickSearchStatus::SELECTED;
      return;
    }
    if (evaluation == PickSearchEvaluation::BACKEND_ERROR) {
      status_ = PickSearchStatus::BACKEND_ERROR;
      return;
    }
    auto & pose = poses_[pose_index_];
    if (evaluation == PickSearchEvaluation::GRASP_IK_NO_SOLUTION) {
      pose.deferred = pose.attempts < seed_limit_;
    } else if (evaluation != PickSearchEvaluation::GRASP_CONTACT_MISMATCH &&
      evaluation != PickSearchEvaluation::GLOBAL_PLANNING_FAILED &&
      pose.attempts < seed_limit_)
    {
      return;  // Preserve the existing alternate-solution retry for other validation failures.
    } else {
      pose.deferred = false;
    }
    ++pose_index_;
    if (!revisiting_ && pose_index_ >= poses_.size()) {
      revisiting_ = true;
      pose_index_ = first_pose_;
    }
    if (revisiting_) {
      while (pose_index_ < poses_.size() && !poses_[pose_index_].deferred) {++pose_index_;}
    }
  }

  PickSearchSeed seed_source() const
  {
    return seed_attempt_ > 1U ? PickSearchSeed::DIVERSIFIED : PickSearchSeed::MEASURED;
  }

  std::size_t pose_index() const {return pose_index_;}
  std::size_t seed_attempt() const {return seed_attempt_;}
  std::size_t attempt_count() const {return attempt_count_;}
  PickSearchStatus status() const {return status_;}

private:
  struct PoseProgress
  {
    std::size_t attempts{0U};
    bool deferred{false};
  };
  const std::size_t seed_limit_;
  const std::size_t first_pose_;
  std::size_t pose_index_;
  std::vector<PoseProgress> poses_;
  bool revisiting_{false};
  std::size_t seed_attempt_{0U};
  std::size_t attempt_count_{0U};
  PickSearchStatus status_{PickSearchStatus::SEARCHING};
};

}  // namespace arm_cell_motion
