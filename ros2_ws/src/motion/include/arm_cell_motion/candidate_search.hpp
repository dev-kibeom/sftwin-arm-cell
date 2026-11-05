#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace arm_cell_motion
{

enum class CandidatePriorityPolicy
{
  DISTANCE_FIRST,
  HEIGHT_FIRST
};

// Values needed to order a candidate are deliberately independent of planning
// or feasibility checks. tray_inward_coordinate is the existing PICK axis
// coordinate (larger values are closer to the tray's inward side).
struct CandidatePriority
{
  std::size_t source_index{0};
  double tray_inward_coordinate{0.0};
  double observed_height_m{0.0};
  std::string stable_id;
};

std::vector<std::size_t> order_candidate_indices(
  const std::vector<CandidatePriority> & candidates,
  CandidatePriorityPolicy policy);

// A caller advances only after its own execution policy permits a transition.
// The cursor does not inspect planning, IK, collision, or execution results.
class CandidateSearchCursor final
{
public:
  void reset(std::vector<std::size_t> ordered_indices);
  bool select_next(std::size_t & source_index);
  std::size_t selected_count() const {return next_index_;}

private:
  std::vector<std::size_t> ordered_indices_;
  std::size_t next_index_{0};
};

}  // namespace arm_cell_motion
