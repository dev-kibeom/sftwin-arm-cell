#include "arm_cell_motion/candidate_search.hpp"

#include <algorithm>
#include <numeric>
#include <utility>

namespace arm_cell_motion
{

std::vector<std::size_t> order_candidate_indices(
  const std::vector<CandidatePriority> & candidates,
  CandidatePriorityPolicy policy)
{
  std::vector<std::size_t> order(candidates.size());
  std::iota(order.begin(), order.end(), 0U);
  if (policy == CandidatePriorityPolicy::HEIGHT_FIRST) {
    std::stable_sort(
      order.begin(), order.end(), [&candidates](std::size_t lhs, std::size_t rhs) {
        return candidates[lhs].observed_height_m > candidates[rhs].observed_height_m;
      });
  } else {
    // Keep the legacy PICK comparison exactly: Tray inward axis descending,
    // observed height descending, then candidate ID ascending.
    std::sort(
      order.begin(), order.end(), [&candidates](std::size_t lhs, std::size_t rhs) {
        const auto & a = candidates[lhs];
        const auto & b = candidates[rhs];
        if (a.tray_inward_coordinate != b.tray_inward_coordinate) {
          return a.tray_inward_coordinate > b.tray_inward_coordinate;
        }
        if (a.observed_height_m != b.observed_height_m) {
          return a.observed_height_m > b.observed_height_m;
        }
        return a.stable_id < b.stable_id;
      });
  }

  std::vector<std::size_t> source_order;
  source_order.reserve(order.size());
  for (const auto index : order) {
    source_order.push_back(candidates[index].source_index);
  }
  return source_order;
}

void CandidateSearchCursor::reset(std::vector<std::size_t> ordered_indices)
{
  ordered_indices_ = std::move(ordered_indices);
  next_index_ = 0;
}

bool CandidateSearchCursor::select_next(std::size_t & source_index)
{
  if (next_index_ >= ordered_indices_.size()) {return false;}
  source_index = ordered_indices_[next_index_++];
  return true;
}

}  // namespace arm_cell_motion
