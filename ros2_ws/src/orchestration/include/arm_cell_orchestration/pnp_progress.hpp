#pragma once

#include <cstdint>
#include <optional>

namespace arm_cell_orchestration
{
// Console projection only: terminal observation precedes the coordinator's count commit.
// A placed material can still have a failed final HOME; do not call that cycle completed.
struct PnpProgress
{
  struct Summary {double elapsed_s; bool completed;};
  double elapsed_s{0.0};
  bool completed{false};
  std::uint64_t reported_count{0U};

  std::optional<Summary> commit(std::uint64_t count)
  {
    if (count <= reported_count) {return std::nullopt;}
    reported_count = count;
    return Summary{elapsed_s, completed};
  }
};
}  // namespace arm_cell_orchestration
