#pragma once

#include <algorithm>
#include <chrono>

namespace arm_cell_motion
{

class PickSearchBudget
{
public:
  using Clock = std::chrono::steady_clock;

  explicit PickSearchBudget(double budget_s, Clock::time_point start = Clock::now())
  : start_(start), deadline_(start + std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(budget_s)))
  {}

  PickSearchBudget(
    double action_s, Clock::time_point action_start,
    double candidate_s, Clock::time_point candidate_start)
  : PickSearchBudget(action_s, action_start)
  {
    deadline_ = std::min(
      deadline_, candidate_start +
      std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(candidate_s)));
  }

  double remaining_s(Clock::time_point now = Clock::now()) const
  {
    return std::max(0.0, std::chrono::duration<double>(deadline_ - now).count());
  }

  double call_timeout_s(double configured_timeout_s, Clock::time_point now = Clock::now()) const
  {
    return std::min(configured_timeout_s, remaining_s(now));
  }

  double elapsed_s(Clock::time_point now = Clock::now()) const
  {
    return std::max(0.0, std::chrono::duration<double>(now - start_).count());
  }

private:
  Clock::time_point start_;
  Clock::time_point deadline_;
};

}  // namespace arm_cell_motion
