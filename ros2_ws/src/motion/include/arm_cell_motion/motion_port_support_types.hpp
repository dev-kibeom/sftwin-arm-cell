#pragma once

#include <cstdint>
#include <string>

namespace arm_cell_motion
{

struct MaterialCandidateSearchAttempt
{
  std::string candidate_id;
  std::uint8_t result_code{0U};
  std::string failure_stage;
  bool grasp_insertion_started{false};
  bool search_began{true};
  std::string acquisition_id{};
  double search_elapsed_s{0.0};
};

}  // namespace arm_cell_motion
