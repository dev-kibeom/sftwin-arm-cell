#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <utility>

namespace arm_cell_orchestration
{

class MotionFeedbackSupervision
{
public:
  using GoalId = std::array<std::uint8_t, 16>;
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;
  static constexpr auto kSilenceLimit = std::chrono::milliseconds(1500);

  enum class Observation {IGNORED, RECEIVED, RECOVERED};

  void accept(const GoalId & id, TimePoint now)
  {
    std::lock_guard<std::mutex> forward_lock(forward_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    goal_id_ = id;
    last_feedback_ = now;
    active_ = true;
    stale_ = false;
    has_phase_ = false;
  }

  Observation observe(const GoalId & id, std::uint8_t phase, TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || id != goal_id_) {
      return Observation::IGNORED;
    }
    const auto recovered = stale_;
    last_gap_ = recovered && now >= last_feedback_ ? now - last_feedback_ : Clock::duration::zero();
    last_feedback_ = now;
    phase_ = phase;
    has_phase_ = true;
    stale_ = false;
    return recovered ? Observation::RECOVERED : Observation::RECEIVED;
  }

  template<typename Forward>
  Observation observe_and_forward(
    const GoalId & id, std::uint8_t phase, TimePoint now, Forward && forward)
  {
    std::lock_guard<std::mutex> forward_lock(forward_mutex_);
    const auto observation = observe(id, phase, now);
    if (observation != Observation::IGNORED) {
      std::forward<Forward>(forward)(observation, phase);
    }
    return observation;
  }

  bool stale(TimePoint now)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || stale_ || now < last_feedback_ ||
      now - last_feedback_ < kSilenceLimit)
    {
      return false;
    }
    stale_ = true;
    return true;
  }

  bool active() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
  }
  bool has_phase() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_phase_;
  }
  std::uint8_t phase() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return phase_;
  }
  TimePoint last_feedback() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_feedback_;
  }
  Clock::duration last_gap() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_gap_;
  }

  bool terminal(const GoalId & id)
  {
    std::lock_guard<std::mutex> forward_lock(forward_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || id != goal_id_) {
      return false;
    }
    active_ = false;
    stale_ = false;
    return true;
  }

private:
  // Serializes upper-layer Feedback delivery with Goal acceptance/termination.
  std::mutex forward_mutex_;
  mutable std::mutex mutex_;
  GoalId goal_id_{};
  TimePoint last_feedback_{};
  Clock::duration last_gap_{};
  std::uint8_t phase_{0};
  bool active_{false};
  bool stale_{false};
  bool has_phase_{false};
};

}  // namespace arm_cell_orchestration
