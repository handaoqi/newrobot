#ifndef LOCALIZATION_CORRECTION_COOLDOWN_GATE_HPP
#define LOCALIZATION_CORRECTION_COOLDOWN_GATE_HPP

#include <algorithm>
#include <cstdint>

namespace localization {

class CorrectionCooldownGate {
public:
  void configure(double min_interval_seconds, double post_suppression_seconds) {
    min_interval_ns_ = secondsToNanoseconds(min_interval_seconds);
    post_suppression_ns_ = secondsToNanoseconds(post_suppression_seconds);
  }

  bool canStart(std::int64_t now_ns) const {
    return remainingNanoseconds(now_ns) == 0;
  }

  void markStarted(std::int64_t now_ns) {
    last_started_ns_ = std::max<std::int64_t>(0, now_ns);
  }

  void markCompleted(std::int64_t now_ns) {
    const auto safe_now = std::max<std::int64_t>(0, now_ns);
    suppressed_until_ns_ = safe_now + post_suppression_ns_;
  }

  double remainingSeconds(std::int64_t now_ns) const {
    return static_cast<double>(remainingNanoseconds(now_ns)) * 1e-9;
  }

  void reset() {
    last_started_ns_ = 0;
    suppressed_until_ns_ = 0;
  }

private:
  static std::int64_t secondsToNanoseconds(double seconds) {
    return static_cast<std::int64_t>(std::max(0.0, seconds) * 1e9);
  }

  std::int64_t remainingNanoseconds(std::int64_t now_ns) const {
    if (now_ns <= 0) {
      return 0;
    }
    const std::int64_t interval_until = last_started_ns_ > 0
      ? last_started_ns_ + min_interval_ns_ : 0;
    const std::int64_t blocked_until = std::max(interval_until, suppressed_until_ns_);
    return std::max<std::int64_t>(0, blocked_until - now_ns);
  }

  std::int64_t min_interval_ns_ = 0;
  std::int64_t post_suppression_ns_ = 0;
  std::int64_t last_started_ns_ = 0;
  std::int64_t suppressed_until_ns_ = 0;
};

}  // namespace localization

#endif  // LOCALIZATION_CORRECTION_COOLDOWN_GATE_HPP
