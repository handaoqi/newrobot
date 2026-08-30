#ifndef LOCALIZATION_GLOBAL_RELOCALIZATION_POLICY_HPP
#define LOCALIZATION_GLOBAL_RELOCALIZATION_POLICY_HPP

#include <cstdint>

namespace localization {

enum class GlobalRelocalizationResultDisposition {
  kApplySuccess,
  kApplyFailure,
  kDiscardStale,
  kDiscardTimedOut,
};

inline bool globalRelocalizationRequestReady(
  std::int64_t now_ns,
  std::int64_t earliest_start_ns) {
  return earliest_start_ns <= 0 || now_ns >= earliest_start_ns;
}

inline GlobalRelocalizationResultDisposition decideGlobalRelocalizationResult(
  std::uint64_t current_generation,
  std::uint64_t result_generation,
  bool success,
  double elapsed_ms,
  double timeout_seconds) {
  if (result_generation != current_generation) {
    return GlobalRelocalizationResultDisposition::kDiscardStale;
  }
  if (timeout_seconds > 0.0 && elapsed_ms > timeout_seconds * 1000.0) {
    return GlobalRelocalizationResultDisposition::kDiscardTimedOut;
  }
  return success
    ? GlobalRelocalizationResultDisposition::kApplySuccess
    : GlobalRelocalizationResultDisposition::kApplyFailure;
}

}  // namespace localization

#endif  // LOCALIZATION_GLOBAL_RELOCALIZATION_POLICY_HPP
