#ifndef LOCALIZATION_SCAN_MATCH_POLICY_HPP
#define LOCALIZATION_SCAN_MATCH_POLICY_HPP

#include <algorithm>
#include <cmath>

namespace localization {

struct ScanMatchRefinePolicy {
  float skip_ndt_score = 0.15f;
  int zero_inlier_skip_count = 2;
  float min_improvement_ratio = 0.05f;
  float max_translation_disagreement_m = 0.30f;
  float max_rotation_disagreement_rad = 0.0872665f;
};

inline bool shouldRunRefinement(
    bool allow_high_quality_skip,
    bool ndt_acceptable,
    float ndt_fitness,
    int consecutive_zero_inlier_count,
    const ScanMatchRefinePolicy& policy) {
  if (policy.zero_inlier_skip_count > 0 &&
      consecutive_zero_inlier_count >= policy.zero_inlier_skip_count) {
    return false;
  }
  return !(allow_high_quality_skip && ndt_acceptable &&
    std::isfinite(ndt_fitness) && ndt_fitness <= policy.skip_ndt_score);
}

inline bool shouldChooseRefinement(
    bool ndt_acceptable,
    float ndt_fitness,
    bool refine_acceptable,
    float refine_fitness,
    float translation_disagreement_m,
    float rotation_disagreement_rad,
    const ScanMatchRefinePolicy& policy) {
  if (!refine_acceptable || !std::isfinite(refine_fitness)) {
    return false;
  }
  if (!ndt_acceptable || !std::isfinite(ndt_fitness)) {
    return true;
  }
  const bool consistent = std::isfinite(translation_disagreement_m) &&
    std::isfinite(rotation_disagreement_rad) &&
    translation_disagreement_m <= policy.max_translation_disagreement_m &&
    rotation_disagreement_rad <= policy.max_rotation_disagreement_rad;
  const float required_score = ndt_fitness *
    (1.0f - std::clamp(policy.min_improvement_ratio, 0.0f, 0.90f));
  return consistent && refine_fitness <= required_score;
}

}  // namespace localization

#endif  // LOCALIZATION_SCAN_MATCH_POLICY_HPP
