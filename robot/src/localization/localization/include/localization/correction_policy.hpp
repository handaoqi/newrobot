#ifndef LOCALIZATION_CORRECTION_POLICY_HPP
#define LOCALIZATION_CORRECTION_POLICY_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <limits>
#include <string>

namespace localization {

enum class CorrectionPolicyMode { ndt, rtk, ukf };
enum class CorrectionSource { none, ndt, rtk, ukf_fused, conflict };

inline const char* correctionSourceName(CorrectionSource source) {
  switch (source) {
    case CorrectionSource::ndt:
      return "ndt_vgicp";
    case CorrectionSource::rtk:
      return "rtk";
    case CorrectionSource::ukf_fused:
      return "ukf_fused";
    case CorrectionSource::conflict:
      return "conflict";
    case CorrectionSource::none:
    default:
      return "none";
  }
}

inline CorrectionPolicyMode parseCorrectionPolicyMode(const std::string& value) {
  std::string normalized = value;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  if (normalized == "rtk") {
    return CorrectionPolicyMode::rtk;
  }
  if (normalized == "ukf") {
    return CorrectionPolicyMode::ukf;
  }
  return CorrectionPolicyMode::ndt;
}

inline const char* correctionPolicyModeName(CorrectionPolicyMode mode) {
  switch (mode) {
    case CorrectionPolicyMode::rtk:
      return "rtk";
    case CorrectionPolicyMode::ukf:
      return "ukf";
    case CorrectionPolicyMode::ndt:
    default:
      return "ndt";
  }
}

inline bool correctionPolicyAllowsNdt(CorrectionPolicyMode mode) {
  return mode != CorrectionPolicyMode::rtk;
}

inline bool correctionPolicyAllowsRtk(CorrectionPolicyMode mode) {
  return mode != CorrectionPolicyMode::ndt;
}

struct CorrectionCandidateSummary {
  bool eligible = false;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  bool yaw_valid = false;
  double horizontal_variance = std::numeric_limits<double>::infinity();
  double orientation_variance = std::numeric_limits<double>::infinity();
  double residual_xy = 0.0;
  double residual_yaw = 0.0;
  std::int64_t stamp_ns = 0;
};

struct CorrectionSelection {
  CorrectionSource source = CorrectionSource::none;
  std::string reason = "no_eligible_source";
};

inline double correctionCandidateMetric(
    const CorrectionCandidateSummary& candidate,
    double drift_xy_m,
    double drift_yaw_rad) {
  const bool yaw_only = candidate.yaw_valid && candidate.residual_xy < drift_xy_m &&
    candidate.residual_yaw >= drift_yaw_rad;
  return yaw_only ? candidate.orientation_variance : candidate.horizontal_variance;
}

inline CorrectionSelection selectCorrectionSource(
    CorrectionPolicyMode mode,
    const CorrectionCandidateSummary& ndt,
    const CorrectionCandidateSummary& rtk,
    double conflict_xy_m,
    double conflict_yaw_rad,
    double drift_xy_m,
    double drift_yaw_rad,
    bool prefer_fixed_rtk = false) {
  if (mode == CorrectionPolicyMode::ndt) {
    return ndt.eligible
      ? CorrectionSelection{CorrectionSource::ndt, "ndt_policy_selected"}
      : CorrectionSelection{};
  }
  if (mode == CorrectionPolicyMode::rtk) {
    return rtk.eligible
      ? CorrectionSelection{CorrectionSource::rtk, "rtk_policy_selected"}
      : CorrectionSelection{};
  }
  if (!ndt.eligible && !rtk.eligible) {
    return CorrectionSelection{};
  }
  if (ndt.eligible && !rtk.eligible) {
    return {CorrectionSource::ndt, "ukf_only_ndt_eligible"};
  }
  if (!ndt.eligible && rtk.eligible) {
    return {CorrectionSource::rtk, "ukf_only_rtk_eligible"};
  }

  // Fixed RTK and NDT both run, but a fixed RTK candidate is authoritative for
  // navigation pose regulation and lost-localization recovery. Prefer RTK even
  // when the sources disagree or NDT reports a lower variance.
  if (prefer_fixed_rtk) {
    return {CorrectionSource::rtk, "ukf_prefer_fixed_rtk"};
  }

  // UKF mode deliberately does not reject two individually valid sources
  // merely because they disagree. Their source-specific freshness, quality
  // and jump gates run before this selector; the anchor filter then weights
  // accepted observations by their covariance.
  (void)conflict_xy_m;
  (void)conflict_yaw_rad;

  const double ndt_metric = correctionCandidateMetric(ndt, drift_xy_m, drift_yaw_rad);
  const double rtk_metric = correctionCandidateMetric(rtk, drift_xy_m, drift_yaw_rad);
  constexpr double kTieTolerance = 1.0e-9;
  if (ndt_metric + kTieTolerance < rtk_metric) {
    return {CorrectionSource::ndt, "ukf_ndt_lower_variance"};
  }
  if (rtk_metric + kTieTolerance < ndt_metric) {
    return {CorrectionSource::rtk, "ukf_rtk_lower_variance"};
  }
  return ndt.stamp_ns >= rtk.stamp_ns
    ? CorrectionSelection{CorrectionSource::ndt, "ukf_variance_tie_ndt_fresher"}
    : CorrectionSelection{CorrectionSource::rtk, "ukf_variance_tie_rtk_fresher"};
}

}  // namespace localization

#endif  // LOCALIZATION_CORRECTION_POLICY_HPP
