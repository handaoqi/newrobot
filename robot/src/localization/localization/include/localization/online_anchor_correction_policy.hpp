#ifndef LOCALIZATION_ONLINE_ANCHOR_CORRECTION_POLICY_HPP
#define LOCALIZATION_ONLINE_ANCHOR_CORRECTION_POLICY_HPP

#include <cmath>
#include <string>

namespace localization {

struct OnlineAnchorCorrectionConfig {
  bool enable = true;
  double max_linear_speed_mps = 0.15;
  double max_yaw_rate_radps = 0.10;
  double min_residual_xy_m = 0.30;
  double max_residual_xy_m = 1.00;
  double min_residual_yaw_rad = 5.0 * M_PI / 180.0;
  double max_residual_yaw_rad = 10.0 * M_PI / 180.0;
};

struct OnlineAnchorCorrectionInput {
  bool policy_allowed = false;
  bool moving = false;
  bool lio_fresh = false;
  bool lio_anomaly = false;
  bool smoothing_active = false;
  bool nominal_profile = true;
  bool motion_valid = false;
  double linear_speed_mps = 0.0;
  double yaw_rate_radps = 0.0;
};

inline std::string onlineAnchorCorrectionRejectionReason(
    const OnlineAnchorCorrectionConfig& config,
    const OnlineAnchorCorrectionInput& input) {
  if (!config.enable) return "online_correction_disabled";
  if (!input.policy_allowed || !input.moving) return "online_policy_not_allowed";
  if (!input.lio_fresh) return "lio_stale";
  if (input.lio_anomaly) return "lio_motion_anomaly";
  if (input.smoothing_active) return "correction_already_active";
  if (!input.nominal_profile) return "fusion_profile_not_nominal";
  if (!input.motion_valid) return "lio_motion_unavailable";
  if (!std::isfinite(input.linear_speed_mps) ||
      input.linear_speed_mps > config.max_linear_speed_mps) {
    return "linear_speed_exceeded";
  }
  if (!std::isfinite(input.yaw_rate_radps) ||
      input.yaw_rate_radps > config.max_yaw_rate_radps) {
    return "yaw_rate_exceeded";
  }
  return "allowed";
}

inline bool onlineAnchorCorrectionAllowed(
    const OnlineAnchorCorrectionConfig& config,
    const OnlineAnchorCorrectionInput& input) {
  return onlineAnchorCorrectionRejectionReason(config, input) == "allowed";
}

inline bool onlineAnchorCorrectionResidualInRange(
    const OnlineAnchorCorrectionConfig& config,
    double residual_xy_m,
    double residual_yaw_rad) {
  const bool xy_in_range = std::isfinite(residual_xy_m) &&
    residual_xy_m >= config.min_residual_xy_m &&
    residual_xy_m < config.max_residual_xy_m;
  const bool yaw_in_range = std::isfinite(residual_yaw_rad) &&
    residual_yaw_rad >= config.min_residual_yaw_rad &&
    residual_yaw_rad < config.max_residual_yaw_rad;
  return xy_in_range || yaw_in_range;
}

inline bool onlineAnchorCorrectionResidualSevere(
    const OnlineAnchorCorrectionConfig& config,
    double residual_xy_m,
    double residual_yaw_rad) {
  return (std::isfinite(residual_xy_m) && residual_xy_m >= config.max_residual_xy_m) ||
    (std::isfinite(residual_yaw_rad) && residual_yaw_rad >= config.max_residual_yaw_rad);
}

}  // namespace localization

#endif
