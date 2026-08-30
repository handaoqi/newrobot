#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace localization {

struct LioMotionSample {
  double yaw_rad = 0.0;
  std::int64_t stamp_ns = 0;
};

struct LioMotionGuardResult {
  bool anomaly = false;
  bool duplicate = false;
  std::string reason = "none";
  double yaw_step_rad = 0.0;
  double yaw_rate_radps = 0.0;
  double dt_s = 0.0;
};

inline double normalizeLioYaw(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline LioMotionGuardResult evaluateLioMotion(
  const LioMotionSample & previous,
  const LioMotionSample & current,
  double max_yaw_step_rad,
  double max_yaw_rate_radps)
{
  LioMotionGuardResult result;
  if (!std::isfinite(previous.yaw_rad) || !std::isfinite(current.yaw_rad)) {
    result.anomaly = true;
    result.reason = "non_finite_yaw";
    return result;
  }
  result.yaw_step_rad = std::fabs(normalizeLioYaw(current.yaw_rad - previous.yaw_rad));
  const std::int64_t dt_ns = current.stamp_ns - previous.stamp_ns;
  if (dt_ns == 0) {
    result.duplicate = result.yaw_step_rad <= 1.0e-9;
    result.anomaly = !result.duplicate;
    result.reason = result.duplicate ? "duplicate_sample" : "duplicate_stamp_pose_change";
    return result;
  }
  if (dt_ns < 0) {
    result.anomaly = true;
    result.reason = "non_monotonic_stamp";
    return result;
  }
  result.dt_s = static_cast<double>(dt_ns) * 1.0e-9;
  result.yaw_rate_radps = result.yaw_step_rad / result.dt_s;
  if (result.yaw_step_rad > max_yaw_step_rad) {
    result.anomaly = true;
    result.reason = "yaw_step_exceeded";
  } else if (!std::isfinite(result.yaw_rate_radps) ||
             result.yaw_rate_radps > max_yaw_rate_radps) {
    result.anomaly = true;
    result.reason = "yaw_rate_exceeded";
  }
  return result;
}

}  // namespace localization
