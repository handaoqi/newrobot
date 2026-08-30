#ifndef LOCALIZATION_POINT_CLOUD_SCHEDULER_HPP
#define LOCALIZATION_POINT_CLOUD_SCHEDULER_HPP

#include <algorithm>
#include <cstdint>
#include <string>

namespace localization {

struct PointCloudScheduleConfig {
  int initialization_stride = 2;
  int stationary_stride = 5;
  int moving_stride = 5;
  double stable_max_rate_hz = 2.0;
  double recovery_max_rate_hz = 5.0;
};

struct PointCloudScheduleInput {
  bool initialized = false;
  bool lidar_matching_paused = false;
  bool rtk_primary = false;
  bool global_relocalization_requested = false;
  bool lidar_odometry_required = false;
  bool lio_primary_enabled = false;
  bool lio_stable = false;
  bool correction_suppressed = false;
  std::string motion_phase = "stationary";
  std::uint64_t frame_index = 0;
  std::int64_t now_ns = 0;
  std::int64_t last_ndt_start_ns = 0;
};

struct PointCloudWorkDecision {
  bool run_ndt = false;
  bool needs_heavy_cloud = false;
  int stride = 1;
  bool stride_due = false;
  bool rate_due = true;
  std::string reason = "idle";
};

inline bool rateLimitDue(
    std::int64_t now_ns,
    std::int64_t last_start_ns,
    double max_rate_hz) {
  if (max_rate_hz <= 0.0 || last_start_ns <= 0 || now_ns <= 0) {
    return true;
  }
  if (now_ns < last_start_ns) {
    return false;
  }
  const double elapsed_seconds = static_cast<double>(now_ns - last_start_ns) * 1e-9;
  return elapsed_seconds + 1e-9 >= 1.0 / max_rate_hz;
}

inline PointCloudWorkDecision decidePointCloudWork(
    const PointCloudScheduleConfig& config,
    const PointCloudScheduleInput& input) {
  PointCloudWorkDecision decision;
  if (!input.lio_primary_enabled) {
    // Preserve legacy NDT-primary behavior: initialization/stationary matching
    // remains continuous, while the existing moving stride is retained.
    decision.stride = input.initialized && input.motion_phase == "moving"
      ? std::max(1, config.moving_stride)
      : 1;
  } else if (!input.initialized || !input.lio_stable) {
    decision.stride = std::max(1, config.initialization_stride);
  } else if (input.motion_phase == "moving") {
    decision.stride = std::max(1, config.moving_stride);
  } else {
    decision.stride = std::max(1, config.stationary_stride);
  }

  decision.stride_due =
    input.frame_index % static_cast<std::uint64_t>(decision.stride) == 0;
  const double max_rate_hz = input.lio_primary_enabled
    ? (input.lio_stable ? config.stable_max_rate_hz : config.recovery_max_rate_hz)
    : 0.0;
  decision.rate_due = rateLimitDue(input.now_ns, input.last_ndt_start_ns, max_rate_hz);
  const bool stable_correction_suppressed = input.lio_primary_enabled && input.lio_stable &&
    input.correction_suppressed;
  decision.run_ndt = !input.lidar_matching_paused && !input.rtk_primary &&
    !stable_correction_suppressed && decision.stride_due && decision.rate_due;
  decision.needs_heavy_cloud = !input.lidar_matching_paused &&
    (decision.run_ndt || input.global_relocalization_requested ||
      input.lidar_odometry_required);
  if (input.lidar_matching_paused || input.rtk_primary) {
    decision.reason = "rtk_primary_paused";
  } else if (stable_correction_suppressed) {
    decision.reason = "correction_suppressed";
  } else if (!decision.stride_due) {
    decision.reason = "stride_skip";
  } else if (!decision.rate_due) {
    decision.reason = "rate_limited";
  } else if (decision.run_ndt) {
    decision.reason = input.lio_stable ? "stable_match" : "recovery_match";
  } else if (decision.needs_heavy_cloud) {
    decision.reason = "forced_heavy";
  }
  return decision;
}

inline bool isPointCloudStale(
    std::int64_t now_ns,
    std::int64_t cloud_stamp_ns,
    double max_age_seconds,
    bool initialized) {
  if (!initialized || max_age_seconds <= 0.0 || now_ns <= 0 || cloud_stamp_ns <= 0) {
    return false;
  }
  const std::int64_t age_ns = now_ns - cloud_stamp_ns;
  if (age_ns < 0) {
    return false;
  }
  return static_cast<double>(age_ns) * 1e-9 > max_age_seconds;
}

}  // namespace localization

#endif  // LOCALIZATION_POINT_CLOUD_SCHEDULER_HPP
