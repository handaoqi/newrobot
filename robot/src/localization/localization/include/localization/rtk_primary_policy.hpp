#ifndef LOCALIZATION_RTK_PRIMARY_POLICY_HPP
#define LOCALIZATION_RTK_PRIMARY_POLICY_HPP

#include <algorithm>
#include <cmath>
#include <string>

namespace localization {

struct RtkPrimaryLatchConfig {
  // Require sustained dual-antenna heading before GPS becomes the continuous
  // pose. Brief heading flashes must not promote and then demote again.
  int promote_samples = 20;
  int demote_samples = 8;
};

struct RtkPrimaryLatchState {
  bool latched = false;
  int good_frames = 0;
  int bad_frames = 0;
};

// Fixed RTK XY is enough to keep outdoor navigation alive. Dual-antenna
// heading is only required when GPS itself drives the continuous pose.
inline bool rtkPositionGoodForNavigation(bool usable, const std::string& quality) {
  return usable && quality == "fixed";
}

inline bool rtkGoodForPrimaryDrive(
    bool usable,
    bool heading_usable,
    const std::string& quality) {
  return rtkPositionGoodForNavigation(usable, quality) && heading_usable;
}

inline bool rtkPrimaryShouldDrive(
    bool source_arbiter_enable,
    bool bridge_active,
    bool latched) {
  return source_arbiter_enable && !bridge_active && latched;
}

// Dual-antenna heading ~180 deg from the current pose is a frame/offset
// error, not LIO drift. GPS may still drive XY; yaw stays on LIO/IMU so
// FollowPath walks the route forward instead of reversing toward a goal
// that looks like it is behind the robot.
inline bool rtkHeadingIsFlip(float drift_yaw_rad) {
  return std::isfinite(drift_yaw_rad) &&
    drift_yaw_rad > static_cast<float>(M_PI / 2.0);
}

// Dual-antenna yaw may only nudge the vehicle heading inside the LIO gate.
// Self-stable fixed RTK is allowed to drive GPS XY; it is not a reason to
// replace lidar/IMU yaw with a 60-90 deg antenna residual. A flip past 90
// deg is never trusted.
inline bool rtkHeadingTrustedForCorrection(
    bool heading_usable,
    float drift_yaw_rad,
    float max_correction_yaw_rad,
    bool /*rtk_self_stable*/) {
  if (!heading_usable || !std::isfinite(drift_yaw_rad) || rtkHeadingIsFlip(drift_yaw_rad)) {
    return false;
  }
  // angularDistance is non-negative; NaN comparisons fail closed.
  return drift_yaw_rad <= max_correction_yaw_rad;
}

// Cruise keeps FAST-LIO/IMU yaw. A ~12 deg dual-antenna residual is inside
// both the 30 deg correction gate and the 90 deg flip gate, but it is enough
// to yank a live Nav2 goal off the planned line. Stopped waypoint correction
// may still use a trusted RTK heading.
inline bool rtkHeadingAllowedForCorrection(bool moving) {
  return !moving;
}

// Latch GPS as the continuous navigation source only while fixed RTK and a
// valid dual-antenna heading are sustained. FAST-LIO remains the continuous
// source when heading flickers; RTK XY still corrects as an auxiliary.
// Do not promote while cruising: a live Nav2 goal must be cancelled, the
// pose jump checked, and the controller reset before a new source may drive
// odometry. Demote still happens immediately so a dead GPS stream cannot
// keep publishing.
inline void updateRtkPrimaryLatch(
    RtkPrimaryLatchState& state,
    const RtkPrimaryLatchConfig& config,
    bool rtk_good_for_primary_drive,
    bool moving = false) {
  const int promote_samples = std::max(1, config.promote_samples);
  const int demote_samples = std::max(1, config.demote_samples);
  if (rtk_good_for_primary_drive) {
    state.bad_frames = 0;
    state.good_frames = std::min(state.good_frames + 1, promote_samples);
    if (!state.latched && state.good_frames >= promote_samples) {
      if (!moving) {
        state.latched = true;
      }
    }
    return;
  }
  state.good_frames = 0;
  state.bad_frames = std::min(state.bad_frames + 1, demote_samples);
  if (state.latched && state.bad_frames >= demote_samples) {
    state.latched = false;
  }
}

}  // namespace localization

#endif
