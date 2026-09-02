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

// Self-stable fixed RTK may correct LIO yaw even when the residual exceeds the
// normal max_correction_yaw gate. Without this, XY-only corrections leave a
// permanent yaw death spiral once LIO and dual-antenna heading disagree by
// more than ~30 deg.
inline bool rtkHeadingTrustedForCorrection(
    bool heading_usable,
    float drift_yaw_rad,
    float max_correction_yaw_rad,
    bool rtk_self_stable) {
  if (!heading_usable) {
    return false;
  }
  if (rtk_self_stable) {
    return true;
  }
  // angularDistance is non-negative; NaN comparisons fail closed.
  return drift_yaw_rad <= max_correction_yaw_rad;
}

// Latch GPS as the continuous navigation source only while fixed RTK and a
// valid dual-antenna heading are sustained. FAST-LIO remains the continuous
// source when heading flickers; RTK XY still corrects as an auxiliary.
inline void updateRtkPrimaryLatch(
    RtkPrimaryLatchState& state,
    const RtkPrimaryLatchConfig& config,
    bool rtk_good_for_primary_drive) {
  const int promote_samples = std::max(1, config.promote_samples);
  const int demote_samples = std::max(1, config.demote_samples);
  if (rtk_good_for_primary_drive) {
    state.bad_frames = 0;
    state.good_frames = std::min(state.good_frames + 1, promote_samples);
    if (!state.latched && state.good_frames >= promote_samples) {
      state.latched = true;
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
