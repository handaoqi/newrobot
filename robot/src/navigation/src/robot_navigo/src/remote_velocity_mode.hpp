#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "zsibot_define.h"

namespace robot_navigo {

// Autonav micro caps from Edge navigation_speed.py, mapped onto the lowest
// observed responsive virtual-remote stick (0.45). Dividing 0.30 m/s and
// 0.525 rad/s by the 3.0 / 5.25 teleop ceiling left a ~0.10 stick that the
// 0.55 floor then lifted into a spin.
constexpr float kNavResponsiveStick = 0.45f;
constexpr double kNavStickVxFullScale = 0.30 / 0.45;
constexpr double kNavStickVyFullScale = 0.225 / 0.45;
constexpr double kNavStickYawFullScale = 0.525 / 0.45;

inline float NormalizeRemoteStick(float value, double max_value, float min_stick,
                                  bool lift_to_min) {
  if (!(max_value > 0.0) || !std::isfinite(max_value) || !std::isfinite(value)) {
    return 0.0f;
  }
  float stick = std::clamp(static_cast<float>(value / max_value), -1.0f, 1.0f);
  if (std::fabs(stick) <= 1e-4f) {
    return 0.0f;
  }
  if (std::fabs(stick) < min_stick) {
    return lift_to_min ? std::copysign(min_stick, stick) : stick;
  }
  return stick;
}

enum class RequestedPosture {
  kNone,
  kStanding,
  kLow,
};

enum class RemoteVelocityDisposition {
  kEmergencyStop,
  kWaitForLowPosture,
  kSendLowPostureVelocity,
  kRequestMoveMode,
  kSendMoveVelocity,
};

constexpr bool IsZeroPlanarVelocity(double vx, double vy, double yaw_rate,
                                    double epsilon = 1e-6) {
  return vx > -epsilon && vx < epsilon && vy > -epsilon && vy < epsilon &&
         yaw_rate > -epsilon && yaw_rate < epsilon;
}

constexpr bool HasLiveMotionCommand(double vx, double vy, double yaw_rate,
                                    int64_t age_ms, int64_t timeout_ms) {
  return age_ms <= timeout_ms &&
         !IsZeroPlanarVelocity(vx, vy, yaw_rate);
}

// The UI's prone/crawl posture is entered with CMD_SIT_DOWN. Its controller
// readback is CM_SIT_DOWN; that readback must not be converted into
// CMD_MOVE_MODE, because doing so exits the low posture and stands the dog.
constexpr RemoteVelocityDisposition DecideRemoteVelocityDisposition(
    RequestedPosture requested_posture, int32_t control_mode) {
  if (control_mode ==
      static_cast<int32_t>(zsibot::ControlMode::CM_EMERGENCY_STOP)) {
    return RemoteVelocityDisposition::kEmergencyStop;
  }
  if (requested_posture == RequestedPosture::kLow) {
    return control_mode ==
               static_cast<int32_t>(zsibot::ControlMode::CM_SIT_DOWN)
               ? RemoteVelocityDisposition::kSendLowPostureVelocity
               : RemoteVelocityDisposition::kWaitForLowPosture;
  }
  return control_mode ==
                 static_cast<int32_t>(zsibot::ControlMode::CM_MOVE_MODE)
             ? RemoteVelocityDisposition::kSendMoveVelocity
             : RemoteVelocityDisposition::kRequestMoveMode;
}

}  // namespace robot_navigo
