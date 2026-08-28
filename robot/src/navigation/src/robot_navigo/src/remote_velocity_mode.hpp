#pragma once

#include <cstdint>

#include "zsibot_define.h"

namespace robot_navigo {

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
