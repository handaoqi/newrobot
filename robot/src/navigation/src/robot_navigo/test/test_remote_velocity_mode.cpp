#include <gtest/gtest.h>

#include "remote_velocity_mode.hpp"

namespace {

using robot_navigo::DecideRemoteVelocityDisposition;
using robot_navigo::HasLiveMotionCommand;
using robot_navigo::IsZeroPlanarVelocity;
using robot_navigo::kNavResponsiveStick;
using robot_navigo::kNavStickVxFullScale;
using robot_navigo::kNavStickYawFullScale;
using robot_navigo::NormalizeRemoteStick;
using robot_navigo::RemoteVelocityDisposition;
using robot_navigo::RequestedPosture;

TEST(RemoteVelocityMode, AllSixDirectionsAreDeliberateVelocityCommands) {
  EXPECT_FALSE(IsZeroPlanarVelocity(0.35, 0.0, 0.0));   // forward
  EXPECT_FALSE(IsZeroPlanarVelocity(-0.35, 0.0, 0.0));  // backward
  EXPECT_FALSE(IsZeroPlanarVelocity(0.0, 0.25, 0.0));   // left shift
  EXPECT_FALSE(IsZeroPlanarVelocity(0.0, -0.25, 0.0));  // right shift
  EXPECT_FALSE(IsZeroPlanarVelocity(0.0, 0.0, 0.45));   // left turn
  EXPECT_FALSE(IsZeroPlanarVelocity(0.0, 0.0, -0.45));  // right turn
  EXPECT_TRUE(IsZeroPlanarVelocity(0.0, 0.0, 0.0));
}

TEST(RemoteVelocityMode, ZeroOrStaleVelocityDoesNotEnterMoveMode) {
  EXPECT_FALSE(HasLiveMotionCommand(0.0, 0.0, 0.0, 0, 500));
  EXPECT_FALSE(HasLiveMotionCommand(0.35, 0.0, 0.0, 501, 500));
  EXPECT_TRUE(HasLiveMotionCommand(0.35, 0.0, 0.0, 500, 500));
}

TEST(RemoteVelocityMode, LowPostureUsesSitDownReadbackWithoutMoveMode) {
  EXPECT_EQ(DecideRemoteVelocityDisposition(
                RequestedPosture::kLow,
                static_cast<int32_t>(zsibot::ControlMode::CM_SIT_DOWN)),
            RemoteVelocityDisposition::kSendLowPostureVelocity);
}

TEST(RemoteVelocityMode, LowPostureWaitsForPostureTransition) {
  EXPECT_EQ(DecideRemoteVelocityDisposition(
                RequestedPosture::kLow,
                static_cast<int32_t>(zsibot::ControlMode::CM_MOVE_MODE)),
            RemoteVelocityDisposition::kWaitForLowPosture);
  EXPECT_EQ(DecideRemoteVelocityDisposition(
                RequestedPosture::kLow,
                static_cast<int32_t>(zsibot::ControlMode::CM_STAND_UP)),
            RemoteVelocityDisposition::kWaitForLowPosture);
}

TEST(RemoteVelocityMode, OrdinaryTeleopStillRequiresMoveMode) {
  EXPECT_EQ(DecideRemoteVelocityDisposition(
                RequestedPosture::kStanding,
                static_cast<int32_t>(zsibot::ControlMode::CM_STAND_UP)),
            RemoteVelocityDisposition::kRequestMoveMode);
  EXPECT_EQ(DecideRemoteVelocityDisposition(
                RequestedPosture::kStanding,
                static_cast<int32_t>(zsibot::ControlMode::CM_MOVE_MODE)),
            RemoteVelocityDisposition::kSendMoveVelocity);
}

TEST(RemoteStick, ManualTeleopStillLiftsDeadZoneStick) {
  EXPECT_FLOAT_EQ(NormalizeRemoteStick(0.30f, 3.0, 0.55f, true), 0.55f);
  EXPECT_FLOAT_EQ(NormalizeRemoteStick(0.35f, 5.25, 0.55f, true), 0.55f);
  EXPECT_FLOAT_EQ(NormalizeRemoteStick(0.0f, 5.25, 0.55f, true), 0.0f);
}

TEST(RemoteStick, AutonavMicroCommandsStayProportional) {
  const float forward = NormalizeRemoteStick(
      0.30f, kNavStickVxFullScale, 0.55f, false);
  const float yaw = NormalizeRemoteStick(
      0.35f, kNavStickYawFullScale, 0.55f, false);
  EXPECT_NEAR(forward, kNavResponsiveStick, 1e-5);
  EXPECT_GT(yaw, 0.25f);
  EXPECT_LT(yaw, 0.55f);
  EXPECT_NEAR(
      NormalizeRemoteStick(0.05f, kNavStickYawFullScale, 0.55f, false),
      0.05f / kNavStickYawFullScale, 1e-5);
}

TEST(RemoteVelocityMode, EmergencyStopAlwaysWins) {
  const auto emergency_stop =
      static_cast<int32_t>(zsibot::ControlMode::CM_EMERGENCY_STOP);
  EXPECT_EQ(DecideRemoteVelocityDisposition(RequestedPosture::kLow,
                                             emergency_stop),
            RemoteVelocityDisposition::kEmergencyStop);
  EXPECT_EQ(DecideRemoteVelocityDisposition(RequestedPosture::kStanding,
                                             emergency_stop),
            RemoteVelocityDisposition::kEmergencyStop);
}

}  // namespace
