#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
#include "remote_velocity_mode.hpp"
#include "robots_dog_msgs/msg/localization.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "zsl-1/highlevel.h"
#include "zsibot_api.h"

class VelCmdUdpPublisher : public rclcpp::Node {
 public:
  VelCmdUdpPublisher()
      : Node("vel_cmd_udp_publisher") {
    teleop_action_subscriber_ =
        this->create_subscription<std_msgs::msg::String>(
            "/teleop_action", 10,
            std::bind(&VelCmdUdpPublisher::HandleTeleopActionCallback, this,
                      std::placeholders::_1));
    remote_teleop_action_subscriber_ =
        this->create_subscription<std_msgs::msg::String>(
            "/remote_teleop_action", 10,
            std::bind(&VelCmdUdpPublisher::HandleRemoteTeleopActionCallback,
                      this, std::placeholders::_1));
    motion_state_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/robot_motion_state", 10);
    control_mode_publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("/robot_ctrl_mode", 10);
    fine_control_subscriber_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/navigation/fine_control",
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
              std::lock_guard<std::mutex> lk(mutex_);
              fine_control_ = msg->data;
              RCLCPP_INFO(this->get_logger(),
                          "navigation fine-control profile %s (minimum stick %.2f)",
                          fine_control_ ? "enabled" : "disabled",
                          fine_control_ ? remote_fine_min_stick_ : remote_min_stick_);
            });

    this->declare_parameter("platform", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("client_ip", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("server_ip", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("client_port", 43988);
    this->declare_parameter("server_port", 43997);
    this->declare_parameter("standup_settle_ms", 4000);
    this->declare_parameter("standup_retry_ms", 5000);
    this->declare_parameter("cmd_timeout_ms", 500);
    this->declare_parameter("inactive_linger_ms", 8000);
    // Keep the vendor virtual-remote stream at 50 Hz. Browser/MQTT commands
    // update the target velocity less frequently; this timer maintains the
    // joystick frames without turning every hold update into a cloud command.
    this->declare_parameter("publish_period_ms", 20);
    this->declare_parameter("sdk_max_vx", 0.5);
    this->declare_parameter("sdk_max_vy", 0.5);
    this->declare_parameter("sdk_max_yaw_rate", 0.5);
    // These scales apply only to virtual-remote teleoperation. Keep the SDK
    // navigation limits above unchanged.
    this->declare_parameter("remote_full_scale_vx", 3.0);
    this->declare_parameter("remote_full_scale_vy", 2.25);
    this->declare_parameter("remote_full_scale_yaw_rate", 5.25);
    // 0.25 remains inside the XG virtual-remote dead zone.  0.45 is the
    // lowest observed responsive command while remaining below the normal
    // navigation floor (0.55) for final docking corrections.
    this->declare_parameter("remote_fine_min_stick", 0.45);
    this->declare_parameter("turn_linear_limit_yaw_rate", 1.0);
    this->declare_parameter("turn_max_linear_speed", 0.5);
    this->declare_parameter("manual_override_ms", 650);
    this->declare_parameter("remote_control_only", false);
    this->declare_parameter("localization_topic",
                            std::string("/localization_info"));
    this->declare_parameter("localization_timeout_ms", 500);
    this->get_parameter("platform", platform_);
    this->get_parameter("client_ip", client_ip_);
    this->get_parameter("server_ip", server_ip_);
    this->get_parameter("client_port", client_port_);
    this->get_parameter("server_port", server_port_);
    this->get_parameter("standup_settle_ms", standup_settle_ms_);
    this->get_parameter("standup_retry_ms", standup_retry_ms_);
    this->get_parameter("cmd_timeout_ms", cmd_timeout_ms_);
    this->get_parameter("inactive_linger_ms", inactive_linger_ms_);
    this->get_parameter("publish_period_ms", publish_period_ms_);
    this->get_parameter("sdk_max_vx", sdk_max_vx_);
    this->get_parameter("sdk_max_vy", sdk_max_vy_);
    this->get_parameter("sdk_max_yaw_rate", sdk_max_yaw_rate_);
    this->get_parameter("remote_full_scale_vx", remote_full_scale_vx_);
    this->get_parameter("remote_full_scale_vy", remote_full_scale_vy_);
    this->get_parameter("remote_full_scale_yaw_rate", remote_full_scale_yaw_rate_);
    this->get_parameter("remote_fine_min_stick", remote_fine_min_stick_);
    this->get_parameter("turn_linear_limit_yaw_rate",
                        turn_linear_limit_yaw_rate_);
    this->get_parameter("turn_max_linear_speed", turn_max_linear_speed_);
    this->get_parameter("manual_override_ms", manual_override_ms_);
    this->get_parameter("remote_control_only", remote_control_only_);
    this->get_parameter("localization_topic", localization_topic_);
    this->get_parameter("localization_timeout_ms", localization_timeout_ms_);

    const char* velocity_topic = remote_control_only_ ? "/teleop_cmd_vel" : "/cmd_vel";
    planner_vel_cmd_subscriber_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            velocity_topic, 10,
            std::bind(&VelCmdUdpPublisher::HandlePlannerVelCallback, this,
                      std::placeholders::_1));
    if (!remote_control_only_) {
      // Keep a single SDK socket owner while Nav2 is running.  The remote
      // control page publishes here, so it does not need a second UDP bridge
      // that would collide with the navigation bridge's local port.
      teleop_vel_cmd_subscriber_ =
          this->create_subscription<geometry_msgs::msg::Twist>(
              "/teleop_cmd_vel", 10,
              std::bind(&VelCmdUdpPublisher::HandleTeleopVelCallback, this,
                        std::placeholders::_1));
    }
    if (!remote_control_only_) {
      mode_switch_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
          "/mode_switch_cmd", 10,
          std::bind(&VelCmdUdpPublisher::HandleModeSwitchCallback, this,
                    std::placeholders::_1));
      localization_subscriber_ =
          this->create_subscription<robots_dog_msgs::msg::Localization>(
              localization_topic_, 10,
              std::bind(&VelCmdUdpPublisher::HandleLocalizationCallback, this,
                        std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),
                  "navigation cmd_vel is held at zero unless localization "
                  "status=3 (topic %s, timeout %dms)",
                  localization_topic_.c_str(), localization_timeout_ms_);
    }

    const std::unordered_map<std::string, std::pair<std::string, std::string>>
        platform_map = {{"NX_XG3588", {"192.168.234.1", "192.168.234.234"}},
                        {"XG3588", {"127.0.0.1", "127.0.0.1"}},
                        {"UE", {"127.0.0.1", "127.0.0.1"}}};

    if (server_ip_.empty() || client_ip_.empty()) {
      auto it = platform_map.find(platform_);
      if (it == platform_map.end()) {
        RCLCPP_ERROR(this->get_logger(),
                     "\033[1;31mError: Unknown PLATFORM type\033[0m");
        rclcpp::shutdown();
        return;
      }

      server_ip_ = it->second.first;
      client_ip_ = it->second.second;
    }

    if (!remote_control_only_) {
      sdk_highlevel_.initRobot(client_ip_, client_port_, server_ip_);
      sdk_connected_ = sdk_highlevel_.checkConnect();
      RCLCPP_INFO(this->get_logger(),
                  "vel_cmd_udp_publisher started via SDK %s:%d -> %s:%d",
                  client_ip_.c_str(), client_port_, server_ip_.c_str(),
                  server_port_);
      RCLCPP_INFO(this->get_logger(), "SDK checkConnect=%s",
                  sdk_connected_ ? "true" : "false");
      if (!sdk_connected_) {
        RCLCPP_WARN(this->get_logger(),
                    "Native SDK is unavailable; navigation will use the vendor "
                    "virtual-remote transport after stand-up");
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "remote-only control bridge started on %s via zsibot remote protocol",
                  velocity_topic);
    }
    StartRemoteExecutorAsync();

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&VelCmdUdpPublisher::PublishLatestVelocity, this));
    control_mode_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&VelCmdUdpPublisher::PublishControlMode, this));
    PublishMotionState("passive");
  }

 private:
  void HandleModeSwitchCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);

    // Nav2's mode topic is also a periodic heartbeat.  It must never revoke
    // a live browser remote-control session by putting the body into passive.
    if (manual_teleop_active_) {
      return;
    }

    // A manual crawl is a body posture selected by the operator.  Nav2 keeps
    // publishing its mode heartbeat even when it has no goal, so it must not
    // convert that heartbeat into a stand-up command.  A later explicit
    // stand_up action (used by task preparation) clears this lock.
    if (manual_crawl_lock_) {
      return;
    }

    if (msg->data == 171) {
      if (crawl_mode_) {
        crawl_mode_ = false;
        StartStandUp("NAV ACTIVE after crawl");
        nav_active_ = true;
      }
      // /mode_switch_cmd is a 60-second status heartbeat after the last
      // Nav2 velocity message, not a request to move.  In particular after a
      // task is cancelled it must not make a freshly restarted bridge stand
      // the dog back up.  A non-zero /cmd_vel or explicit stand_up action is
      // the only valid way to enter active motion from idle.
      return;
    }

    if (!nav_active_) {
      return;
    }

    if (last_cmd_) {
      const auto idle_time = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - last_cmd_time_);
      if (idle_time < std::chrono::milliseconds(inactive_linger_ms_)) {
        return;
      }
    }

    const auto ret = sdk_connected_ ? sdk_highlevel_.passive() : 0;
    if (!sdk_connected_) {
      RemoteSetRemote({}, {});
    }
    crawl_mode_ = false;
    nav_active_ = false;
    standing_up_ = false;
    last_cmd_.reset();
    PublishMotionState("passive");
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "NAV INACTIVE: SDK passive()");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "NAV INACTIVE: SDK passive() returned 0x%x", ret);
    }
  }

  void HandlePlannerVelCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!sdk_connected_ && emergency_stop_latched_) {
      // Suppress queued Nav2/teleop velocity while the dog reports its
      // hardware safety latch. Re-enable command handling only after the
      // dog-side state itself has left CM_EMERGENCY_STOP.
      if (!RemoteReady() || RemoteControlMode() ==
                                static_cast<int32_t>(zsibot::ControlMode::CM_EMERGENCY_STOP)) {
        return;
      }
      emergency_stop_latched_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "virtual remote emergency-stop latch cleared by dog-side state");
    }
    if (!remote_control_only_ && manual_crawl_lock_) {
      return;
    }
    if (!remote_control_only_ &&
        std::chrono::steady_clock::now() < manual_override_until_) {
      return;
    }

    const bool is_zero_command =
        std::fabs(msg->linear.x) < 1e-6 &&
        std::fabs(msg->linear.y) < 1e-6 &&
        std::fabs(msg->angular.z) < 1e-6;

    if (!LocalizationAllowsNavLocked()) {
      if (!is_zero_command) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "dropping planner cmd_vel while localization is not Normal "
            "(status=%u seen=%s)",
            last_loc_status_, loc_info_seen_ ? "true" : "false");
      }
      if (nav_active_) {
        HoldPlannerVelocityLocked(std::chrono::steady_clock::now(),
                                  "localization not Normal");
      }
      return;
    }

    if (!nav_active_ && is_zero_command) {
      return;
    }

    if (remote_control_only_ && !nav_active_) {
      if (!is_zero_command) {
        last_cmd_ = *msg;
        last_cmd_time_ = std::chrono::steady_clock::now();
        nav_active_ = true;
        if (requested_posture_ == robot_navigo::RequestedPosture::kLow) {
          // The operator explicitly selected the low posture. Preserve it
          // and pass the stick through; only an explicit stand action exits.
          PublishMotionState("low_posture_moving");
          return;
        }

        // Outside low posture, a deliberate non-zero remote stick input is
        // the request to stand before the held stick frames are applied.
        queued_stand_cmd_ = *msg;
        queued_stand_cmd_time_ = last_cmd_time_;
        // SetRemote() is honored only while the motion controller is in its
        // vendor remote-control function mode. A stand-up action can leave
        // that mode, so acquire it before sending the posture command.
        RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
        RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
        standing_up_ = true;
        stand_start_time_ = last_cmd_time_;
        PublishMotionState("standing_up");
      }
      return;
    }

    last_cmd_ = *msg;
    last_cmd_time_ = std::chrono::steady_clock::now();
    if (remote_control_only_ && standing_up_ && !is_zero_command) {
      // The browser may release a directional button before the physical
      // stand-up action completes. Keep the latest deliberate input briefly
      // so it is applied as soon as the robot reports standing.
      queued_stand_cmd_ = *msg;
      queued_stand_cmd_time_ = last_cmd_time_;
    }

    if (!remote_control_only_ && !nav_active_) {
      StartStandUp("First /cmd_vel");
      nav_active_ = true;
    }
  }

  void HandleTeleopActionCallback(
      const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    const auto& action = msg->data;
    uint32_t ret = 0;

    if (SetRemoteSpeedLevel(action)) {
      return;
    }

    if (remote_control_only_) {
      HandleRemoteTeleopAction(action);
      return;
    }

    if (action == "stand_up") {
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kStanding;
      StartStandUp("Teleop stand_up");
      geometry_msgs::msg::Twist hold_cmd;
      last_cmd_ = hold_cmd;
      last_cmd_time_ = std::chrono::steady_clock::now();
      nav_active_ = true;
      return;
    }
    if (action == "lie_down") {
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kLow;
      ret = sdk_highlevel_.lieDown();
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      PublishMotionState(ret == 0 ? "lying_down" : "lie_down_failed");
    } else if (action == "crawl_forward") {
      // Enter the manufacturer's crawl gait. Subsequent /cmd_vel messages are
      // translated to virtual remote joystick values until another posture is selected.
      RemoteSetCmd(zsibot::CmdCode::CMD_SDK_CONTROL_RIGHT);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RemoteSetCmd(zsibot::CmdCode::CMD_CRAWL_FORWARD);
      crawl_mode_ = true;
      manual_crawl_lock_ = true;
      requested_posture_ = robot_navigo::RequestedPosture::kLow;
      remote_move_mode_requested_ = false;
      crawl_zero_sent_ = false;
      nav_active_ = true;
      standing_up_ = false;
      last_cmd_ = geometry_msgs::msg::Twist{};
      last_cmd_time_ = std::chrono::steady_clock::now();
      PublishMotionState("crawling");
      RCLCPP_INFO(this->get_logger(),
                  "Crawl gait enabled via CMD_CRAWL_FORWARD; /cmd_vel uses SetRemote");
      return;
    } else if (action == "passive") {
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kNone;
      ret = sdk_connected_ ? sdk_highlevel_.passive() : 0;
      if (!sdk_connected_) {
        RemoteSetRemote({}, {});
      }
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      PublishMotionState(ret == 0 ? "passive" : "passive_failed");
    } else if (action == "release_remote") {
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kNone;
      ret = sdk_connected_ ? sdk_highlevel_.passive() : 0;
      if (!sdk_connected_) {
        RemoteSetRemote({}, {});
      }
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      if (ret != 0) {
        PublishMotionState("remote_control_failed");
        RCLCPP_WARN(this->get_logger(),
                    "Teleop release passive() returned 0x%x", ret);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      PublishMotionState("remote_control");
      RCLCPP_INFO(this->get_logger(),
                  "Teleop released via CMD_REMOTE_CONTROL_RIGHT");
      return;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown /teleop_action: %s",
                  action.c_str());
      return;
    }

    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "Teleop action %s succeeded",
                  action.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Teleop action %s returned 0x%x",
                  action.c_str(), ret);
    }
  }

  void HandleTeleopVelCallback(
      const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    manual_teleop_active_ = true;
    last_cmd_ = *msg;
    last_cmd_time_ = std::chrono::steady_clock::now();
    manual_override_until_ = last_cmd_time_ +
        std::chrono::milliseconds(manual_override_ms_);
    if (!nav_active_) {
      const bool is_zero_command = robot_navigo::IsZeroPlanarVelocity(
          msg->linear.x, msg->linear.y, msg->angular.z);
      if (is_zero_command) {
        // Button release/stop frames must not turn a selected posture into a
        // stand-up request.
        last_cmd_.reset();
        return;
      }
      if (requested_posture_ == robot_navigo::RequestedPosture::kLow) {
        // The UI enters its prone/crawl posture with CMD_SIT_DOWN. Preserve
        // that posture and pass the deliberate stick command through; only
        // an explicit stand_up action changes requested_posture_.
        standing_up_ = false;
        nav_active_ = true;
        PublishMotionState("low_posture_moving");
        return;
      }
      // The virtual remote accepts stand-up only after the remote-control
      // function mode has been selected.  The former SDK/navigation path sent
      // only CMD_STAND_UP, which left the dog in PASSIVE/idle safety mode.
      RemoteSetRemote({}, {});
      RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
      standing_up_ = true;
      stand_start_time_ = last_cmd_time_;
      PublishMotionState("standing_up");
      nav_active_ = true;
    }
  }

  void HandleRemoteTeleopActionCallback(
      const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    HandleRemoteTeleopAction(msg->data);
  }

  void StartRemoteExecutorAsync() {
    std::thread([this]() {
      auto executor = std::make_shared<zsibot::ZsibotExecutor>(zsibot::Role::ROLE_REMOTE);
      std::lock_guard<std::mutex> lk(mutex_);
      remote_executor_ = std::move(executor);
      RCLCPP_INFO(this->get_logger(), "vendor remote protocol is ready");
    }).detach();
  }

  bool RemoteReady() const {
    return static_cast<bool>(remote_executor_);
  }

  void RemoteSetCmd(zsibot::CmdCode command) {
    if (!RemoteReady()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "vendor remote protocol is waiting for robot model state");
      return;
    }
    remote_executor_->SetCmd(command);
  }

  void RemoteSetRemote(const std::array<float, 4>& joystick,
                       const std::array<float, 14>& buttons) {
    if (!RemoteReady()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "vendor remote protocol is waiting for robot model state");
      return;
    }
    remote_executor_->SetRemote(joystick, buttons);
  }

  int32_t RemoteControlMode() const {
    return RemoteReady()
               ? static_cast<int32_t>(remote_executor_->GetControlMode())
               : -1;
  }

  void HandleLocalizationCallback(
      const robots_dog_msgs::msg::Localization::SharedPtr msg) {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lk(mutex_);
    loc_info_seen_ = true;
    last_loc_status_ = msg->status;
    last_loc_time_ = std::chrono::steady_clock::now();
  }

  bool LocalizationAllowsNavLocked() const {
    if (remote_control_only_ || manual_teleop_active_) {
      return true;
    }
    if (std::chrono::steady_clock::now() < manual_override_until_) {
      return true;
    }
    if (!loc_info_seen_) {
      return false;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - last_loc_time_)
                            .count();
    if (age_ms > localization_timeout_ms_) {
      return false;
    }
    return last_loc_status_ == 3;
  }

  void HoldPlannerVelocityLocked(
      const std::chrono::steady_clock::time_point& now, const char* reason) {
    last_cmd_ = geometry_msgs::msg::Twist{};
    last_cmd_time_ = now;
    queued_stand_cmd_.reset();
    if (crawl_mode_ || !sdk_connected_) {
      RemoteSetRemote({}, {});
      crawl_zero_sent_ = true;
    } else {
      (void)sdk_highlevel_.move(0.0f, 0.0f, 0.0f);
    }
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "holding zero velocity: %s (loc status=%u seen=%s)", reason,
        last_loc_status_, loc_info_seen_ ? "true" : "false");
  }

  void PublishLatestVelocity() {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!LocalizationAllowsNavLocked()) {
      if (nav_active_) {
        HoldPlannerVelocityLocked(std::chrono::steady_clock::now(),
                                  "localization not Normal");
      }
      return;
    }

    if (!nav_active_ || !last_cmd_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (remote_control_only_ || manual_teleop_active_ || !sdk_connected_ ||
        manual_crawl_lock_ || now < manual_override_until_) {
      PublishLatestRemoteVelocity(now);
      return;
    }

    if (standing_up_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - stand_start_time_);
      const auto ctrl_mode = sdk_highlevel_.getCurrentCtrlmode();
      const bool sdk_reports_standing = ctrl_mode == 1 || ctrl_mode == 18;
      // On NX_XG3588 the high-level command acknowledgement is reliable, but
      // getCurrentCtrlmode() can remain at 0 after a successful standUp().
      // Do not leave navigation blocked forever on that stale readback: after
      // the normal settling interval, accept a successful stand-up command as
      // a bounded fallback.  A failed command still retries and never reaches
      // this branch.
      const bool settled_successful_standup =
          standup_command_accepted_ &&
          elapsed >= std::chrono::milliseconds(standup_settle_ms_);
      if ((sdk_reports_standing &&
           elapsed >= std::chrono::milliseconds(standup_settle_ms_)) ||
          settled_successful_standup) {
        standing_up_ = false;
        PublishMotionState("standing");
        if (sdk_reports_standing) {
          RCLCPP_INFO(this->get_logger(),
                      "standUp confirmed by SDK, current_ctrlmode=%u", ctrl_mode);
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "standUp settled after a successful SDK command, but "
                      "current_ctrlmode remains %u; using bounded fallback",
                      ctrl_mode);
        }
      } else if (elapsed < std::chrono::milliseconds(standup_retry_ms_)) {
        return;
      } else {
        const auto ret = sdk_highlevel_.standUp();
        standup_command_accepted_ = ret == 0;
        stand_start_time_ = now;
        RCLCPP_WARN(this->get_logger(),
                    "standUp not confirmed after %ldms (mode=%u); retry returned 0x%x",
                    elapsed.count(), ctrl_mode, ret);
        return;
      }
    }

    const auto cmd_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_cmd_time_);
    if (cmd_age > std::chrono::milliseconds(cmd_timeout_ms_)) {
      if (crawl_mode_ && !crawl_zero_sent_) {
        RemoteSetRemote({}, {});
        crawl_zero_sent_ = true;
      }
      return;
    }

    float vx =
        std::fabs(last_cmd_->linear.x) < 0.085 ? 0.0f : last_cmd_->linear.x;
    float vy =
        std::fabs(last_cmd_->linear.y) < 0.10 ? 0.0f : last_cmd_->linear.y;
    float yaw_rate = last_cmd_->angular.z;
    if (std::fabs(yaw_rate) > 1e-4f && std::fabs(yaw_rate) < 0.025f) {
      yaw_rate = std::copysign(0.025f, yaw_rate);
    }
    vx = std::clamp(vx, static_cast<float>(-sdk_max_vx_),
                    static_cast<float>(sdk_max_vx_));
    vy = std::clamp(vy, static_cast<float>(-sdk_max_vy_),
                    static_cast<float>(sdk_max_vy_));
    yaw_rate = std::clamp(yaw_rate, static_cast<float>(-sdk_max_yaw_rate_),
                          static_cast<float>(sdk_max_yaw_rate_));
    if (std::fabs(yaw_rate) >= turn_linear_limit_yaw_rate_) {
      vx = std::clamp(vx, static_cast<float>(-turn_max_linear_speed_),
                      static_cast<float>(turn_max_linear_speed_));
      vy = std::clamp(vy, static_cast<float>(-turn_max_linear_speed_),
                      static_cast<float>(turn_max_linear_speed_));
    }
    if (crawl_mode_) {
      std::array<float, 4> joystick{
          static_cast<float>(vx / sdk_max_vx_),
          static_cast<float>(yaw_rate / sdk_max_yaw_rate_),
          static_cast<float>(vy / sdk_max_vy_), 0.0f};
      joystick[0] = std::clamp(joystick[0], -1.0f, 1.0f);
      joystick[1] = std::clamp(joystick[1], -1.0f, 1.0f);
      joystick[2] = std::clamp(joystick[2], -1.0f, 1.0f);
      RemoteSetRemote(joystick, {});
      crawl_zero_sent_ = false;
      return;
    }
    const auto ret = sdk_highlevel_.move(vx, vy, yaw_rate);
    if (ret != 0) {
      const auto ctrl_mode = sdk_highlevel_.getCurrentCtrlmode();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "SDK move() returned 0x%x, current_ctrlmode=%u, "
                           "cmd=(%.3f, %.3f, %.3f), raw=(%.3f, %.3f, %.3f)",
                           ret, ctrl_mode, vx, vy, yaw_rate,
                           last_cmd_->linear.x, last_cmd_->linear.y,
                           last_cmd_->angular.z);
      if (ret == 0x3007 && ctrl_mode != 1 && ctrl_mode != 18) {
        StartStandUp("SDK move rejected while not standing");
      }
    }
  }

  void StartStandUp(const char* reason) {
    requested_posture_ = robot_navigo::RequestedPosture::kStanding;
    if (!sdk_connected_) {
      // The robot may already be owned by the vendor SDK app. In that case
      // HighLevel::standUp()/move() return 0x3007 even though Nav2 is healthy.
      // The vendor virtual remote is a separate, supported command path and
      // keeps the navigation bridge functional without a second SDK socket.
      // This controller also reports CM_EMERGENCY_STOP while its virtual
      // remote session has not yet selected a posture/motion mode.  It is not
      // by itself evidence of a physical e-stop, so a user-requested stand-up
      // is allowed to issue the documented zero-stick mode-selection sequence.
      RemoteSetRemote({}, {});
      RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
      emergency_stop_latched_ = false;
      standup_command_accepted_ = true;
      standing_up_ = true;
      remote_move_mode_requested_ = false;
      stand_start_time_ = std::chrono::steady_clock::now();
      PublishMotionState("standing_up");
      RCLCPP_WARN(this->get_logger(),
                  "%s: native SDK unavailable; requested stand-up via virtual remote",
                  reason);
      return;
    }
    const auto ret = sdk_highlevel_.standUp();
    standup_command_accepted_ = ret == 0;
    standing_up_ = true;
    stand_start_time_ = std::chrono::steady_clock::now();
    PublishMotionState(ret == 0 ? "standing_up" : "stand_up_retrying");
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "%s: SDK standUp()", reason);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s: SDK standUp() returned 0x%x",
                  reason, ret);
    }
  }

  void HandleRemoteTeleopAction(const std::string& action) {
    if (SetRemoteSpeedLevel(action)) {
      return;
    }
    if (action == "stand_up") {
      manual_teleop_active_ = true;
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kStanding;
      RemoteSetRemote({}, {});
      RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
      emergency_stop_latched_ = false;
      nav_active_ = true;
      standing_up_ = true;
      remote_move_mode_requested_ = false;
      stand_start_time_ = std::chrono::steady_clock::now();
      queued_stand_cmd_.reset();
      last_cmd_ = geometry_msgs::msg::Twist{};
      last_cmd_time_ = stand_start_time_;
      PublishMotionState("standing_up");
      return;
    }
    if (action == "two_leg_stand" || action == "shake_hand") {
      const bool two_leg_stand = action == "two_leg_stand";
      manual_teleop_active_ = true;
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kNone;
      RemoteSetRemote({}, {});
      RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RemoteSetCmd(two_leg_stand ? zsibot::CmdCode::CMD_TWO_LEG_STAND
                                 : zsibot::CmdCode::CMD_GREET);
      emergency_stop_latched_ = false;
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      queued_stand_cmd_.reset();
      PublishMotionState(two_leg_stand ? "two_leg_standing" : "greeting");
      RCLCPP_INFO(this->get_logger(), "remote %s requested via vendor command",
                  two_leg_stand ? "two-leg stand" : "greet");
      return;
    }
    if (action == "crawl_forward") {
      manual_teleop_active_ = true;
      // The vendor virtual-remote library rejects CMD_CRAWL_FORWARD on the
      // point-foot XG model. Do not claim that crawl is active: a following
      // joystick frame is otherwise interpreted as ordinary standing gait.
      if (remote_executor_ && remote_executor_->GetModel() == zsibot::Model::MODEL_XG) {
        crawl_mode_ = false;
        manual_crawl_lock_ = false;
        requested_posture_ = robot_navigo::RequestedPosture::kNone;
        last_cmd_.reset();
        PublishMotionState("crawl_unsupported_xg");
        RCLCPP_ERROR(this->get_logger(),
                     "XG virtual remote does not support crawl; refusing joystick fallback");
        return;
      }
      RemoteSetCmd(zsibot::CmdCode::CMD_CRAWL_FORWARD);
      crawl_mode_ = true;
      manual_crawl_lock_ = true;
      requested_posture_ = robot_navigo::RequestedPosture::kLow;
      remote_move_mode_requested_ = false;
      crawl_zero_sent_ = false;
      nav_active_ = true;
      standing_up_ = false;
      last_cmd_ = geometry_msgs::msg::Twist{};
      last_cmd_time_ = std::chrono::steady_clock::now();
      PublishMotionState("crawling");
      return;
    }
    if (action == "passive") {
      RemoteSetRemote({}, {});
      // Keep passive on the same vendor virtual-remote path as stand_up and
      // lie_down. A zero joystick frame only releases the current velocity;
      // it does not change the controller mode. CMD_EMERGENCY_STOP is the
      // vendor remote's motor-free/passive safety state (CM_EMERGENCY_STOP),
      // which is also the state observed after the normal remote idle timeout.
      RemoteSetCmd(zsibot::CmdCode::CMD_EMERGENCY_STOP);
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kNone;
      manual_teleop_active_ = false;
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      queued_stand_cmd_.reset();
      emergency_stop_latched_ = true;
      PublishMotionState("passive");
      RCLCPP_INFO(this->get_logger(),
                  "remote passive requested via CMD_EMERGENCY_STOP");
      return;
    }
    if (action == "lie_down") {
      RemoteSetRemote({}, {});
      RemoteSetCmd(zsibot::CmdCode::CMD_SIT_DOWN);
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kLow;
      remote_move_mode_requested_ = false;
      manual_teleop_active_ = false;
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      queued_stand_cmd_.reset();
      PublishMotionState("lying_down");
      return;
    }
    if (action == "release_remote") {
      RemoteSetRemote({}, {});
      RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      requested_posture_ = robot_navigo::RequestedPosture::kNone;
      manual_teleop_active_ = false;
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      queued_stand_cmd_.reset();
      PublishMotionState("remote_control");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "Unknown remote /teleop_action: %s",
                action.c_str());
  }

  bool SetRemoteSpeedLevel(const std::string& action) {
    zsibot::CmdCode command = zsibot::CmdCode::CMD_NULL;
    const char* state = nullptr;
    if (action == "speed_micro") {
      command = zsibot::CmdCode::CMD_SLOW_SPEED;
      state = "speed_micro";
      remote_min_stick_ = 0.70f;
    } else if (action == "speed_slow") {
      command = zsibot::CmdCode::CMD_SLOW_SPEED;
      state = "speed_slow";
      remote_min_stick_ = 1.0f;
    } else if (action == "speed_normal") {
      command = zsibot::CmdCode::CMD_NORMAL_SPEED;
      state = "speed_normal";
      remote_min_stick_ = 1.0f;
    } else if (action == "speed_fast") {
      command = zsibot::CmdCode::CMD_FAST_SPEED;
      state = "speed_fast";
      remote_min_stick_ = 0.55f;
    } else {
      return false;
    }
    RemoteSetCmd(command);
    PublishMotionState(state);
    RCLCPP_INFO(this->get_logger(), "Vendor remote speed level set: %s", state);
    return true;
  }

  void PublishLatestRemoteVelocity(
      const std::chrono::steady_clock::time_point& now) {
    if (standing_up_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - stand_start_time_);
      const auto ctrl_mode = RemoteControlMode();
      const bool remote_reports_ready =
          ctrl_mode == static_cast<int32_t>(zsibot::ControlMode::CM_STAND_UP) ||
          ctrl_mode == static_cast<int32_t>(zsibot::ControlMode::CM_MOVE_MODE);
      if (remote_reports_ready) {
        standing_up_ = false;
        if (queued_stand_cmd_ &&
            now - queued_stand_cmd_time_ <= std::chrono::milliseconds(1500)) {
          last_cmd_ = *queued_stand_cmd_;
          last_cmd_time_ = now;
        }
        queued_stand_cmd_.reset();
        // Reaching CM_STAND_UP completes the posture action. Do not request
        // MOVE_MODE until a fresh non-zero velocity arrives: the dog-side FSM
        // rejects an immediate standup -> rlmix transition while the posture
        // is still settling, and may fall back to PASSIVE.
        remote_move_mode_requested_ = false;
        PublishMotionState("standing");
        RCLCPP_INFO(this->get_logger(),
                    "stand-up confirmed; holding standing posture until a non-zero velocity command");
        return;
      } else if (elapsed >= std::chrono::milliseconds(standup_retry_ms_)) {
        if (ctrl_mode ==
            static_cast<int32_t>(zsibot::ControlMode::CM_EMERGENCY_STOP)) {
          // For the vendor virtual remote this can be its idle mode after a
          // stand-up request.  Keep the sticks at zero and retry the documented
          // remote-control -> stand-up transition; a real hardware interlock
          // will simply reject it and no velocity is ever sent here.
          RemoteSetRemote({}, {});
          RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
          emergency_stop_latched_ = false;
          stand_start_time_ = now;
          PublishMotionState("stand_up_retrying");
          RCLCPP_WARN(this->get_logger(),
                      "virtual remote is in its idle safety mode; re-requested "
                      "remote-control and stand-up while holding zero");
          return;
        }
        // The virtual remote may not have connected when the first stand-up
        // request was made. Retry after the transport is available and keep
        // all joystick outputs at zero meanwhile.
        RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
        stand_start_time_ = now;
        RCLCPP_WARN(this->get_logger(),
                    "virtual remote control mode=%d; re-requested stand-up and holding velocity",
                    ctrl_mode);
        return;
      } else {
        return;
      }
    }

    // Do not transmit virtual-stick velocity while the vendor controller is
    // in an emergency-stop or posture state.
    const auto remote_ctrl_mode = RemoteControlMode();
    const auto mode_disposition =
        robot_navigo::DecideRemoteVelocityDisposition(requested_posture_,
                                                       remote_ctrl_mode);
    if (mode_disposition ==
        robot_navigo::RemoteVelocityDisposition::kEmergencyStop) {
      RemoteSetRemote({}, {});
      emergency_stop_latched_ = true;
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      queued_stand_cmd_.reset();
      remote_move_mode_requested_ = false;
      PublishMotionState("emergency_stop_latched");
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "virtual remote emergency stop is latched; "
                            "holding zero and issuing no recovery commands");
      return;
    }
    const auto cmd_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_cmd_time_);
    const bool has_live_motion_command = robot_navigo::HasLiveMotionCommand(
        last_cmd_->linear.x, last_cmd_->linear.y, last_cmd_->angular.z,
        cmd_age.count(), cmd_timeout_ms_);
    if (!has_live_motion_command) {
      // A posture command carries an intentional zero Twist. Keep the current
      // posture and do not turn that zero into a MOVE_MODE request.
      if (!crawl_zero_sent_) {
        RemoteSetRemote({}, {});
        crawl_zero_sent_ = true;
      }
      return;
    }
    if (mode_disposition ==
        robot_navigo::RemoteVelocityDisposition::kWaitForLowPosture) {
      // CMD_SIT_DOWN transitions asynchronously. Hold zero until the
      // controller reports CM_SIT_DOWN, and never request MOVE_MODE here:
      // MOVE_MODE exits the low posture and physically stands the dog.
      if (!crawl_zero_sent_) {
        RemoteSetRemote({}, {});
        crawl_zero_sent_ = true;
      }
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "low posture requested; waiting for CM_SIT_DOWN readback (current mode=%d)",
          remote_ctrl_mode);
      return;
    }
    // Outside a preserved low posture, retry MOVE_MODE at a bounded rate so
    // a dropped command can recover without flooding the controller.
    if (mode_disposition ==
        robot_navigo::RemoteVelocityDisposition::kRequestMoveMode) {
      if (!remote_move_mode_requested_ ||
          now - last_remote_move_mode_request_ >= std::chrono::seconds(1)) {
        RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RemoteSetCmd(zsibot::CmdCode::CMD_MOVE_MODE);
        remote_move_mode_requested_ = true;
        last_remote_move_mode_request_ = now;
        RCLCPP_WARN(this->get_logger(),
                    "virtual remote control mode=%d; requested MOVE_MODE and holding velocity",
                    remote_ctrl_mode);
      }
      return;
    }

    const auto normalize_stick = [this](float value, double max_value) {
      float stick = std::clamp(static_cast<float>(value / max_value), -1.0f, 1.0f);
      // The robot ignores small virtual-stick values. Keep a deliberate UI
      // direction above its dead zone while preserving an exact zero command.
      // The dock-contact profile has its own much smaller floor so that Nav2
      // can make a genuine low-speed pose correction.
      const float min_stick = fine_control_ ? remote_fine_min_stick_ : remote_min_stick_;
      if (std::fabs(stick) > 1e-4f && std::fabs(stick) < min_stick) {
        stick = std::copysign(min_stick, stick);
      }
      return stick;
    };
    const float forward = normalize_stick(last_cmd_->linear.x, remote_full_scale_vx_);
    const float lateral = normalize_stick(last_cmd_->linear.y, remote_full_scale_vy_);
    const float yaw = normalize_stick(last_cmd_->angular.z, remote_full_scale_yaw_rate_);

    // XG virtual remote uses [forward, lateral, yaw, 0]. The prior ordering
    // swapped the lateral and yaw channels, so left/right shift became turn.
    RemoteSetRemote({forward, lateral, yaw, 0.0f}, {});
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Remote stick sent forward=%.2f lateral=%.2f yaw=%.2f packet=(%.2f, %.2f, %.2f), function_mode=%d ctrl_mode=%d speed_level=%d body_velocity=(%.3f, %.3f, %.3f)",
        forward, lateral, yaw, forward, lateral, yaw,
        static_cast<int>(remote_executor_->GetFunctionMode()),
        RemoteControlMode(),
        static_cast<int>(remote_executor_->GetSpeedLevel()),
        remote_executor_->GetBodyVelocity()[0],
        remote_executor_->GetBodyVelocity()[1],
        remote_executor_->GetBodyVelocity()[2]);
    crawl_zero_sent_ = false;
  }

  void PublishMotionState(const std::string& state) {
    std_msgs::msg::String msg;
    msg.data = state;
    motion_state_publisher_->publish(msg);
  }

  void PublishControlMode() {
    std::lock_guard<std::mutex> lk(mutex_);
    std_msgs::msg::Int32 msg;
    msg.data = remote_control_only_
                   ? RemoteControlMode()
                   : static_cast<int32_t>(sdk_highlevel_.getCurrentCtrlmode());
    control_mode_publisher_->publish(msg);
  }

  std::string server_ip_;
  std::string client_ip_;
  std::string platform_;
  int client_port_ = 43988;
  int server_port_ = 43997;
  int standup_settle_ms_ = 4000;
  int standup_retry_ms_ = 5000;
  int cmd_timeout_ms_ = 500;
  int inactive_linger_ms_ = 8000;
  int manual_override_ms_ = 650;
  int publish_period_ms_ = 20;
  double sdk_max_vx_ = 0.5;
  double sdk_max_vy_ = 0.5;
  double sdk_max_yaw_rate_ = 0.5;
  double remote_full_scale_vx_ = 3.0;
  double remote_full_scale_vy_ = 2.25;
  double remote_full_scale_yaw_rate_ = 5.25;
  float remote_fine_min_stick_ = 0.45f;
  double turn_linear_limit_yaw_rate_ = 1.0;
  double turn_max_linear_speed_ = 0.5;
  float remote_min_stick_ = 0.55f;
  bool sdk_connected_ = false;
  bool remote_control_only_ = false;
  std::mutex mutex_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      planner_vel_cmd_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      teleop_vel_cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_switch_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      teleop_action_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      remote_teleop_action_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fine_control_subscriber_;
  rclcpp::Subscription<robots_dog_msgs::msg::Localization>::SharedPtr
      localization_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_mode_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr control_mode_timer_;

  mc_sdk::zsl_1::HighLevel sdk_highlevel_;
  std::shared_ptr<zsibot::ZsibotExecutor> remote_executor_;
  bool nav_active_ = false;
  bool manual_teleop_active_ = false;
  bool standing_up_ = false;
  bool standup_command_accepted_ = false;
  bool crawl_mode_ = false;
  bool remote_move_mode_requested_ = false;
  bool emergency_stop_latched_ = false;
  bool fine_control_ = false;
  std::chrono::steady_clock::time_point last_remote_move_mode_request_{};
  bool manual_crawl_lock_ = false;
  robot_navigo::RequestedPosture requested_posture_ =
      robot_navigo::RequestedPosture::kNone;
  bool crawl_zero_sent_ = false;
  std::chrono::steady_clock::time_point stand_start_time_;
  std::chrono::steady_clock::time_point manual_override_until_;
  std::optional<geometry_msgs::msg::Twist> last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  std::optional<geometry_msgs::msg::Twist> queued_stand_cmd_;
  std::chrono::steady_clock::time_point queued_stand_cmd_time_;
  std::string localization_topic_ = "/localization_info";
  int localization_timeout_ms_ = 500;
  bool loc_info_seen_ = false;
  uint8_t last_loc_status_ = 0;
  std::chrono::steady_clock::time_point last_loc_time_{};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelCmdUdpPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
