#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
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
    this->declare_parameter("turn_linear_limit_yaw_rate", 0.35);
    this->declare_parameter("turn_max_linear_speed", 0.10);
    this->declare_parameter("manual_override_ms", 650);
    this->declare_parameter("remote_control_only", false);
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
    this->get_parameter("turn_linear_limit_yaw_rate",
                        turn_linear_limit_yaw_rate_);
    this->get_parameter("turn_max_linear_speed", turn_max_linear_speed_);
    this->get_parameter("manual_override_ms", manual_override_ms_);
    this->get_parameter("remote_control_only", remote_control_only_);

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
      const bool connected = sdk_highlevel_.checkConnect();
      RCLCPP_INFO(this->get_logger(),
                  "vel_cmd_udp_publisher started via SDK %s:%d -> %s:%d",
                  client_ip_.c_str(), client_port_, server_ip_.c_str(),
                  server_port_);
      RCLCPP_INFO(this->get_logger(), "SDK checkConnect=%s",
                  connected ? "true" : "false");
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
      } else if (!nav_active_) {
        StartStandUp("NAV ACTIVE");
      }
      nav_active_ = true;
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

    const auto ret = sdk_highlevel_.passive();
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
    if (!nav_active_ && is_zero_command) {
      return;
    }

    if (remote_control_only_ && !nav_active_) {
      if (!is_zero_command) {
        last_cmd_ = *msg;
        last_cmd_time_ = std::chrono::steady_clock::now();
        nav_active_ = true;
        if (low_posture_lock_) {
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
      low_posture_lock_ = false;
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
      ret = sdk_highlevel_.passive();
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      PublishMotionState(ret == 0 ? "passive" : "passive_failed");
    } else if (action == "release_remote") {
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      ret = sdk_highlevel_.passive();
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
    last_cmd_ = *msg;
    last_cmd_time_ = std::chrono::steady_clock::now();
    manual_override_until_ = last_cmd_time_ +
        std::chrono::milliseconds(manual_override_ms_);
    if (!nav_active_) {
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

  void PublishLatestVelocity() {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!nav_active_ || !last_cmd_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (remote_control_only_ || manual_crawl_lock_ || now < manual_override_until_) {
      PublishLatestRemoteVelocity(now);
      return;
    }

    if (standing_up_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - stand_start_time_);
      const auto ctrl_mode = sdk_highlevel_.getCurrentCtrlmode();
      if ((ctrl_mode == 1 || ctrl_mode == 18) &&
          elapsed >= std::chrono::milliseconds(standup_settle_ms_)) {
        standing_up_ = false;
        PublishMotionState("standing");
        RCLCPP_INFO(this->get_logger(),
                    "standUp confirmed by SDK, current_ctrlmode=%u", ctrl_mode);
      } else if (elapsed < std::chrono::milliseconds(standup_retry_ms_)) {
        return;
      } else {
        const auto ret = sdk_highlevel_.standUp();
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
    const auto ret = sdk_highlevel_.standUp();
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
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      RemoteSetCmd(zsibot::CmdCode::CMD_REMOTE_CONTROL_RIGHT);
      RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
      nav_active_ = true;
      standing_up_ = true;
      stand_start_time_ = std::chrono::steady_clock::now();
      queued_stand_cmd_.reset();
      last_cmd_ = geometry_msgs::msg::Twist{};
      last_cmd_time_ = stand_start_time_;
      PublishMotionState("standing_up");
      return;
    }
    if (action == "crawl_forward") {
      // The vendor virtual-remote library rejects CMD_CRAWL_FORWARD on the
      // point-foot XG model. Do not claim that crawl is active: a following
      // joystick frame is otherwise interpreted as ordinary standing gait.
      if (remote_executor_ && remote_executor_->GetModel() == zsibot::Model::MODEL_XG) {
        crawl_mode_ = false;
        manual_crawl_lock_ = false;
        last_cmd_.reset();
        PublishMotionState("crawl_unsupported_xg");
        RCLCPP_ERROR(this->get_logger(),
                     "XG virtual remote does not support crawl; refusing joystick fallback");
        return;
      }
      RemoteSetCmd(zsibot::CmdCode::CMD_CRAWL_FORWARD);
      crawl_mode_ = true;
      manual_crawl_lock_ = true;
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
      RemoteSetCmd(zsibot::CmdCode::CMD_EMERGENCY_STOP);
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      low_posture_lock_ = false;
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      queued_stand_cmd_.reset();
      PublishMotionState("passive");
      return;
    }
    if (action == "lie_down") {
      RemoteSetRemote({}, {});
      RemoteSetCmd(zsibot::CmdCode::CMD_SIT_DOWN);
      crawl_mode_ = false;
      manual_crawl_lock_ = false;
      low_posture_lock_ = true;
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
      low_posture_lock_ = false;
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
      if (ctrl_mode == 1 || elapsed >= std::chrono::milliseconds(standup_settle_ms_)) {
        standing_up_ = false;
        if (queued_stand_cmd_ &&
            now - queued_stand_cmd_time_ <= std::chrono::milliseconds(1500)) {
          last_cmd_ = *queued_stand_cmd_;
          last_cmd_time_ = now;
        }
        queued_stand_cmd_.reset();
        PublishMotionState("standing");
      } else if (elapsed >= std::chrono::milliseconds(standup_retry_ms_)) {
        RemoteSetCmd(zsibot::CmdCode::CMD_STAND_UP);
        stand_start_time_ = now;
        return;
      } else {
        return;
      }
    }
    const auto cmd_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_cmd_time_);
    if (cmd_age > std::chrono::milliseconds(cmd_timeout_ms_)) {
      if (!crawl_zero_sent_) {
        RemoteSetRemote({}, {});
        crawl_zero_sent_ = true;
      }
      return;
    }
    const auto normalize_stick = [this](float value, double max_value) {
      float stick = std::clamp(static_cast<float>(value / max_value), -1.0f, 1.0f);
      // The robot ignores small virtual-stick values. Keep a deliberate UI
      // direction above its dead zone while preserving an exact zero command.
      if (std::fabs(stick) > 1e-4f && std::fabs(stick) < remote_min_stick_) {
        stick = std::copysign(remote_min_stick_, stick);
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
  double turn_linear_limit_yaw_rate_ = 0.35;
  double turn_max_linear_speed_ = 0.10;
  float remote_min_stick_ = 0.55f;
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_mode_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr control_mode_timer_;

  mc_sdk::zsl_1::HighLevel sdk_highlevel_;
  std::shared_ptr<zsibot::ZsibotExecutor> remote_executor_;
  bool nav_active_ = false;
  bool standing_up_ = false;
  bool crawl_mode_ = false;
  bool manual_crawl_lock_ = false;
  bool low_posture_lock_ = false;
  bool crawl_zero_sent_ = false;
  std::chrono::steady_clock::time_point stand_start_time_;
  std::chrono::steady_clock::time_point manual_override_until_;
  std::optional<geometry_msgs::msg::Twist> last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  std::optional<geometry_msgs::msg::Twist> queued_stand_cmd_;
  std::chrono::steady_clock::time_point queued_stand_cmd_time_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelCmdUdpPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
