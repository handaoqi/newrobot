#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "zsl-1/highlevel.h"

class VelCmdUdpPublisher : public rclcpp::Node {
 public:
  VelCmdUdpPublisher() : Node("vel_cmd_udp_publisher") {
    planner_vel_cmd_subscriber_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&VelCmdUdpPublisher::HandlePlannerVelCallback, this,
                      std::placeholders::_1));
    mode_switch_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        "/mode_switch_cmd", 10,
        std::bind(&VelCmdUdpPublisher::HandleModeSwitchCallback, this,
                  std::placeholders::_1));
    teleop_action_subscriber_ =
        this->create_subscription<std_msgs::msg::String>(
            "/teleop_action", 10,
            std::bind(&VelCmdUdpPublisher::HandleTeleopActionCallback, this,
                      std::placeholders::_1));
    motion_state_publisher_ =
        this->create_publisher<std_msgs::msg::String>("/robot_motion_state", 10);

    this->declare_parameter("platform", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("client_ip", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("server_ip", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("client_port", 43988);
    this->declare_parameter("server_port", 43997);
    this->declare_parameter("standup_settle_ms", 4000);
    this->declare_parameter("standup_retry_ms", 5000);
    this->declare_parameter("cmd_timeout_ms", 500);
    this->declare_parameter("inactive_linger_ms", 8000);
    this->declare_parameter("publish_period_ms", 20);
    this->declare_parameter("sdk_max_vx", 0.5);
    this->declare_parameter("sdk_max_vy", 0.5);
    this->declare_parameter("sdk_max_yaw_rate", 0.5);
    this->declare_parameter("turn_linear_limit_yaw_rate", 0.35);
    this->declare_parameter("turn_max_linear_speed", 0.10);
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
    this->get_parameter("turn_linear_limit_yaw_rate",
                        turn_linear_limit_yaw_rate_);
    this->get_parameter("turn_max_linear_speed", turn_max_linear_speed_);

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

    sdk_highlevel_.initRobot(client_ip_, client_port_, server_ip_);

    const bool connected = sdk_highlevel_.checkConnect();
    RCLCPP_INFO(this->get_logger(),
                "vel_cmd_udp_publisher started via SDK %s:%d -> %s:%d",
                client_ip_.c_str(), client_port_, server_ip_.c_str(),
                server_port_);
    RCLCPP_INFO(this->get_logger(), "SDK checkConnect=%s",
                connected ? "true" : "false");

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&VelCmdUdpPublisher::PublishLatestVelocity, this));
    PublishMotionState("passive");
  }

 private:
  void HandleModeSwitchCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);

    if (msg->data == 171) {
      if (!nav_active_) {
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

    last_cmd_ = *msg;
    last_cmd_time_ = std::chrono::steady_clock::now();

    if (!nav_active_) {
      StartStandUp("First /cmd_vel");
      nav_active_ = true;
    }
  }

  void HandleTeleopActionCallback(
      const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(mutex_);
    const auto& action = msg->data;
    uint32_t ret = 0;

    if (action == "stand_up") {
      StartStandUp("Teleop stand_up");
      geometry_msgs::msg::Twist hold_cmd;
      last_cmd_ = hold_cmd;
      last_cmd_time_ = std::chrono::steady_clock::now();
      nav_active_ = true;
      return;
    }
    if (action == "lie_down") {
      ret = sdk_highlevel_.lieDown();
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      PublishMotionState(ret == 0 ? "lying_down" : "lie_down_failed");
    } else if (action == "passive") {
      ret = sdk_highlevel_.passive();
      nav_active_ = false;
      standing_up_ = false;
      last_cmd_.reset();
      PublishMotionState(ret == 0 ? "passive" : "passive_failed");
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

  void PublishLatestVelocity() {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!nav_active_ || !last_cmd_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();

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

  void PublishMotionState(const std::string& state) {
    std_msgs::msg::String msg;
    msg.data = state;
    motion_state_publisher_->publish(msg);
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
  int publish_period_ms_ = 20;
  double sdk_max_vx_ = 0.5;
  double sdk_max_vy_ = 0.5;
  double sdk_max_yaw_rate_ = 0.5;
  double turn_linear_limit_yaw_rate_ = 0.35;
  double turn_max_linear_speed_ = 0.10;
  std::mutex mutex_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      planner_vel_cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_switch_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      teleop_action_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_state_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  mc_sdk::zsl_1::HighLevel sdk_highlevel_;
  bool nav_active_ = false;
  bool standing_up_ = false;
  std::chrono::steady_clock::time_point stand_start_time_;
  std::optional<geometry_msgs::msg::Twist> last_cmd_;
  std::chrono::steady_clock::time_point last_cmd_time_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelCmdUdpPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
