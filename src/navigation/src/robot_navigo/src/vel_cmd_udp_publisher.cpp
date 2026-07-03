#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
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

    this->declare_parameter("platform", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("client_ip", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("server_ip", rclcpp::ParameterValue(std::string("")));
    this->declare_parameter("client_port", 43988);
    this->declare_parameter("server_port", 43997);
    this->declare_parameter("standup_settle_ms", 4000);
    this->declare_parameter("cmd_timeout_ms", 500);
    this->declare_parameter("publish_period_ms", 20);
    this->get_parameter("platform", platform_);
    this->get_parameter("client_ip", client_ip_);
    this->get_parameter("server_ip", server_ip_);
    this->get_parameter("client_port", client_port_);
    this->get_parameter("server_port", server_port_);
    this->get_parameter("standup_settle_ms", standup_settle_ms_);
    this->get_parameter("cmd_timeout_ms", cmd_timeout_ms_);
    this->get_parameter("publish_period_ms", publish_period_ms_);

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

    const auto ret = sdk_highlevel_.passive();
    nav_active_ = false;
    standing_up_ = false;
    last_cmd_.reset();
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

  void PublishLatestVelocity() {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!nav_active_ || !last_cmd_) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();

    if (standing_up_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - stand_start_time_);
      if (elapsed < std::chrono::milliseconds(standup_settle_ms_)) {
        return;
      }
      standing_up_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "standUp settled, forwarding /cmd_vel through SDK move()");
    }

    const auto cmd_age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_cmd_time_);
    if (cmd_age > std::chrono::milliseconds(cmd_timeout_ms_)) {
      return;
    }

    const float vx =
        std::fabs(last_cmd_->linear.x) < 0.085 ? 0.0f : last_cmd_->linear.x;
    const float vy =
        std::fabs(last_cmd_->linear.y) < 0.085 ? 0.0f : last_cmd_->linear.y;
    const float yaw_rate = last_cmd_->angular.z;
    const auto ret = sdk_highlevel_.move(vx, vy, yaw_rate);
    if (ret != 0) {
      const auto ctrl_mode = sdk_highlevel_.getCurrentCtrlmode();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "SDK move() returned 0x%x, current_ctrlmode=%u",
                           ret, ctrl_mode);
    }
  }

  void StartStandUp(const char* reason) {
    const auto ret = sdk_highlevel_.standUp();
    standing_up_ = true;
    stand_start_time_ = std::chrono::steady_clock::now();
    if (ret == 0) {
      RCLCPP_INFO(this->get_logger(), "%s: SDK standUp()", reason);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s: SDK standUp() returned 0x%x",
                  reason, ret);
    }
  }

  std::string server_ip_;
  std::string client_ip_;
  std::string platform_;
  int client_port_ = 43988;
  int server_port_ = 43997;
  int standup_settle_ms_ = 4000;
  int cmd_timeout_ms_ = 500;
  int publish_period_ms_ = 20;
  std::mutex mutex_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      planner_vel_cmd_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_switch_subscriber_;
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
