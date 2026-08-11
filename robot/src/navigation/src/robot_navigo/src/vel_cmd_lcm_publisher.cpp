#include "gamepad_lcmt.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

#include <lcm/lcm-cpp.hpp>
#include <rclcpp/rclcpp.hpp>

class VelCmdLcmPublisher : public rclcpp::Node
{
public:
    VelCmdLcmPublisher()
        : Node("vel_cmd_lcm_publisher"),
          lc("udpm://239.255.76.67:7667?ttl=255")
    {
        planner_vel_cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&VelCmdLcmPublisher::HandlPlannerVelCallback, this, std::placeholders::_1));

        mode_switch_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "/mode_switch_cmd", 10, std::bind(&VelCmdLcmPublisher::HandleModeSwitchCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "vel_cmd_lcm_publisher started (with mode_switch support)");
    }

private:
    void HandleModeSwitchCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(planner_vel_mutex_);
        int32_t new_mode = msg->data;
        if (new_mode == 171) {
            navigation_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Navigation mode ACTIVE (mode=%d)", new_mode);
        } else {
            navigation_active_ = false;
            RCLCPP_INFO(this->get_logger(), "Navigation mode INACTIVE (mode=%d)", new_mode);
        }
        // Send an immediate mode switch LCM message
        gamepad_lcmt lcmt;
        lcmt.robot_state_flag = navigation_active_ ? 1 : 0;
        lcmt.navigation_mode  = navigation_active_ ? 171 : 170;
        lc.publish("vel_cmd_lcm_data", &lcmt);
    }

    void HandlPlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(planner_vel_mutex_);
        gamepad_lcmt lcmt;
        if (!navigation_active_) {
            return;  // Don't send velocity if not in navigation mode
        }
        lcmt.leftStickAnalog[1]  = std::fabs(msg->linear.x) < 0.085 ? 0.0 : msg->linear.x;
        lcmt.leftStickAnalog[0]  = std::fabs(msg->linear.y) < 0.085 ? 0.0 : msg->linear.y;
        lcmt.rightStickAnalog[0] = -msg->angular.z;
        lcmt.robot_state_flag    = 1;
        lcmt.navigation_mode     = 171;

        lc.publish("vel_cmd_lcm_data", &lcmt);
    }

    lcm::LCM lc;
    std::mutex planner_vel_mutex_;
    bool navigation_active_ = false;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr planner_vel_cmd_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_switch_subscriber;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelCmdLcmPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
