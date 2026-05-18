#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>

class NavigationNode : public rclcpp::Node
{
public:
    NavigationNode() : Node("navigation_node")
    {
        // Initialize publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Initialize subscribers
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&NavigationNode::laserCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&NavigationNode::odomCallback, this, std::placeholders::_1));
        
        // Create timer for periodic control commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&NavigationNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Navigation node started");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process laser scan data
        last_scan_ = msg;
        processLaserData();
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Process odometry data
        last_odom_ = msg;
    }
    
    void processLaserData()
    {
        if (!last_scan_) return;
        
        // Simple obstacle avoidance logic
        float front_distance = last_scan_->ranges[last_scan_->ranges.size() / 2];
        
        if (front_distance < 0.5) {
            // Obstacle ahead, stop
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected, distance: %.2f m", front_distance);
        } else {
            // Path clear, move forward
            cmd_vel_.linear.x = 0.2;
            cmd_vel_.angular.z = 0.0;
        }
    }
    
    void timerCallback()
    {
        // Publish velocity commands periodically
        cmd_vel_pub_->publish(cmd_vel_);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Twist cmd_vel_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
