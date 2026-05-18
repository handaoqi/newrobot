#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class SensorTestNode : public rclcpp::Node
{
public:
    SensorTestNode() : Node("sensor_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "Sensor test node started");
        
        // Test publishers to see what topics are available
        test_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&SensorTestNode::testCallback, this));
    }

private:
    void testCallback()
    {
        // List all available topics
        auto topic_names_and_types = this->get_topic_names_and_types();
        
        RCLCPP_INFO(this->get_logger(), "=== Available Topics ===");
        for (const auto& [topic, types] : topic_names_and_types) {
            RCLCPP_INFO(this->get_logger(), "Topic: %s, Types: %s", 
                       topic.c_str(), types.begin()->c_str());
        }
    }
    
    rclcpp::TimerBase::SharedPtr test_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
