#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

class OdomToTFBroadcaster : public rclcpp::Node
{
public:
    OdomToTFBroadcaster()
        : Node("odom_to_tf_broadcaster")
    {
        input_odom_topic_ = this->declare_parameter<std::string>("input_odom_topic", "/odom/mc_odom");
        publish_map_to_odom_ = this->declare_parameter<bool>("publish_map_to_odom", false);
        use_current_time_ = this->declare_parameter<bool>("use_current_time", true);

        // Initialize the TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create a subscription to the odom topic
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_, 10, std::bind(&OdomToTFBroadcaster::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "Odom to TF Broadcaster started, input=%s, publish_map_to_odom=%s, use_current_time=%s",
                    input_odom_topic_.c_str(), publish_map_to_odom_ ? "true" : "false",
                    use_current_time_ ? "true" : "false");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create a TransformStamped message
        geometry_msgs::msg::TransformStamped t;

        // Set the timestamp to the time of the received message
        if (use_current_time_) {
            t.header.stamp = this->get_clock()->now();
        } else {
            t.header.stamp = msg->header.stamp;
        }

        // Set the frame IDs
        t.header.frame_id = "odom";
        t.child_frame_id  = "base_link";

        // Set the translation
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        // Set the rotation
        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);

        if (publish_map_to_odom_) {
            geometry_msgs::msg::TransformStamped t_map;
            t_map.header.stamp = msg->header.stamp;
            t_map.header.frame_id = "map";
            t_map.child_frame_id  = "odom";
            t_map.transform.translation.x = 0.0;
            t_map.transform.translation.y = 0.0;
            t_map.transform.translation.z = 0.0;
            t_map.transform.rotation.x = 0.0;
            t_map.transform.rotation.y = 0.0;
            t_map.transform.rotation.z = 0.0;
            t_map.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t_map);
        }
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::string input_odom_topic_;
    bool publish_map_to_odom_;
    bool use_current_time_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OdomToTFBroadcaster>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
