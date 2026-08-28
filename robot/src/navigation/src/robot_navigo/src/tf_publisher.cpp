#include <algorithm>
#include <cmath>
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
        input_odom_topic_ = this->declare_parameter<std::string>("input_odom_topic", "/odom/localization_odom");
        output_odom_topic_ = this->declare_parameter<std::string>("output_odom_topic", "/odom/nav2");
        output_odom_frame_ = this->declare_parameter<std::string>("output_odom_frame", "odom");
        output_base_frame_ = this->declare_parameter<std::string>("output_base_frame", "base_link");
        publish_map_to_odom_ = this->declare_parameter<bool>("publish_map_to_odom", false);
        use_current_time_ = this->declare_parameter<bool>("use_current_time", true);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_odom_topic_, 10, std::bind(&OdomToTFBroadcaster::odom_callback, this, std::placeholders::_1));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, 10);

        RCLCPP_INFO(this->get_logger(),
                    "Odom to TF Broadcaster started, input=%s, output=%s, frames=%s->%s, publish_map_to_odom=%s, use_current_time=%s",
                    input_odom_topic_.c_str(), output_odom_topic_.c_str(),
                    output_odom_frame_.c_str(), output_base_frame_.c_str(),
                    publish_map_to_odom_ ? "true" : "false",
                    use_current_time_ ? "true" : "false");
    }

private:
    static double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
    {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    static geometry_msgs::msg::Quaternion yawOnly(const geometry_msgs::msg::Quaternion & q)
    {
        const double yaw = yawFromQuat(q);
        geometry_msgs::msg::Quaternion out;
        out.x = 0.0;
        out.y = 0.0;
        out.z = std::sin(yaw * 0.5);
        out.w = std::cos(yaw * 0.5);
        return out;
    }

    bool sendPlanarTransforms(const rclcpp::Time & stamp, const nav_msgs::msg::Odometry & msg)
    {
        if (stamp.nanoseconds() <= 0) {
            return false;
        }
        if (have_last_tf_ && stamp <= last_tf_stamp_) {
            return false;
        }
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = output_odom_frame_;
        t.child_frame_id = output_base_frame_;
        t.transform.translation.x = msg.pose.pose.position.x;
        t.transform.translation.y = msg.pose.pose.position.y;
        t.transform.translation.z = 0.0;
        t.transform.rotation = yawOnly(msg.pose.pose.orientation);
        tf_broadcaster_->sendTransform(t);

        if (publish_map_to_odom_) {
            geometry_msgs::msg::TransformStamped t_map;
            t_map.header.stamp = stamp;
            t_map.header.frame_id = "map";
            t_map.child_frame_id = output_odom_frame_;
            t_map.transform.translation.x = 0.0;
            t_map.transform.translation.y = 0.0;
            t_map.transform.translation.z = 0.0;
            t_map.transform.rotation.x = 0.0;
            t_map.transform.rotation.y = 0.0;
            t_map.transform.rotation.z = 0.0;
            t_map.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(t_map);
        }
        last_tf_stamp_ = stamp;
        have_last_tf_ = true;
        return true;
    }

    void fillTwistFromPoseDelta(nav_msgs::msg::Odometry & nav2_odom, const rclcpp::Time & stamp)
    {
        const double x = nav2_odom.pose.pose.position.x;
        const double y = nav2_odom.pose.pose.position.y;
        const double yaw = yawFromQuat(nav2_odom.pose.pose.orientation);
        if (have_last_pose_) {
            const double dt = (stamp - last_stamp_).seconds();
            if (std::isfinite(dt) && dt > 0.02 && dt < 1.00) {
                const double dx = x - last_x_;
                const double dy = y - last_y_;
                const double dist = std::hypot(dx, dy);
                const double dyaw = std::atan2(std::sin(yaw - last_yaw_), std::cos(yaw - last_yaw_));
                // A localization jump must not look like a 2 m/s command to MPPI.
                if (dist < 1.0 && std::fabs(dyaw) < 1.2) {
                    const double cy = std::cos(yaw);
                    const double sy = std::sin(yaw);
                    const double vx = (dx * cy + dy * sy) / dt;
                    const double vy = (-dx * sy + dy * cy) / dt;
                    nav2_odom.twist.twist.linear.x = std::clamp(vx, -2.0, 2.0);
                    nav2_odom.twist.twist.linear.y = std::clamp(vy, -2.0, 2.0);
                    nav2_odom.twist.twist.angular.z = std::clamp(dyaw / dt, -2.0, 2.0);
                }
            }
        }
        last_stamp_ = stamp;
        last_x_ = x;
        last_y_ = y;
        last_yaw_ = yaw;
        have_last_pose_ = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto msg_stamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
        const auto now_stamp = this->get_clock()->now();
        const bool have_msg_stamp = msg_stamp.nanoseconds() > 0;
        // Keep /odom/nav2 on the lidar/odom stamp so Nav2 and TF share one timeline.
        const auto odom_stamp = have_msg_stamp ? msg_stamp : now_stamp;
        auto nav2_odom = *msg;
        nav2_odom.header.stamp = odom_stamp;
        nav2_odom.header.frame_id = output_odom_frame_;
        nav2_odom.child_frame_id = output_base_frame_;
        nav2_odom.pose.pose.position.z = 0.0;
        nav2_odom.pose.pose.orientation = yawOnly(msg->pose.pose.orientation);
        fillTwistFromPoseDelta(nav2_odom, odom_stamp);
        odom_publisher_->publish(nav2_odom);

        // tf2 rejects a stamp older than the last published transform for the
        // same frames. Publish the measurement stamp first, then optionally
        // now() so the controller can look up "now" without dropping lidar
        // scans that still carry the measurement time.
        sendPlanarTransforms(odom_stamp, nav2_odom);
        if (use_current_time_ && now_stamp > odom_stamp) {
            sendPlanarTransforms(now_stamp, nav2_odom);
        }
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::string input_odom_topic_;
    std::string output_odom_topic_;
    std::string output_odom_frame_;
    std::string output_base_frame_;
    bool publish_map_to_odom_;
    bool use_current_time_;
    bool have_last_pose_ = false;
    bool have_last_tf_ = false;
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_yaw_ = 0.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
