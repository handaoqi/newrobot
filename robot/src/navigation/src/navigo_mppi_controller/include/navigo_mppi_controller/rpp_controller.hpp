#ifndef NAVIGO_MPPI_CONTROLLER__RPP_CONTROLLER_HPP_
#define NAVIGO_MPPI_CONTROLLER__RPP_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigo_core/controller.hpp"
#include "navigo_core/goal_checker.hpp"
#include "navigo_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace navigo_mppi_controller
{

/** A compact Regulated Pure Pursuit controller for low-latency path tracking. */
class RPPController : public navigo_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    navigo_core::GoalChecker * goal_checker) override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp::Logger logger_{rclcpp::get_logger("RPPController")};
  std::string name_;
  nav_msgs::msg::Path plan_;
  double desired_linear_vel_{0.25};
  double min_linear_vel_{0.05};
  double lookahead_dist_{0.65};
  double min_lookahead_dist_{0.35};
  double max_angular_vel_{0.45};
  double rotate_to_heading_threshold_{0.785};
  double rotate_to_heading_angular_vel_{0.35};
  bool curvature_speed_regulation_{true};
  double speed_limit_scale_{1.0};
};

}  // namespace navigo_mppi_controller

#endif  // NAVIGO_MPPI_CONTROLLER__RPP_CONTROLLER_HPP_
