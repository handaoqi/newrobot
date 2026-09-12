#ifndef NAVIGO_MPPI_CONTROLLER__ILQR_CONTROLLER_HPP_
#define NAVIGO_MPPI_CONTROLLER__ILQR_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "Eigen/Core"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "navigo_core/controller.hpp"
#include "navigo_costmap_2d/costmap_2d_ros.hpp"

namespace navigo_mppi_controller
{

/** Iterative finite-horizon LQR path tracker for a differential-drive body. */
class ILQRController : public navigo_core::Controller
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
  std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path plan_;
  std::string name_;
  double desired_linear_vel_{0.20};
  double max_angular_vel_{0.35};
  double dt_{0.05};
  int horizon_{24};
  int iterations_{4};
  double position_weight_{8.0};
  double yaw_weight_{3.0};
  double velocity_weight_{0.6};
  double angular_weight_{0.4};
  double speed_limit_scale_{1.0};
};

}  // namespace navigo_mppi_controller

#endif  // NAVIGO_MPPI_CONTROLLER__ILQR_CONTROLLER_HPP_
