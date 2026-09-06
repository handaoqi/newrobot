#include "navigo_mppi_controller/rpp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace navigo_mppi_controller
{

void RPPController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer>,
  const std::shared_ptr<navigo_costmap_2d::Costmap2DROS>)
{
  parent_ = parent;
  name_ = std::move(name);
  auto node = parent_.lock();
  auto parameter = [node, this](auto & value, const std::string & suffix, const auto & fallback) {
      const auto key = name_ + "." + suffix;
      if (!node->has_parameter(key)) {
        node->declare_parameter(key, fallback);
      }
      node->get_parameter(key, value);
    };
  parameter(desired_linear_vel_, "desired_linear_vel", desired_linear_vel_);
  parameter(min_linear_vel_, "min_linear_vel", min_linear_vel_);
  parameter(lookahead_dist_, "lookahead_dist", lookahead_dist_);
  parameter(min_lookahead_dist_, "min_lookahead_dist", min_lookahead_dist_);
  parameter(max_angular_vel_, "max_angular_vel", max_angular_vel_);
  parameter(rotate_to_heading_threshold_, "rotate_to_heading_threshold", rotate_to_heading_threshold_);
  parameter(rotate_to_heading_angular_vel_, "rotate_to_heading_angular_vel", rotate_to_heading_angular_vel_);
  parameter(curvature_speed_regulation_, "curvature_speed_regulation", curvature_speed_regulation_);
  RCLCPP_INFO(logger_, "Configured RPP controller: %s", name_.c_str());
}

void RPPController::cleanup()
{
  plan_.poses.clear();
  speed_limit_scale_ = 1.0;
}

void RPPController::activate() {}
void RPPController::deactivate() {}

void RPPController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
}

void RPPController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit < 0.0) {
    speed_limit_scale_ = 1.0;
  } else if (percentage) {
    speed_limit_scale_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else {
    speed_limit_scale_ = std::clamp(speed_limit / std::max(desired_linear_vel_, 1e-3), 0.0, 1.0);
  }
}

geometry_msgs::msg::TwistStamped RPPController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist &,
  navigo_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped command;
  command.header = robot_pose.header;
  if (plan_.poses.empty()) {
    return command;
  }
  const auto & goal = plan_.poses.back();
  if (goal_checker && goal_checker->isGoalReached(robot_pose.pose, goal.pose, geometry_msgs::msg::Twist{})) {
    return command;
  }

  size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan_.poses.size(); ++i) {
    const auto dx = plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    const auto dy = plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    const auto distance = dx * dx + dy * dy;
    if (distance < nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }

  const auto lookahead = std::max(min_lookahead_dist_, lookahead_dist_);
  size_t target = nearest;
  for (size_t i = nearest; i < plan_.poses.size(); ++i) {
    const auto dx = plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    const auto dy = plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    if (std::hypot(dx, dy) >= lookahead) {
      target = i;
      break;
    }
    target = i;
  }
  const auto & target_pose = plan_.poses[target].pose;
  const auto dx = target_pose.position.x - robot_pose.pose.position.x;
  const auto dy = target_pose.position.y - robot_pose.pose.position.y;
  const auto heading = tf2::getYaw(robot_pose.pose.orientation);
  const auto target_heading = std::atan2(dy, dx);
  const auto heading_error = angles::shortest_angular_distance(heading, target_heading);
  if (std::abs(heading_error) > rotate_to_heading_threshold_) {
    command.twist.angular.z = std::clamp(
      std::copysign(rotate_to_heading_angular_vel_, heading_error),
      -max_angular_vel_, max_angular_vel_);
    return command;
  }

  const auto distance = std::max(std::hypot(dx, dy), min_lookahead_dist_);
  const auto curvature = 2.0 * std::sin(heading_error) / distance;
  auto speed = desired_linear_vel_ * speed_limit_scale_;
  if (curvature_speed_regulation_) {
    speed /= 1.0 + 2.0 * std::abs(curvature);
  }
  speed = std::clamp(speed, min_linear_vel_ * speed_limit_scale_, desired_linear_vel_ * speed_limit_scale_);
  command.twist.linear.x = speed;
  command.twist.angular.z = std::clamp(curvature * speed, -max_angular_vel_, max_angular_vel_);
  return command;
}

}  // namespace navigo_mppi_controller

PLUGINLIB_EXPORT_CLASS(navigo_mppi_controller::RPPController, navigo_core::Controller)
