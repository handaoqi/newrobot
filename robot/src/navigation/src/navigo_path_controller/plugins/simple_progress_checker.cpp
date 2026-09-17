// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "navigo_path_controller/plugins/simple_progress_checker.hpp"
#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <sstream>
#include "angles/angles.h"
#include "navigo_core/exceptions.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "navigo_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace navigo_path_controller
{
void SimpleProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  clock_ = node->get_clock();

  navigo_util::declare_parameter_if_not_declared(
    node, plugin_name + ".required_movement_radius", rclcpp::ParameterValue(0.5));
  navigo_util::declare_parameter_if_not_declared(
    node, plugin_name + ".movement_time_allowance", rclcpp::ParameterValue(10.0));
  navigo_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rotation_time_allowance", rclcpp::ParameterValue(30.0));
  navigo_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rotation_yaw_threshold", rclcpp::ParameterValue(0.03));
  navigo_util::declare_parameter_if_not_declared(
    node, plugin_name + ".rotation_progress_enabled", rclcpp::ParameterValue(true));
  // Scale is set to 0 by default, so if it was not set otherwise, set to 0
  node->get_parameter_or(plugin_name + ".required_movement_radius", radius_, 0.5);
  double time_allowance_param = 0.0;
  node->get_parameter_or(plugin_name + ".movement_time_allowance", time_allowance_param, 10.0);
  time_allowance_ = rclcpp::Duration::from_seconds(time_allowance_param);
  double rotation_time_param = 30.0;
  node->get_parameter_or(plugin_name + ".rotation_time_allowance", rotation_time_param, 30.0);
  rotation_time_allowance_ = rclcpp::Duration::from_seconds(rotation_time_param);
  node->get_parameter_or(plugin_name + ".rotation_yaw_threshold", rotation_yaw_threshold_, 0.03);
  node->get_parameter_or(plugin_name + ".rotation_progress_enabled", rotation_progress_enabled_, true);
  status_pub_ = node->create_publisher<std_msgs::msg::String>("/navigation/progress_status", 10);

  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&SimpleProgressChecker::dynamicParametersCallback, this, _1));
}

bool SimpleProgressChecker::check(geometry_msgs::msg::PoseStamped & current_pose)
{
  // relies on short circuit evaluation to not call is_robot_moved_enough if
  // baseline_pose is not set.
  geometry_msgs::msg::Pose2D current_pose2d;
  current_pose2d = nav_2d_utils::poseToPose2D(current_pose.pose);

  const auto now = clock_->now();
  if (!baseline_pose_set_) {
    resetBaselinePose(current_pose2d);
    if (status_pub_) {
      std_msgs::msg::String status;
      status.data = "baseline_initialized";
      status_pub_->publish(status);
    }
    return true;
  }
  if (isRobotMovedEnough(current_pose2d)) {
    resetBaselinePose(current_pose2d);
    if (status_pub_) {
      std_msgs::msg::String status;
      status.data = "translation_progress";
      status_pub_->publish(status);
    }
    return true;
  }
  if (rotation_progress_enabled_ && isRobotRotating(current_pose2d, now)) {
    if (status_pub_) {
      std_msgs::msg::String status;
      std::ostringstream stream;
      stream << "rotation_progress;rotation_timeout_s=" << rotation_time_allowance_.seconds();
      status.data = stream.str();
      status_pub_->publish(status);
    }
    return true;
  }

  const auto allowance = rotation_active_ ? rotation_time_allowance_ : time_allowance_;
  const auto start_time = rotation_active_ ? last_rotation_time_ : baseline_time_;
  const bool within_allowance = !((now - start_time) > allowance);
  if (status_pub_) {
    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << (within_allowance ? "waiting_for_progress" : "no_translation_or_rotation_progress")
           << ";rotation_active=" << (rotation_active_ ? "true" : "false")
           << ";timeout_s=" << allowance.seconds();
    status.data = stream.str();
    status_pub_->publish(status);
  }
  return within_allowance;
}

void SimpleProgressChecker::reset()
{
  baseline_pose_set_ = false;
  rotation_active_ = false;
  last_rotation_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void SimpleProgressChecker::resetBaselinePose(const geometry_msgs::msg::Pose2D & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = clock_->now();
  last_yaw_ = pose.theta;
  rotation_active_ = false;
  baseline_pose_set_ = true;
}

bool SimpleProgressChecker::isRobotRotating(
  const geometry_msgs::msg::Pose2D & pose, const rclcpp::Time & now)
{
  const double yaw_delta = std::abs(angles::shortest_angular_distance(last_yaw_, pose.theta));
  last_yaw_ = pose.theta;
  if (yaw_delta < std::max(0.001, rotation_yaw_threshold_)) {
    return false;
  }
  rotation_active_ = true;
  last_rotation_time_ = now;
  return true;
}

bool SimpleProgressChecker::isRobotMovedEnough(const geometry_msgs::msg::Pose2D & pose)
{
  return pose_distance(pose, baseline_pose_) > radius_;
}

double SimpleProgressChecker::pose_distance(
  const geometry_msgs::msg::Pose2D & pose1,
  const geometry_msgs::msg::Pose2D & pose2)
{
  double dx = pose1.x - pose2.x;
  double dy = pose1.y - pose2.y;

  return std::hypot(dx, dy);
}

rcl_interfaces::msg::SetParametersResult
SimpleProgressChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".required_movement_radius") {
        radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".movement_time_allowance") {
        time_allowance_ = rclcpp::Duration::from_seconds(parameter.as_double());
      } else if (name == plugin_name_ + ".rotation_time_allowance") {
        rotation_time_allowance_ = rclcpp::Duration::from_seconds(parameter.as_double());
      } else if (name == plugin_name_ + ".rotation_yaw_threshold") {
        rotation_yaw_threshold_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL &&
      name == plugin_name_ + ".rotation_progress_enabled") {
      rotation_progress_enabled_ = parameter.as_bool();
    }
  }
  result.successful = true;
  return result;
}

}  // namespace navigo_path_controller

PLUGINLIB_EXPORT_CLASS(navigo_path_controller::SimpleProgressChecker, navigo_core::ProgressChecker)
