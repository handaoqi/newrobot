// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2022 Joshua Wallace
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

#ifndef NAVIGO_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_
#define NAVIGO_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "navigo_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "navigo_util/node_utils.hpp"

namespace navigo_behaviors
{

/**
 * @class navigo_behaviors::DriveOnHeading
 * @brief An action server Behavior for spinning in
 */
template<typename ActionT = nav2_msgs::action::DriveOnHeading>
class DriveOnHeading : public TimedBehavior<ActionT>
{
public:
  /**
   * @brief A constructor for navigo_behaviors::DriveOnHeading
   */
  DriveOnHeading()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    command_distance_(0.0),
    command_unit_x_(1.0),
    command_unit_y_(0.0),
    command_speed_(0.0),
    simulate_ahead_time_(0.0)
  {
  }

  ~DriveOnHeading() = default;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    if (command->target.z != 0.0 ||
      (std::abs(command->target.x) > 1e-6 && std::abs(command->target.y) > 1e-6))
    {
      RCLCPP_INFO(this->logger_, "DriveOnHeading accepts one planar body axis at a time.");
      return Status::FAILED;
    }

    // Ensure that both the speed and direction have the same sign
    const double signed_distance = std::abs(command->target.y) > 1e-6 ?
      command->target.y : command->target.x;
    if (std::abs(signed_distance) <= 1e-6 ||
      !((signed_distance > 0.0) == (command->speed > 0.0)))
    {
      RCLCPP_ERROR(this->logger_, "Speed and command sign did not match");
      return Status::FAILED;
    }

    command_distance_ = std::abs(signed_distance);
    command_unit_x_ = std::abs(command->target.y) > 1e-6 ? 0.0 : std::copysign(
      1.0,
      command->target.x);
    command_unit_y_ = std::abs(command->target.y) >
      1e-6 ? std::copysign(1.0, command->target.y) : 0.0;
    command_speed_ = command->speed;
    command_time_allowance_ = command->time_allowance;

    end_time_ = this->clock_->now() + command_time_allowance_;

    if (!navigo_util::getCurrentPose(
        initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }

    return Status::SUCCEEDED;
  }

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override
  {
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "Exceeded time allowance before reaching the DriveOnHeading goal - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!navigo_util::getCurrentPose(
        current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }

    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = std::hypot(diff_x, diff_y);

    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

    if (distance >= command_distance_) {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;

    if (acceleration_limit_ == 0.0 || deceleration_limit_ == 0.0) {
      RCLCPP_INFO_ONCE(this->logger_, "DriveOnHeading: no acceleration or deceleration limits set");
      const double speed = std::abs(command_speed_);
      cmd_vel->linear.x = command_unit_x_ * speed;
      cmd_vel->linear.y = command_unit_y_ * speed;
    } else {
      const double current_speed = last_vel_ ==
        std::numeric_limits<double>::max() ? 0.0 : last_vel_;
      const double limited_speed = std::clamp(
        std::abs(command_speed_), 0.0,
        current_speed + acceleration_limit_ / this->cycle_frequency_);
      cmd_vel->linear.x = command_unit_x_ * limited_speed;
      cmd_vel->linear.y = command_unit_y_ * limited_speed;

      // Check if we need to slow down to avoid overshooting
      auto remaining_distance = command_distance_ - distance;
      double max_vel_to_stop = std::sqrt(-2.0 * deceleration_limit_ * remaining_distance);
      if (max_vel_to_stop < std::hypot(cmd_vel->linear.x, cmd_vel->linear.y)) {
        cmd_vel->linear.x = command_unit_x_ * max_vel_to_stop;
        cmd_vel->linear.y = command_unit_y_ * max_vel_to_stop;
      }
    }

    // Ensure we don't go below minimum speed
    if (std::hypot(cmd_vel->linear.x, cmd_vel->linear.y) < minimum_speed_) {
      cmd_vel->linear.x = command_unit_x_ * minimum_speed_;
      cmd_vel->linear.y = command_unit_y_ * minimum_speed_;
    }

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = current_pose.pose.position.x;
    pose2d.y = current_pose.pose.position.y;
    pose2d.theta = tf2::getYaw(current_pose.pose.orientation);

    if (!isCollisionFree(distance, cmd_vel.get(), pose2d)) {
      this->stopRobot();
      RCLCPP_WARN(this->logger_, "Collision Ahead - Exiting DriveOnHeading");
      return Status::FAILED;
    }

    last_vel_ = std::hypot(cmd_vel->linear.x, cmd_vel->linear.y);
    this->vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

  void onCleanup() override {last_vel_ = std::numeric_limits<double>::max();}

  void onActionCompletion() override
  {
    last_vel_ = std::numeric_limits<double>::max();
  }

protected:
  /**
   * @brief Check if pose is collision free
   * @param distance Distance to check forward
   * @param cmd_vel current commanded velocity
   * @param pose2d Current pose
   * @return is collision free or not
   */
  bool isCollisionFree(
    const double & distance,
    geometry_msgs::msg::Twist * cmd_vel,
    geometry_msgs::msg::Pose2D & pose2d)
  {
    // Simulate ahead by simulate_ahead_time_ in this->cycle_frequency_ increments
    int cycle_count = 0;
    double sim_position_change;
    const double diff_dist = command_distance_ - distance;
    const int max_cycle_count = static_cast<int>(this->cycle_frequency_ * simulate_ahead_time_);
    geometry_msgs::msg::Pose2D init_pose = pose2d;
    bool fetch_data = true;

    while (cycle_count < max_cycle_count) {
      const double dt = cycle_count / this->cycle_frequency_;
      const double body_x = cmd_vel->linear.x * dt;
      const double body_y = cmd_vel->linear.y * dt;
      sim_position_change = std::hypot(body_x, body_y);
      pose2d.x = init_pose.x + body_x * std::cos(init_pose.theta) -
        body_y * std::sin(init_pose.theta);
      pose2d.y = init_pose.y + body_x * std::sin(init_pose.theta) +
        body_y * std::cos(init_pose.theta);
      cycle_count++;

      if (diff_dist - std::abs(sim_position_change) <= 0.) {
        break;
      }

      if (!this->collision_checker_->isCollisionFree(pose2d, fetch_data)) {
        return false;
      }
      fetch_data = false;
    }
    return true;
  }

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override
  {
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    navigo_util::declare_parameter_if_not_declared(
      node,
      "simulate_ahead_time", rclcpp::ParameterValue(2.0));
    node->get_parameter("simulate_ahead_time", simulate_ahead_time_);

    navigo_util::declare_parameter_if_not_declared(
      node, this->behavior_name_ + ".acceleration_limit",
      rclcpp::ParameterValue(0.0));
    navigo_util::declare_parameter_if_not_declared(
      node, this->behavior_name_ + ".deceleration_limit",
      rclcpp::ParameterValue(0.0));
    navigo_util::declare_parameter_if_not_declared(
      node, this->behavior_name_ + ".minimum_speed",
      rclcpp::ParameterValue(0.0));
    node->get_parameter(this->behavior_name_ + ".acceleration_limit", acceleration_limit_);
    node->get_parameter(this->behavior_name_ + ".deceleration_limit", deceleration_limit_);
    node->get_parameter(this->behavior_name_ + ".minimum_speed", minimum_speed_);
    if (acceleration_limit_ < 0.0 || deceleration_limit_ > 0.0) {
      RCLCPP_ERROR(
        this->logger_,
        "DriveOnHeading: acceleration_limit and deceleration_limit must be "
        "positive and negative respectively");
      acceleration_limit_ = std::abs(acceleration_limit_);
      deceleration_limit_ = -std::abs(deceleration_limit_);
    }
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  double command_distance_;
  double command_unit_x_;
  double command_unit_y_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double simulate_ahead_time_;
  double acceleration_limit_;
  double deceleration_limit_;
  double minimum_speed_;
  double last_vel_ = std::numeric_limits<double>::max();
};

}  // namespace navigo_behaviors

#endif  // NAVIGO_BEHAVIORS__PLUGINS__DRIVE_ON_HEADING_HPP_
