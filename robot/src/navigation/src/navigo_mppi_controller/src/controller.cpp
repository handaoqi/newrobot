// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
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

#include <stdint.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>
#include "navigo_mppi_controller/controller.hpp"
#include "navigo_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace navigo_mppi_controller
{

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);
  getParam(
    performance_log_interval_seconds_, "performance_log_interval_seconds", 10.0);
  performance_log_interval_seconds_ =
    std::max(1.0, performance_log_interval_seconds_);
  performance_window_started_ = std::chrono::steady_clock::now();
  performance_samples_ms_.clear();
  performance_publisher_ = node->create_publisher<std_msgs::msg::String>(
    "/mppi/performance", rclcpp::QoS(10));

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  performance_publisher_.reset();
  performance_samples_ms_.clear();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  performance_publisher_->on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  performance_publisher_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  navigo_core::GoalChecker * goal_checker)
{
  const auto cycle_started_at = std::chrono::steady_clock::now();

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
    // Do not combine a short old goal with a new task hours later into one
    // misleading percentile window.
    performance_samples_ms_.clear();
    performance_window_started_ = cycle_started_at;
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  geometry_msgs::msg::Pose goal = path_handler_.getTransformedGoal(robot_pose.header.stamp).pose;

  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);

  navigo_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<navigo_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal, goal_checker);

  visualize(std::move(transformed_plan));
  const auto cycle_finished_at = std::chrono::steady_clock::now();
  recordPerformance(
    std::chrono::duration<double, std::milli>(
      cycle_finished_at - cycle_started_at).count());

  return cmd;
}

void MPPIController::recordPerformance(double duration_ms)
{
  if (!std::isfinite(duration_ms) || duration_ms < 0.0) {
    return;
  }
  performance_samples_ms_.push_back(duration_ms);
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_seconds =
    std::chrono::duration<double>(now - performance_window_started_).count();
  if (elapsed_seconds < performance_log_interval_seconds_) {
    return;
  }

  std::sort(performance_samples_ms_.begin(), performance_samples_ms_.end());
  const auto percentile = [this](double quantile) {
      const auto count = performance_samples_ms_.size();
      const auto rank = static_cast<size_t>(std::ceil(quantile * count));
      return performance_samples_ms_[std::min(count - 1, std::max<size_t>(1, rank) - 1)];
    };
  const double sum = std::accumulate(
    performance_samples_ms_.begin(), performance_samples_ms_.end(), 0.0);
  std::ostringstream payload;
  payload << std::fixed << std::setprecision(3)
          << "{\"schema\":\"roamerx.mppi-performance.v1\""
          << ",\"window_seconds\":" << elapsed_seconds
          << ",\"sample_count\":" << performance_samples_ms_.size()
          << ",\"mean_ms\":" << sum / performance_samples_ms_.size()
          << ",\"p50_ms\":" << percentile(0.50)
          << ",\"p90_ms\":" << percentile(0.90)
          << ",\"p99_ms\":" << percentile(0.99)
          << ",\"max_ms\":" << performance_samples_ms_.back()
          << ",\"target_p99_ms\":40.0"
          << ",\"target_met\":" << (percentile(0.99) < 40.0 ? "true" : "false")
          << "}";
  std_msgs::msg::String message;
  message.data = payload.str();
  if (performance_publisher_ && performance_publisher_->is_activated()) {
    performance_publisher_->publish(message);
  }
  RCLCPP_INFO(
    logger_, "MPPI performance samples=%zu mean=%.3fms p90=%.3fms p99=%.3fms max=%.3fms",
    performance_samples_ms_.size(), sum / performance_samples_ms_.size(),
    percentile(0.90), percentile(0.99), performance_samples_ms_.back());
  performance_samples_ms_.clear();
  performance_window_started_ = now;
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  // The two marker arrays are the expensive half: batch_size trajectories over
  // time_steps samples, decimated by TrajectoryVisualizer's steps, is still a
  // few thousand Marker constructions per control cycle at controller_frequency
  // 20 Hz. They stay behind `visualize`, which is false in navigo_params.yaml.
  //
  // The transformed plan is one Path copy and is the only record of which local
  // segment the controller was tracking, so diagnostic rosbags need it without
  // paying for the markers. TrajectoryVisualizer publishes nothing when no one
  // is subscribed, so leaving this call unconditional costs nothing in normal
  // operation.
  if (visualize_) {
    trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
    trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  }
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace navigo_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(navigo_mppi_controller::MPPIController, navigo_core::Controller)
