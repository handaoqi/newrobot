#include "navigo_mppi_controller/ilqr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <Eigen/LU>

#include "navigo_core/exceptions.hpp"
#include "navigo_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace navigo_mppi_controller
{
namespace
{
double angleError(double current, double target)
{
  return std::atan2(std::sin(current - target), std::cos(current - target));
}
}  // namespace

void ILQRController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer>,
  const std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = std::move(name);
  costmap_ros_ = costmap_ros;
  auto node = parent_.lock();
  auto read = [node, this](auto & value, const std::string & suffix, const auto & fallback) {
      const auto key = name_ + "." + suffix;
      if (!node->has_parameter(key)) {node->declare_parameter(key, fallback);}
      node->get_parameter(key, value);
    };
  read(desired_linear_vel_, "desired_linear_vel", desired_linear_vel_);
  read(max_angular_vel_, "max_angular_vel", max_angular_vel_);
  read(dt_, "model_dt", dt_);
  read(horizon_, "time_steps", horizon_);
  read(iterations_, "iteration_count", iterations_);
  read(position_weight_, "position_weight", position_weight_);
  read(yaw_weight_, "yaw_weight", yaw_weight_);
  read(velocity_weight_, "velocity_weight", velocity_weight_);
  read(angular_weight_, "angular_weight", angular_weight_);
  horizon_ = std::max(4, horizon_);
  iterations_ = std::max(1, iterations_);
  dt_ = std::max(0.01, dt_);
}

void ILQRController::cleanup()
{
  plan_.poses.clear();
  costmap_ros_.reset();
  speed_limit_scale_ = 1.0;
}

void ILQRController::activate() {}
void ILQRController::deactivate() {}
void ILQRController::setPlan(const nav_msgs::msg::Path & path) {plan_ = path;}

void ILQRController::setSpeedLimit(const double & limit, const bool & percentage)
{
  if (limit < 0.0) {speed_limit_scale_ = 1.0; return;}
  speed_limit_scale_ = percentage ? std::clamp(limit / 100.0, 0.0, 1.0) :
    std::clamp(limit / std::max(desired_linear_vel_, 1e-3), 0.0, 1.0);
}

geometry_msgs::msg::TwistStamped ILQRController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  navigo_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped output;
  output.header = robot_pose.header;
  if (plan_.poses.empty()) {
    throw navigo_core::PlannerException("ILQR received an empty path");
  }
  if (goal_checker && goal_checker->isGoalReached(
      robot_pose.pose, plan_.poses.back().pose, geometry_msgs::msg::Twist{}))
  {
    return output;
  }

  size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan_.poses.size(); ++i) {
    const double dx = plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    const double distance = dx * dx + dy * dy;
    if (distance < nearest_distance) {nearest = i; nearest_distance = distance;}
  }

  using State = Eigen::Vector3d;
  using Control = Eigen::Vector2d;
  std::vector<State> reference(static_cast<size_t>(horizon_));
  for (int k = 0; k < horizon_; ++k) {
    const size_t index = std::min(nearest + static_cast<size_t>(k), plan_.poses.size() - 1);
    const auto & pose = plan_.poses[index].pose;
    double yaw = tf2::getYaw(pose.orientation);
    if (index + 1 < plan_.poses.size()) {
      const auto & next = plan_.poses[index + 1].pose.position;
      yaw = std::atan2(next.y - pose.position.y, next.x - pose.position.x);
    }
    reference[static_cast<size_t>(k)] = State(pose.position.x, pose.position.y, yaw);
  }

  const double nominal_v = desired_linear_vel_ * speed_limit_scale_;
  std::vector<Control> controls(static_cast<size_t>(horizon_), Control(nominal_v, 0.0));
  std::vector<State> states(static_cast<size_t>(horizon_ + 1));
  states.front() = State(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation));
  const Eigen::Matrix3d q =
    (Eigen::Vector3d(position_weight_, position_weight_, yaw_weight_)).asDiagonal();
  const Eigen::Matrix2d r =
    (Eigen::Vector2d(velocity_weight_, angular_weight_)).asDiagonal();

  for (int iteration = 0; iteration < iterations_; ++iteration) {
    for (int k = 0; k < horizon_; ++k) {
      const auto & state = states[static_cast<size_t>(k)];
      const auto & control = controls[static_cast<size_t>(k)];
      states[static_cast<size_t>(k + 1)] = State(
        state.x() + control.x() * std::cos(state.z()) * dt_,
        state.y() + control.x() * std::sin(state.z()) * dt_,
        state.z() + control.y() * dt_);
    }
    Eigen::Matrix3d value = q;
    for (int k = horizon_ - 1; k >= 0; --k) {
      const auto & state = states[static_cast<size_t>(k)];
      const auto & control = controls[static_cast<size_t>(k)];
      Eigen::Matrix3d a = Eigen::Matrix3d::Identity();
      a(0, 2) = -control.x() * std::sin(state.z()) * dt_;
      a(1, 2) = control.x() * std::cos(state.z()) * dt_;
      Eigen::Matrix<double, 3, 2> b = Eigen::Matrix<double, 3, 2>::Zero();
      b(0, 0) = std::cos(state.z()) * dt_;
      b(1, 0) = std::sin(state.z()) * dt_;
      b(2, 1) = dt_;
      const Eigen::Matrix2d inverse = (r + b.transpose() * value * b).inverse();
      const Eigen::Matrix<double, 2, 3> gain = inverse * b.transpose() * value * a;
      State error = state - reference[static_cast<size_t>(k)];
      error.z() = angleError(state.z(), reference[static_cast<size_t>(k)].z());
      Control updated = Control(nominal_v, 0.0) - gain * error;
      updated.x() = std::clamp(updated.x(), 0.0, nominal_v);
      updated.y() = std::clamp(updated.y(), -max_angular_vel_, max_angular_vel_);
      controls[static_cast<size_t>(k)] = 0.5 * control + 0.5 * updated;
      value = q + a.transpose() * value * (a - b * gain);
    }
  }

  const Control command = controls.front();
  auto * costmap = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
  if (costmap) {
    State predicted = states.front();
    for (int k = 0; k < std::min(horizon_, 10); ++k) {
      predicted.x() += command.x() * std::cos(predicted.z()) * dt_;
      predicted.y() += command.x() * std::sin(predicted.z()) * dt_;
      predicted.z() += command.y() * dt_;
      unsigned int mx = 0, my = 0;
      if (!costmap->worldToMap(predicted.x(), predicted.y(), mx, my) ||
        costmap->getCost(mx, my) >= navigo_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        throw navigo_core::PlannerException("ILQR prediction intersects an obstacle");
      }
    }
  }
  output.twist.linear.x = command.x();
  output.twist.linear.y = 0.0;
  output.twist.angular.z = command.y();
  (void)robot_speed;
  return output;
}

}  // namespace navigo_mppi_controller

PLUGINLIB_EXPORT_CLASS(navigo_mppi_controller::ILQRController, navigo_core::Controller)
