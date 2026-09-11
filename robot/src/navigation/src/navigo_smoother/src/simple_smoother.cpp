#include "navigo_smoother/simple_smoother.hpp"

#include <chrono>
#include <cmath>

#include "navigo_costmap_2d/cost_values.hpp"
#include "navigo_util/geometry_utils.hpp"
#include "navigo_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace navigo_smoother
{

void SimpleSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<navigo_costmap_2d::FootprintSubscriber>)
{
  costmap_sub_ = std::move(costmap_sub);
  const auto node = parent.lock();
  logger_ = node->get_logger();
  navigo_util::declare_parameter_if_not_declared(
    node, name + ".tolerance", rclcpp::ParameterValue(tolerance_));
  navigo_util::declare_parameter_if_not_declared(
    node, name + ".max_its", rclcpp::ParameterValue(max_iterations_));
  navigo_util::declare_parameter_if_not_declared(
    node, name + ".w_data", rclcpp::ParameterValue(data_weight_));
  navigo_util::declare_parameter_if_not_declared(
    node, name + ".w_smooth", rclcpp::ParameterValue(smooth_weight_));
  navigo_util::declare_parameter_if_not_declared(
    node, name + ".do_refinement", rclcpp::ParameterValue(do_refinement_));
  node->get_parameter(name + ".tolerance", tolerance_);
  node->get_parameter(name + ".max_its", max_iterations_);
  node->get_parameter(name + ".w_data", data_weight_);
  node->get_parameter(name + ".w_smooth", smooth_weight_);
  node->get_parameter(name + ".do_refinement", do_refinement_);
}

bool SimpleSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (path.poses.size() < 3 || !costmap_sub_) {return true;}
  const auto original = path;
  auto candidate = path;
  const auto costmap = costmap_sub_->getCostmap();
  const auto start = std::chrono::steady_clock::now();
  const int passes = do_refinement_ ? 2 : 1;

  for (int pass = 0; pass < passes; ++pass) {
    double change = tolerance_;
    for (int iteration = 0; change >= tolerance_; ++iteration) {
      if (iteration >= max_iterations_ ||
        std::chrono::steady_clock::now() - start >
        std::chrono::duration<double>(max_time.seconds())) {
        path = original;
        return false;
      }
      change = 0.0;
      auto next = candidate;
      for (size_t i = 1; i + 1 < candidate.poses.size(); ++i) {
        auto & point = next.poses[i].pose.position;
        const auto & before = candidate.poses[i - 1].pose.position;
        const auto & current = candidate.poses[i].pose.position;
        const auto & after = candidate.poses[i + 1].pose.position;
        const double x = current.x + data_weight_ * (original.poses[i].pose.position.x - current.x) +
          smooth_weight_ * (before.x + after.x - 2.0 * current.x);
        const double y = current.y + data_weight_ * (original.poses[i].pose.position.y - current.y) +
          smooth_weight_ * (before.y + after.y - 2.0 * current.y);
        if (costmap) {
          unsigned int mx = 0, my = 0;
          if (!costmap->worldToMap(x, y, mx, my) ||
            (costmap->getCost(mx, my) > navigo_costmap_2d::MAX_NON_OBSTACLE &&
             costmap->getCost(mx, my) != navigo_costmap_2d::NO_INFORMATION)) {
            path = original;
            return false;
          }
        }
        change += std::abs(x - current.x) + std::abs(y - current.y);
        point.x = x;
        point.y = y;
      }
      candidate = std::move(next);
    }
  }
  if (!updateOrientations(candidate)) {
    path = original;
    return false;
  }
  path = std::move(candidate);
  return true;
}

bool SimpleSmoother::updateOrientations(nav_msgs::msg::Path & path) const
{
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & current = path.poses[i].pose.position;
    const auto & next = path.poses[i + 1].pose.position;
    const double dx = next.x - current.x;
    const double dy = next.y - current.y;
    if (std::hypot(dx, dy) < 1e-4) {continue;}
    path.poses[i].pose.orientation = navigo_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
  }
  return true;
}

}  // namespace navigo_smoother

PLUGINLIB_EXPORT_CLASS(navigo_smoother::SimpleSmoother, navigo_core::Smoother)
