#include "navigo_smoother/savitzky_golay_smoother.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#include "navigo_costmap_2d/cost_values.hpp"
#include "navigo_util/geometry_utils.hpp"
#include "navigo_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigo_smoother
{

void SavitzkyGolaySmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<navigo_costmap_2d::FootprintSubscriber>)
{
  costmap_sub_ = std::move(costmap_sub);
  auto node = parent.lock();
  logger_ = node->get_logger();
  navigo_util::declare_parameter_if_not_declared(
    node, name + ".refinement_passes", rclcpp::ParameterValue(refinement_passes_));
  navigo_util::declare_parameter_if_not_declared(
    node, name + ".max_point_deviation", rclcpp::ParameterValue(max_point_deviation_));
  node->get_parameter(name + ".refinement_passes", refinement_passes_);
  node->get_parameter(name + ".max_point_deviation", max_point_deviation_);
  refinement_passes_ = std::clamp(refinement_passes_, 1, 4);
  max_point_deviation_ = std::max(0.01, max_point_deviation_);
}

bool SavitzkyGolaySmoother::smooth(
  nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (path.poses.size() < 9) {return true;}
  static constexpr std::array<double, 9> coefficients{
    -21.0 / 231.0, 14.0 / 231.0, 39.0 / 231.0, 54.0 / 231.0,
    59.0 / 231.0, 54.0 / 231.0, 39.0 / 231.0, 14.0 / 231.0, -21.0 / 231.0};
  const auto original = path;
  auto candidate = path;
  const auto started = std::chrono::steady_clock::now();
  auto costmap = costmap_sub_ ? costmap_sub_->getCostmap() : nullptr;

  for (int pass = 0; pass < refinement_passes_; ++pass) {
    auto filtered = candidate;
    for (size_t i = 4; i + 4 < candidate.poses.size(); ++i) {
      double x = 0.0;
      double y = 0.0;
      for (int offset = -4; offset <= 4; ++offset) {
        const auto & point = candidate.poses[static_cast<size_t>(
              static_cast<int>(i) + offset)].pose.position;
        const double coefficient = coefficients[static_cast<size_t>(offset + 4)];
        x += coefficient * point.x;
        y += coefficient * point.y;
      }
      const auto & source = original.poses[i].pose.position;
      const double displacement = std::hypot(x - source.x, y - source.y);
      if (displacement > max_point_deviation_) {
        const double scale = max_point_deviation_ / displacement;
        x = source.x + (x - source.x) * scale;
        y = source.y + (y - source.y) * scale;
      }
      if (costmap) {
        unsigned int mx = 0, my = 0;
        if (!costmap->worldToMap(x, y, mx, my) ||
          (costmap->getCost(mx, my) >= navigo_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
          costmap->getCost(mx, my) != navigo_costmap_2d::NO_INFORMATION))
        {
          path = original;
          return false;
        }
      }
      filtered.poses[i].pose.position.x = x;
      filtered.poses[i].pose.position.y = y;
    }
    candidate = std::move(filtered);
    if (std::chrono::steady_clock::now() - started >
      std::chrono::duration<double>(max_time.seconds()))
    {
      path = original;
      return false;
    }
  }

  for (size_t i = 0; i + 1 < candidate.poses.size(); ++i) {
    const auto & current = candidate.poses[i].pose.position;
    const auto & next = candidate.poses[i + 1].pose.position;
    if (std::hypot(next.x - current.x, next.y - current.y) > 1e-4) {
      candidate.poses[i].pose.orientation =
        navigo_util::geometry_utils::orientationAroundZAxis(
        std::atan2(next.y - current.y, next.x - current.x));
    }
  }
  candidate.poses.front().pose.position = original.poses.front().pose.position;
  candidate.poses.back() = original.poses.back();
  path = std::move(candidate);
  return true;
}

}  // namespace navigo_smoother

PLUGINLIB_EXPORT_CLASS(
  navigo_smoother::SavitzkyGolaySmoother, navigo_core::Smoother)
