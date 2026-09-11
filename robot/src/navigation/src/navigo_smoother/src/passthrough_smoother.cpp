#include "navigo_smoother/passthrough_smoother.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace navigo_smoother
{

void PassthroughSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>)
{
  parent_ = parent;
  name_ = std::move(name);
}

void PassthroughSmoother::cleanup() {}
void PassthroughSmoother::activate() {}
void PassthroughSmoother::deactivate() {}

bool PassthroughSmoother::smooth(nav_msgs::msg::Path &, const rclcpp::Duration &)
{
  return true;
}

}  // namespace navigo_smoother

PLUGINLIB_EXPORT_CLASS(navigo_smoother::PassthroughSmoother, nav2_core::Smoother)
