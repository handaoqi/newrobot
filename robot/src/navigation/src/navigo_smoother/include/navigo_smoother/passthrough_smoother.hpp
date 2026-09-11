#ifndef NAVIGO_SMOOTHER__PASSTHROUGH_SMOOTHER_HPP_
#define NAVIGO_SMOOTHER__PASSTHROUGH_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "navigo_core/smoother.hpp"

namespace navigo_smoother
{

// Deliberately preserves the planner geometry for RTK straight legs, docking
// and precision approach. It still participates in the same SmootherServer
// action path, so BT telemetry and fallback behavior remain uniform.
class PassthroughSmoother : public navigo_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<navigo_costmap_2d::FootprintSubscriber> footprint_sub) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
};

}  // namespace navigo_smoother

#endif  // NAVIGO_SMOOTHER__PASSTHROUGH_SMOOTHER_HPP_
