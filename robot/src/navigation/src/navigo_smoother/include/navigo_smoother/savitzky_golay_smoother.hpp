#ifndef NAVIGO_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
#define NAVIGO_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "navigo_core/smoother.hpp"

namespace navigo_smoother
{

/** Quadratic nine-sample Savitzky-Golay path smoother. */
class SavitzkyGolaySmoother : public navigo_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<navigo_costmap_2d::FootprintSubscriber> footprint_sub) override;
  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}
  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

private:
  std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("SavitzkyGolaySmoother")};
  int refinement_passes_{2};
  double max_point_deviation_{0.30};
};

}  // namespace navigo_smoother

#endif  // NAVIGO_SMOOTHER__SAVITZKY_GOLAY_SMOOTHER_HPP_
