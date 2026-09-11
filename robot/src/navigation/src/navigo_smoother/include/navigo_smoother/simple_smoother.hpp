#ifndef NAVIGO_SMOOTHER__SIMPLE_SMOOTHER_HPP_
#define NAVIGO_SMOOTHER__SIMPLE_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "navigo_core/smoother.hpp"

namespace navigo_smoother
{

class SimpleSmoother : public navigo_core::Smoother
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<navigo_costmap_2d::FootprintSubscriber> footprint_sub) override;
  void cleanup() override {costmap_sub_.reset();}
  void activate() override {}
  void deactivate() override {}
  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

private:
  bool updateOrientations(nav_msgs::msg::Path & path) const;
  double tolerance_{1e-4};
  double data_weight_{0.2};
  double smooth_weight_{0.3};
  int max_iterations_{200};
  bool do_refinement_{false};
  std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("navigo_simple_smoother")};
};

}  // namespace navigo_smoother

#endif  // NAVIGO_SMOOTHER__SIMPLE_SMOOTHER_HPP_
