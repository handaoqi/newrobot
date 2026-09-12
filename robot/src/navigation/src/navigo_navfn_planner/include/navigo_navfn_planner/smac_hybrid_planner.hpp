#ifndef NAVIGO_NAVFN_PLANNER__SMAC_HYBRID_PLANNER_HPP_
#define NAVIGO_NAVFN_PLANNER__SMAC_HYBRID_PLANNER_HPP_

#include <memory>
#include <string>

#include "navigo_navfn_planner/navfn_planner.hpp"

namespace navigo_navfn_planner
{

/** Cost-aware SE(2) Hybrid-A* planner for the NaviGo plugin ABI. */
class SmacHybridPlanner : public NavfnPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros) override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  int angle_bins_{72};
  int max_iterations_{200000};
  double minimum_turning_radius_{0.50};
  double step_size_{0.10};
  double cost_penalty_{2.0};
  double reverse_penalty_{2.0};
  double change_penalty_{0.20};
  bool allow_reverse_{false};
};

}  // namespace navigo_navfn_planner

#endif  // NAVIGO_NAVFN_PLANNER__SMAC_HYBRID_PLANNER_HPP_
