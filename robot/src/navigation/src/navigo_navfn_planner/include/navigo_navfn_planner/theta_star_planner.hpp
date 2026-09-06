#ifndef NAVIGO_NAVFN_PLANNER__THETA_STAR_PLANNER_HPP_
#define NAVIGO_NAVFN_PLANNER__THETA_STAR_PLANNER_HPP_

#include "navigo_navfn_planner/navfn_planner.hpp"

namespace navigo_navfn_planner
{

/**
 * Theta* global planner.
 *
 * NavFn supplies the cost-aware graph path.  This plugin then performs the
 * Theta* line-of-sight relaxation over that path, retaining only shortcuts
 * whose complete rasterized segment is free of lethal/inflated cells.  The
 * result is a real planner plugin with a distinct selector id, while keeping
 * the existing NavFn implementation as the safe fallback for blocked maps.
 */
class ThetaStarPlanner : public NavfnPlanner
{
public:
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  bool lineOfSight(
    const geometry_msgs::msg::PoseStamped & from,
    const geometry_msgs::msg::PoseStamped & to);
};

}  // namespace navigo_navfn_planner

#endif  // NAVIGO_NAVFN_PLANNER__THETA_STAR_PLANNER_HPP_
