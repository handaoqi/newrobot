#include "navigo_navfn_planner/theta_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "navigo_costmap_2d/cost_values.hpp"
#include "navigo_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace navigo_navfn_planner
{

bool ThetaStarPlanner::lineOfSight(
  const geometry_msgs::msg::PoseStamped & from,
  const geometry_msgs::msg::PoseStamped & to)
{
  unsigned int x0, y0, x1, y1;
  if (!worldToMap(from.pose.position.x, from.pose.position.y, x0, y0) ||
    !worldToMap(to.pose.position.x, to.pose.position.y, x1, y1))
  {
    return false;
  }

  const int dx = std::abs(static_cast<int>(x1) - static_cast<int>(x0));
  const int dy = std::abs(static_cast<int>(y1) - static_cast<int>(y0));
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx - dy;
  int x = static_cast<int>(x0);
  int y = static_cast<int>(y0);

  while (true) {
    const auto cost = costmap_->getCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
    if (cost >= navigo_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
    if (x == static_cast<int>(x1) && y == static_cast<int>(y1)) {
      break;
    }
    const int twice_error = 2 * error;
    if (twice_error > -dy) {
      error -= dy;
      x += sx;
    }
    if (twice_error < dx) {
      error += dx;
      y += sy;
    }
  }
  return true;
}

nav_msgs::msg::Path ThetaStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const auto navfn_path = NavfnPlanner::createPlan(start, goal);
  if (navfn_path.poses.size() < 3) {
    return navfn_path;
  }

  nav_msgs::msg::Path relaxed;
  relaxed.header = navfn_path.header;
  size_t anchor = 0;
  relaxed.poses.push_back(navfn_path.poses.front());

  while (anchor < navfn_path.poses.size() - 1) {
    size_t best = anchor + 1;
    for (size_t candidate = navfn_path.poses.size() - 1; candidate > anchor + 1; --candidate) {
      if (lineOfSight(navfn_path.poses[anchor], navfn_path.poses[candidate])) {
        best = candidate;
        break;
      }
    }
    auto pose = navfn_path.poses[best];
    if (best + 1 < navfn_path.poses.size()) {
      const auto & next = navfn_path.poses[best + 1].pose.position;
      const auto & current = pose.pose.position;
      pose.pose.orientation = navigo_util::geometry_utils::orientationAroundZAxis(
        std::atan2(next.y - current.y, next.x - current.x));
    }
    relaxed.poses.push_back(pose);
    anchor = best;
  }
  relaxed.poses.back().pose.orientation = navfn_path.poses.back().pose.orientation;
  return relaxed;
}

}  // namespace navigo_navfn_planner

PLUGINLIB_EXPORT_CLASS(navigo_navfn_planner::ThetaStarPlanner, navigo_core::GlobalPlanner)
