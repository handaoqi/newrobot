#include "navigo_navfn_planner/smac_hybrid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navigo_costmap_2d/cost_values.hpp"
#include "navigo_util/geometry_utils.hpp"
#include "navigo_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace navigo_navfn_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

struct SearchNode
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double g{0.0};
  std::uint64_t parent{0};
  int steering{0};
  bool has_parent{false};
};

struct QueueItem
{
  double f{0.0};
  std::uint64_t key{0};
  bool operator<(const QueueItem & other) const {return f > other.f;}
};
}  // namespace

void SmacHybridPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<navigo_costmap_2d::Costmap2DROS> costmap_ros)
{
  NavfnPlanner::configure(parent, name, std::move(tf), std::move(costmap_ros));
  auto node = parent.lock();
  auto read = [node, this](auto & value, const std::string & suffix, const auto & fallback) {
      const auto key = name_ + "." + suffix;
      navigo_util::declare_parameter_if_not_declared(node, key, rclcpp::ParameterValue(fallback));
      node->get_parameter(key, value);
    };
  read(angle_bins_, "angle_quantization_bins", angle_bins_);
  read(max_iterations_, "max_iterations", max_iterations_);
  read(minimum_turning_radius_, "minimum_turning_radius", minimum_turning_radius_);
  read(step_size_, "step_size", step_size_);
  read(cost_penalty_, "cost_penalty", cost_penalty_);
  read(reverse_penalty_, "reverse_penalty", reverse_penalty_);
  read(change_penalty_, "change_penalty", change_penalty_);
  read(allow_reverse_, "allow_reverse", allow_reverse_);
  angle_bins_ = std::max(16, angle_bins_);
  max_iterations_ = std::max(1000, max_iterations_);
  minimum_turning_radius_ = std::max(0.10, minimum_turning_radius_);
  step_size_ = std::max(costmap_->getResolution(), step_size_);
}

nav_msgs::msg::Path SmacHybridPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = global_frame_;
  empty.header.stamp = clock_->now();
  unsigned int start_mx = 0, start_my = 0, goal_mx = 0, goal_my = 0;
  if (!worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my) ||
    !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
  {
    return empty;
  }

  const double bin_size = 2.0 * kPi / static_cast<double>(angle_bins_);
  const auto yaw_bin = [this, bin_size](double yaw) {
      int bin = static_cast<int>(std::floor((normalizeAngle(yaw) + kPi) / bin_size));
      return std::clamp(bin, 0, angle_bins_ - 1);
    };
  const auto make_key = [](unsigned int mx, unsigned int my, int bin) {
      return (static_cast<std::uint64_t>(bin) << 48) |
             (static_cast<std::uint64_t>(my) << 24) | static_cast<std::uint64_t>(mx);
    };
  const auto heuristic = [&goal](double x, double y) {
      return std::hypot(goal.pose.position.x - x, goal.pose.position.y - y);
    };
  const auto traversable = [this](double x, double y) {
      unsigned int mx = 0, my = 0;
      if (!worldToMap(x, y, mx, my)) {return false;}
      const auto cost = costmap_->getCost(mx, my);
      return cost < navigo_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
             (allow_unknown_ && cost == navigo_costmap_2d::NO_INFORMATION);
    };
  if (!traversable(start.pose.position.x, start.pose.position.y) ||
    !traversable(goal.pose.position.x, goal.pose.position.y))
  {
    return NavfnPlanner::createPlan(start, goal);
  }

  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const auto start_key = make_key(start_mx, start_my, yaw_bin(start_yaw));
  std::unordered_map<std::uint64_t, SearchNode> nodes;
  nodes.reserve(32768);
  nodes.emplace(
    start_key, SearchNode{
      start.pose.position.x, start.pose.position.y, start_yaw, 0.0, 0, 0, false});
  std::priority_queue<QueueItem> open;
  open.push({heuristic(start.pose.position.x, start.pose.position.y), start_key});
  std::unordered_map<std::uint64_t, bool> closed;
  closed.reserve(32768);
  std::uint64_t reached_key = 0;
  bool reached = false;
  int iterations = 0;

  while (!open.empty() && iterations++ < max_iterations_) {
    const auto current_item = open.top();
    open.pop();
    if (closed[current_item.key]) {continue;}
    closed[current_item.key] = true;
    const SearchNode current = nodes.at(current_item.key);
    if (heuristic(current.x, current.y) <= tolerance_) {
      reached_key = current_item.key;
      reached = true;
      break;
    }

    const int direction_count = allow_reverse_ ? 2 : 1;
    for (int direction_index = 0; direction_index < direction_count; ++direction_index) {
      const double direction = direction_index == 0 ? 1.0 : -1.0;
      for (int steering = -1; steering <= 1; ++steering) {
        const double distance = direction * step_size_;
        const double delta_yaw = steering ==
          0 ? 0.0 : distance / minimum_turning_radius_ * steering;
        const double mid_yaw = current.yaw + 0.5 * delta_yaw;
        const double next_x = current.x + distance * std::cos(mid_yaw);
        const double next_y = current.y + distance * std::sin(mid_yaw);
        const double next_yaw = normalizeAngle(current.yaw + delta_yaw);
        if (!traversable(
            0.5 * (current.x + next_x), 0.5 * (current.y + next_y)) ||
          !traversable(next_x, next_y))
        {
          continue;
        }
        unsigned int mx = 0, my = 0;
        if (!worldToMap(next_x, next_y, mx, my)) {continue;}
        const auto key = make_key(mx, my, yaw_bin(next_yaw));
        const double cell_cost = static_cast<double>(costmap_->getCost(mx, my)) / 252.0;
        const double transition = step_size_ * (direction < 0.0 ? reverse_penalty_ : 1.0) +
          cost_penalty_ * std::max(0.0, cell_cost) +
          (current.has_parent && current.steering != steering ? change_penalty_ : 0.0);
        const double next_g = current.g + transition;
        const auto found = nodes.find(key);
        if (found != nodes.end() && found->second.g <= next_g) {continue;}
        nodes[key] = SearchNode{
          next_x, next_y, next_yaw, next_g, current_item.key, steering, true};
        open.push({next_g + heuristic(next_x, next_y), key});
      }
    }
  }

  if (!reached) {
    RCLCPP_WARN(
      logger_, "%s: Hybrid-A* exhausted %d iterations; using NavFn fallback",
      name_.c_str(), iterations);
    return NavfnPlanner::createPlan(start, goal);
  }

  std::vector<SearchNode> reversed;
  for (auto key = reached_key;; ) {
    const auto & node = nodes.at(key);
    reversed.push_back(node);
    if (!node.has_parent) {break;}
    key = node.parent;
  }
  std::reverse(reversed.begin(), reversed.end());
  nav_msgs::msg::Path path;
  path.header = empty.header;
  path.poses.reserve(reversed.size() + 1);
  for (const auto & node : reversed) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    pose.pose.orientation = navigo_util::geometry_utils::orientationAroundZAxis(node.yaw);
    path.poses.push_back(std::move(pose));
  }
  auto final_pose = goal;
  final_pose.header = path.header;
  path.poses.push_back(final_pose);
  return path;
}

}  // namespace navigo_navfn_planner

PLUGINLIB_EXPORT_CLASS(
  navigo_navfn_planner::SmacHybridPlanner, navigo_core::GlobalPlanner)
