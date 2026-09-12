#ifndef NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__ADAPTIVE_SPIN_HPP_
#define NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__ADAPTIVE_SPIN_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "localization/msg/scan_matching_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "robots_dog_msgs/srv/navigation_self_healing.hpp"

namespace navigo_behavior_tree
{

class AdaptiveSpin : public BT::DecoratorNode
{
public:
  AdaptiveSpin(const std::string & name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  void onScanMatchingStatus(const localization::msg::ScanMatchingStatus::SharedPtr msg);
  bool ndtHealthy() const;
  bool edgePermissionReady();
  void resetPermission();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<localization::msg::ScanMatchingStatus>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<robots_dog_msgs::srv::NavigationSelfHealing>::SharedPtr permission_client_;
  std::shared_future<robots_dog_msgs::srv::NavigationSelfHealing::Response::SharedPtr>
    permission_future_;
  std::atomic<bool> converged_{false};
  std::atomic<double> score_{0.0};
  std::atomic<int64_t> received_ns_{0};
  double score_threshold_{0.4};
  double max_age_seconds_{0.75};
  double permission_timeout_seconds_{1.0};
  bool permission_sent_{false};
  bool permission_confirmed_{false};
  rclcpp::Time permission_started_;
  std::string episode_id_;
  std::string fault_label_;
};

}  // namespace navigo_behavior_tree

#endif  // NAVIGO_BEHAVIOR_TREE__PLUGINS__DECORATOR__ADAPTIVE_SPIN_HPP_
