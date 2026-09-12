#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/decorator/recovery_lease_scope.hpp"

namespace navigo_behavior_tree
{

using namespace std::chrono_literals;

RecoveryLeaseScope::RecoveryLeaseScope(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<robots_dog_msgs::srv::NavigationRecoveryLease>(
    "/navigation/recovery/lease");
  self_heal_client_ = node_->create_client<robots_dog_msgs::srv::NavigationSelfHealing>(
    "/navigation/self_healing");
  getInput("owner", owner_);
  getInput("reason", reason_);
  auto timeout = config().blackboard->get<std::chrono::milliseconds>("server_timeout");
  getInput("server_timeout", timeout);
  server_timeout_ = timeout;
}

BT::PortsList RecoveryLeaseScope::providedPorts()
{
  return {
    BT::InputPort<std::string>("owner", std::string("BT_NAVIGATOR"), "Lease owner"),
    BT::InputPort<std::string>("reason", std::string("bt_recovery"), "Lease reason"),
    BT::InputPort<std::string>("episode_id", std::string(""), "Self-healing episode"),
    BT::InputPort<unsigned>("level", 2u, "Recovery level"),
    BT::InputPort<std::string>("action_type", std::string(""), "Concrete action type"),
    BT::InputPort<std::chrono::milliseconds>("server_timeout")};
}

bool RecoveryLeaseScope::ensureActionReported()
{
  if (action_reported_ || action_reporting_unavailable_) {
    return true;
  }
  if (!action_start_sent_) {
    if (!self_heal_client_->service_is_ready()) {
      RCLCPP_WARN(node_->get_logger(), "Self-healing action service is unavailable");
      action_reporting_unavailable_ = true;
      return true;
    }
    getInput("episode_id", episode_id_);
    getInput("level", level_);
    getInput("action_type", action_type_);
    if (action_type_.empty()) {
      action_type_ = reason_;
    }
    auto request = std::make_shared<robots_dog_msgs::srv::NavigationSelfHealing::Request>();
    request->operation = robots_dog_msgs::srv::NavigationSelfHealing::Request::ACTION_STARTED;
    request->episode_id = episode_id_;
    request->fault_label = "navigation_failed";
    request->level = static_cast<uint8_t>(std::min(level_, 3u));
    request->action_type = action_type_;
    action_start_future_ = self_heal_client_->async_send_request(request).share();
    action_start_time_ = node_->now();
    action_start_sent_ = true;
    return false;
  }
  if (action_start_future_.wait_for(0ms) != std::future_status::ready) {
    if ((node_->now() - action_start_time_).nanoseconds() <
      server_timeout_.count() * 1000000LL)
    {
      return false;
    }
    RCLCPP_WARN(node_->get_logger(), "Self-healing action report timed out");
    action_start_sent_ = false;
    action_reporting_unavailable_ = true;
    return true;
  }
  const auto response = action_start_future_.get();
  action_start_sent_ = false;
  if (response && response->accepted) {
    action_id_ = response->action_id;
    episode_id_ = response->episode_id;
    action_reported_ = true;
  } else {
    action_reporting_unavailable_ = true;
  }
  return true;
}

BT::NodeStatus RecoveryLeaseScope::tick()
{
  setStatus(BT::NodeStatus::RUNNING);
  if (!acquired_) {
    if (!acquire_sent_) {
      if (!client_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Recovery lease service is unavailable");
        return BT::NodeStatus::FAILURE;
      }
      auto request = std::make_shared<robots_dog_msgs::srv::NavigationRecoveryLease::Request>();
      request->operation = robots_dog_msgs::srv::NavigationRecoveryLease::Request::ACQUIRE;
      request->owner = owner_;
      request->reason = reason_;
      acquire_future_ = client_->async_send_request(request).share();
      acquire_started_ = node_->now();
      acquire_sent_ = true;
      return BT::NodeStatus::RUNNING;
    }
    if (acquire_future_.wait_for(0ms) != std::future_status::ready) {
      if ((node_->now() - acquire_started_).nanoseconds() < server_timeout_.count() * 1000000LL) {
        return BT::NodeStatus::RUNNING;
      }
      RCLCPP_WARN(node_->get_logger(), "Recovery lease request timed out");
      acquire_sent_ = false;
      return BT::NodeStatus::FAILURE;
    }
    const auto response = acquire_future_.get();
    acquire_sent_ = false;
    if (!response->granted) {
      RCLCPP_WARN(node_->get_logger(), "Recovery lease rejected: attempts=%u exhausted=%s",
        response->attempts, response->exhausted ? "true" : "false");
      return BT::NodeStatus::FAILURE;
    }
    generation_ = response->generation;
    acquired_ = true;
  }

  if (!ensureActionReported()) {
    return BT::NodeStatus::RUNNING;
  }

  const auto child_status = child_node_->executeTick();
  if (child_status != BT::NodeStatus::RUNNING) {
    requestRelease(
      child_status == BT::NodeStatus::SUCCESS,
      child_status == BT::NodeStatus::SUCCESS ? "health_condition_recovered" :
      "recovery_action_exhausted");
  }
  return child_status;
}

void RecoveryLeaseScope::requestRelease(bool success, const std::string & detail)
{
  if (!acquired_) {
    return;
  }
  if (action_reported_ && self_heal_client_->service_is_ready()) {
    auto action_request =
      std::make_shared<robots_dog_msgs::srv::NavigationSelfHealing::Request>();
    action_request->operation =
      robots_dog_msgs::srv::NavigationSelfHealing::Request::ACTION_FINISHED;
    action_request->episode_id = episode_id_;
    action_request->action_id = action_id_;
    action_request->fault_label = "navigation_failed";
    action_request->level = static_cast<uint8_t>(std::min(level_, 3u));
    action_request->action_type = action_type_;
    action_request->success = success;
    action_request->detail = detail;
    self_heal_client_->async_send_request(action_request);
  }
  auto request = std::make_shared<robots_dog_msgs::srv::NavigationRecoveryLease::Request>();
  request->operation = robots_dog_msgs::srv::NavigationRecoveryLease::Request::RELEASE;
  request->owner = owner_;
  request->generation = generation_;
  // Releasing is best-effort: retaining a lease until the supervised task
  // reset is safer than allowing another owner after a transport failure.
  client_->async_send_request(request);
  acquired_ = false;
  generation_ = 0;
  action_start_sent_ = false;
  action_reported_ = false;
  action_reporting_unavailable_ = false;
  action_id_.clear();
}

void RecoveryLeaseScope::halt()
{
  // Stop the motion-producing child before releasing ownership. Releasing
  // first leaves a small window where another recovery owner can be granted
  // while the previous child is still publishing velocity commands.
  if (child_node_) {
    child_node_->halt();
  }
  requestRelease(false, "behavior_tree_halted");
  acquire_sent_ = false;
  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::RecoveryLeaseScope>("RecoveryLeaseScope");
}
