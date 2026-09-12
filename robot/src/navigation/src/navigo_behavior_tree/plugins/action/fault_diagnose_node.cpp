#include <algorithm>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/action/fault_diagnose_node.hpp"

namespace navigo_behavior_tree
{

FaultDiagnoseNode::FaultDiagnoseNode(
  const std::string & name, const BT::NodeConfiguration & conf)
: BtServiceNode(name, conf, "/navigation/self_healing")
{}

BT::PortsList FaultDiagnoseNode::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>(
      "episode_id", std::string(""), "Existing self-healing episode"),
    BT::InputPort<std::string>(
      "fault_label", std::string("navigation_failed"), "Raw fault label"),
    BT::InputPort<unsigned>("level", 0u, "Requested recovery level"),
    BT::OutputPort<std::string>("diagnosed_episode_id"),
    BT::OutputPort<std::string>("diagnosed_fault"),
    BT::OutputPort<std::string>("scene_mode"),
    BT::OutputPort<std::string>("action_type"),
    BT::OutputPort<bool>("forbid_spin"),
    BT::OutputPort<bool>("wait_for_rtk"),
    BT::OutputPort<bool>("use_lio_hold"),
    BT::OutputPort<bool>("search_laser"),
    BT::OutputPort<bool>("recovered")});
}

void FaultDiagnoseNode::on_tick()
{
  std::string episode_id;
  std::string fault_label = "navigation_failed";
  unsigned level = 0;
  getInput("episode_id", episode_id);
  getInput("fault_label", fault_label);
  getInput("level", level);
  request_->operation = robots_dog_msgs::srv::NavigationSelfHealing::Request::DIAGNOSE;
  request_->episode_id = episode_id;
  request_->fault_label = fault_label;
  request_->level = static_cast<uint8_t>(std::min(level, 3u));
}

BT::NodeStatus FaultDiagnoseNode::on_completion(
  std::shared_ptr<robots_dog_msgs::srv::NavigationSelfHealing::Response> response)
{
  if (!response || !response->accepted) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("diagnosed_episode_id", response->episode_id);
  setOutput("diagnosed_fault", response->fault_label);
  setOutput("scene_mode", response->scene_mode);
  setOutput("action_type", response->action_type);
  setOutput("forbid_spin", response->forbid_spin);
  setOutput("wait_for_rtk", response->wait_for_rtk);
  setOutput("use_lio_hold", response->use_lio_hold);
  setOutput("search_laser", response->search_laser);
  setOutput("recovered", response->recovered);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::FaultDiagnoseNode>("FaultDiagnoseNode");
}
