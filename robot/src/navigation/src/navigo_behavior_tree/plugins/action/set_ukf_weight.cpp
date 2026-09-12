#include <algorithm>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "navigo_behavior_tree/plugins/action/set_ukf_weight.hpp"

namespace navigo_behavior_tree
{

SetUkfWeight::SetUkfWeight(
  const std::string & name, const BT::NodeConfiguration & conf)
: BtServiceNode(name, conf, "/localization/set_fusion_profile")
{}

BT::PortsList SetUkfWeight::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<bool>("enabled", true, "Whether profile switching is allowed"),
    BT::InputPort<std::string>(
      "profile", std::string("lio_hold"), "Restricted fusion profile"),
    BT::InputPort<double>("duration_seconds", 10.0, "Temporary profile lifetime"),
    BT::InputPort<std::string>(
      "reason", std::string("self_healing"), "Audit reason")});
}

void SetUkfWeight::on_tick()
{
  bool enabled = true;
  getInput("enabled", enabled);
  if (!enabled) {
    should_send_request_ = false;
    return;
  }
  std::string profile = "lio_hold";
  std::string reason = "self_healing";
  double duration_seconds = 10.0;
  getInput("profile", profile);
  getInput("reason", reason);
  getInput("duration_seconds", duration_seconds);
  using Request = robots_dog_msgs::srv::SetLocalizationFusionProfile::Request;
  if (profile == "balanced") {
    request_->profile = Request::PROFILE_BALANCED;
  } else if (profile == "lio_hold") {
    request_->profile = Request::PROFILE_LIO_HOLD;
  } else {
    request_->profile = Request::PROFILE_NOMINAL;
  }
  request_->reason = reason;
  request_->duration_seconds = request_->profile == Request::PROFILE_NOMINAL ?
    0.0F : static_cast<float>(std::max(1.0, std::min(180.0, duration_seconds)));
}

BT::NodeStatus SetUkfWeight::on_completion(
  std::shared_ptr<robots_dog_msgs::srv::SetLocalizationFusionProfile::Response> response)
{
  return response && response->accepted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace navigo_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigo_behavior_tree::SetUkfWeight>("SetUkfWeight");
}
