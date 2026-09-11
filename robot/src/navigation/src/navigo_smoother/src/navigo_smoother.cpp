#include "navigo_smoother/navigo_smoother.hpp"

#include <chrono>
#include <utility>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "navigo_util/node_utils.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"

namespace navigo_smoother
{

SmootherServer::SmootherServer(const rclcpp::NodeOptions & options)
: navigo_util::LifecycleNode("smoother_server", "", options),
  loader_("navigo_core", "navigo_core::Smoother")
{
  declare_parameter("costmap_topic", "global_costmap/costmap_raw");
  declare_parameter("footprint_topic", "global_costmap/published_footprint");
  declare_parameter("robot_base_frame", "base_link");
  declare_parameter("transform_tolerance", 0.1);
  declare_parameter("smoother_plugins", std::vector<std::string>{"simple_smoother"});
}

SmootherServer::~SmootherServer() = default;

navigo_util::CallbackReturn SmootherServer::on_configure(const rclcpp_lifecycle::State &)
{
  get_parameter("smoother_plugins", smoother_ids_);
  if (smoother_ids_.empty()) {
    RCLCPP_ERROR(get_logger(), "smoother_plugins must not be empty");
    return navigo_util::CallbackReturn::FAILURE;
  }

  auto node = shared_from_this();
  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  std::string costmap_topic, footprint_topic, base_frame;
  double transform_tolerance = 0.1;
  get_parameter("costmap_topic", costmap_topic);
  get_parameter("footprint_topic", footprint_topic);
  get_parameter("robot_base_frame", base_frame);
  get_parameter("transform_tolerance", transform_tolerance);
  costmap_sub_ = std::make_shared<navigo_costmap_2d::CostmapSubscriber>(node, costmap_topic);
  footprint_sub_ = std::make_shared<navigo_costmap_2d::FootprintSubscriber>(
    node, footprint_topic, *tf_, base_frame, transform_tolerance);
  collision_checker_ = std::make_shared<navigo_costmap_2d::CostmapTopicCollisionChecker>(
    *costmap_sub_, *footprint_sub_, get_name());

  if (!loadSmootherPlugins()) {
    return navigo_util::CallbackReturn::FAILURE;
  }
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan_smoothed", 1);
  action_server_ = std::make_unique<ActionServer>(
    node, "smooth_path", std::bind(&SmootherServer::smoothPlan, this), nullptr,
    std::chrono::milliseconds(500), true);
  return navigo_util::CallbackReturn::SUCCESS;
}

bool SmootherServer::loadSmootherPlugins()
{
  const auto node = shared_from_this();
  for (const auto & id : smoother_ids_) {
    try {
      const auto default_type = id == "simple_smoother" ?
        "navigo_smoother::SimpleSmoother" : "navigo_smoother::PassthroughSmoother";
      navigo_util::declare_parameter_if_not_declared(
        node, id + ".plugin", rclcpp::ParameterValue(default_type));
      const auto type = navigo_util::get_plugin_type_param(node, id);
      auto smoother = loader_.createUniqueInstance(type);
      smoother->configure(node, id, tf_, costmap_sub_, footprint_sub_);
      smoothers_.emplace(id, std::move(smoother));
      available_ids_ += id + " ";
    } catch (const pluginlib::PluginlibException & error) {
      RCLCPP_ERROR(get_logger(), "Unable to load smoother '%s': %s", id.c_str(), error.what());
      return false;
    }
  }
  RCLCPP_INFO(get_logger(), "NaviGo smoother server loaded: %s", available_ids_.c_str());
  return true;
}

navigo_util::CallbackReturn SmootherServer::on_activate(const rclcpp_lifecycle::State &)
{
  plan_publisher_->on_activate();
  for (auto & entry : smoothers_) {entry.second->activate();}
  action_server_->activate();
  createBond();
  return navigo_util::CallbackReturn::SUCCESS;
}

navigo_util::CallbackReturn SmootherServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  action_server_->deactivate();
  for (auto & entry : smoothers_) {entry.second->deactivate();}
  plan_publisher_->on_deactivate();
  destroyBond();
  return navigo_util::CallbackReturn::SUCCESS;
}

navigo_util::CallbackReturn SmootherServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (auto & entry : smoothers_) {entry.second->cleanup();}
  smoothers_.clear();
  available_ids_.clear();
  action_server_.reset();
  plan_publisher_.reset();
  collision_checker_.reset();
  footprint_sub_.reset();
  costmap_sub_.reset();
  transform_listener_.reset();
  tf_.reset();
  return navigo_util::CallbackReturn::SUCCESS;
}

navigo_util::CallbackReturn SmootherServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  return navigo_util::CallbackReturn::SUCCESS;
}

bool SmootherServer::findSmootherId(const std::string & requested, std::string & selected) const
{
  if (requested.empty() && smoothers_.size() == 1) {
    selected = smoothers_.begin()->first;
    return true;
  }
  if (smoothers_.count(requested) != 0U) {
    selected = requested;
    return true;
  }
  RCLCPP_ERROR(get_logger(), "Unknown smoother '%s'; available: %s", requested.c_str(), available_ids_.c_str());
  return false;
}

void SmootherServer::smoothPlan()
{
  const auto started = now();
  const auto goal = action_server_->get_current_goal();
  if (!goal) {return;}
  auto result = std::make_shared<Action::Result>();
  std::string id;
  if (!findSmootherId(goal->smoother_id, id)) {
    action_server_->terminate_current();
    return;
  }
  try {
    result->path = goal->path;
    result->was_completed = smoothers_.at(id)->smooth(result->path, goal->max_smoothing_duration);
    result->smoothing_duration = now() - started;
    plan_publisher_->publish(result->path);
    if (goal->check_for_collisions) {
      bool fetch_data = true;
      for (const auto & pose : result->path.poses) {
        geometry_msgs::msg::Pose2D pose2d;
        pose2d.x = pose.pose.position.x;
        pose2d.y = pose.pose.position.y;
        pose2d.theta = tf2::getYaw(pose.pose.orientation);
        if (!collision_checker_->isCollisionFree(pose2d, fetch_data)) {
          RCLCPP_ERROR(get_logger(), "Rejected smoothed path: collision at %.3f, %.3f", pose2d.x, pose2d.y);
          action_server_->terminate_current(result);
          return;
        }
        fetch_data = false;
      }
    }
    action_server_->succeeded_current(result);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Smoothing failed: %s", error.what());
    action_server_->terminate_current(result);
  }
}

}  // namespace navigo_smoother

RCLCPP_COMPONENTS_REGISTER_NODE(navigo_smoother::SmootherServer)
