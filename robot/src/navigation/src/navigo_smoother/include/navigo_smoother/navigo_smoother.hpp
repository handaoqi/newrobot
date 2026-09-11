#ifndef NAVIGO_SMOOTHER__NAVIGO_SMOOTHER_HPP_
#define NAVIGO_SMOOTHER__NAVIGO_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "nav2_msgs/action/smooth_path.hpp"
#include "navigo_core/smoother.hpp"
#include "navigo_costmap_2d/costmap_subscriber.hpp"
#include "navigo_costmap_2d/costmap_topic_collision_checker.hpp"
#include "navigo_costmap_2d/footprint_subscriber.hpp"
#include "navigo_util/lifecycle_node.hpp"
#include "navigo_util/simple_action_server.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navigo_smoother
{

class SmootherServer : public navigo_util::LifecycleNode
{
public:
  using Action = nav2_msgs::action::SmoothPath;
  using ActionServer = navigo_util::SimpleActionServer<Action>;
  using SmootherMap = std::unordered_map<std::string, navigo_core::Smoother::Ptr>;

  explicit SmootherServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SmootherServer() override;

protected:
  navigo_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  navigo_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  navigo_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  navigo_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  navigo_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  bool loadSmootherPlugins();
  bool findSmootherId(const std::string & requested, std::string & selected) const;
  void smoothPlan();

  pluginlib::ClassLoader<navigo_core::Smoother> loader_;
  SmootherMap smoothers_;
  std::vector<std::string> smoother_ids_;
  std::string available_ids_;
  std::unique_ptr<ActionServer> action_server_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  std::shared_ptr<navigo_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<navigo_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<navigo_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
};

}  // namespace navigo_smoother

#endif  // NAVIGO_SMOOTHER__NAVIGO_SMOOTHER_HPP_
