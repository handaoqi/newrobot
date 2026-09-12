// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "navigo_collision_monitor/collision_monitor_node.hpp"

#include <algorithm>
#include <exception>
#include <cmath>
#include <utility>
#include <functional>

#include "tf2_ros/create_timer_ros.h"

#include "navigo_util/node_utils.hpp"

#include "navigo_collision_monitor/kinematics.hpp"

namespace navigo_collision_monitor
{

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: navigo_util::LifecycleNode("collision_monitor", "", options),
  process_active_(false), robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}},
  stop_stamp_{0, 0, get_clock()->get_clock_type()}, stop_pub_timeout_(1.0, 0.0),
  stop_confirmation_cycles_(1), stop_detection_count_(0),
  allow_rotation_recovery_(false), allow_reverse_recovery_(false),
  require_healthy_localization_(true), localization_timeout_(0.50),
  loc_seen_(false), loc_status_(0),
  last_loc_stamp_{0, 0, get_clock()->get_clock_type()}
{
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
}

navigo_util::CallbackReturn
CollisionMonitor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic)) {
    return navigo_util::CallbackReturn::FAILURE;
  }

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic, 1,
    std::bind(&CollisionMonitor::cmdVelInCallback, this, std::placeholders::_1));
  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_out_topic, 1);
  state_pub_ = this->create_publisher<std_msgs::msg::String>("/collision_monitor/state", 10);

  if (require_healthy_localization_) {
    std::string localization_topic;
    navigo_util::declare_parameter_if_not_declared(
      shared_from_this(), "localization_topic",
      rclcpp::ParameterValue(std::string("/localization_info")));
    localization_topic = get_parameter("localization_topic").as_string();
    localization_sub_ = this->create_subscription<robots_dog_msgs::msg::Localization>(
      localization_topic, 10,
      std::bind(&CollisionMonitor::localizationCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "Collision monitor will zero cmd_vel unless localization status=3 (topic %s, timeout %.2fs)",
      localization_topic.c_str(), localization_timeout_);
  }

  return navigo_util::CallbackReturn::SUCCESS;
}

navigo_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();
  state_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return navigo_util::CallbackReturn::SUCCESS;
}

navigo_util::CallbackReturn
CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}};
  stop_detection_count_ = 0;

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();
  state_pub_->on_deactivate();

  // Destroying bond connection
  destroyBond();

  return navigo_util::CallbackReturn::SUCCESS;
}

navigo_util::CallbackReturn
CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cmd_vel_in_sub_.reset();
  cmd_vel_out_pub_.reset();
  state_pub_.reset();
  localization_sub_.reset();

  polygons_.clear();
  sources_.clear();

  tf_listener_.reset();
  tf_buffer_.reset();

  return navigo_util::CallbackReturn::SUCCESS;
}

navigo_util::CallbackReturn
CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return navigo_util::CallbackReturn::SUCCESS;
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  // If message contains NaN or Inf, ignore
  if (!navigo_util::validateTwist(*msg)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  process({msg->linear.x, msg->linear.y, msg->angular.z});
}

void CollisionMonitor::publishVelocity(const Action & robot_action)
{
  if (robot_action.req_vel.isZero()) {
    if (!robot_action_prev_.req_vel.isZero()) {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = this->now();
    } else if (this->now() - stop_stamp_ > stop_pub_timeout_) {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel.
      return;
    }
  }

  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg =
    std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
}

bool CollisionMonitor::getParameters(
  std::string & cmd_vel_in_topic,
  std::string & cmd_vel_out_topic)
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();

  navigo_util::declare_parameter_if_not_declared(
    node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_raw"));
  cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
  navigo_util::declare_parameter_if_not_declared(
    node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();

  navigo_util::declare_parameter_if_not_declared(
    node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_parameter("base_frame_id").as_string();
  navigo_util::declare_parameter_if_not_declared(
    node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_parameter("odom_frame_id").as_string();
  navigo_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance =
    tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
  navigo_util::declare_parameter_if_not_declared(
    node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout =
    rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());
  navigo_util::declare_parameter_if_not_declared(
    node, "base_shift_correction", rclcpp::ParameterValue(true));
  const bool base_shift_correction =
    get_parameter("base_shift_correction").as_bool();

  navigo_util::declare_parameter_if_not_declared(
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  navigo_util::declare_parameter_if_not_declared(
    node, "stop_confirmation_cycles", rclcpp::ParameterValue(1));
  stop_confirmation_cycles_ =
    std::max(1, static_cast<int>(get_parameter("stop_confirmation_cycles").as_int()));
  navigo_util::declare_parameter_if_not_declared(
    node, "allow_rotation_recovery", rclcpp::ParameterValue(false));
  allow_rotation_recovery_ = get_parameter("allow_rotation_recovery").as_bool();
  navigo_util::declare_parameter_if_not_declared(
    node, "allow_reverse_recovery", rclcpp::ParameterValue(false));
  allow_reverse_recovery_ = get_parameter("allow_reverse_recovery").as_bool();
  navigo_util::declare_parameter_if_not_declared(
    node, "require_healthy_localization", rclcpp::ParameterValue(true));
  require_healthy_localization_ = get_parameter("require_healthy_localization").as_bool();
  navigo_util::declare_parameter_if_not_declared(
    node, "localization_timeout", rclcpp::ParameterValue(0.50));
  localization_timeout_ = get_parameter("localization_timeout").as_double();

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    return false;
  }

  if (
    !configureSources(
      base_frame_id, odom_frame_id, transform_tolerance, source_timeout, base_shift_correction))
  {
    return false;
  }

  return true;
}

bool CollisionMonitor::configurePolygons(
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
{
  try {
    auto node = shared_from_this();

    navigo_util::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::ParameterValue(std::vector<std::string>()));
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      navigo_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        polygons_.push_back(
          std::make_shared<Polygon>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "circle") {
        polygons_.push_back(
          std::make_shared<Circle>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown polygon type: %s",
          polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(
  const std::string & base_frame_id,
  const std::string & odom_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout,
  const bool base_shift_correction)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    navigo_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      navigo_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        p->configure();

        sources_.push_back(p);
      } else if (source_type == "range") {
        std::shared_ptr<Range> r = std::make_shared<Range>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout, base_shift_correction);

        r->configure();

        sources_.push_back(r);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown source type: %s",
          source_name.c_str(), source_type.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::process(const Velocity & cmd_vel_in)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  if (!localizationAllowsMotion()) {
    uint8_t status = 0;
    bool seen = false;
    {
      std::lock_guard<std::mutex> lock(localization_mutex_);
      status = loc_status_;
      seen = loc_seen_;
    }
    Action stop_action{STOP, {0.0, 0.0, 0.0}};
    if (robot_action_prev_.action_type != STOP) {
      RCLCPP_WARN(
        get_logger(),
        "Zeroing cmd_vel: localization is not Normal (status=%u seen=%s)",
        status, seen ? "true" : "false");
    }
    publishVelocity(stop_action);
    publishPolygons();
    robot_action_prev_ = stop_action;
    return;
  }

  if (!sourcesAllowMotion(curr_time) && !cmd_vel_in.isZero()) {
    Action stop_action{STOP, {0.0, 0.0, 0.0}};
    if (robot_action_prev_.action_type != STOP) {
      RCLCPP_WARN(get_logger(), "Zeroing cmd_vel: an enabled collision source is stale");
    }
    publishVelocity(stop_action);
    publishPolygons();
    robot_action_prev_ = stop_action;
    return;
  }

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_) {
    if (source->getEnabled()) {
      source->getData(curr_time, collision_points);
    }
  }

  // By default - there is no action
  Action robot_action{DO_NOTHING, cmd_vel_in};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (!polygon->getEnabled()) {
      continue;
    }
    if (!polygonAppliesToVelocity(*polygon, cmd_vel_in)) {
      continue;
    }
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN) {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processStopSlowdown(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    }
  }

  if (robot_action.action_type == STOP) {
    const bool pure_rotation =
      std::abs(cmd_vel_in.x) < 0.02 && std::abs(cmd_vel_in.y) < 0.02 &&
      std::abs(cmd_vel_in.tw) > 0.01;
    const bool reversing = cmd_vel_in.x < -0.01;

    if ((allow_rotation_recovery_ && pure_rotation) ||
      (allow_reverse_recovery_ && reversing))
    {
      // This stop polygon is in front. Permit commands that increase clearance.
      robot_action = {DO_NOTHING, cmd_vel_in};
      action_polygon.reset();
      stop_detection_count_ = 0;
    } else {
      ++stop_detection_count_;
      if (stop_detection_count_ < stop_confirmation_cycles_) {
        robot_action = {DO_NOTHING, cmd_vel_in};
        action_polygon.reset();
      }
    }
  } else {
    stop_detection_count_ = 0;
  }

  if (robot_action.action_type != robot_action_prev_.action_type) {
    // Report changed robot behavior
    printAction(robot_action, action_polygon);
    publishState(robot_action);
  }

  // Publish required robot velocity
  publishVelocity(robot_action);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

bool CollisionMonitor::sourcesAllowMotion(const rclcpp::Time & curr_time) const
{
  for (const std::shared_ptr<Source> & source : sources_) {
    if (source->getEnabled() && !source->isFresh(curr_time)) {
      return false;
    }
  }
  return true;
}

bool CollisionMonitor::polygonAppliesToVelocity(const Polygon & polygon, const Velocity & velocity)
{
  const std::string scope = polygon.getMotionScope();
  if (scope == "any") {
    return true;
  }
  const bool rotation = std::abs(velocity.x) < 0.02 && std::abs(velocity.y) < 0.02 &&
    std::abs(velocity.tw) > 0.01;
  if (scope == "rotation") {
    return rotation;
  }
  if (scope == "forward") {
    return velocity.x > 0.01;
  }
  if (scope == "reverse") {
    return velocity.x < -0.01;
  }
  if (scope == "left") {
    return velocity.y > 0.01;
  }
  if (scope == "right") {
    return velocity.y < -0.01;
  }
  return std::abs(velocity.y) > 0.01;
}

void CollisionMonitor::publishState(const Action & robot_action)
{
  if (!state_pub_ || !state_pub_->is_activated()) {
    return;
  }
  const char * state = "CLEAR";
  if (robot_action.action_type == STOP) {
    state = "STOP";
  } else if (robot_action.action_type == SLOWDOWN || robot_action.action_type == APPROACH) {
    state = "SLOW";
  }
  std_msgs::msg::String message;
  message.data = std::string("{\"schema\":\"roamerx.collision-state.v1\",\"state\":\"") +
    state + "\",\"velocity_x\":" + std::to_string(robot_action.req_vel.x) +
    ",\"velocity_y\":" + std::to_string(robot_action.req_vel.y) +
    ",\"velocity_w\":" + std::to_string(robot_action.req_vel.tw) + "}";
  state_pub_->publish(message);
}

bool CollisionMonitor::processStopSlowdown(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  polygon->updatePolygon();

  // Obtain time before a collision
  const double collision_time = polygon->getCollisionTime(collision_points, velocity);
  if (collision_time >= 0.0) {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }

  return false;
}

void CollisionMonitor::printAction(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to stop due to %s polygon",
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to slowdown for %f percents due to %s polygon",
      action_polygon->getSlowdownRatio() * 100,
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_INFO(
      get_logger(),
      "Robot to approach for %f seconds away from collision",
      action_polygon->getTimeBeforeCollision());
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_INFO(
      get_logger(),
      "Robot to continue normal operation");
  }
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (polygon->getEnabled()) {
      polygon->publish();
    }
  }
}

void CollisionMonitor::localizationCallback(
  robots_dog_msgs::msg::Localization::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(localization_mutex_);
  loc_seen_ = true;
  loc_status_ = msg->status;
  last_loc_stamp_ = this->now();
}

bool CollisionMonitor::localizationAllowsMotion() const
{
  if (!require_healthy_localization_) {
    return true;
  }
  std::lock_guard<std::mutex> lock(localization_mutex_);
  if (!loc_seen_) {
    return false;
  }
  const double age = (this->now() - last_loc_stamp_).seconds();
  if (!std::isfinite(age) || age > localization_timeout_) {
    return false;
  }
  return loc_status_ == 3;
}

}  // namespace navigo_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(navigo_collision_monitor::CollisionMonitor)
