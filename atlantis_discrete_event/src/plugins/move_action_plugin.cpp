// Copyright 2026 Atlantis

#include <atlantis_discrete_event/plugins/move_action_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <cmath>
#include <thread>

namespace atlantis_simulator
{

void MoveActionPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode * node,
  std::shared_ptr<atlantis_core::SimulationWorld> world,
  const atlantis_core::PluginConfig & config)
{
  node_ = node;
  world_ = world;
  config_ = config;
  robot_name_ = config.name.substr(0, config.name.find('.'));

  node_->declare_parameter(config.name + ".execution_sec", 1);
  node_->get_parameter(config.name + ".execution_sec", execution_sec_);

  server_ = rclcpp_action::create_server<Action>(
    node_,
    config.topic,
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<GoalHandle> gh) {
      std::thread{[this, gh]() { this->execute(gh); }}.detach();
    });
}

void MoveActionPlugin::cleanup()
{
  server_.reset();
}

std::string MoveActionPlugin::getName() const
{
  return "move";
}

void MoveActionPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Action::Result>();

  double gx = goal->target_pose.pose.position.x;
  double gy = goal->target_pose.pose.position.y;

  double qw = goal->target_pose.pose.orientation.w;
  double qx = goal->target_pose.pose.orientation.x;
  double qy = goal->target_pose.pose.orientation.y;
  double qz = goal->target_pose.pose.orientation.z;
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  double theta = std::atan2(siny_cosp, cosy_cosp);
  auto start_location = world_->getRobotLocation(robot_name_);
  RCLCPP_INFO(node_->get_logger(), "Start Waypoint: (%f, %f, %f)", start_location.x, start_location.y, start_location.theta);
  RCLCPP_INFO(node_->get_logger(), "Goal Waypoint: (%f, %f, %f)", gx, gy, theta);
  atlantis_core::Waypoint target = world_->findWaypoint(gx, gy, theta);
  world_->setRobotLocation(robot_name_, target);
  std::this_thread::sleep_for(std::chrono::seconds(execution_sec_));

  goal_handle->succeed(result);
}

}  // namespace atlantis_simulator

PLUGINLIB_EXPORT_CLASS(
  atlantis_simulator::MoveActionPlugin,
  atlantis_core::ActionPlugin)
