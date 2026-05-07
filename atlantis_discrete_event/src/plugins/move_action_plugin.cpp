// Copyright 2026 Atlantis

#include <atlantis_core/action_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <standard_msgs/action/move_to_pose.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace atlantis_simulator
{

class MoveActionPlugin : public atlantis_core::ActionPlugin
{
public:
  using Action = standard_msgs::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  void initialize(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<atlantis_core::SimulationWorld> world,
    const atlantis_core::PluginConfig & config) override
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

  void cleanup() override { server_.reset(); }

  std::string getName() const override { return "move"; }

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
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
    RCLCPP_INFO(node_->get_logger(),"Start Waypoint: %s", world_->getRobotLocation(robot_name_).c_str());
    RCLCPP_INFO(node_->get_logger(),  "Goal Waypoint: (%f, %f, %f)", gx, gy, theta);
    std::string target = world_->findWaypoint(gx, gy, theta);
    if (target == "-1") {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Goal location not found for pose (%f, %f, %f)", gx, gy, theta);
      goal_handle->abort(result);
      return;
    }

    RCLCPP_INFO(
      node_->get_logger(), "%s: %s -> %s",
      robot_name_.c_str(),
      world_->getRobotLocation(robot_name_).c_str(),
      target.c_str());

    world_->setRobotLocation(robot_name_, target);
    std::this_thread::sleep_for(std::chrono::seconds(execution_sec_));

    goal_handle->succeed(result);
  }

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  std::string robot_name_;
  int execution_sec_{1};
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace atlantis_simulator

PLUGINLIB_EXPORT_CLASS(
  atlantis_simulator::MoveActionPlugin,
  atlantis_core::ActionPlugin)
