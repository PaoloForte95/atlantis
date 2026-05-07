// Copyright 2026 Atlantis

#include <atlantis_core/action_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <standard_msgs/action/dump.hpp>

#include <memory>
#include <string>
#include <thread>

namespace atlantis_simulator
{

class DumpActionPlugin : public atlantis_core::ActionPlugin
{
public:
  using Action = standard_msgs::action::Dump;
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

  std::string getName() const override { return "dump"; }

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<Action::Feedback>();
    auto result = std::make_shared<Action::Result>();
    auto goal = goal_handle->get_goal();

    const auto & material_name = goal->target;
    const auto & location = goal->location;
    double delta = world_->getLoadedAmount(robot_name_);

    if (delta <= 0.0) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Robot %s cannot dump without having loaded any material",
        robot_name_.c_str());
      result->success = false;
      result->payload = 0.0;
      goal_handle->abort(result);
      return;
    }

    double current = world_->getMaterialAmount(material_name, location);
    if (current < 0.0) {
      current = 0.0;
    }

    const int num_steps = 5;
    for (int step = 0; step < num_steps; ++step) {
      current += delta / num_steps;
      world_->setMaterialAmount(material_name, location, current);
      feedback->phase = "dumping";
      feedback->progress = static_cast<float>(step + 1) / num_steps;
      goal_handle->publish_feedback(feedback);
    }

    world_->setLoadedAmount(robot_name_, 0.0);
    RCLCPP_INFO(
      node_->get_logger(), "Dumped %f of %s at %s",
      delta, material_name.c_str(), location.c_str());

    result->payload = delta;
    result->success = true;
    goal_handle->succeed(result);
  }

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  std::string robot_name_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace atlantis_simulator

PLUGINLIB_EXPORT_CLASS(
  atlantis_simulator::DumpActionPlugin,
  atlantis_core::ActionPlugin)
