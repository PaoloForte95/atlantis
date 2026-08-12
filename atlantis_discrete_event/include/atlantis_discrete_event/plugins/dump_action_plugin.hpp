// Copyright 2026 Atlantis

#ifndef ATLANTIS_DISCRETE_EVENT__PLUGINS__DUMP_ACTION_PLUGIN_HPP_
#define ATLANTIS_DISCRETE_EVENT__PLUGINS__DUMP_ACTION_PLUGIN_HPP_

#include <atlantis_core/action_plugin.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <standard_msgs/action/dump.hpp>

#include <memory>
#include <string>

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
    const atlantis_core::PluginConfig & config) override;

  void cleanup() override;

  std::string getName() const override;

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  std::string robot_name_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace atlantis_simulator

#endif  // ATLANTIS_DISCRETE_EVENT__PLUGINS__DUMP_ACTION_PLUGIN_HPP_
