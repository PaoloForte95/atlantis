// Copyright 2026 Atlantis

#ifndef ATLANTIS_DISCRETE_EVENT__PLUGINS__LOAD_ACTION_PLUGIN_HPP_
#define ATLANTIS_DISCRETE_EVENT__PLUGINS__LOAD_ACTION_PLUGIN_HPP_

#include <atlantis_core/action_plugin.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <standard_msgs/action/load.hpp>

#include <memory>
#include <string>

namespace atlantis_simulator
{

class LoadActionPlugin : public atlantis_core::ActionPlugin
{
public:
  using Action = standard_msgs::action::Load;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  void initialize(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<atlantis_core::SimulationWorld> world,
    const atlantis_core::PluginConfig & config) override;

  void cleanup() override;

  std::string getName() const override;

private:
  double generateRandomValue(double mean, double stddev);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  std::string robot_name_;
  bool refilling_{false};
  bool randomness_{false};
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace atlantis_simulator

#endif  // ATLANTIS_DISCRETE_EVENT__PLUGINS__LOAD_ACTION_PLUGIN_HPP_
