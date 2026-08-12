// Copyright 2026 Atlantis

#ifndef ATLANTIS_BASE__PLUGINS__LOAD_ACTION_PLUGIN_HPP_
#define ATLANTIS_BASE__PLUGINS__LOAD_ACTION_PLUGIN_HPP_

#include <atlantis_base/base_simulator.hpp>
#include <atlantis_core/action_plugin.hpp>

#include <material_handler_msgs/action/load_material.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <string>

namespace atlantis_base
{

class LoadActionPlugin : public atlantis_core::ActionPlugin
{
public:
  using Action = material_handler_msgs::action::LoadMaterial;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  void initialize(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<atlantis_core::SimulationWorld> world,
    const atlantis_core::PluginConfig & config) override;

  void cleanup() override;

  std::string getName() const override { return "base_load"; }

private:
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
  double generateRandomValue(double mean, double stddev);

  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  BaseSimulator * base_sim_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  std::string robot_name_;
  bool refilling_{false};
  bool randomness_{false};
  double uncertainty_{0.0};
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace atlantis_base

#endif  // ATLANTIS_BASE__PLUGINS__LOAD_ACTION_PLUGIN_HPP_
