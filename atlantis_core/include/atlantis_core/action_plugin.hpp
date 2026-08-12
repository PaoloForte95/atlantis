#ifndef ATLANTIS_CORE__ACTION_PLUGIN_HPP_
#define ATLANTIS_CORE__ACTION_PLUGIN_HPP_

#include <atlantis_core/plugin_config.hpp>
#include <atlantis_core/simulation_world.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <memory>
#include <string>

namespace atlantis_core
{

class ActionPlugin
{
public:
  virtual ~ActionPlugin() = default;

  virtual void initialize(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<SimulationWorld> world,
    const PluginConfig & config) = 0;

  virtual void activate() {}
  virtual void deactivate() {}
  virtual void cleanup() {}

  virtual std::string getName() const = 0;
};

}  // namespace atlantis_core

#endif  // ATLANTIS_CORE__ACTION_PLUGIN_HPP_