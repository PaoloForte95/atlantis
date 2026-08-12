// Copyright 2026 Atlantis

#ifndef ATLANTIS_DISCRETE_EVENT__PLUGINS__MATERIAL_AMOUNT_SERVICE_PLUGIN_HPP_
#define ATLANTIS_DISCRETE_EVENT__PLUGINS__MATERIAL_AMOUNT_SERVICE_PLUGIN_HPP_

#include <atlantis_core/service_plugin.hpp>

#include <material_handler_msgs/srv/get_material_amount.hpp>

#include <memory>
#include <string>

namespace atlantis_simulator
{

class MaterialAmountServicePlugin : public atlantis_core::ServicePlugin
{
public:
  using Service = material_handler_msgs::srv::GetMaterialAmount;

  void initialize(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<atlantis_core::SimulationWorld> world,
    const atlantis_core::PluginConfig & config) override;

  void cleanup() override;

  std::string getName() const override;

private:
  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  rclcpp::Service<Service>::SharedPtr server_;
};

}  // namespace atlantis_simulator

#endif  // ATLANTIS_DISCRETE_EVENT__PLUGINS__MATERIAL_AMOUNT_SERVICE_PLUGIN_HPP_
