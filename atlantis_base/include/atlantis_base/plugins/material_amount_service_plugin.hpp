// Copyright 2026 Atlantis

#ifndef ATLANTIS_BASE__PLUGINS__MATERIAL_AMOUNT_SERVICE_PLUGIN_HPP_
#define ATLANTIS_BASE__PLUGINS__MATERIAL_AMOUNT_SERVICE_PLUGIN_HPP_

#include <atlantis_core/service_plugin.hpp>

#include <material_handler_msgs/srv/get_material_amount.hpp>

#include <memory>
#include <string>

namespace atlantis_base
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

  std::string getName() const override { return "material_amount"; }

private:
  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  rclcpp::Service<Service>::SharedPtr server_;
};

}  // namespace atlantis_base

#endif  // ATLANTIS_BASE__PLUGINS__MATERIAL_AMOUNT_SERVICE_PLUGIN_HPP_
