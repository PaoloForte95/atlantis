// Copyright 2026 Atlantis

#include <atlantis_discrete_event/plugins/material_amount_service_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace atlantis_simulator
{

void MaterialAmountServicePlugin::initialize(
  rclcpp_lifecycle::LifecycleNode * node,
  std::shared_ptr<atlantis_core::SimulationWorld> world,
  const atlantis_core::PluginConfig & config)
{
  node_ = node;
  world_ = world;
  config_ = config;

  server_ = node_->create_service<Service>(
    config.topic,
    [this](
      const std::shared_ptr<Service::Request> request,
      std::shared_ptr<Service::Response> response) {
      response->amount = world_->getMaterialAmount(
        request->pile_id, request->pile_location);
      RCLCPP_INFO(
        node_->get_logger(),
        "Material amount for %s at %s: %f",
        request->pile_id.c_str(),
        request->pile_location.c_str(),
        response->amount);
    });
}

void MaterialAmountServicePlugin::cleanup()
{
  server_.reset();
}

std::string MaterialAmountServicePlugin::getName() const
{
  return "material_amount";
}

}  // namespace atlantis_simulator

PLUGINLIB_EXPORT_CLASS(
  atlantis_simulator::MaterialAmountServicePlugin,
  atlantis_core::ServicePlugin)
