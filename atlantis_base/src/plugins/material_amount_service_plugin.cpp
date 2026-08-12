// Copyright 2026 Atlantis

#include <atlantis_base/plugins/material_amount_service_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace atlantis_base
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
    });
}

void MaterialAmountServicePlugin::cleanup()
{
  server_.reset();
}

}  // namespace atlantis_base

PLUGINLIB_EXPORT_CLASS(
  atlantis_base::MaterialAmountServicePlugin,
  atlantis_core::ServicePlugin)
