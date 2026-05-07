// Copyright 2026 Atlantis

#include <atlantis_core/service_plugin.hpp>

#include <material_handler_msgs/srv/get_material_amount.hpp>
#include <pluginlib/class_list_macros.hpp>

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
    const atlantis_core::PluginConfig & config) override
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

  void cleanup() override { server_.reset(); }

  std::string getName() const override { return "material_amount"; }

private:
  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  rclcpp::Service<Service>::SharedPtr server_;
};

}  // namespace atlantis_simulator

PLUGINLIB_EXPORT_CLASS(
  atlantis_simulator::MaterialAmountServicePlugin,
  atlantis_core::ServicePlugin)
