// Copyright 2026 Atlantis

#ifndef ATLANTIS_BASE__PLUGINS__WAYPOINT_LIST_SERVICE_PLUGIN_HPP_
#define ATLANTIS_BASE__PLUGINS__WAYPOINT_LIST_SERVICE_PLUGIN_HPP_

#include <atlantis_core/service_plugin.hpp>

#include <location_msgs/srv/get_waypoint_list.hpp>

#include <memory>
#include <string>

namespace atlantis_base
{

class WaypointListServicePlugin : public atlantis_core::ServicePlugin
{
public:
  using Service = location_msgs::srv::GetWaypointList;

  void initialize(
    rclcpp_lifecycle::LifecycleNode * node,
    std::shared_ptr<atlantis_core::SimulationWorld> world,
    const atlantis_core::PluginConfig & config) override;

  void cleanup() override;

  std::string getName() const override { return "waypoint_list"; }

private:
  rclcpp_lifecycle::LifecycleNode * node_{nullptr};
  std::shared_ptr<atlantis_core::SimulationWorld> world_;
  atlantis_core::PluginConfig config_;
  rclcpp::Service<Service>::SharedPtr server_;
};

}  // namespace atlantis_base

#endif  // ATLANTIS_BASE__PLUGINS__WAYPOINT_LIST_SERVICE_PLUGIN_HPP_
