// Copyright 2026 Atlantis

#include <atlantis_base/plugins/waypoint_list_service_plugin.hpp>

#include <location_msgs/msg/waypoint.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>

namespace atlantis_base
{

void WaypointListServicePlugin::initialize(
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
      const std::shared_ptr<Service::Request> /*request*/,
      std::shared_ptr<Service::Response> response) {
      for (const auto & wp : world_->getWaypoints()) {
        location_msgs::msg::Waypoint wp_msg;
        wp_msg.name = wp.name;
        wp_msg.pose.position.x = wp.x;
        wp_msg.pose.position.y = wp.y;
        wp_msg.pose.orientation.z = std::sin(wp.theta / 2.0);
        wp_msg.pose.orientation.w = std::cos(wp.theta / 2.0);
        response->list.push_back(wp_msg);
      }
    });
}

void WaypointListServicePlugin::cleanup()
{
  server_.reset();
}

}  // namespace atlantis_base

PLUGINLIB_EXPORT_CLASS(
  atlantis_base::WaypointListServicePlugin,
  atlantis_core::ServicePlugin)
