// Copyright 2026 Atlantis

#ifndef ATLANTIS_CORE__SIMULATOR_CORE_HPP_
#define ATLANTIS_CORE__SIMULATOR_CORE_HPP_

#include <atlantis_core/action_plugin.hpp>
#include <atlantis_core/service_plugin.hpp>
#include <atlantis_core/simulation_world.hpp>

#include <location_msgs/msg/waypoint_array.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace atlantis_core
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SimulatorCore : public rclcpp_lifecycle::LifecycleNode
{
public:
  SimulatorCore(
    const std::string & node_name,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~SimulatorCore() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  // Hooks for layer specific behavior.
  virtual void onConfigureExtra() {}
  virtual void onActivateExtra() {}
  virtual void onDeactivateExtra() {}
  virtual void onCleanupExtra() {}

  // Layer specific defaults, used only when YAML does not provide them.
  virtual std::vector<std::string> defaultActions() const { return {}; }
  virtual std::vector<std::string> defaultServices() const { return {}; }

  // Helpers a subclass may use.
  std::shared_ptr<SimulationWorld> world() const { return world_; }
  const std::vector<std::string> & robotIds() const { return robots_ids_; }

private:
  void loadCommonParameters();
  void buildWorld();
  void loadActions();
  void loadServices();
  void setupMetrics();

  // ---- Common parameters ----
  std::vector<std::string> robots_ids_;
  std::vector<std::string> materials_ids_;
  std::vector<std::string> waypoints_ids_;
  std::vector<std::string> action_names_;
  std::vector<std::string> service_names_;
  std::vector<std::string> metrics_;

  // ---- Shared state ----
  std::shared_ptr<SimulationWorld> world_;

  // ---- Plugin ----
  std::unique_ptr<pluginlib::ClassLoader<ActionPlugin>> action_loader_;
  std::unique_ptr<pluginlib::ClassLoader<ServicePlugin>> service_loader_;
  std::vector<std::shared_ptr<ActionPlugin>> action_plugins_;
  std::vector<std::shared_ptr<ServicePlugin>> service_plugins_;

  // ---- Metrics ----
  std::map<std::string, bool> metrics_to_evaluate_;
  std::map<std::string,
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr> metrics_pubs_;

  // ---- Waypoint publisher ----
  std::string waypoint_topic_;
  std::string waypoint_frame_id_;
  rclcpp_lifecycle::LifecyclePublisher<location_msgs::msg::WaypointArray>::SharedPtr
    waypoint_pub_;
};

}  // namespace atlantis_core

#endif  // ATLANTIS_CORE__SIMULATOR_CORE_HPP_