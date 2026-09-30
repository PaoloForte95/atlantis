// Copyright 2026 Atlantis

#include <atlantis_discrete_event/discrete_event_simulator.hpp>
#include <fstream>

namespace atlantis_simulator
{

DiscreteEventSimulator::DiscreteEventSimulator(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: atlantis_core::SimulatorCore(node_name, ns, options)
{
}

std::vector<std::string> DiscreteEventSimulator::defaultActions() const
{
  return {};
}

std::vector<std::string> DiscreteEventSimulator::defaultServices() const
{
  return {};
}

void DiscreteEventSimulator::onConfigureExtra()
{
  const auto waypoints = world()->getWaypoints();
  std::ofstream costs("path_costs.csv");
  if (!costs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Cannot create path_costs.csv");
    return;
  }
  costs << "location";
  for (const auto & wp : waypoints) {
    costs << "," << wp.name;
  }
  costs << "\n";
  for (const auto & wp : waypoints) {
    costs << wp.name;
    for (const auto & wp2 : waypoints) {
      costs << "," << (wp.name == wp2.name ? "0" : "1");
    }
    costs << "\n";
  }
  RCLCPP_INFO(get_logger(), "Path costs written to path_costs.csv");
}

}  // namespace atlantis_simulator