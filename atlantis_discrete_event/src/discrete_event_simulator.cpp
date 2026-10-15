// Copyright 2026 Atlantis

#include <atlantis_discrete_event/discrete_event_simulator.hpp>

#include <fstream>
#include <iomanip>
#include <locale>
#include <string>
#include <vector>

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

void DiscreteEventSimulator::loadParameters()
{
  if (!has_parameter("path_costs")) {
    declare_parameter("path_costs", std::string("path_costs.csv"));
  }
  if (!has_parameter("compute_path_costs")) {
    declare_parameter("compute_path_costs", false);
  }
  get_parameter("path_costs", path_costs_);
  get_parameter("compute_path_costs", compute_path_costs_);
}

void DiscreteEventSimulator::writePathCosts()
{
  const auto waypoints = world()->getWaypoints();

  std::ofstream costs(path_costs_);
  if (!costs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Cannot create %s", path_costs_.c_str());
    return;
  }

  costs.imbue(std::locale::classic());
  costs << std::fixed << std::setprecision(3);
  costs << "# unit: m\n";
  costs << "location";
  for (const auto & wp : waypoints) {
    costs << "," << wp.name;
  }
  costs << "\n";

  for (const auto & from : waypoints) {
    costs << from.name;
    for (const auto & to : waypoints) {
      costs << "," << (from.name == to.name ? 0.0 : 1.0);
    }
    costs << "\n";
  }

  RCLCPP_INFO(get_logger(), "Path costs written to %s", path_costs_.c_str());
}

void DiscreteEventSimulator::onConfigureExtra()
{
  loadParameters();
  if (compute_path_costs_) {
    writePathCosts();
  }
}

}  // namespace atlantis_simulator