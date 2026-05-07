// Copyright 2026 Atlantis

#include <atlantis_discrete_event/discrete_event_simulator.hpp>

namespace atlantis_simulator
{

DiscreteEventSimulator::DiscreteEventSimulator(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: atlantis_core::SimulatorCore(node_name, ns, options)
{
}

// The defaults are empty because the YAML file describes what instances
// to create. If the user provides no YAML, the simulator simply runs with
// no actions or services, which is a safe and obvious failure mode.
std::vector<std::string> DiscreteEventSimulator::defaultActions() const
{
  return {};
}

std::vector<std::string> DiscreteEventSimulator::defaultServices() const
{
  return {};
}

}  // namespace atlantis_simulator