// Copyright 2026 Atlantis

#ifndef ATLANTIS_DISCRETE_EVENT__DISCRETE_EVENT_SIMULATOR_HPP_
#define ATLANTIS_DISCRETE_EVENT__DISCRETE_EVENT_SIMULATOR_HPP_

#include <atlantis_core/simulator_core.hpp>

#include <string>
#include <vector>

namespace atlantis_simulator
{

class DiscreteEventSimulator : public atlantis_core::SimulatorCore
{
public:
  DiscreteEventSimulator(
    const std::string & node_name,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  std::vector<std::string> defaultActions() const override;
  std::vector<std::string> defaultServices() const override;
};

}  // namespace atlantis_simulator

#endif  // ATLANTIS_DISCRETE_EVENT__DISCRETE_EVENT_SIMULATOR_HPP_