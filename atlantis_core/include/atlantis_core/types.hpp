// Copyright 2026 Atlantis
//
// Shared data types used across simulator layers.

#ifndef ATLANTIS_CORE__TYPES_HPP_
#define ATLANTIS_CORE__TYPES_HPP_

#include <map>
#include <string>

namespace atlantis_core
{

/// A named pose in the world.
struct Waypoint
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

/// A material pile that can exist at one or more locations,
/// each with its own amount.
struct Material
{
  std::string name;
  int id{0};
  std::map<std::string, double> amounts;  // location name -> amount

  double getAmount(const std::string & location) const
  {
    auto it = amounts.find(location);
    if (it == amounts.end()) {
      return -1.0;
    }
    return it->second;
  }

  void setAmount(const std::string & location, double amount)
  {
    amounts[location] = amount;
  }
};

// The state of a single robot.
struct RobotState
{
  std::string name;
  std::string type;
  Waypoint current_location;

  //Planner
  double minimum_turning_radius;
  std::string model;
  std::string footprint;
  std::string planner;

  //Capacity
  double loaded_amount;
  double capacity;
};

}  // namespace atlantis_core

#endif  // ATLANTIS_CORE__TYPES_HPP_
