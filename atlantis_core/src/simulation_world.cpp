// Copyright 2026 Atlantis

#include <atlantis_core/simulation_world.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace atlantis_core
{

void SimulationWorld::addRobot(const RobotState & robot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  robots_[robot.name] = robot;
}

bool SimulationWorld::hasRobot(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robots_.find(name) != robots_.end();
}

std::string SimulationWorld::getRobotLocation(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    return "";
  }
  return it->second.current_location;
}

void SimulationWorld::setRobotLocation(const std::string & name, const std::string & location)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it != robots_.end()) {
    it->second.current_location = location;
  }
}

double SimulationWorld::getCapacity(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    return 0.0;
  }
  return it->second.capacity;
}

double SimulationWorld::getLoadedAmount(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    return -1.0;
  }
  return it->second.loaded_amount;
}

void SimulationWorld::setLoadedAmount(const std::string & name, double amount)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it != robots_.end()) {
    it->second.loaded_amount = amount;
  }
}

void SimulationWorld::addMaterial(const Material & material)
{
  std::lock_guard<std::mutex> lock(mutex_);
  materials_.push_back(material);
}

double SimulationWorld::getMaterialAmount(
  const std::string & material_name,
  const std::string & location) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & mat : materials_) {
    if (mat.name == material_name) {
      return mat.getAmount(location);
    }
  }
  return -1.0;
}

void SimulationWorld::setMaterialAmount(
  const std::string & material_name,
  const std::string & location,
  double amount)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & mat : materials_) {
    if (mat.name == material_name) {
      mat.setAmount(location, amount);
      return;
    }
  }
}

void SimulationWorld::addWaypoint(const Waypoint & waypoint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  waypoints_.push_back(waypoint);
}

std::vector<Waypoint> SimulationWorld::getWaypoints() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return waypoints_;
}

std::string SimulationWorld::findWaypoint(
  double x, double y, double theta, double tolerance) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & wp : waypoints_) {
    double dx = x - wp.x;
    double dy = y - wp.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance <= tolerance && std::abs(theta - wp.theta) <= tolerance) {
      return wp.name;
    }
  }
  return "-1";
}

}  // namespace atlantis_core