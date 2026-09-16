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
  robots_[robot.info.name] = robot;
}

bool SimulationWorld::hasRobot(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robots_.find(name) != robots_.end();
}

Waypoint SimulationWorld::getRobotLocation(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    throw std::out_of_range("Robot not found: " + name);
  }
  return it->second.current_location;
}

void SimulationWorld::setRobotLocation(const std::string & name, const Waypoint & location)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it != robots_.end()) {
    it->second.current_location = location;
  }
}

RobotState SimulationWorld::getRobotInfo(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    throw std::out_of_range("Robot not found: " + name);
  }
  return it->second;
}

double SimulationWorld::getCapacity(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    throw std::out_of_range("Robot not found: " + name);
  }
  return it->second.info.capacity;
}

double SimulationWorld::getLoadedAmount(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    throw std::out_of_range("Robot not found: " + name);
  }
  return it->second.loaded_amount;
}

std::vector<Material> SimulationWorld::getMaterials() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return materials_;
}

void SimulationWorld::setLoadedAmount(const std::string & name, double amount)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = robots_.find(name);
  if (it == robots_.end()) {
    throw std::out_of_range("Robot not found: " + name);
  }
  it->second.loaded_amount = amount;
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
      auto it = mat.amounts.find(location);
      if (it == mat.amounts.end()) {
        return 0.0;
      }
      return it->second;
    }
  }
  throw std::out_of_range("Material not found: " + material_name);
}

void SimulationWorld::setMaterialAmount(
  const std::string & material_name,
  const std::string & location,
  double amount)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = std::find_if(
    materials_.begin(), materials_.end(),
    [&material_name](const Material & mat) {
      return mat.name == material_name;
    });

  if (it == materials_.end()) {
    throw std::out_of_range("Material not found: " + material_name);
  }
  it->setAmount(location, amount);
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

Waypoint SimulationWorld::findWaypoint(
  double x, double y, double theta) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Waypoint best_match;

  double best_distance = std::numeric_limits<double>::max();
  for (const auto & wp : waypoints_) {
    double dx = x - wp.x;
    double dy = y - wp.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    double dtheta = std::fabs(theta - wp.theta);
    if ((distance + dtheta) <= best_distance) {
      best_distance = distance;
      best_match.name = wp.name;
      best_match.x = wp.x;
      best_match.y = wp.y;
      best_match.theta = wp.theta;
    }
  }
  return best_match;
}

}  // namespace atlantis_core