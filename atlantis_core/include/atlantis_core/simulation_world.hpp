#ifndef ATLANTIS_CORE__SIMULATION_WORLD_HPP_
#define ATLANTIS_CORE__SIMULATION_WORLD_HPP_

#include <atlantis_core/types.hpp>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace atlantis_core
{

class SimulationWorld
{
public:
  SimulationWorld() = default;
  ~SimulationWorld() = default;

  // ---- Robots ----
  void addRobot(const RobotState & robot);
  bool hasRobot(const std::string & name) const;
  Waypoint getRobotLocation(const std::string & name) const;
  RobotState getRobotInfo(const std::string & name) const;
  void setRobotLocation(const std::string & name, const Waypoint & location);
  double getCapacity(const std::string & name) const;
  double getLoadedAmount(const std::string & name) const;
  void setLoadedAmount(const std::string & name, double amount);

  // ---- Materials ----
  void addMaterial(const Material & material);
  double getMaterialAmount(const std::string & material_name, const std::string & location) const;
  void setMaterialAmount(
    const std::string & material_name,
    const std::string & location,
    double amount);

  // ---- Waypoints ----
  void addWaypoint(const Waypoint & waypoint);
  std::vector<Waypoint> getWaypoints() const;
  Waypoint findWaypoint(double x, double y, double theta) const;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, RobotState> robots_;
  std::vector<Material> materials_;
  std::vector<Waypoint> waypoints_;
};

} 

#endif  