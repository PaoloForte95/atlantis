// Copyright 2026 Atlantis

#include <atlantis_base/base_simulator.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <locale>


double pathLength(const navigo::Path & path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

namespace atlantis_base
{


navigo::Pose BaseSimulator::readFirstPose(const std::string & file_path) const
{
  std::ifstream file(file_path);
  std::string line;
  if (file.is_open() && std::getline(file, line)) {
    std::stringstream ss(line);
    navigo::Pose pose;
    ss >> pose.x >> pose.y >> pose.theta;
    return pose;
  }
  throw std::runtime_error("Could not read the first line of " + file_path);
}

navigo::Pose BaseSimulator::readLastPose(const std::string & file_path) const
{
  std::ifstream file(file_path);
  std::string line, last_line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      last_line = line;
    }
  }
  if (!last_line.empty()) {
    std::stringstream ss(last_line);
    navigo::Pose pose;
    ss >> pose.x >> pose.y >> pose.theta;
    return pose;
  }
  throw std::runtime_error("Could not read the last line of " + file_path);
}

bool BaseSimulator::posesMatch(const navigo::Pose & a, const navigo::Pose & b) const
{
  const double tolerance = 0.1;
  return std::fabs(a.x - b.x) < tolerance &&
         std::fabs(a.y - b.y) < tolerance &&
         std::fabs(a.theta - b.theta) < tolerance;
}

std::string BaseSimulator::findPathFile(
  const std::string & folder,
  const navigo::Pose & start,
  const navigo::Pose & goal) const
{
  if (!std::filesystem::exists(folder)) {
    RCLCPP_ERROR(get_logger(), "Precomputed paths folder does not exist: %s", folder.c_str());
    return "";
  }

  for (const auto & entry : std::filesystem::directory_iterator(folder)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".txt") {
      continue;
    }
    try {
      auto file_start = readFirstPose(entry.path().string());
      auto file_goal = readLastPose(entry.path().string());
      if (posesMatch(file_start, start) && posesMatch(file_goal, goal)) {
        RCLCPP_INFO(get_logger(), "Found precomputed path %s", entry.path().c_str());
        return entry.path().string();
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(get_logger(), "Skipping %s: %s", entry.path().c_str(), e.what());
    }
  }

  RCLCPP_ERROR(
    get_logger(), "No precomputed path from (%f, %f, %f) to (%f, %f, %f)",
    start.x, start.y, start.theta, goal.x, goal.y, goal.theta);
  return "";
}

navigo::Path BaseSimulator::loadPath(const std::string & file_path) const
{
  navigo::Path path;
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Could not open %s", file_path.c_str());
    return path;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    navigo::Pose pose;
    ss >> pose.x >> pose.y >> pose.theta;
    path.push_back(pose);
  }
  return path;
}

navigo::Path BaseSimulator::loadPrecomputedPath(
  const navigo::Pose & start,
  const navigo::Pose & goal)
{
  auto folder = atlantis::util::resolve_pkg_uri(precomputed_paths_folder_);
  auto file = findPathFile(folder, start, goal);
  if (file.empty()) {
    return navigo::Path();
  }
  return loadPath(file);
}
  

BaseSimulator::BaseSimulator(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: atlantis_core::SimulatorCore(node_name, ns, options)
{
}

BaseSimulator::~BaseSimulator() = default;

navigo::CarPlanner * BaseSimulator::getPlanner(const std::string & robot_name)
{
  auto it = base_planners_.find(robot_name);
  if (it == base_planners_.end()) {
    return nullptr;
  }
  return it->second;
}

const std::vector<geometry_msgs::msg::Point> &
BaseSimulator::getFootprint(const std::string & robot_name) const
{
  static const std::vector<geometry_msgs::msg::Point> empty;
  auto it = robot_footprints_.find(robot_name);
  if (it == robot_footprints_.end()) {
    return empty;
  }
  return it->second;
}

double BaseSimulator::getSimTime() const
{
  std::lock_guard<std::mutex> lock(sim_time_mutex_);
  return sim_time_;
}

void BaseSimulator::advanceSimTime(double dt)
{
  std::lock_guard<std::mutex> lock(sim_time_mutex_);
  sim_time_ += dt;
}

rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
BaseSimulator::getRobotPosePublisher(const std::string & robot_name)
{
  auto it = robot_pose_pubs_.find(robot_name);
  if (it == robot_pose_pubs_.end()) {
    return nullptr;
  }
  return it->second;
}

rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
BaseSimulator::getRobotPathPublisher(const std::string & robot_name)
{
  auto it = robot_path_pubs_.find(robot_name);
  if (it == robot_path_pubs_.end()) {
    return nullptr;
  }
  return it->second;
}

void BaseSimulator::loadFootprints()
{
  for (const auto & robot : robotIds()) {
    std::string key = robot + ".footprint";
    if (!has_parameter(key)) {
      declare_parameter(key, std::string(""));
    }

    std::string footprint_str;
    get_parameter(key, footprint_str);
    if (footprint_str.empty()) {
      RCLCPP_WARN(get_logger(), "No footprint for %s", robot.c_str());
      continue;
    }

    std::string error;
    std::vector<std::vector<float>> parsed = atlantis::util::parseVVF(footprint_str, error);

    if (!error.empty()) {
      RCLCPP_ERROR(get_logger(), "Footprint for %s invalid: %s",
                   robot.c_str(), error.c_str());
      continue;
    }

    std::vector<geometry_msgs::msg::Point> points;
    for (const auto & corner : parsed) {
      if (corner.size() != 2) {
        RCLCPP_ERROR(get_logger(), "Footprint corner for %s is not a pair",
                     robot.c_str());
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = corner[0];
      p.y = corner[1];
      p.z = 0.0;
      points.push_back(p);
    }

    robot_footprints_[robot] = points;
    RCLCPP_INFO(get_logger(), "Loaded %zu footprint points for %s",
                points.size(), robot.c_str());
  }
}


void BaseSimulator::loadBaseParameters()
{
  declare_parameter("map", std::string(""));
  declare_parameter("lattice_primitives", std::string(""));
  declare_parameter("primitives_dir", std::string(""));
  declare_parameter("use_precomputed_paths", false);
  declare_parameter("precomputed_paths_folder", std::string(""));
  declare_parameter("use_trajectory_dt", false);
  declare_parameter("goal_tolerance", 0.5);
  declare_parameter("max_planning_time", 5.0);
  declare_parameter("dt", 0.1);
  declare_parameter("real_time_factor", 1.0);
  declare_parameter("max_sim_time", 0.0);
  declare_parameter("path_costs", "path_costs.csv");
  declare_parameter("compute_path_costs", false);

  get_parameter("map", map_yaml_);
  get_parameter("lattice_primitives", lattice_primitives_);
  get_parameter("primitives_dir", primitives_dir_);
  get_parameter("use_precomputed_paths", use_precomputed_paths_);
  get_parameter("precomputed_paths_folder", precomputed_paths_folder_);
  get_parameter("use_trajectory_dt", use_trajectory_dt_);
  get_parameter("goal_tolerance", goal_tolerance_);
  get_parameter("max_planning_time", max_planning_time_);
  get_parameter("dt", dt_);
  get_parameter("real_time_factor", real_time_factor_);
  get_parameter("max_sim_time", max_sim_time_);
  get_parameter("path_costs", path_costs_);
  get_parameter("compute_path_costs", compute_path_costs_);
  rviz_viz_.startVisualization();
}

void BaseSimulator::buildCostmap()
{

  RCLCPP_INFO(get_logger(), "Building costmap");
  auto file = atlantis::util::resolve_pkg_uri(map_yaml_);
  RCLCPP_INFO(get_logger(), "Resolved map file: %s", file.c_str());
  oc_ = new navigo::CostMap(file);
  RCLCPP_INFO(get_logger(), "Map info... %s", oc_->getDebugString().c_str());
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

}

void BaseSimulator::buildPlanners()
{
  for (const auto & robot : robotIds()) {
    auto rb = world()->getRobotInfo(robot);
    RCLCPP_INFO(get_logger(), "Setting up planner for %s", robot.c_str());
    navigo::SearchParams search_info;
    navigo::PlannerParams planning_params;
    planning_params.tolerance = goal_tolerance_;
    planning_params.max_planning_time = max_planning_time_;
    search_info.minimum_turning_radius = rb.info.minimum_turning_radius;
    auto car_planner = new navigo::CarPlanner(
      "CarPlanner", navigo::MotionModel::REEDS_SHEPP, search_info, planning_params,
      navigo::PlanningAlgorithm::RRTstar);
    const auto & footprint = getFootprint(robot);
    std::vector<double> xcoords, ycoords;
    for (const auto & p : footprint) {
      xcoords.push_back(p.x);
      ycoords.push_back(p.y);
    }

    auto checker = std::make_unique<navigo::GridCollisionChecker>(getCostmap());
    checker->setFootprint(navigo::Footprint(xcoords, ycoords));
    car_planner->setCollisionChecker(checker.get());
    collision_checkers_[robot] = std::move(checker);
    base_planners_[robot] = car_planner;
  }
}

void BaseSimulator::buildPerRobotPublishers()
{
  for (const auto & robot : robotIds()) {
    auto pose_topic = robot + "/current_pose";
    auto path_topic = robot + "/path";

    robot_pose_pubs_[robot] = create_publisher<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    robot_path_pubs_[robot] = create_publisher<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  }
}

void BaseSimulator::publishRobotMarkers()
{
  int index = 0;
  for (const auto & robot : robots_) {
    auto loc = world()->getRobotLocation(robot.name);

    int id = index;
    size_t pos = robot.name.find_first_of("0123456789");
    if (pos != std::string::npos) {
      id = std::stoi(robot.name.substr(pos));
    }

    rviz_viz_.publishRobot(id, loc.x, loc.y, loc.theta, robot.model, 2);
    ++index;
  }
}

void BaseSimulator::publishClock()
{
  rosgraph_msgs::msg::Clock msg;
  double t = getSimTime();
  msg.clock.sec = static_cast<int32_t>(t);
  msg.clock.nanosec = static_cast<uint32_t>((t - msg.clock.sec) * 1e9);
  clock_pub_->publish(msg);
  advanceSimTime(dt_);
}

void BaseSimulator::publishRobotPoses()
{
  for (auto & pair : robot_pose_pubs_) {
    const auto & robot = pair.first;
    auto & pub = pair.second;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    // The plugin (move) is responsible for keeping the world updated.
    // Here we just publish whatever location the world has, by name.
    // For a real pose, the plugin would store interpolated coordinates
    // somewhere readable.
    auto loc = world()->getRobotLocation(robot);
    auto quat = atlantis::util::rpyToQuaternion(0.0, 0.0, loc.theta);
    msg.pose.position.x = loc.x;
    msg.pose.position.y = loc.y;
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();

    pub->publish(msg);
  }
}

void BaseSimulator::onConfigureExtra()
{
  RCLCPP_INFO(get_logger(), "BaseSimulator: configuring");
  loadBaseParameters();
  loadFootprints();
  buildCostmap();
  buildPlanners();
  buildPerRobotPublishers();

  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::QoS(rclcpp::KeepLast(10)));
  material_flow_pub_ = create_publisher<material_handler_msgs::msg::MaterialFlow>(
    "material_flow", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  navigo::CarPlanner * planner = nullptr;
  if (!base_planners_.empty()) {
    const std::string robot = base_planners_.begin()->first;
    planner = getPlanner(robot);
    RCLCPP_INFO(get_logger(), "Path costs: using the planner of %s for missing paths", robot.c_str());
  }

  if(compute_path_costs_) {
      const auto waypoints = world()->getWaypoints();
      std::vector<std::vector<double>> lengths(waypoints.size(), std::vector<double>(waypoints.size(), 0.0));
      for (size_t i = 0; i < waypoints.size(); ++i) {
        for (size_t j = i + 1; j < waypoints.size(); ++j) {
          navigo::Pose start(waypoints[i].x, waypoints[i].y, waypoints[i].theta);
          navigo::Pose goal(waypoints[j].x, waypoints[j].y, waypoints[j].theta);
          navigo::Path path = loadPrecomputedPath(start, goal);
          if (path.empty() && planner != nullptr) {
            path = planner->computePath(start, goal);
          }
          double length = std::numeric_limits<double>::infinity();
          if (path.empty()) {
            RCLCPP_WARN(get_logger(), "No path between %s and %s", waypoints[i].name.c_str(), waypoints[j].name.c_str());
          } else {
            length = pathLength(path);
          }
          lengths[i][j] = length;
          lengths[j][i] = length;
        }
      }

      std::ofstream costs("path_costs.csv");
      if (!costs.is_open()) {
        RCLCPP_ERROR(get_logger(), "Cannot create path_costs.csv");
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
      for (size_t i = 0; i < waypoints.size(); ++i) {
        costs << waypoints[i].name;
        for (size_t j = 0; j < waypoints.size(); ++j) {
          costs << ",";
          if (std::isinf(lengths[i][j])) {
            costs << "inf";
          } else {
            costs << lengths[i][j];
          }
        }
        costs << "\n";
      }
      RCLCPP_INFO(get_logger(), "Path costs written to path_costs.csv");
  }


}

void BaseSimulator::onActivateExtra()
{
  RCLCPP_INFO(get_logger(), "BaseSimulator: activating");

  clock_pub_->on_activate();
  map_pub_->on_activate();
  material_flow_pub_->on_activate();

  for (auto & pair : robot_pose_pubs_) {
    pair.second->on_activate();
  }
  for (auto & pair : robot_path_pubs_) {
    pair.second->on_activate();
  }

  // Clock at real_time_factor * dt period.
  auto clock_period = std::chrono::duration<double>(dt_ / real_time_factor_);
  clock_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(clock_period),
    [this]() { publishClock(); });

  // Pose publisher at ~10 Hz.
  pose_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { publishRobotPoses(); });

  marker_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() { publishRobotMarkers(); });

  tf_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() { publishRobotTransforms(); });

  nav_msgs::msg::OccupancyGrid map_msg;
  rviz_viz_.convertCostMapToMsg(oc_->getResolution(), oc_->getSizeInCellsX(),oc_->getSizeInCellsY(), oc_->getData() , map_msg);
  map_msg.header.frame_id = "map";
  map_pub_->publish(map_msg);
}

void BaseSimulator::onDeactivateExtra()
{
  RCLCPP_INFO(get_logger(), "BaseSimulator: deactivating");

  clock_timer_.reset();
  pose_timer_.reset();
  marker_timer_.reset();
  tf_timer_.reset();

  map_pub_->on_deactivate();

  for (auto & pair : robot_path_pubs_) {
    pair.second->on_deactivate();
  }
  for (auto & pair : robot_pose_pubs_) {
    pair.second->on_deactivate();
  }
  material_flow_pub_->on_deactivate();
  clock_pub_->on_deactivate();
}

void BaseSimulator::onCleanupExtra()
{
  RCLCPP_INFO(get_logger(), "BaseSimulator: cleaning up");

  for (auto & pair : base_planners_) {
    delete pair.second;
  }
  base_planners_.clear();

  delete oc_;
  oc_ = nullptr;

  robot_pose_pubs_.clear();
  robot_path_pubs_.clear();
  material_flow_pub_.reset();
  clock_pub_.reset();
  map_pub_.reset();
}

void BaseSimulator::publishRobotTransforms()
{
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(robots_.size());

  for (const auto & robot : robots_) {
    auto loc = world()->getRobotLocation(robot.name);
    auto quat = atlantis::util::rpyToQuaternion(0.0, 0.0, loc.theta);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = robot.name + "/base_link";
    tf.transform.translation.x = loc.x;
    tf.transform.translation.y = loc.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();
    tf.transform.rotation.w = quat.w();
    transforms.push_back(tf);
  }

  tf_broadcaster_->sendTransform(transforms);
}

}  // namespace atlantis_base