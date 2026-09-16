// Copyright 2026 Atlantis

#include <atlantis_core/simulator_core.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include <algorithm>
#include <cmath>
#include <set>

namespace atlantis_core
{

SimulatorCore::SimulatorCore(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
{
}

SimulatorCore::~SimulatorCore()
{
  RCLCPP_INFO(get_logger(), "Destroying simulator core");
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    this->deactivate();
  }
  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    this->cleanup();
  }
}


std::vector<Waypoint> SimulatorCore::loadWaypointsFromFile(const std::string & file_path)
{
  YAML::Node root = YAML::LoadFile(file_path);
  std::vector<Waypoint> waypoints;
  for (const auto & entry : root) {
    Waypoint wp;
    wp.name = entry.first.as<std::string>();
    wp.x = entry.second["x"].as<double>();
    wp.y = entry.second["y"].as<double>();
    wp.theta = entry.second["yaw"].as<double>();
    waypoints.push_back(wp);
  }
  return waypoints;
}

void SimulatorCore::loadCommonParameters()
{
  std::vector<std::string> default_ids;
  std::string default_path;
  metrics_ = {"execution_time", "action_time"};

  declare_parameter("robots", default_ids);
  declare_parameter("materials", default_ids);
  declare_parameter("waypoints", default_ids);
  declare_parameter("waypoints_file", default_path);
  declare_parameter("actions", defaultActions());
  declare_parameter("services", defaultServices());
  declare_parameter("metrics", metrics_);

  get_parameter("robots", robots_ids_);
  get_parameter("materials", materials_ids_);
  get_parameter("waypoints", waypoints_ids_);
  get_parameter("waypoints_file", waypoints_path_);
  get_parameter("actions", action_names_);
  get_parameter("services", service_names_);

  std::vector<std::string> extra_metrics;
  get_parameter("metrics", extra_metrics);
  for (const auto & m : extra_metrics) {
    if (std::find(metrics_.begin(), metrics_.end(), m) == metrics_.end()) {
      metrics_.push_back(m);
    }
  }

  declare_parameter("waypoint_publisher.topic", std::string("waypoints"));
  declare_parameter("waypoint_publisher.frame_id", std::string("map"));
  get_parameter("waypoint_publisher.topic", waypoint_topic_);
  get_parameter("waypoint_publisher.frame_id", waypoint_frame_id_);
}

void SimulatorCore::buildWorld()
{
  world_ = std::make_shared<SimulationWorld>();
  
  if(!waypoints_path_.empty()) {
    auto file = atlantis::util::resolve_pkg_uri(waypoints_path_);
    RCLCPP_INFO(get_logger(), "Loading waypoints from %s", file.c_str()); 
    auto waypoints = loadWaypointsFromFile(file);
    for (const auto & wp : waypoints) {
      world_->addWaypoint(wp);
      RCLCPP_INFO(get_logger(), "Added waypoint %s at (%f, %f, %f)",
        wp.name.c_str(), wp.x, wp.y, wp.theta);
    }
  }
  else{
    // Waypoints
    for (const auto & wp_name : waypoints_ids_) {
      Waypoint wp;
      wp.name = wp_name;

      declare_parameter(wp_name + ".x", 0.0);
      declare_parameter(wp_name + ".y", 0.0);
      declare_parameter(wp_name + ".yaw", 0.0);
      get_parameter(wp_name + ".x", wp.x);
      get_parameter(wp_name + ".y", wp.y);
      get_parameter(wp_name + ".yaw", wp.theta);

      world_->addWaypoint(wp);
      RCLCPP_INFO(
        get_logger(), "Added waypoint %s at (%f, %f, %f)",
        wp_name.c_str(), wp.x, wp.y, wp.theta);
    }

  }



  // Materials
  std::vector<std::string> material_locs;
  for (const auto & name : materials_ids_) {
    int id = 0;
    declare_parameter(name + ".ID", 0);
    declare_parameter(name + ".locations", std::vector<std::string>{});
    get_parameter(name + ".ID", id);
    get_parameter(name + ".locations", material_locs);

    Material material;
    material.name = name;
    material.id = id;

    for (const auto & loc : material_locs) {
      double amount = 0.0;
      declare_parameter(name + "." + loc + ".amount", 0.0);
      get_parameter(name + "." + loc + ".amount", amount);
      material.amounts[loc] = amount;
      RCLCPP_INFO(
        get_logger(), "Material %s at %s: %f", name.c_str(), loc.c_str(), amount);
    }
    world_->addMaterial(material);
  }

  // Robots
  for (const auto & name : robots_ids_) {
    Waypoint start_location;
    std::string empty, type;
    std::string model, footprint_points;
    double capacity, minimum_turning_radius;
    declare_parameter(name+".initial_pose.x", 0.0);
    declare_parameter(name+".initial_pose.y", 0.0);
    declare_parameter(name+".initial_pose.yaw", 0.0);
    declare_parameter(name+".minimum_turning_radius", 0.0);
    declare_parameter(name+".footprint", empty);
    declare_parameter(name + ".model", empty);
    declare_parameter(name + ".type", empty);
    declare_parameter(name + ".capacity", 0.0);
    declare_parameter(name + ".material_publisher.topic", std::string("material_stock"));
    declare_parameter(name + ".material_publisher.rate", 1.0);


    get_parameter(name + ".initial_pose.x", start_location.x);
    get_parameter(name + ".initial_pose.y", start_location.y);
    get_parameter(name + ".initial_pose.yaw", start_location.theta);
    get_parameter(name + ".footprint", footprint_points);
    get_parameter(name + ".capacity", capacity);
    get_parameter(name + ".model", model);
    get_parameter(name + ".type", type);
    get_parameter(name + ".minimum_turning_radius", minimum_turning_radius);
    get_parameter(name + ".material_publisher.topic", material_topic_);
    get_parameter(name + ".material_publisher.rate", material_publish_rate_);

    RobotState robot;
    robot.info.name = name;
    robot.info.type = type;
    robot.info.capacity = capacity;
    robot.info.model = model;
    robot.info.minimum_turning_radius = minimum_turning_radius;
    robot.info.footprint = footprint_points;
    
    robot.current_location = start_location;
    robot.loaded_amount = 0.0;
    world_->addRobot(robot);
    robots_.push_back(robot.info);

    RCLCPP_INFO(get_logger(), "Robot %s starts at (%f, %f, %f)",name.c_str(), start_location.x, start_location.y, start_location.theta);
  }
}

void SimulatorCore::publishMaterials()
{
  material_handler_msgs::msg::MaterialStockArray msg;
  for (const auto & material : world_->getMaterials()) {
    for (const auto & entry : material.amounts) {
      material_handler_msgs::msg::MaterialStock stock;
      stock.material = material.name;
      stock.location = entry.first;
      stock.amount = entry.second;
      msg.stocks.push_back(stock);
    }
  }
  material_pub_->publish(msg);
}

void SimulatorCore::loadActions()
{
  action_loader_ = std::make_unique<pluginlib::ClassLoader<ActionPlugin>>(
    "atlantis_core", "atlantis_core::ActionPlugin");

  std::set<std::string> seen_topics;
  for (const auto & robot : robots_) {
    for (const auto & action : action_names_) {
      PluginConfig config;
      config.name = robot.name + "." + action;

      std::string topic_suffix;
      if (!has_parameter(action + ".type")) {
        declare_parameter(action + ".type", std::string(""));
      }
      if (!has_parameter(action + ".topic")) {
        declare_parameter(action + ".topic", std::string(""));
      }
      get_parameter(action + ".type", config.type);
      get_parameter(action + ".topic", topic_suffix);

      if (config.type.empty()) {
        RCLCPP_ERROR(
          get_logger(), "Action %s has no .type parameter, skipping",
          action.c_str());
        continue;
      }

      // Default the topic to the action name if not given.
      if (topic_suffix.empty()) {
        topic_suffix = action;
      }
      config.topic = robot.name  + "/" + topic_suffix;

      if (!seen_topics.insert(config.topic).second) {
        RCLCPP_ERROR(
          get_logger(),
          "Duplicate action topic %s, skipping",
          config.topic.c_str());
        continue;
      }

      try {
        auto plugin = action_loader_->createSharedInstance(config.type);
        plugin->initialize(this, world_, config);
        action_plugins_.push_back(plugin);
        RCLCPP_INFO(
          get_logger(), "Loaded action %s (type %s) on topic %s",
          config.name.c_str(), config.type.c_str(), config.topic.c_str());
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(
          get_logger(), "Failed to load action %s (type %s): %s",
          config.name.c_str(), config.type.c_str(), ex.what());
      }
    }
  }
}

void SimulatorCore::loadServices()
{
  service_loader_ = std::make_unique<pluginlib::ClassLoader<ServicePlugin>>(
    "atlantis_core", "atlantis_core::ServicePlugin");

  std::set<std::string> seen_topics;
  for (const auto & robot : robots_) {
    for (const auto & service : service_names_) {
      PluginConfig config;
      config.name = robot.name + "." + service;

      std::string topic_suffix;
      if (!has_parameter(service + ".type")) {
        declare_parameter(service + ".type", std::string(""));
      }
      if (!has_parameter(service + ".topic")) {
        declare_parameter(service + ".topic", std::string(""));
      }
      get_parameter(service + ".type", config.type);
      get_parameter(service + ".topic", topic_suffix);

      if (config.type.empty()) {
        RCLCPP_ERROR(
          get_logger(), "Service %s has no .type parameter, skipping",
          service.c_str());
        continue;
      }

      if (topic_suffix.empty()) {
        topic_suffix = service;
      }
      config.topic = robot.name  + "/" + topic_suffix;

      if (!seen_topics.insert(config.topic).second) {
        RCLCPP_ERROR(
          get_logger(),
          "Duplicate service topic %s, skipping",
          config.topic.c_str());
        continue;
      }

      try {
        auto plugin = service_loader_->createSharedInstance(config.type);
        plugin->initialize(this, world_, config);
        service_plugins_.push_back(plugin);
        RCLCPP_INFO(
          get_logger(), "Loaded service %s (type %s) on topic %s",
          config.name.c_str(), config.type.c_str(), config.topic.c_str());
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(
          get_logger(), "Failed to load service %s (type %s): %s",
          config.name.c_str(), config.type.c_str(), ex.what());
      }
    }
  }
}

void SimulatorCore::setupMetrics()
{
  for (const auto & metric : metrics_) {
    bool evaluate = true;
    std::string topic;
    declare_parameter(metric + ".enable", evaluate);
    declare_parameter(metric + ".topic", std::string(get_name()) + "/" + metric);
    get_parameter(metric + ".enable", evaluate);
    get_parameter(metric + ".topic", topic);

    if (evaluate) {
      metrics_to_evaluate_[metric] = true;
      metrics_pubs_[metric] = create_publisher<std_msgs::msg::Float64>(
        topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
      RCLCPP_INFO(
        get_logger(), "Evaluating metric %s on topic %s",
        metric.c_str(), topic.c_str());
    }
  }
}

CallbackReturn SimulatorCore::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  loadCommonParameters();
  buildWorld();
  loadActions();
  loadServices();
  setupMetrics();

  waypoint_pub_ = create_publisher<location_msgs::msg::WaypointArray>(
    waypoint_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  
  material_pub_ = create_publisher<material_handler_msgs::msg::MaterialStockArray>(
    material_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  onConfigureExtra();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimulatorCore::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  for (auto & pair : metrics_pubs_) {
    pair.second->on_activate();
  }
  waypoint_pub_->on_activate();

  // Publish the waypoint list once. Transient local QoS ensures
  // late subscribers still receive it.
  location_msgs::msg::WaypointArray msg;
  msg.header.stamp = now();
  msg.header.frame_id = waypoint_frame_id_;
  for (const auto & wp : world_->getWaypoints()) {
    location_msgs::msg::Waypoint wp_msg;
    wp_msg.name = wp.name;
    wp_msg.header.stamp = msg.header.stamp;
    wp_msg.header.frame_id = waypoint_frame_id_;
    wp_msg.pose.position.x = wp.x;
    wp_msg.pose.position.y = wp.y;
    wp_msg.pose.position.z = 0.0;
    // Convert yaw to quaternion (only z and w are non-zero for a yaw rotation).
    wp_msg.pose.orientation.z = std::sin(wp.theta / 2.0);
    wp_msg.pose.orientation.w = std::cos(wp.theta / 2.0);
    msg.waypoints.push_back(wp_msg);
  }
  waypoint_pub_->publish(msg);
  RCLCPP_INFO(
    get_logger(), "Published %zu waypoints on %s",
    msg.waypoints.size(), waypoint_topic_.c_str());

  for (auto & plugin : action_plugins_) {
    plugin->activate();
  }
  for (auto & plugin : service_plugins_) {
    plugin->activate();
  }
  material_pub_->on_activate();
  material_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / material_publish_rate_),
    std::bind(&SimulatorCore::publishMaterials, this));

  onActivateExtra();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimulatorCore::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  onDeactivateExtra();
  for (auto & plugin : service_plugins_) {
    plugin->deactivate();
  }
  for (auto & plugin : action_plugins_) {
    plugin->deactivate();
  }
  material_timer_.reset();
  material_pub_->on_deactivate();
  waypoint_pub_->on_deactivate();
  for (auto & pair : metrics_pubs_) {
    pair.second->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimulatorCore::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  onCleanupExtra();
  for (auto & plugin : service_plugins_) {
    plugin->cleanup();
  }
  for (auto & plugin : action_plugins_) {
    plugin->cleanup();
  }
  service_plugins_.clear();
  action_plugins_.clear();
  service_loader_.reset();
  action_loader_.reset();
  for (auto & pair : metrics_pubs_) {
    pair.second.reset();
  }
  metrics_pubs_.clear();
  waypoint_pub_.reset();
  material_pub_.reset();
  world_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SimulatorCore::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

}  // namespace atlantis_core