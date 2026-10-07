#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <fstream>
#include <utility>
#include <cctype>

#include "atlantis_scene_graph/scene_graph_generators/agent_scene_graph_generator.hpp"

namespace atlantis_scene_graph
{

namespace
{

const char * const kDefaultTypes = R"(
locatable - object
location  - object
robot - locatable
item - locatable
container - locatable
)";

const char * const kDefaultPredicates = R"yaml(
at:
  intent: "An entity is located at a place."
  arguments: "(?e - locatable ?l - location)"

on_top:
  intent: "An item is resting on top of another entity."
  arguments: "(?top - item ?bottom - locatable)"

in:
  intent: "An item is inside a container."
  arguments: "(?o - item ?c - container)"

holding:
  intent: "A robot is currently holding an item."
  arguments: "(?a - robot ?o - item)"

robot_free:
  intent: "A robot is not currently holding any item."
  arguments: "(?r - robot)"
)yaml";

std::string toLower(const std::string & text)
{
  std::string result = text;
  std::transform(
    result.begin(), result.end(), result.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return result;
}

std::vector<std::pair<std::string, std::string>> parseTypedList(const std::string & text)
{
  std::vector<std::string> tokens;
  std::istringstream lines(text);
  std::string line;
  while (std::getline(lines, line)) {
    auto cut = line.find_first_of("#;");
    if (cut != std::string::npos) {
      line = line.substr(0, cut);
    }
    std::replace(line.begin(), line.end(), '(', ' ');
    std::replace(line.begin(), line.end(), ')', ' ');
    std::istringstream stream(line);
    std::string token;
    while (stream >> token) {
      tokens.push_back(token);
    }
  }

  std::vector<std::pair<std::string, std::string>> items;
  size_t pending = 0;
  for (size_t i = 0; i < tokens.size(); ++i) {
    if (tokens[i] == "-") {
      std::string type = (i + 1 < tokens.size()) ? toLower(tokens[i + 1]) : "object";
      for (size_t k = items.size() - pending; k < items.size(); ++k) {
        items[k].second = type;
      }
      pending = 0;
      ++i;
    } else {
      items.emplace_back(toLower(tokens[i]), "object");
      ++pending;
    }
  }
  return items;
}

std::vector<std::string> parseArgumentTypes(const std::string & arguments)
{
  std::vector<std::string> types;
  for (const auto & item : parseTypedList(arguments)) {
    types.push_back(item.second);
  }
  return types;
}

}

AgentSceneGraphGenerator::AgentSceneGraphGenerator()
{

}

AgentSceneGraphGenerator::~AgentSceneGraphGenerator()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type AgentSceneGraphGenerator",
    name_.c_str());
}

void AgentSceneGraphGenerator::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;
  auto node_name = std::string(node->get_name());
  std::vector<std::string> default_list;
  node->declare_parameter("scene_graph_frame", std::string("map"));
  node->get_parameter("scene_graph_frame", scene_graph_frame_);
  node->declare_parameter("robot_type", std::string("robot"));
  node->get_parameter("robot_type", robot_type_);
  robot_type_ = toLower(robot_type_);
  node->declare_parameter("waypoint_type", std::string("location"));
  node->get_parameter("waypoint_type", waypoint_type_);
  waypoint_type_ = toLower(waypoint_type_);
  node->declare_parameter("types_file", std::string(""));
  node->get_parameter("types_file", types_file_);
  node->declare_parameter("predicates_file", std::string(""));
  node->get_parameter("predicates_file", predicates_file_);
  std::vector<std::string> object_names;
  node->declare_parameter("initial_objects", default_list);
  node->get_parameter("initial_objects", object_names);

  initial_objects_.clear();
  for (const auto & object : object_names) {
    const std::string type_param = object + ".type";
    if (!node->has_parameter(type_param)) {
      node->declare_parameter(type_param, std::string("item"));
    }
    std::string type;
    node->get_parameter(type_param, type);
    initial_objects_.emplace_back(object, toLower(type));
  }
  node->declare_parameter("initial_facts", default_list);
  node->get_parameter("initial_facts", initial_facts_);

  loadTypes();
  loadPredicates();
  checkSchema();
  loadInitialState();

  robot_ids_sub_ = node->create_subscription<standard_msgs::msg::StringMultiArray>(
    "robot_ids",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](standard_msgs::msg::StringMultiArray msg) {
      robotIdsCallback(msg);
    });

  waypoints_sub_ = node->create_subscription<location_msgs::msg::WaypointArray>(
          "waypoints",
          rclcpp::SensorDataQoS(),
          [this](location_msgs::msg::WaypointArray msg) {
            waypointArrayCallback(msg);
          }
        );

  plan_sub_ = node->create_subscription<standard_msgs::msg::Plan>(
    "/dispatched_plan",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](standard_msgs::msg::Plan msg) {
      planCallback(msg);
    });

  event_sub_ = node->create_subscription<standard_msgs::msg::Event>(
    "/plan_actions",
    rclcpp::QoS(rclcpp::KeepLast(1000)).transient_local().reliable(),
    [this](standard_msgs::msg::Event msg) {
      eventCallback(msg);
    });

  RCLCPP_INFO(logger_, "Configuring %s of type AgentSceneGraphGenerator", name.c_str());

}

void AgentSceneGraphGenerator::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type AgentSceneGraphGenerator", name_.c_str());
}

void AgentSceneGraphGenerator::deactivate()
{
  RCLCPP_INFO( logger_, "Deactivating plugin %s of type AgentSceneGraphGenerator", name_.c_str());
}

void AgentSceneGraphGenerator::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type AgentSceneGraphGenerator", name_.c_str());
}

void AgentSceneGraphGenerator::loadTypes()
{
  std::string text = kDefaultTypes;

  if (types_file_.empty()) {
    RCLCPP_INFO(logger_, "No types file given, using the default types");
  } else {
    std::ifstream file(types_file_);
    if (file.is_open()) {
      std::stringstream buffer;
      buffer << file.rdbuf();
      text = buffer.str();
      RCLCPP_INFO(logger_, "Loading types from %s", types_file_.c_str());
    } else {
      RCLCPP_ERROR(
        logger_, "Cannot open types file %s, using the default types",
        types_file_.c_str());
    }
  }

  type_parents_.clear();
  for (const auto & [type, parent] : parseTypedList(text)) {
    type_parents_[type] = parent;
  }
}

void AgentSceneGraphGenerator::loadPredicates()
{
  const YAML::Node defaults = YAML::Load(kDefaultPredicates);
  YAML::Node root = defaults;

  if (predicates_file_.empty()) {
    RCLCPP_INFO(logger_, "No predicates file given, using the default predicates");
  } else {
    try {
      root = YAML::LoadFile(predicates_file_);
      RCLCPP_INFO(logger_, "Loading predicates from %s", predicates_file_.c_str());
    } catch (const YAML::Exception & ex) {
      RCLCPP_ERROR(
        logger_, "Cannot read predicates file %s: %s. Using the default predicates",
        predicates_file_.c_str(), ex.what());
    }
  }

  try {
    readPredicates(root);
  } catch (const YAML::Exception & ex) {
    RCLCPP_ERROR(
      logger_, "Predicates file %s has a wrong format: %s. Using the default predicates",
      predicates_file_.c_str(), ex.what());
    readPredicates(defaults);
  }
}

void AgentSceneGraphGenerator::readPredicates(const YAML::Node & root)
{
  predicates_.clear();

  for (const auto & entry : root) {
    PredicateSchema schema;
    const YAML::Node value = entry.second;
    if (value["intent"]) {
      schema.intent = value["intent"].as<std::string>();
    }
    if (value["arguments"]) {
      schema.types = parseArgumentTypes(value["arguments"].as<std::string>());
    }
    predicates_[toLower(entry.first.as<std::string>())] = schema;
  }
}

void AgentSceneGraphGenerator::checkSchema()
{
  auto known = [this](const std::string & type) {
      return type == "object" || type_parents_.count(type) > 0;
    };

  for (const auto & [type, parent] : type_parents_) {
    if (!known(parent)) {
      RCLCPP_WARN(logger_, "Type %s has an unknown parent type %s", type.c_str(), parent.c_str());
    }
  }

  for (const auto & [predicate, schema] : predicates_) {
    for (const auto & type : schema.types) {
      if (!known(type)) {
        RCLCPP_WARN(
          logger_, "Predicate %s uses an unknown type %s",
          predicate.c_str(), type.c_str());
      }
    }
  }

  if (!known(robot_type_)) {
    RCLCPP_WARN(logger_, "Robot type %s is not a known type", robot_type_.c_str());
  }

  if (!known(waypoint_type_)) {
    RCLCPP_WARN(logger_, "Waypoint type %s is not a known type", waypoint_type_.c_str());
  }

  if (predicates_.count("at") == 0) {
    RCLCPP_WARN(
      logger_,
      "There is no 'at' predicate, the initial robot locations will not match the predicates");
  }

  RCLCPP_INFO(
    logger_, "Loaded %zu types and %zu predicates",
    type_parents_.size(), predicates_.size());
}

void AgentSceneGraphGenerator::loadInitialState()
{
  std::lock_guard<std::mutex> lock(mutex_);

  size_t object_count = 0;
  for (const auto & [id, type] : initial_objects_) {
    if (type != "object" && type_parents_.count(type) == 0) {
      RCLCPP_WARN(logger_, "Object %s has an unknown type %s", id.c_str(), type.c_str());
    }
    setNodeType(id, type);
    ++object_count;
  }

  size_t fact_count = 0;
  for (const auto & entry : initial_facts_) {
    auto fact = tokenize(entry);
    if (fact.empty()) {
      continue;
    }

    if (fact[0] == "not") {
      RCLCPP_WARN(
        logger_, "Initial fact %s is negative and is ignored, missing facts are already false",
        entry.c_str());
      continue;
    }

    if (!isValidFact(fact)) {
      continue;
    }

    addFactNodes(fact);
    const std::vector<std::string> args(fact.begin() + 1, fact.end());
    if (!hasEdge(fact[0], args)) {
      addEdge(fact[0], args);
      ++fact_count;
    }
  }

  RCLCPP_INFO(
    logger_, "Loaded %zu initial objects and %zu initial facts",
    object_count, fact_count);
}

scene_graph_msgs::msg::SceneGraph AgentSceneGraphGenerator::generateSceneGraph()
{
  scene_graph_msgs::msg::SceneGraph msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = graph_;
  }

  auto node = node_.lock();
  if (node) {
    msg.header.stamp = node->now();
  }
  msg.header.frame_id = scene_graph_frame_;
  return msg;
}

bool AgentSceneGraphGenerator::isReady()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return robot_ids_received_ && allRobotsLocated();
}

void AgentSceneGraphGenerator::robotIdsCallback(standard_msgs::msg::StringMultiArray msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  robot_ids_received_ = true;

  for (const auto & robot : msg.data) {
    if (robot.empty()) {
      continue;
    }

    if (std::find(robots_ids_.begin(), robots_ids_.end(), robot) != robots_ids_.end()) {
      continue;
    }

    robots_ids_.push_back(robot);
    setNodeType(robot, robot_type_);

    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      robot + "/current_pose",
      rclcpp::SensorDataQoS(),
      [this, robot](geometry_msgs::msg::PoseStamped pose) {
        currentPoseCallback(pose, robot);
      });
    position_subs_.push_back(sub);

    RCLCPP_INFO(logger_, "Added robot %s", robot.c_str());
  }
}

void AgentSceneGraphGenerator::currentPoseCallback(
  geometry_msgs::msg::PoseStamped msg,
  const std::string & robot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  updateNodePose(robot, msg.pose);

  if (waypoints_.empty()) {
    return;
  }

  if (initial_state_built_ && hasLocation(robot)) {
    return;
  }

  Eigen::Quaterniond quaternion;
  quaternion.x() = msg.pose.orientation.x;
  quaternion.y() = msg.pose.orientation.y;
  quaternion.z() = msg.pose.orientation.z;
  quaternion.w() = msg.pose.orientation.w;
  auto rpy = atlantis::util::quaternionToEulerAngles(quaternion);
  auto theta = rpy[2]; // Yaw
  auto nearest = findNearestWaypoint(msg.pose.position.x, msg.pose.position.y, theta);

  const std::vector<std::string> args = {robot, nearest.name};
  if (!hasEdge("at", args)) {
    removeEdges("at", robot);
    addEdge("at", args);
  }

  if (!initial_state_built_ && allRobotsLocated()) {
    initial_state_built_ = true;
    RCLCPP_INFO(logger_, "Initial state built, the state now changes only through action effects");
  }
}

void AgentSceneGraphGenerator::waypointArrayCallback(location_msgs::msg::WaypointArray msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(logger_, "Received waypoint array for robot");
    for (auto wp: msg.waypoints){
        atlantis_core::Waypoint waypoint;
        waypoint.name = wp.name;
        waypoint.x = wp.pose.position.x;
        waypoint.y = wp.pose.position.y;
        waypoints_.push_back(waypoint);
        setNodeType(wp.name, waypoint_type_);
        updateNodePose(wp.name, wp.pose);
    }

}

void AgentSceneGraphGenerator::planCallback(standard_msgs::msg::Plan msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg == plan_) {
    return;
  }
  plan_ = msg;
  applied_actions_.clear();
  RCLCPP_INFO(logger_, "Received plan with %zu actions", plan_.actions.size());
}

void AgentSceneGraphGenerator::eventCallback(standard_msgs::msg::Event msg)
{
  if (msg.kind != standard_msgs::msg::Event::ACTION) {
    return;
  }

  if (msg.status == standard_msgs::msg::Event::FAILURE) {
    RCLCPP_WARN(logger_, "Action %d (%s) failed, effects not applied", msg.id, msg.name.c_str());
    return;
  }

  if (msg.status != standard_msgs::msg::Event::SUCCESS) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (applied_actions_.count(msg.id) > 0) {
    return;
  }

  auto it = std::find_if(
    plan_.actions.begin(), plan_.actions.end(),
    [&](const standard_msgs::msg::Action & a) {return a.action_id == msg.id;});

  if (it == plan_.actions.end()) {
    RCLCPP_WARN(logger_, "Action %d (%s) not found in the plan", msg.id, msg.name.c_str());
    return;
  }

  applyEffects(it->effects);
  applied_actions_.insert(msg.id);
}

void AgentSceneGraphGenerator::applyEffects(const std::vector<std::string> & effects)
{
  std::vector<std::vector<std::string>> adds;
  std::vector<std::vector<std::string>> deletes;

  for (const auto & effect : effects) {
    auto tokens = tokenize(effect);
    if (tokens.empty()) {
      continue;
    }
    if (tokens[0] == "not") {
      tokens.erase(tokens.begin());
      if (!tokens.empty()) {
        deletes.push_back(tokens);
      }
    } else {
      adds.push_back(tokens);
    }
  }

  for (const auto & fact : deletes) {
    if (!isValidFact(fact)) {
      continue;
    }
    removeEdge(fact[0], std::vector<std::string>(fact.begin() + 1, fact.end()));
  }

  for (const auto & fact : adds) {
    if (!isValidFact(fact)) {
      continue;
    }
    addFactNodes(fact);
    const std::vector<std::string> args(fact.begin() + 1, fact.end());
    if (!hasEdge(fact[0], args)) {
      addEdge(fact[0], args);
    }
  }
}

bool AgentSceneGraphGenerator::isValidFact(const std::vector<std::string> & fact) const
{
  if (predicates_.empty()) {
    return true;
  }

  auto it = predicates_.find(toLower(fact[0]));
  if (it == predicates_.end()) {
    RCLCPP_WARN(logger_, "Predicate %s is not known, fact ignored", fact[0].c_str());
    return false;
  }

  if (it->second.types.size() != fact.size() - 1) {
    RCLCPP_WARN(
      logger_, "Predicate %s expects %zu arguments but got %zu, fact ignored",
      fact[0].c_str(), it->second.types.size(), fact.size() - 1);
    return false;
  }

  return true;
}

void AgentSceneGraphGenerator::addFactNodes(const std::vector<std::string> & fact)
{
  std::vector<std::string> types;
  auto it = predicates_.find(toLower(fact[0]));
  if (it != predicates_.end()) {
    types = it->second.types;
  }

  for (size_t i = 1; i < fact.size(); ++i) {
    const auto & id = fact[i];
    bool exists = std::any_of(
      graph_.nodes.begin(), graph_.nodes.end(),
      [&](const scene_graph_msgs::msg::Node & n) {return n.id == id;});
    if (exists) {
      continue;
    }

    scene_graph_msgs::msg::Node node;
    node.id = id;
    node.type = (i - 1 < types.size()) ? types[i - 1] : "object";
    graph_.nodes.push_back(node);
  }
}

bool AgentSceneGraphGenerator::hasLocation(const std::string & robot) const
{
  return std::any_of(
    graph_.edges.begin(), graph_.edges.end(),
    [&](const scene_graph_msgs::msg::Edge & e) {
      return e.relation == "at" && !e.args.empty() && e.args[0] == robot;
    });
}

bool AgentSceneGraphGenerator::allRobotsLocated() const
{
  return std::all_of(
    robots_ids_.begin(), robots_ids_.end(),
    [&](const std::string & robot) {return hasLocation(robot);});
}

std::vector<std::string> AgentSceneGraphGenerator::tokenize(const std::string & fact) const
{
  std::string clean = fact;
  std::replace(clean.begin(), clean.end(), '(', ' ');
  std::replace(clean.begin(), clean.end(), ')', ' ');

  std::istringstream stream(clean);
  std::vector<std::string> tokens;
  std::string token;
  while (stream >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

atlantis_core::Waypoint AgentSceneGraphGenerator::findNearestWaypoint(double x, double y, double theta)
{
    double best_distance = std::numeric_limits<double>::max();
    atlantis_core::Waypoint best_match;

    for (const auto & wp : waypoints_) {
        double dx = x - wp.x;
        double dy = y - wp.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double dtheta = std::fabs(theta - wp.theta);
        if ((distance + dtheta) <= best_distance) {
            best_distance = distance;
            best_match = wp;
        }
    }

    //RCLCPP_INFO(logger_, "Nearest waypoint: %s (x=%f, y=%f)", best_match.name.c_str(), best_match.x, best_match.y);
    return best_match;
}

void AgentSceneGraphGenerator::updateNodePose(
  const std::string & id, const geometry_msgs::msg::Pose & pose)
{
  auto it = std::find_if(
    graph_.nodes.begin(), graph_.nodes.end(),
    [&](const scene_graph_msgs::msg::Node & n) {return n.id == id;});

  if (it != graph_.nodes.end()) {
    it->pose = pose;
  }
}

void AgentSceneGraphGenerator::setNodeType(const std::string & id, const std::string & type)
{
  auto it = std::find_if(
    graph_.nodes.begin(), graph_.nodes.end(),
    [&](const scene_graph_msgs::msg::Node & n) {return n.id == id;});

  if (it != graph_.nodes.end()) {
    it->type = type;
    return;
  }

  scene_graph_msgs::msg::Node new_node;
  new_node.id = id;
  new_node.type = type;
  graph_.nodes.push_back(new_node);
}

bool AgentSceneGraphGenerator::hasEdge(
  const std::string & relation, const std::vector<std::string> & args) const
{
  return std::any_of(
    graph_.edges.begin(), graph_.edges.end(),
    [&](const scene_graph_msgs::msg::Edge & e) {
      return e.relation == relation && e.args == args;
    });
}

void AgentSceneGraphGenerator::removeEdge(
  const std::string & relation, const std::vector<std::string> & args)
{
  graph_.edges.erase(
    std::remove_if(
      graph_.edges.begin(), graph_.edges.end(),
      [&](const scene_graph_msgs::msg::Edge & e) {
        return e.relation == relation && e.args == args;
      }),
    graph_.edges.end());
}

void AgentSceneGraphGenerator::removeEdges(const std::string & relation, const std::string & first_arg)
{
  graph_.edges.erase(
    std::remove_if(
      graph_.edges.begin(), graph_.edges.end(),
      [&](const scene_graph_msgs::msg::Edge & e) {
        return e.relation == relation && !e.args.empty() && e.args[0] == first_arg;
      }),
    graph_.edges.end());
}

void AgentSceneGraphGenerator::addEdge(
  const std::string & relation, const std::vector<std::string> & args)
{
  scene_graph_msgs::msg::Edge edge;
  edge.relation = relation;
  edge.args = args;
  graph_.edges.push_back(edge);
}

rcl_interfaces::msg::SetParametersResult
AgentSceneGraphGenerator::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    bool reinit_a_star = false;
    bool reinit_downsampler = false;

    for (auto parameter : parameters) {
        const auto & type = parameter.get_type();
        const auto & name = parameter.get_name();
    }


    result.successful = true;
    return result;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(atlantis_scene_graph::AgentSceneGraphGenerator, atlantis_scene_graph::SceneGraphGenerator)