#include <atlantis_discrete_event/discrete_event_simulator.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include "nlohmann/json.hpp"
#include <regex>
#include <fstream>

using namespace atlantis::util;

namespace atlantis_simulator
{


int getLocationIndex(std::string location, std::vector<std::string> material_locations){
  auto it = std::find(material_locations.begin(), material_locations.end(), location);
  if(it == material_locations.end()){
    return -1;
  }
  return std::distance(material_locations.begin(), it);
}


DiscreteEventSimulator::DiscreteEventSimulator(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
{
    std::string waypoints_file;
    std::vector<std::string> default_ids, new_metrics;
    metrics_ = {"execution_time", "action_time"};
    declare_parameter("robots", default_ids);
    declare_parameter("materials", default_ids);
    declare_parameter("metrics", metrics_);
    declare_parameter("waypoints", default_ids);
    declare_parameter("waypoints_file", "");
    declare_parameter("refilling", false);
    declare_parameter("randomness", false);
    declare_parameter("execution_sec", 1);

    get_parameter("robots", robots_ids_);
    get_parameter("materials", materials_ids_);
    get_parameter("metrics", new_metrics);
    get_parameter("waypoints", waypoints_ids_);
    get_parameter("waypoints_file", waypoints_file);
    get_parameter("refilling", refilling_);
    get_parameter("randomness", randomness_);
    get_parameter("execution_sec", execution_sec_);
    if(waypoints_file != "" && !waypoints_file.empty() && waypoints_ids_.empty()){
      waypoints_ = parseWaypoints(waypoints_file);
    }

    for (size_t i = 0; i < new_metrics.size(); ++i) {
      if(std::find(metrics_.begin(), metrics_.end(), new_metrics[i]) == metrics_.end()){
        metrics_.push_back(new_metrics[i]);
      }
    }
    for (size_t i = 0; i < metrics_.size(); ++i) {
      bool evaluate = true;
      std::string topic;
      RCLCPP_INFO(this->get_logger(), "Added metric %s", metrics_[i].c_str());
      declare_parameter(metrics_[i]+".enable", evaluate);
      get_parameter(metrics_[i]+".enable", evaluate);
      declare_parameter(metrics_[i]+".topic", node_name + "/" + metrics_[i]);
      get_parameter(metrics_[i]+".topic", topic);
      if(evaluate){
        metrics_to_evaluate_.insert(std::make_pair(metrics_[i],true));
        RCLCPP_INFO(this->get_logger(), "Evaluating metric %s", metrics_[i].c_str());
        metrics_pubs_.insert(std::make_pair(metrics_[i],create_publisher<std_msgs::msg::Float64>(topic,rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable())));
      }
    }
}

std::string DiscreteEventSimulator::find_waypoint(double x, double y, double theta, double tolerance) {
    for (auto wp: waypoints_){
        double wp_x = wp.x;
        double wp_y = wp.y;
        double distance = std::sqrt(std::pow(x - wp_x, 2) + std::pow(y - wp_y, 2));
        if (distance <= tolerance && std::abs(theta - wp.theta) <= tolerance) {
            return wp.name;
        }
    }
    return "-1";
}


double DiscreteEventSimulator::getMaterialAmount(std::string pile, std::string pile_loc){
  for(auto mat: materials_){
    if (mat.name_ == pile){
      return mat.getAmount(pile_loc);
    }
  }  
  return -1;  
}

CallbackReturn
DiscreteEventSimulator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring executor...");
    std::vector<std::string> default_ids, material_locs;

    if(waypoints_.empty()){
      for (size_t i = 0; i < waypoints_ids_.size(); ++i) {
            auto wp = waypoints_ids_[i];
            RCLCPP_INFO(this->get_logger(), "Added waypoint %s", wp.c_str());
            Waypoint waypoint;
            waypoint.x = 0;
            waypoint.y = 0;
            waypoint.theta = 0;
            waypoint.name = wp;
            waypoints_.push_back(waypoint);
        }
    }
   
    for (size_t i = 0; i < materials_ids_.size(); ++i) {
        int marker_id = 1;
        auto name = materials_ids_[i];
        Material material(name);
        int ID;
        double amount;
        std::map<std::string, double> location_amounts;
        std::string loc;
        declare_parameter(name+".ID", 0);
        declare_parameter(name+".locations", default_ids);

        get_parameter(name+".ID", ID);
        get_parameter(name+".locations", material_locs);
       
        for (size_t j = 0; j < material_locs.size(); ++j) {
          loc = material_locs[j];
          declare_parameter(name+"."+ loc + ".amount", 0.);
          get_parameter(name+"."+ loc + ".amount", amount);
          RCLCPP_INFO(get_logger(), "Got material %s at location %s with amount %f...", name.c_str(), loc.c_str(), amount);
          location_amounts[loc] = amount;
        }

        material.id = ID;
        material.amounts = location_amounts;
        materials_.push_back(material);
    }
   
    for (size_t i = 0; i < robots_ids_.size(); ++i) {
        auto name = robots_ids_[i];
        int robotID;
        std::string start_location;
        double capacity;
        RCLCPP_INFO(get_logger(), "Configuring executor for %s...", name.c_str());

        declare_parameter(name+".start_location", "home");
        declare_parameter(name+".capacity", 0.0);
        get_parameter(name+".start_location", start_location);
        get_parameter(name+".capacity", capacity);

        RCLCPP_INFO(get_logger(), "Start location for %s: %s", name.c_str(), start_location.c_str());

        auto action_name = name + "/move_to_pose";
        auto action_load_name = name + "/send_load";
        auto action_dump_name = name + "/send_dump";

        material_amount_servers_.push_back(create_service<material_handler_msgs::srv::GetMaterialAmount>(
                name + std::string("/get_material_amount"),
                std::bind(&DiscreteEventSimulator::getMaterialAmountCallback, this, std::placeholders::_1, std::placeholders::_2)));

        capacities_.insert(std::make_pair(name, capacity));
        robots_current_locations_.insert(std::make_pair(name, start_location));
        amount_loaded_.insert(std::make_pair(name, -1.0));

        // Navigate action server (MoveToPose)
        action_servers_.push_back(rclcpp_action::create_server<standard_msgs::action::MoveToPose>(
            this,
            action_name,
            std::bind(&DiscreteEventSimulator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DiscreteEventSimulator::handle_cancel, this, std::placeholders::_1),
            std::bind(&DiscreteEventSimulator::handle_accepted, this, std::placeholders::_1, name)));

        // Load action server
        load_action_servers_.push_back(rclcpp_action::create_server<standard_msgs::action::Load>(
            this,
            action_load_name,
            std::bind(&DiscreteEventSimulator::handle_load_cmd_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DiscreteEventSimulator::handle_load_cmd_cancel, this, std::placeholders::_1),
            std::bind(&DiscreteEventSimulator::handle_load_cmd_accepted, this, std::placeholders::_1, name)));

        // Dump action server
        dump_action_servers_.push_back(rclcpp_action::create_server<standard_msgs::action::Dump>(
            this,
            action_dump_name,
            std::bind(&DiscreteEventSimulator::handle_dump_cmd_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DiscreteEventSimulator::handle_dump_cmd_cancel, this, std::placeholders::_1),
            std::bind(&DiscreteEventSimulator::handle_dump_cmd_accepted, this, std::placeholders::_1, name)));
    }

  return CallbackReturn::SUCCESS;
}

DiscreteEventSimulator::~DiscreteEventSimulator()
{
  RCLCPP_INFO(get_logger(), "Destroying");

    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        this->deactivate();
    }

    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        this->cleanup();
    }
}

CallbackReturn
DiscreteEventSimulator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  for (const auto& pair : metrics_pubs_) {
    pair.second->on_activate();
  }
  return CallbackReturn::SUCCESS;
}


CallbackReturn
DiscreteEventSimulator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  for (const auto& pair : metrics_pubs_) {
    pair.second->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}


CallbackReturn
DiscreteEventSimulator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  for (const auto& pair : metrics_pubs_) {
    auto pub = pair.second;
    pub.reset();
  }
  for(size_t i = 0; i < action_servers_.size(); i++){
    action_servers_[i].reset();
    load_action_servers_[i].reset();
    dump_action_servers_[i].reset();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DiscreteEventSimulator::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}


void DiscreteEventSimulator::execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle, const std::string & name){
   
    RCLCPP_INFO(get_logger(), "Executing move goal for %s", name.c_str());
    auto robot = name;
    auto num = std::remove_if(robot.begin(), robot.end(), [](char c) {
      return !std::isdigit(c);
    });
    robot.erase(num, robot.end());
    int robotID = std::stoi(robot);

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<standard_msgs::action::MoveToPose::Result>();

    auto gx = goal->target_pose.pose.position.x;
    auto gy = goal->target_pose.pose.position.y;

    // Extract yaw from quaternion
    auto qw = goal->target_pose.pose.orientation.w;
    auto qx = goal->target_pose.pose.orientation.x;
    auto qy = goal->target_pose.pose.orientation.y;
    auto qz = goal->target_pose.pose.orientation.z;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double theta = std::atan2(siny_cosp, cosy_cosp);

    std::string start_location = robots_current_locations_[name];
    std::string goal_location = find_waypoint(gx, gy, theta);

    RCLCPP_INFO(get_logger(), "Start Location: %s", start_location.c_str());
    RCLCPP_INFO(get_logger(), "Goal Location: %s", goal_location.c_str());

    if(goal_location == "-1"){
      RCLCPP_ERROR(get_logger(), "Goal location not found for pose (%f, %f, %f)", gx, gy, theta);
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "Move goal failed");
      return;
    }

    // Update robot location (discrete event: instant move)
    robots_current_locations_[name] = goal_location;

    std::this_thread::sleep_for(std::chrono::seconds(execution_sec_));

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Move goal succeeded: %s moved to %s", name.c_str(), goal_location.c_str());
}

rclcpp_action::GoalResponse DiscreteEventSimulator::handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const standard_msgs::action::MoveToPose::Goal> goal)
{
    auto goal_x = goal->target_pose.pose.position.x;
    auto goal_y = goal->target_pose.pose.position.y;
    RCLCPP_INFO(this->get_logger(), "Received move goal request with pose (%f, %f)", goal_x, goal_y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DiscreteEventSimulator::handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel move goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DiscreteEventSimulator::handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle, const std::string & server_name)
{
    using namespace std::placeholders;
    std::thread{std::bind(&DiscreteEventSimulator::execute, this, _1, server_name), goal_handle}.detach();
}


// ==================== Load Action ====================

void DiscreteEventSimulator::executeLoadCmd(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const std::string & robotID){
    auto feedback = std::make_shared<standard_msgs::action::Load::Feedback>();
    auto result = std::make_shared<standard_msgs::action::Load::Result>();
    const auto goal = goal_handle->get_goal();

    auto obj_name = goal->target;
    auto location = goal->location;
    auto name = robotID;
    double amount_to_load = capacities_[robotID];

    if (!materials_.empty()) {
      auto it = std::find_if(materials_.begin(), materials_.end(),
                        [&obj_name](const Material& mat) {
                            return mat.name_ == obj_name;
                        });

      if (it == materials_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Object %s not found", obj_name.c_str());
        result->success = false;
        result->payload = 0.0;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Action load failed");
        return;
      }

      Material material = *it;
      double available_amount = material.getAmount(location);

      if (available_amount <= 0.){
        RCLCPP_ERROR(this->get_logger(), "Object %s is not available at location %s", obj_name.c_str(), location.c_str());
        result->success = false;
        result->payload = 0.0;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Action load failed");
        return;
      }

      if(amount_to_load < 0){
        RCLCPP_ERROR(this->get_logger(), "Cannot load a negative amount!");
        result->success = false;
        result->payload = 0.0;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Action load failed");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Material %s amount %f at location %s", obj_name.c_str(), available_amount, location.c_str());
      
      if(goal->amount > 0){
        amount_to_load = std::min(amount_to_load, goal->amount);
      }

      double amount_loaded = 0.0;
      if(available_amount > amount_to_load){
        if(randomness_){
          amount_loaded = generateRandomValue(0.8 * amount_to_load, 0.2);
        } else {
          amount_loaded = amount_to_load;
        }
      } else {
        amount_loaded = available_amount;
      }

      amount_loaded_[robotID] = amount_loaded;
      RCLCPP_INFO(this->get_logger(), "Amount to load: %f", amount_loaded);

      double amount = available_amount;
      int num_steps = 5;
      double current_amount_loaded = 0.0;

      for(int step = 0; step < num_steps; step++){
        if(!refilling_){
          amount -= amount_loaded / num_steps;
        }
        current_amount_loaded += amount_loaded / num_steps;
        
        feedback->phase = "loading";
        feedback->progress = static_cast<float>(step + 1) / static_cast<float>(num_steps);
        goal_handle->publish_feedback(feedback);
        
        RCLCPP_INFO(this->get_logger(), "Loading material %s... %f", obj_name.c_str(), current_amount_loaded);
        
        material.setAmount(location, amount);
        *it = material;
      }
      result->payload = amount_loaded;
      result->success = true;
      RCLCPP_INFO(this->get_logger(), "Material %s amount %f at location %s", obj_name.c_str(), material.getAmount(location), location.c_str());
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Loading material %s from location %s", obj_name.c_str(), location.c_str());
      result->payload = 1;
      feedback->phase = "loading";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::seconds(execution_sec_));
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Action load succeeded");
}


rclcpp_action::GoalResponse DiscreteEventSimulator::handle_load_cmd_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const standard_msgs::action::Load::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request to load material");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DiscreteEventSimulator::handle_load_cmd_cancel(
    const std::shared_ptr<GoalHandleLoadCommand> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel load action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DiscreteEventSimulator::handle_load_cmd_accepted(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const std::string & robotID)
{
    using namespace std::placeholders;
    std::thread{std::bind(&DiscreteEventSimulator::executeLoadCmd, this, _1, robotID), goal_handle}.detach();
}


void DiscreteEventSimulator::executeDumpCmd(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const std::string & robotID){
    auto feedback = std::make_shared<standard_msgs::action::Dump::Feedback>();
    auto result = std::make_shared<standard_msgs::action::Dump::Result>();
    const auto goal = goal_handle->get_goal();

    auto obj_name = goal->target;
    auto location = goal->location;


    if (!materials_.empty()) {
      auto it = std::find_if(materials_.begin(), materials_.end(),
                        [&obj_name](const Material& mat) {
                            return mat.name_ == obj_name;
                        });

      if (it == materials_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Material %s not found", obj_name.c_str());
        result->success = false;
        result->payload = 0.0;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Action dump failed");
        return;
      }

      Material material = *it;
      double delta = amount_loaded_[robotID];

      if (delta <= 0){
        RCLCPP_ERROR(this->get_logger(), "Robot %s cannot dump without having loaded any material!", robotID.c_str());
        result->success = false;
        result->payload = 0.0;
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Action dump failed");
        return;
      }

      double amount = getMaterialAmount(obj_name, location);
      if(amount == -1){
        // New dump location — start from 0
        amount = 0.0;
      }

      int num_steps = 5;
      double current_amount_dumped = delta;

      for(int step = 0; step < num_steps; step++){
        amount += delta / num_steps;
        current_amount_dumped -= delta / num_steps;
        
        feedback->phase = "dumping";
        feedback->progress = static_cast<float>(step + 1) / static_cast<float>(num_steps);
        goal_handle->publish_feedback(feedback);
        
        RCLCPP_INFO(this->get_logger(), "Dumping material %s... %f remaining", obj_name.c_str(), current_amount_dumped);
        
        material.setAmount(location, amount);
        *it = material;
      }

      // Update material flow tracking
      material_flow_amount_ += delta;
      amount_loaded_[robotID] = 0;

      RCLCPP_INFO(this->get_logger(), "Dumped %f m³ of material %s at location %s", delta, obj_name.c_str(), location.c_str());
      result->payload = delta;
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Dumping material %s at location %s", obj_name.c_str(), location.c_str());
      result->payload = 1;
      feedback->phase = "dumping";
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(std::chrono::seconds(execution_sec_));
    }
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Action dump succeeded");
}


rclcpp_action::GoalResponse DiscreteEventSimulator::handle_dump_cmd_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const standard_msgs::action::Dump::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request to dump material");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DiscreteEventSimulator::handle_dump_cmd_cancel(
    const std::shared_ptr<GoalHandleDumpCommand> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel dump action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DiscreteEventSimulator::handle_dump_cmd_accepted(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const std::string & robotID)
{
    using namespace std::placeholders;
    std::thread{std::bind(&DiscreteEventSimulator::executeDumpCmd, this, _1, robotID), goal_handle}.detach();
}



void DiscreteEventSimulator::getMaterialAmountCallback(const std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Request> request,
  std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Response> response){
  auto pile_id = request->pile_id;
  auto pile_loc = request->pile_location;
  double amount = getMaterialAmount(pile_id, pile_loc);
  response->amount = amount;      
}

}