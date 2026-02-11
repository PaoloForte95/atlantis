#include <atlantis_discrete_event/discrete_event_simulator.hpp>
#include "lifecycle_msgs/msg/state.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nlohmann/json.hpp"
#include <regex>
#include <fstream>

using namespace atlantis::util;

namespace atlantis_simulator
{


// Function to find a waypoint within a tolerance



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
    declare_parameter("map", "");
    declare_parameter("robots", default_ids);
    declare_parameter("materials", default_ids);
    declare_parameter("metrics", metrics_);
    declare_parameter("waypoints", default_ids);
    declare_parameter("waypoints_file", "");
    

    get_parameter("robots", robots_ids_);
    get_parameter("materials", materials_ids_);
    get_parameter("metrics", new_metrics);
    get_parameter("waypoints", waypoints_ids_);
    get_parameter("waypoints_file", waypoints_file);

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
    return "-1"; // Return -1 if no match is found
}


double DiscreteEventSimulator::getMaterialAmount(std::string pile, std::string pile_loc){
  double amount = 0;
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
            double x,y,z,yaw;
            declare_parameter(wp+".x", 0.0);
            declare_parameter(wp+".y", 0.0);
            declare_parameter(wp+".z", 0.0);
            declare_parameter(wp+".yaw", 0.0);
            get_parameter(wp+".x", x);
            get_parameter(wp+".y", y);
            get_parameter(wp+".z", z);
            get_parameter(wp+".yaw", yaw);
            Waypoint waypoint;
            waypoint.x = x;
            waypoint.y = y;
            waypoint.theta = yaw;
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
       
        for (size_t i = 0; i < material_locs.size(); ++i) {
          
          loc = material_locs[i];
          declare_parameter(name+"."+ loc + ".amount", 0.);
          get_parameter(name+"."+ loc + ".amount", amount);
          RCLCPP_INFO(get_logger(), "Got material %s at location %s with amount %f...", name.c_str(), loc.c_str(), amount);
          auto loc_copy = loc;
          int loc_ID;
          auto num = std::remove_if(loc_copy.begin(), loc_copy.end(), [](char c) {
            return !std::isdigit(c);
          });
          loc_copy.erase(num, loc_copy.end());
          if(loc_copy.empty() || loc_copy == ""){
            loc_ID = marker_id;
            marker_id += 1;
          }else{
            loc_ID = std::stoi(loc_copy);
          }
          
          location_amounts[loc] = amount;
          
        }
        
  
        material.id = ID;
        material.amounts = location_amounts;
        materials_.push_back(material);
    }
   
    for (size_t i = 0; i < robots_ids_.size(); ++i) {
        auto name = robots_ids_[i];
        int robotID;
        std::string start_location = "home" + std::to_string(robotID);
        double capacity;
        RCLCPP_INFO(get_logger(), "Configuring executor for %s...", name.c_str());

        
        declare_parameter(name+".ID", 0);
        declare_parameter(name+".start_location", start_location);
        declare_parameter(name+".capacity", 0.0);
        declare_parameter(name+".initial_pose.x", 0.0);
        declare_parameter(name+".initial_pose.y", 0.0);
        declare_parameter(name+".initial_pose.z", 0.0);
        declare_parameter(name+".initial_pose.yaw", 0.0);
        get_parameter(name+".ID", robotID);
        get_parameter(name+".start_location", start_location);
        get_parameter(name+".capacity", capacity);

        RCLCPP_INFO(get_logger(), "Initial Pose for %s... (%s)", name.c_str(), 
        start_location.c_str());

        auto action_name = "robot" + std::to_string(robotID) + "/navigate_to_pose";
        auto action_load_name = "robot" + std::to_string(robotID) + "/send_load";
        auto action_dump_name = "robot" + std::to_string(robotID) + "/send_dump";

        auto critical_point_topic = "robot" + std::to_string(robotID) + "/critical_point";
        // auto sub = create_subscription<std_msgs::msg::Int64MultiArray>(
        //   critical_point_topic, 
        //   rclcpp::SensorDataQoS(),std::bind(&DiscreteEventSimulator::criticalPointCallback, 
        //   this, std::placeholders::_1));
        material_amount_servers_.push_back(create_service<athena_exe_msgs::srv::GetMaterialAmount>( name + std::string("/get_material_amount"),
                std::bind(&DiscreteEventSimulator::getMaterialAmountCallback, this, std::placeholders::_1, std::placeholders::_2)));

      capacities_.insert(std::make_pair(robotID,capacity));
      robots_current_locations_.insert(std::make_pair(name,start_location));
      amount_loaded_.insert(std::make_pair(robotID,-1.0));

      action_servers_.push_back(rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            action_name,
            std::bind(&DiscreteEventSimulator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DiscreteEventSimulator::handle_cancel, this, std::placeholders::_1),
            std::bind(&DiscreteEventSimulator::handle_accepted, this, std::placeholders::_1, name)));


      dump_action_servers_.push_back(rclcpp_action::create_server<athena_exe_msgs::action::MoveJoint>(
          this,
          action_dump_name,
          std::bind(&DiscreteEventSimulator::handle_bucket_cmd_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&DiscreteEventSimulator::handle_bucket_cmd_cancel, this, std::placeholders::_1),
          std::bind(&DiscreteEventSimulator::handle_bucket_cmd_accepted, this, std::placeholders::_1, robotID)));
      
      load_action_servers_.push_back(rclcpp_action::create_server<athena_exe_msgs::action::MoveJoint>(
          this,
          action_load_name,
          std::bind(&DiscreteEventSimulator::handle_bucket_cmd_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&DiscreteEventSimulator::handle_bucket_cmd_cancel, this, std::placeholders::_1),
          std::bind(&DiscreteEventSimulator::handle_bucket_cmd_accepted, this, std::placeholders::_1, robotID)));


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

  for (const auto& pair : metrics_pubs_) {
  auto pub =  pair.second;
  pub.reset();
  }
  for(int i=0; i< action_servers_.size(); i++){
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

void DiscreteEventSimulator::execute(const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle, const std::string & name ){
   
   
    rclcpp::Rate r(10);
    RCLCPP_INFO(get_logger(), "Executing goal for %s ", name.c_str());
    auto robot = name;
    auto num = std::remove_if(robot.begin(), robot.end(), [](char c) {
      return !std::isdigit(c);
    });
    robot.erase(num, robot.end());
    int robotID = std::stoi(robot);

    bool can_be_executed = true;


    //Check if the action can be execute by the robot
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    //Add the goal pose
    auto quat_w = goal.get()->pose.pose.orientation.w;
    auto quat_x = goal.get()->pose.pose.orientation.x;
    auto quat_y = goal.get()->pose.pose.orientation.y;
    auto quat_z = goal.get()->pose.pose.orientation.z;
    Eigen::Quaternion q(quat_w, quat_x,quat_y,quat_z);
    
    auto euler = quaternionToEulerAngles(q);
    auto gx = goal.get()->pose.pose.position.x;
    auto gy =  goal.get()->pose.pose.position.y;
    auto theta = euler[2];

    std::string start_location = robots_current_locations_[name];
    std::string goal_location = find_waypoint(gx, gy, theta);
    RCLCPP_INFO(get_logger(), "Start Location (%s) " , start_location.c_str());
    RCLCPP_INFO(get_logger(), "Goal Location (%s) " , goal_location.c_str());

    robots_current_locations_[name] = goal_location;


    if(can_be_executed){
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }else{
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Failed");
    }        
}

  rclcpp_action::GoalResponse DiscreteEventSimulator::handle_goal(const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal)
        {
            auto goal_x = goal->pose.pose.position.x;
            auto goal_y = goal->pose.pose.position.y;
            RCLCPP_INFO(this->get_logger(), "Received goal request with pose (%f,%f)", goal_x, goal_y );
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }


 rclcpp_action::CancelResponse DiscreteEventSimulator::handle_cancel(
    const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel move goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

void DiscreteEventSimulator::handle_accepted(const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle, const std::string & server_name)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DiscreteEventSimulator::execute, this, _1, server_name), goal_handle}.detach();
}

 void DiscreteEventSimulator::executeBucketCmd(const std::shared_ptr<GoalHandleBucketCommand> goal_handle, const int robotID){
        auto feedback = std::make_shared<athena_exe_msgs::action::MoveJoint::Feedback>();
        auto result = std::make_shared<athena_exe_msgs::action::MoveJoint::Result>();
        const auto goal = goal_handle->get_goal();
        int step = 0;
      
        double delta = 0;
        auto mat_id = goal->material_id;
        auto location = goal->location;
        std::string mat_name = "mat" + std::to_string(int(mat_id));
        double amount = 0.0;
        auto it = std::find_if(materials_.begin(), materials_.end(),
                           [&mat_name](const Material& mat) {
                               return mat.name_ == mat_name;
                           });

        if (it == materials_.end()) {
           RCLCPP_INFO(this->get_logger(), "Material not found");
           goal_handle->abort(result);
        }
        auto index = std::distance(materials_.begin(), it);
        Material material = *it;
        RCLCPP_INFO(this->get_logger(), "Material %s amount %f at waypoint %s ", mat_name.c_str(), material.getAmount(location), location.c_str());
        if(goal->operation == 2){
          delta = amount_loaded_[robotID];
          if (delta == -1){
            RCLCPP_ERROR(this->get_logger(), "Robot %d cannot dump without having loaded any material!", robotID );
            goal_handle->abort(result);
          }
          amount = getMaterialAmount(mat_name, location);
          if(amount == -1){
            RCLCPP_ERROR(this->get_logger(), "Material %s not found! ", mat_name.c_str());
            goal_handle->abort(result);
          }

          int markerID = getLocationIndex(location, material.getLocations());
          if(markerID != -1){
              markerID += 1 ;
            }else{
              markerID = 1 + material.getLocations().size();
          }
          auto loc_copy = location;
          int loc_ID;
          auto num = std::remove_if(loc_copy.begin(), loc_copy.end(), [](char c) {
            return !std::isdigit(c);
          });
          loc_copy.erase(num, loc_copy.end());
          if(loc_copy.empty() || loc_copy == ""){
            loc_ID = markerID;
          }else{
            loc_ID = std::stoi(loc_copy);
          }
          while( step < 5){
            amount += delta/5;
            step+= 1;
            sleep(0.5);
          }
          amount_loaded_[robotID] = 0;
        }
        else{
          double capacity = capacities_[robotID];
          amount = getMaterialAmount(mat_name, location);
          delta = generateRandomValue(0.8*capacity,0.2);
          double amount_loaded = std::min(amount, delta);
          //To avoid loading very little material
          if(amount < capacity/2){
            amount_loaded = amount;
          }
            
          amount_loaded_[robotID] = amount_loaded;
          
          int markerID = 1 + getLocationIndex(location, material.getLocations());
          auto loc_copy = location;
          int loc_ID;
          auto num = std::remove_if(loc_copy.begin(), loc_copy.end(), [](char c) {
            return !std::isdigit(c);
          });
          loc_copy.erase(num, loc_copy.end());
          if(loc_copy.empty() || loc_copy == ""){
            loc_ID = markerID;
          }else{
            loc_ID = std::stoi(loc_copy);
          }

          while( step < 5){
              amount -= delta/5;
              step+= 1;
              sleep(0.5);
          }
        }
        material.setAmount(location,amount);
        *it = material;
        RCLCPP_INFO(this->get_logger(), "Material amount: %f", material.getAmount(location) );
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Moving bucket goal succeeded");
 }


  rclcpp_action::GoalResponse DiscreteEventSimulator::handle_bucket_cmd_goal(const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const athena_exe_msgs::action::MoveJoint::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request to move the bucket" );
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

rclcpp_action::CancelResponse DiscreteEventSimulator::handle_bucket_cmd_cancel(
    const std::shared_ptr<GoalHandleBucketCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel bucket cmd goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }


void DiscreteEventSimulator::handle_bucket_cmd_accepted(const std::shared_ptr<GoalHandleBucketCommand> goal_handle, const int robotID)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&DiscreteEventSimulator::executeBucketCmd, this, _1, robotID), goal_handle}.detach();
}

void DiscreteEventSimulator::getMaterialAmountCallback(const std::shared_ptr<athena_exe_msgs::srv::GetMaterialAmount::Request> request,
  std::shared_ptr<athena_exe_msgs::srv::GetMaterialAmount::Response> response){
  auto pile_id = request->pile_id;
  auto pile_loc = request->pile_location;
  double amount = getMaterialAmount(pile_id,pile_loc);
  response->amount = amount;      
}

}