#include <atlantis_base/base_simulator.hpp>
#include "lifecycle_msgs/msg/state.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nlohmann/json.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace atlantis::util;
using namespace navigo;
namespace atlantis
{

bool makeFootprintFromString(
  const std::string & footprint_string,
  std::vector<geometry_msgs::msg::Point> & footprint)
{
  std::string error;
  std::vector<std::vector<float>> vvf = atlantis::util::parseVVF(footprint_string, error);

  if (error != "") {
    RCLCPP_ERROR(rclcpp::get_logger("atlantis_base_simulator"), "Error parsing footprint parameter: %s. Footprint string was %s", error.c_str(), footprint_string.c_str());
    return false;
  }

  if (vvf.size() < 3) {
    RCLCPP_ERROR(rclcpp::get_logger("atlantis_base_simulator"),"You must specify at least three points for the robot footprint!");
    return false;
  }
  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      geometry_msgs::msg::Point point;
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      point.z = 0;
      footprint.push_back(point);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("atlantis_base_simulator"),"Points in the footprint specification must be pairs of numbers. Found a point with %d numbers.", static_cast<int>(vvf[i].size()));
      return false;
    }
  }

  return true;
}


// Function to read the first line of a file
Pose getFirstLine(const std::string& filePath) {
    std::ifstream file(filePath);
    if (file.is_open()) {
        std::string line;
        if (std::getline(file, line)) {
            file.close();
            std::stringstream ss(line);
            Pose wp;
            ss >> wp.x >> wp.y >> wp.theta;
            return wp;
        }
        file.close();
    }
    throw std::runtime_error("Could not read the first line of file: " + filePath);
}

// Function to read the last line of a file
Pose getLastLine(const std::string& filePath) {
    std::ifstream file(filePath);
    if (file.is_open()) {
        std::string lastLine;
        std::string currentLine;
        while (std::getline(file, currentLine)) {
            lastLine = currentLine;
        }
        file.close();
        if (!lastLine.empty()) {
            std::stringstream ss(lastLine);
            Pose wp;
            ss >> wp.x >> wp.y >> wp.theta;
            return wp;
        }
    }
    throw std::runtime_error("Could not read the last line of file: " + filePath);
}


// Function to find the path file given a start and goal waypoint
std::string findPathFile(const std::string& folderPath, navigo::Pose start, navigo::Pose goal) {
    for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            try {
                Pose fileStart = getFirstLine(entry.path().string());
                Pose fileGoal = getLastLine(entry.path().string());
                if (fileStart == start && fileGoal == goal) {
                   std::cout << "Found Path " << entry.path().string() << std::endl;
                    return entry.path().string();
                }
            } catch (const std::runtime_error& e) {
                std::cerr << "Error processing file " << entry.path() << ": " << e.what() << std::endl;
                continue; // Skip to the next file if there's an error reading this one
            }
        }
    }
    std::cerr << "No Matching path is found!" << std::endl;
    return ""; // Return an empty string if no matching path is found
}


navigo::Path loadPath(const std::string& filePath) {
    navigo::Path path;
    std::ifstream file(filePath);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            navigo::Pose pose;
            ss >> pose.x >> pose.y >> pose.theta;
            path.push_back(pose);
            //path.insert(path.begin(), pose);
        }
        file.close();
    } else {
        throw std::runtime_error("Could not open file for loading path: " + filePath);
    }
    return path;
}


double getHeigth(std::string mat_name, std::string location, std::vector<Material> materials){
  double height = 0;
      for(auto material: materials){
      if (material.name_ != mat_name){
        auto locs = material.getLocations();
        if(std::find(locs.begin(), locs.end(), location) != locs.end()){
          auto current_amount = material.getAmount(location);
          height +=current_amount;
      }
    }
  }
  return height;
}

void convertPathToMsg(const navigo::Path &path, nav_msgs::msg::Path& msg){
  geometry_msgs::msg::PoseStamped msg_pose;
  for(auto pose : path){
    msg_pose.header = msg.header;
    msg_pose.pose.position.x = pose.x;
    msg_pose.pose.position.y = pose.y;
    msg_pose.pose.position.z = 0;
    Eigen::Quaterniond quat = rpyToQuaternion(0,0,pose.theta);
    msg_pose.pose.orientation.x = quat.x();
    msg_pose.pose.orientation.y = quat.y();
    msg_pose.pose.orientation.z = quat.z();
    msg_pose.pose.orientation.w = quat.w();
    msg.poses.push_back(msg_pose);
  }
}

int getLocationIndex(std::string location, std::vector<std::string> material_locations){
  auto it = std::find(material_locations.begin(), material_locations.end(), location);
  if(it == material_locations.end()){
    return -1;
  }
  return std::distance(material_locations.begin(), it);
}


double calculateDistance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}


navigo::MotionModel fromString(const std::string & n)
{
  if (n == "2D") {
    return navigo::MotionModel::TWOD;
  } else if (n == "DUBIN") {
    return navigo::MotionModel::DUBIN;
  } else if (n == "REEDS_SHEPP") {
    return navigo::MotionModel::REEDS_SHEPP;
  } else if (n == "STATE_LATTICE") {
    return navigo::MotionModel::STATE_LATTICE;
  } else {
    return navigo::MotionModel::UNKNOWN;
  }
}

/**
 * @brief Construct a new Base Simulator:: Base Simulator object
 * 
 * @param node_name 
 * @param ns 
 * @param options 
 */
BaseSimulator::BaseSimulator(
  const std::string & node_name,
  const std::string & ns,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, ns, options)
{
    std::string waypoints_file, scenario;
    std::vector<std::string> default_ids, new_metrics;
    metrics_ = {"execution_time", "action_time"};
    declare_parameter("scenario", "empty");
    declare_parameter("use_precomputed_paths", false);
    declare_parameter("precomputed_paths_folder", "");
    declare_parameter("map", "empty");
    declare_parameter("robots", default_ids);
    declare_parameter("materials", default_ids);
    declare_parameter("goal_tolerance", 0.5);
    declare_parameter("waypoints", default_ids);
    declare_parameter("waypoints_file", "");
    declare_parameter("metrics", metrics_);
    declare_parameter("planners", default_ids);
    declare_parameter("max_planning_time", 10.0);
    //Simulation Parameters
    declare_parameter("dt", 0.1);
    declare_parameter("real_time_factor", 2.);
    declare_parameter("max_sim_time", -1.);
    declare_parameter("refilling", false);
    declare_parameter("randomness", false);

    get_parameter("scenario", scenario);
    get_parameter("use_precomputed_paths", use_precomputed_paths_);
    get_parameter("precomputed_paths_folder", precomputed_paths_folder_);
    get_parameter("map", map_yaml_);
    get_parameter("robots", robots_ids_);
    get_parameter("materials", materials_ids_);
    get_parameter("waypoints", waypoints_ids_);
    get_parameter("waypoints_file", waypoints_file);
    get_parameter("metrics", new_metrics);
    get_parameter("planners", planners_ids_);
    get_parameter("goal_tolerance", goal_tolerance_);
    get_parameter("max_planning_time", max_planning_time_);
    //Simulation Parameters
    get_parameter("dt", dt_);
    get_parameter("real_time_factor", real_time_factor_);
    get_parameter("max_sim_time", max_sim_time_);
    get_parameter("refilling", refilling_);
    get_parameter("randomness", randomness_);
    


    auto collection_dir = ament_index_cpp::get_package_share_directory("atlantis_collection");
    
    map_dir_ = collection_dir + "/scenarios/" + scenario + "/";

    auto waypoint_dir = collection_dir + "/scenarios/" + scenario + "/";
    waypoints_file = waypoint_dir + waypoints_file;


    //Create the pubs
    for (size_t i = 0; i < robots_ids_.size(); ++i) {
      auto name = robots_ids_[i];
      auto current_pose_topic = name + "/current_pose";
      auto path_topic = name + "/path";
      auto current_path_idx = name + "/current_path_idx";
      robots_current_pose_.insert(std::make_pair(name,create_publisher<geometry_msgs::msg::PoseStamped>(current_pose_topic,10)));
      robots_paths_.insert(std::make_pair(name,create_publisher<nav_msgs::msg::Path>(path_topic,10)));
      current_path_idx_pubs_.insert(std::make_pair(name,create_publisher<std_msgs::msg::Int64>(current_path_idx,10)));
    }

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    if(waypoints_file != "" && !waypoints_file.empty() && waypoints_ids_.empty()){
      waypoints_ = parseWaypoints(waypoints_file);
    }

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
     material_flow_pub_ =  create_publisher<material_handler_msgs::msg::MaterialFlow>("material_flow",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    last_real_time_ = system_clock.now().seconds();
    timer_sim_time_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(this->dt_ / this->real_time_factor_ * 1000)),
            std::bind(&BaseSimulator::publishClock, this)); 
    timer_material_flow_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&BaseSimulator::publishMaterialFlow, this)); 
    timer_robot_pose_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BaseSimulator::publishRobotPose, this)); 
    timer_material_publisher_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&BaseSimulator::publishMaterial, this));

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
    waypoint_list_server_ = create_service<location_msgs::srv::GetWaypointList>(std::string("/get_waypoint_list"),
                std::bind(&BaseSimulator::getWaypointList, this, std::placeholders::_1, std::placeholders::_2));
    
}


geometry_msgs::msg::Pose BaseSimulator::getWaypoint(std::string wp){
  geometry_msgs::msg::Pose location;
  for(auto waypoint:waypoints_){
    if (waypoint.name == wp){
      location.position.x = waypoint.x;
      location.position.y = waypoint.y;
    }
  }
  return location;
}


double BaseSimulator::getMaterialAmount(std::string pile, std::string pile_loc){
  double amount = 0;
  for(auto mat: materials_){
    if (mat.name_ == pile){
      return mat.getAmount(pile_loc);
    }
  }  
  return -1;  
}

CallbackReturn
BaseSimulator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring executor...");
     std::vector<std::string> default_ids, material_locs;
    rviz_viz_.startVisualization();

    oc_ = new navigo::CostMap(map_dir_ + map_yaml_);

    RCLCPP_INFO(get_logger(), "Map info... %s", oc_->getDebugString().c_str());

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
   
    int color = 1;
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
          auto wp = getWaypoint(loc);
          double height = getHeigth(name,loc,materials_);
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
          //rviz_viz_.publishMaterial(name, wp, loc_ID, amount, height, color);
          materials_viz_.insert(std::make_pair(name,color));
          
          location_amounts[loc] = amount;
          
        }
        color +=1;
        
  
        material.id = ID;
        material.amounts = location_amounts;
        materials_.push_back(material);
    }
   
    for (size_t i = 0; i < robots_ids_.size(); ++i) {
        auto name = robots_ids_[i];
        int robotID;
        std::string model, footprint_points;
        double capacity, minimum_turning_radius;
        RCLCPP_INFO(get_logger(), "Configuring executor for %s...", name.c_str());

        navigo::Pose start_pose;

        declare_parameter(name+".ID", 0);
        declare_parameter(name+".footprint", rclcpp::ParameterValue(std::string("[[1.0, -1.0], [1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0]]")));
        declare_parameter(name+".initial_pose.x", 0.0);
        declare_parameter(name+".initial_pose.y", 0.0);
        declare_parameter(name+".initial_pose.z", 0.0);
        declare_parameter(name+".initial_pose.yaw", 0.0);
        declare_parameter(name+".model", "");
        declare_parameter(name+".minimum_turning_radius", 1.0);
        declare_parameter(name+".capacity", 1.0);
        get_parameter(name+".ID", robotID);
        get_parameter(name+".initial_pose.x", start_pose.x);
        get_parameter(name+".initial_pose.y", start_pose.y);
        get_parameter(name+".initial_pose.yaw", start_pose.theta);
        get_parameter(name+".model", model);
        get_parameter(name+".footprint", footprint_points);
        get_parameter(name+".capacity", capacity);
        get_parameter(name+".minimum_turning_radius", minimum_turning_radius);

        RCLCPP_INFO(get_logger(), "Initial Pose for %s... (%f, %f, %f)",name.c_str(), 
        start_pose.x,
        start_pose.y,
        start_pose.theta);

        start_poses_.insert(std::make_pair(name, start_pose));
        current_poses_.insert(std::make_pair(name, start_pose));
        critical_points_.insert(std::make_pair(robotID,-1));
        path_index_.insert(std::make_pair(name,-1));

        auto action_name = "robot" + std::to_string(robotID) + "/navigate_to_pose";
        auto action_load_name = "robot" + std::to_string(robotID) + "/send_load";
        auto action_dump_name = "robot" + std::to_string(robotID) + "/send_dump";

        auto critical_point_topic = "robot" + std::to_string(robotID) + "/critical_point";
        
        


      std::vector<geometry_msgs::msg::Point> footprint;
      if (footprint_points != "" && footprint_points != "[]") {
        if (!makeFootprintFromString(footprint_points, footprint)) {
            RCLCPP_ERROR( get_logger(), "The footprint parameter is invalid: \"%s\"!",footprint_points.c_str() );
        } 
      robot_footprints_.insert(std::make_pair(name,footprint));
      }

      rviz_viz_.publishRobot(robotID, start_pose.x, start_pose.y, start_pose.theta, model);
      rviz_viz_.publishFootprint(start_pose.x, start_pose.y,start_pose.theta,footprint,"world",robotID,0,2);
      models_.insert(std::make_pair(robotID,model));
      capacities_.insert(std::make_pair(robotID,capacity));
      amount_loaded_.insert(std::make_pair(robotID,-1.0));
      navigo::SearchParams search_info;
      navigo::PlannerParams planning_params;
      planning_params.tolerance = goal_tolerance_;
      planning_params.max_planning_time = max_planning_time_;
      search_info.minimum_turning_radius = minimum_turning_radius;
      auto car_planner = new navigo::CarPlanner("CarPlanner", navigo::MotionModel::REEDS_SHEPP, search_info,planning_params, navigo::PlanningAlgorithm::RRTstar);

      base_planners_.insert(std::make_pair(robotID,car_planner));
      
      auto sub = this->create_subscription<std_msgs::msg::Int64>(
        critical_point_topic,
        rclcpp::SensorDataQoS(),
        [this, robotID](std_msgs::msg::Int64 msg) {
          criticalPointCallback(msg, robotID);
        }
      );

        critical_points_subs_.push_back(sub);
        action_servers_.push_back(rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            action_name,
            std::bind(&BaseSimulator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BaseSimulator::handle_cancel, this, std::placeholders::_1),
            std::bind(&BaseSimulator::handle_accepted, this, std::placeholders::_1, name)));

        dump_action_servers_.push_back(rclcpp_action::create_server<material_handler_msgs::action::DumpMaterial>(
            this,
            action_dump_name,
            std::bind(&BaseSimulator::handle_dump_cmd_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BaseSimulator::handle_dump_cmd_cancel, this, std::placeholders::_1),
            std::bind(&BaseSimulator::handle_dump_cmd_accepted, this, std::placeholders::_1, robotID)));
        
        load_action_servers_.push_back(rclcpp_action::create_server<material_handler_msgs::action::LoadMaterial>(
            this,
            action_load_name,
            std::bind(&BaseSimulator::handle_load_cmd_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BaseSimulator::handle_load_cmd_cancel, this, std::placeholders::_1),
            std::bind(&BaseSimulator::handle_load_cmd_accepted, this, std::placeholders::_1, robotID)));


    }

    material_amount_server_ = create_service<material_handler_msgs::srv::GetMaterialAmount>(std::string("/get_material_amount"),
                std::bind(&BaseSimulator::getMaterialAmountCallback, this, std::placeholders::_1, std::placeholders::_2));

    nav_msgs::msg::OccupancyGrid map_msg;
    rviz_viz_.convertCostMapToMsg(oc_->getResolution(), oc_->getSizeInCellsX(),oc_->getSizeInCellsY(), oc_->getData() , map_msg);
    map_msg.header.frame_id = "map";
    map_pub_->publish(map_msg);
    
  return CallbackReturn::SUCCESS;
}

BaseSimulator::~BaseSimulator()
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
BaseSimulator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  map_pub_->on_activate();
  clock_pub_->on_activate();
  material_flow_pub_->on_activate();
   for (const auto& pair : robots_current_pose_) {
      pair.second->on_activate();
   }
   for (const auto& pair : robots_paths_) {
      pair.second->on_activate();
   }
   for (const auto& pair : current_path_idx_pubs_) {
      pair.second->on_activate();
   }
   for (const auto& pair : metrics_pubs_) {
      pair.second->on_activate();
   }
  return CallbackReturn::SUCCESS;
}


CallbackReturn
BaseSimulator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  map_pub_->on_deactivate();
  clock_pub_->on_deactivate();
  material_flow_pub_->on_deactivate();
  for (const auto& pair : robots_current_pose_) {
      pair.second->on_deactivate();
   }
   for (const auto& pair : robots_paths_) {
      pair.second->on_deactivate();
   }
   for (const auto& pair : current_path_idx_pubs_) {
      pair.second->on_deactivate();
   }
    for (const auto& pair : metrics_pubs_) {
      pair.second->on_deactivate();
   }
  return CallbackReturn::SUCCESS;
}


CallbackReturn
BaseSimulator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  map_pub_.reset();
  clock_pub_.reset();
  material_flow_pub_.reset();
  for (const auto& pair : robots_current_pose_) {
    auto pub =  pair.second;
    pub.reset();
   }
  for (const auto& pair : robots_paths_) {
    auto pub =  pair.second;
    pub.reset();
   }
  for (const auto& pair : current_path_idx_pubs_) {
    auto pub =  pair.second;
    pub.reset();
   }
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
BaseSimulator::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

void BaseSimulator::execute(const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle, const std::string & name ){
   
   
    RCLCPP_INFO(get_logger(), "Executing goal for %s ", name.c_str());
    auto robot = name;
    auto num = std::remove_if(robot.begin(), robot.end(), [](char c) {
      return !std::isdigit(c);
    });
    robot.erase(num, robot.end());
    int robotID = std::stoi(robot);


    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    //Add the goal pose
    navigo::Pose goal_pose;
    auto quat_w = goal.get()->pose.pose.orientation.w;
    auto quat_x = goal.get()->pose.pose.orientation.x;
    auto quat_y = goal.get()->pose.pose.orientation.y;
    auto quat_z = goal.get()->pose.pose.orientation.z;
    Eigen::Quaternion q(quat_w, quat_x,quat_y,quat_z);
    
    auto euler = quaternionToEulerAngles(q);
    goal_pose.x = goal.get()->pose.pose.position.x;
    goal_pose.y = goal.get()->pose.pose.position.y;
    goal_pose.theta = euler[2];
    auto footprint = robot_footprints_[name];
    std::vector<double> xcoords, ycoords;
    double fp = 0.5;
    for (auto point: footprint){
      xcoords.push_back(point.x);
      ycoords.push_back(point.y);
    }
    navigo::Pose start_pose ;
    auto it = start_poses_.find(name);
    if (it != start_poses_.end()) {
        start_pose = it->second;
    } else {
        RCLCPP_ERROR(get_logger(), "Start Pose not found for %s " , name.c_str());
    }
    RCLCPP_INFO(get_logger(), "Start Pose (%f, %f, %f) " , start_pose.x, start_pose.y, start_pose.theta);
    RCLCPP_INFO(get_logger(), "Goal Pose (%f, %f, %f) " , goal_pose.x, goal_pose.y, goal_pose.theta);
    navigo::Pose current_pose;
    std::map<std::string, std::vector<atlantis::util::MetricData>> results;
    auto planner = base_planners_[robotID];
    //std::vector<atlantis::util::MetricData> metrics_results;
    auto start_action_time = std::chrono::high_resolution_clock::now();
    std::unique_ptr<navigo::GridCollisionChecker> checker = std::make_unique<navigo::GridCollisionChecker>(oc_);
    auto front = navigo::Footprint(xcoords, ycoords);
    checker->setFootprint(front);
    planner->setCollisionChecker(checker.get());

    navigo::Path path;
    if(use_precomputed_paths_){
      RCLCPP_INFO(get_logger(), "Using precomputed paths!");
      std::string foundFilePath = findPathFile(precomputed_paths_folder_, start_pose, goal_pose);
      path = loadPath(foundFilePath);
    }
    else {
      path = planner->computePath(start_pose,goal_pose);
    }

    int pathIndex = 0;

    nav_msgs::msg::Path plan;
    plan.header = goal.get()->pose.header;
    convertPathToMsg(path,plan);

    auto path_pub = robots_paths_[name];
    path_pub->publish(plan);

    double dt = this->dt_;
    rclcpp::Rate r((1./dt)*real_time_factor_);
    geometry_msgs::msg::PoseStamped pose;
    auto start_exe_time = std::chrono::high_resolution_clock::now();
    while (pathIndex < path.size()) {

      bool incr = true;
      current_pose = path[pathIndex];
      RCLCPP_DEBUG(get_logger(), "Current Pose (%f, %f, %f) " , current_pose.x, current_pose.y, current_pose.theta);
      rviz_viz_.publishRobot(robotID, current_pose.x, current_pose.y, current_pose.theta, models_[robotID]);
      auto distance_to_goal = calculateDistance(goal_pose.x, goal_pose.y, current_pose.x, current_pose.y);
      double amount_loaded = amount_loaded_[robotID];
      if(amount_loaded < 0){
        amount_loaded = 0;
      }
      rviz_viz_.publishRobotInfo(current_pose.x, current_pose.y, current_pose.theta,distance_to_goal,amount_loaded,"map",robotID);
      rviz_viz_.publishFootprint(current_pose.x, current_pose.y,current_pose.theta,footprint,"map",robotID,0,2);

      
      //Check for the critcialPoint
      int critical_point = critical_points_[robotID];
      if (critical_point >= 0)
      {
        if (pathIndex >= critical_point)
        {
          incr = false;
        }
      }
      
      {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        current_poses_[name] = current_pose;
        path_index_[name] = pathIndex; 
      }

      if(incr){
        pathIndex ++;
      }

      r.sleep();
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_action = end_time - start_exe_time;
    //plan_execution_time_ += duration_action.count();
      //Publish the metrics
      // for(auto metric: metrics_){
      //   atlantis::util::MetricData data;
      //   data.name = metric;
      //   data.value = 0;
      //   if(metrics_to_evaluate_[metric]){
      //     auto metric_msg = std_msgs::msg::Float64();
      //     auto metric_pub = metrics_pubs_ [metric];
      //     double value = -1;
      //     if(metric == "computational_time"){
      //       value = planner->getComputationalTime();
      //     }
      //     else if(metric == "path_length"){
      //       value = planner->getPathLength();
      //     }
      //     else if(metric == "action_time"){
      //       auto end_time = std::chrono::high_resolution_clock::now();
      //       std::chrono::duration<double> duration_action = end_time - start_action_time;
      //       value = duration_action.count();
      //     }
      //     data.value = value;
      //     metric_msg.data = value;
      //     metric_pub->publish(metric_msg);
      //   }
      //   //metrics_results.push_back(data);
      // }
    //results.insert(std::make_pair(planner->getName(),metrics_results));
    

    // std::string filename = "metrics_" + name +".csv";
    // atlantis::util::saveMetricsToCSV(filename, metrics_, results);

    //     for(auto metric: metrics_){
    //   if(metric == "execution_time"){
    //         auto metric_msg = std_msgs::msg::Float64();
    //         auto metric_pub = metrics_pubs_ ["execution_time"];
    //         //metric_msg.data = plan_execution_time_;
    //         atlantis::util::MetricData data;
    //         data.name = metric;
    //         //data.value = plan_execution_time_;
    //         std::vector<atlantis::util::MetricData> metrics_data;
    //         metrics_data.push_back(data);
    //         atlantis::util::appendRowToCSV(filename,get_name(),metrics_data,metrics_);
    //         }
    // }
  

    auto dist = calculateDistance(goal_pose.x, goal_pose.y, current_pose.x, current_pose.y);
    start_pose.x = current_pose.x;
    start_pose.y = current_pose.y;
    start_pose.theta = current_pose.theta;
    start_poses_[name] = start_pose;


    if(dist < goal_tolerance_){
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }else{
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "Goal Failed");
    }        
}

  rclcpp_action::GoalResponse BaseSimulator::handle_goal(const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal)
        {
            auto goal_x = goal->pose.pose.position.x;
            auto goal_y = goal->pose.pose.position.y;
            RCLCPP_INFO(this->get_logger(), "Received goal request with pose (%f,%f)", goal_x, goal_y );
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }


 rclcpp_action::CancelResponse BaseSimulator::handle_cancel(
    const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel move goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

void BaseSimulator::handle_accepted(const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle, const std::string & server_name)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&BaseSimulator::execute, this, _1, server_name), goal_handle}.detach();
}

void BaseSimulator::executeLoadCmd(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const int robotID){
        auto feedback = std::make_shared<material_handler_msgs::action::LoadMaterial::Feedback>();
        auto result = std::make_shared<material_handler_msgs::action::LoadMaterial::Result>();
        const auto goal = goal_handle->get_goal();
        int step = 0;
      
        auto mat_name = goal->name;
        auto location = goal->location;
        auto name = "robot" + std::to_string(robotID);
        auto current_location = start_poses_[name];
        double amount_to_load = capacities_[robotID];
        double amount_loaded = 0.;
        auto wp = getWaypoint(location);
        auto it = std::find_if(materials_.begin(), materials_.end(),
                           [&mat_name](const Material& mat) {
                               return mat.name_ == mat_name;
                           });

        if (it == materials_.end()) {
           RCLCPP_INFO(this->get_logger(), "Material not found");
           goal_handle->abort(result);
           RCLCPP_ERROR(this->get_logger(), "Action load failed");
          return;
        }
        if(amount_to_load < 0){
          RCLCPP_ERROR(this->get_logger(), "Cannot load a negative amount!");
          goal_handle->abort(result);
          RCLCPP_ERROR(this->get_logger(), "Action load failed");
          return;
        }
        Material material = *it;
        if (material.getAmount(location) <= 0.){
          RCLCPP_ERROR(this->get_logger(), "Material %s is not at waypoint %s ", mat_name.c_str(), location.c_str());
          goal_handle->abort(result);
          RCLCPP_ERROR(this->get_logger(), "Action load failed");
          return;
        }

        RCLCPP_INFO(this->get_logger(), "Material %s amount %f at waypoint %s ", mat_name.c_str(), material.getAmount(location), location.c_str());
        
        
        if(goal->amount > 0){
          amount_to_load = std::min(amount_to_load, goal->amount);
        }

        double amount = getMaterialAmount(mat_name, location);

        if(amount > amount_loaded){
          if(randomness_){
            amount_loaded = generateRandomValue(0.8*amount_to_load,0.2);
            }
          else{
            amount_loaded = amount_to_load;
          }
        }else {
          amount_loaded = amount;
        } 
        feedback->amount_loaded = amount_loaded;
        amount_loaded_[robotID] = amount_loaded;
        RCLCPP_INFO(this->get_logger(), "Amount loaded: %f", amount_loaded);
        double current_amount_loaded = 0.0;
        int num_step = 5;
        double dt = this->dt_;
        rclcpp::Rate r((1./dt)*real_time_factor_);
        while( step < num_step){
            if(!refilling_){
              amount -= amount_loaded/num_step;
            }
            current_amount_loaded += amount_loaded/num_step;
            RCLCPP_INFO(this->get_logger(), "Loading material...%f", current_amount_loaded);
            rviz_viz_.publishRobotInfo(current_location.x, current_location.y, current_location.theta,0.0,current_amount_loaded,"map",robotID);
            step+= 1;
            material.setAmount(location,amount);
            *it = material;
            r.sleep();
        }
        
        RCLCPP_INFO(this->get_logger(), "Material %s amount %f at waypoint %s ", mat_name.c_str(), material.getAmount(location), location.c_str());
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action load succeeded");
    
 }


void BaseSimulator::executeDumpCmd(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const int robotID){
        auto feedback = std::make_shared<material_handler_msgs::action::DumpMaterial::Feedback>();
        auto result = std::make_shared<material_handler_msgs::action::DumpMaterial::Result>();
        const auto goal = goal_handle->get_goal();
        int step = 0;
      
        double delta = 0;
        auto mat_name = goal->name;
        auto location = goal->location;
        double amount = 0.0;
        auto wp = getWaypoint(location);
        auto it = std::find_if(materials_.begin(), materials_.end(),
                           [&mat_name](const Material& mat) {
                               return mat.name_ == mat_name;
                           });

        if (it == materials_.end()) {
           RCLCPP_ERROR(this->get_logger(), "Material not found");
           goal_handle->abort(result);
           RCLCPP_ERROR(this->get_logger(), "Action dump failed");
           return;
        }

        Material material = *it;
        delta = amount_loaded_[robotID];
        feedback->amount_dumped = delta;
          if (delta == -1){
            RCLCPP_ERROR(this->get_logger(), "Robot %d cannot dump without having loaded any material!", robotID );
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Action dump failed");
            return;
          }
          amount = getMaterialAmount(mat_name, location);
          int num_step = 5;
          double current_amount_dumped = delta;
          double dt = this->dt_;
          rclcpp::Rate r((1./dt)*real_time_factor_);
          while( step < num_step){
            amount += delta/num_step;
            current_amount_dumped -= delta/num_step;
            RCLCPP_INFO(this->get_logger(), "Dumping material %s...%f", mat_name.c_str(), current_amount_dumped);
            step+= 1;
            material.setAmount(location,amount);
             *it = material;
            r.sleep();
          }
        //Update the material_flow_amount   
        material_flow_amount_ += delta;
        amount_loaded_[robotID] = 0;
        
        RCLCPP_INFO(this->get_logger(), "Dump %f m³ of material %s at waypoint %s ", mat_name.c_str(), material.getAmount(location), location.c_str());
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Action dump succeeded");
 }




  rclcpp_action::GoalResponse BaseSimulator::handle_load_cmd_goal(const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const material_handler_msgs::action::LoadMaterial::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request to load material" );
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

    rclcpp_action::GoalResponse BaseSimulator::handle_dump_cmd_goal(const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const material_handler_msgs::action::DumpMaterial::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request to dump material" );
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

rclcpp_action::CancelResponse BaseSimulator::handle_load_cmd_cancel(
    const std::shared_ptr<GoalHandleLoadCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel load action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse BaseSimulator::handle_dump_cmd_cancel(
    const std::shared_ptr<GoalHandleDumpCommand> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel dump action");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }


void BaseSimulator::handle_dump_cmd_accepted(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const int robotID)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&BaseSimulator::executeDumpCmd, this, _1, robotID), goal_handle}.detach();
}


void BaseSimulator::handle_load_cmd_accepted(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const int robotID)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&BaseSimulator::executeLoadCmd, this, _1, robotID), goal_handle}.detach();
}


void BaseSimulator::getMaterialAmountCallback(const std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Request> request,
  std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Response> response){
  auto pile_id = request->pile_id;
  auto pile_loc = request->pile_location;
  double amount = getMaterialAmount(pile_id,pile_loc);
  response->amount = amount;      
}

void BaseSimulator::criticalPointCallback(std_msgs::msg::Int64 msg, int robotID)
{
    auto it = critical_points_.find(robotID);
    if (it != critical_points_.end()) {
      critical_points_[it->first] = msg.data;
    } else {
       critical_points_[it->first] = -1;
    }
}

void BaseSimulator::publishClock()
{
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  double now = system_clock.now().seconds(); // Note: this->now() - will report the published /clock time below
  double real_time_elapsed = now - last_real_time_;

  sim_time_ += real_time_elapsed * real_time_factor_;
  last_real_time_ = now;

  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = rclcpp::Time(static_cast<int64_t>(sim_time_ * 1e9));
  clock_pub_->publish(clock_msg);
  if (sim_time_ >= max_sim_time_ && max_sim_time_ != -1.){
    RCLCPP_INFO(get_logger(), "Max simulation time elapsed, shutting down the simulator...");
    rclcpp::shutdown();
  }
}

void BaseSimulator::publishRobotPose(){
  {
  std::lock_guard<std::mutex> lock(poses_mutex_);
  for(auto robot : current_poses_){
      auto current_pose = robot.second;
      auto pose = geometry_msgs::msg::PoseStamped();
      pose.header.frame_id = "map";
      pose.header.stamp.sec = sim_time_;
      pose.pose.position.x = current_pose.x;
      pose.pose.position.y = current_pose.y;
      pose.pose.position.z = 0;

      Eigen::Quaterniond quat = Eigen::AngleAxisd(current_pose.theta, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
      pose.pose.orientation.w = quat.w(); 
      pose.pose.orientation.x = quat.x();  
      pose.pose.orientation.y = quat.y();  
      pose.pose.orientation.z = quat.z(); 
      auto robot_pub = robots_current_pose_[robot.first];
      robot_pub->publish(pose);
      auto msg = std_msgs::msg::Int64();
      msg.data = path_index_[robot.first];
      current_path_idx_pubs_[robot.first]->publish(msg);
  }
}
     
}

void BaseSimulator::publishMaterial(){
  //Publish the material
  int markerID = 0;
  int locationID = 0;
  for(auto material : materials_){
    auto mat_name = material.name_;
    for (auto location : material.getLocations()){
     
      auto ID = markerID*material.getLocations().size() + locationID;
      double amount = getMaterialAmount(mat_name, location);
      double height = getHeigth(mat_name, location,materials_);
      auto wp = getWaypoint(location);
      rviz_viz_.publishMaterial(mat_name, wp, ID, amount, height, materials_viz_[mat_name]);
      locationID++;
    }
  markerID++;
  }
}

void BaseSimulator::getWaypointList(const std::shared_ptr<location_msgs::srv::GetWaypointList::Request> request, std::shared_ptr<location_msgs::srv::GetWaypointList::Response> response){
  location_msgs::msg::Waypoint wp;
  for(auto waypoint:waypoints_){
    wp.name = waypoint.name;
    wp.pose.position.x = waypoint.x;
    wp.pose.position.y = waypoint.y;
    auto quat = atlantis::util::eulerToQuaternion(0,0,waypoint.theta);
    wp.pose.orientation.w = quat.w(); 
    wp.pose.orientation.x = quat.x();  
    wp.pose.orientation.y = quat.y();  
    wp.pose.orientation.z = quat.z();   
    response->list.push_back(wp);
  }
}


void BaseSimulator::publishMaterialFlow(){
  //Publish the material amount flow
  auto flow_msg = material_handler_msgs::msg::MaterialFlow();
  flow_msg.flow = material_flow_amount_; // m³
  flow_msg.stamp.sec = sim_time_; //s
  material_flow_pub_->publish(flow_msg);
}

}