
#include <atlantis_base/rviz_visualization.hpp>
#include <Eigen/Geometry>
#include <atlantis_util/utils.h>
namespace po = boost::program_options;

namespace atlantis_base
{




void assignColor(visualization_msgs::msg::Marker &m, int color)
{
  double r, g, b = 0.;
  int color_mod = color % 4;

  switch (color_mod)
  {
    case 0:
      r = 1.;
      g = 0.;
      b = 0.;
      break;
    case 1:
      r = 0.;
      g = 1.;
      b = 0.;
      break;
    case 2:
      r = 0.;
      g = 0.;
      b = 1.;
      break;
    case 3:
      r = 1.;
      g = 0.;
      b = 1.;
      break;
    default:
      r = 0.5;
      g = 0.5;
      b = 0.5;
      break;
  };

  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.3;

  if (color < 0)
  {
    m.color.r = 1.;
    m.color.g = 1.;
    m.color.b = 1.;
    m.color.a = 1.;
  }
}


RvizVisualization::RvizVisualization()
{ 
}

RvizVisualization::~RvizVisualization()
{
}

void RvizVisualization::startVisualization()
{

  nh_ = rclcpp::Node::make_shared("rviz_visualization");
  logger_ = nh_->get_logger();
  RCLCPP_INFO(logger_, "Creating RVIZ Base Simulator");

  //Generate publisher 
  materials_marker_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("materials_visualization_markers_array", 1000);
  
}

void RvizVisualization::publishRobot(int ID, double x, double y, double theta,std::string mesh_model, int color ){


        auto marker = visualization_msgs::msg::Marker();
        if(mesh_model != "" && !mesh_model.empty()){
            assignColor(marker, color);
            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.mesh_resource = mesh_model;
            marker.scale.x = 0.8;
            marker.scale.y = 0.8;
            marker.scale.z = 0.8;
        }
        else{
            marker.type = visualization_msgs::msg::Marker::CUBE;
            assignColor(marker, color);

            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
        }
        marker.id = ID;
        marker.ns = "robot" + std::to_string (ID);
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        marker.header.frame_id = "world";

        marker.pose.position.x = x; 
        marker.pose.position.y = y; 
        marker.pose.position.z = 0.0;  // 
        auto quat = atlantis::util::eulerToQuaternion(0.,0.,theta);
        marker.pose.orientation.w = quat.w(); 
        marker.pose.orientation.x = quat.x();  
        marker.pose.orientation.y = quat.y();  
        marker.pose.orientation.z = quat.z();   

        auto it = robots_pub_.find(ID);
          if (it == robots_pub_.end()) {
          auto marker_pub = nh_->create_publisher<visualization_msgs::msg::Marker>("robot" + std::to_string(ID) + "/marker", 10); // Creates a topic with the name <robot_id>/marker
          robots_pub_[ID] = marker_pub;
          marker_pub->publish(marker);
        } else {
          auto marker_pub = it->second;
          marker_pub->publish(marker);
      }
  }


  void RvizVisualization::publishMaterial(std::string name, geometry_msgs::msg::Pose pose, int ID, double amount, double start_z, int color){
    auto material_marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = nh_->get_clock()->now();
    marker.ns = name + "_pile";
    marker.id = ID;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    double multiplier = 0.05* amount;
    float height = multiplier*0.1f;       // Height of the cone
    float base_radius = multiplier*1.0f;  // Radius of the base
    int sections = 32;         // Number of segments approximating the circular base

    // For TRIANGLE_LIST markers, the marker.scale is typically left at (1,1,1)
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Define the tip of the cone (apex)
    geometry_msgs::msg::Point tip;
    tip.x = 0;
    tip.y = 0;
    tip.z = height;

    // Generate points for the circular base (lying on z=0)
    std::vector<geometry_msgs::msg::Point> base_points;
    for (int i = 0; i < sections; ++i) {
      float angle = 2.0f * M_PI * static_cast<float>(i) / static_cast<float>(sections);
      geometry_msgs::msg::Point p;
      p.x = base_radius * std::cos(angle);
      p.y = base_radius * std::sin(angle);
      p.z = 0.0f;
      base_points.push_back(p);
    }

    // Build the lateral surface: Create a triangle for each section that connects the tip and two adjacent base vertices.
    for (int i = 0; i < sections; ++i) {
      // First vertex: tip
      marker.points.push_back(tip);
      // Second vertex: current base point
      marker.points.push_back(base_points[i]);
      // Third vertex: next base point (using modulo for wrap-around)
      marker.points.push_back(base_points[(i + 1) % sections]);
    }

    marker.pose = pose;
    marker.pose.position.z = start_z;

    marker.scale.x = 1.0;  
    marker.scale.y = 1.0; 
    marker.scale.z = amount;  // Height of the pile

    // Set the color -- be sure to set alpha to something non-zero!
    assignColor(marker, color);

    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Marker never expires
    material_marker_array.markers.push_back(marker);
    materials_marker_pub_->publish(material_marker_array);
    
  }


  void RvizVisualization::publishFootprint(double x, double y, double theta,
                                            std::vector<geometry_msgs::msg::Point> footprint,
                                            const std::string &frame_id,
                                            int id,
                                            double start_z,
                                            int color)
{
  auto marker = visualization_msgs::msg::Marker();
  marker.header.frame_id = "world";
  marker.scale.x = 0.05;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.lifetime = rclcpp::Duration(60., 0.0);
  assignColor(marker, color);
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.ns = "footprint" + std::to_string (id);
  marker.id = id;
  marker.header.frame_id = frame_id;
  auto quat = atlantis::util::eulerToQuaternion(0.,0.,theta);
  marker.pose.orientation.w = quat.w(); 
  marker.pose.orientation.x = quat.x();  
  marker.pose.orientation.y = quat.y();  
  marker.pose.orientation.z = quat.z();   

  marker.pose.position.x = x; 
  marker.pose.position.y = y; 
  marker.pose.position.z = 0.0;  // 

  geometry_msgs::msg::Point p;
  p.z = start_z;
  for (auto point: footprint)
  {
    p.x = point.x;
    p.y = point.y;
    p.z = 0;
    marker.points.push_back(p);
  }
  marker.points.push_back(marker.points[0]);  // Need to add the first one to 'close the loop'.
  auto it = robots_footprints_pubs_.find(id);
  if (it == robots_footprints_pubs_.end()) {
    auto marker_pub = nh_->create_publisher<visualization_msgs::msg::Marker>("robot" + std::to_string(id) + "/footprint_marker", 10); // Creates a topic with the name <robot_id>/marker
    robots_footprints_pubs_[id] = marker_pub;
    marker_pub->publish(marker);
  } else {
    auto marker_pub = it->second;
    marker_pub->publish(marker);
  }
}

  void  RvizVisualization::convertCostMapToMsg(const double resolution,
                            const double x_cells,
                            const double y_cells,
                            const std::vector<unsigned char> data,
                            nav_msgs::msg::OccupancyGrid& msg){

  msg.info.resolution = resolution;
  msg.info.width = x_cells;
  msg.info.height = y_cells;

  std::vector<unsigned char> occupancyMap = data;

  int size = msg.info.width*msg.info.height;
  msg.data.resize(size);
  for (int i = 0; i < size; i++)
  {
    // 0 - 254 -> convert to 0 - 100
    if (static_cast<int>(occupancyMap[i]) >= 254) {
      msg.data[i] = (char)100;
      continue;
    }
    msg.data[i] = (char)0; //occupancyMap[i];
  }
}

void RvizVisualization::publishRobotInfo(double x, double y, double theta, double distance_to_goal, double amount_loaded, const std::string &frame_id, int id){

        auto marker = visualization_msgs::msg::Marker();
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = id;
        marker.ns = "robot_info_marker";
        //marker.lifetime = rclcpp::Duration(10.0, 0.0);

        marker.scale.z = 0.3;  // Font size
        marker.color.a = 1.0;  // Alpha (transparency)
        marker.color.r = 0.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 0.0;  // Blue
        marker.header.frame_id = "map";

        marker.pose.position.x = x; 
        marker.pose.position.y = y; 
        marker.pose.position.z = 5.0;  // 
        // Construct the text string
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "Robot " << id << "\n"
          << "Position: (" << x << ", " << y << ", " << theta << ")\n"
          << "Amount loaded: " << amount_loaded << " m3 \n"
          << "Distance to Goal: " << distance_to_goal << " m";
        marker.text = ss.str();
        auto it = robots_info_pub_.find(id);
        if (it == robots_info_pub_.end()) {
          auto marker_pub = nh_->create_publisher<visualization_msgs::msg::Marker>("robot" + std::to_string(id) + "/info_marker", 10); // Creates a topic with the name <robot_id>/marker
          robots_info_pub_[id] = marker_pub;
          marker_pub->publish(marker);
        } else {
          auto marker_pub = it->second;
          marker_pub->publish(marker);
        }
}



}
