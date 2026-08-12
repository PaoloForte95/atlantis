#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>

#include "atlantis_state/state_generators/pddl_state_generator.hpp"

namespace atlantis_state
{



PddlStateGenerator::PddlStateGenerator()
{

}

PddlStateGenerator::~PddlStateGenerator()
{
  RCLCPP_INFO(
    logger_, "Destroying plugin %s of type PddlStateGenerator",
    name_.c_str());
}

void PddlStateGenerator::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;
  auto node_name = std::string(node->get_name());
  std::vector<std::string> default_ids;
  node->declare_parameter("robots", default_ids);
  node->get_parameter("robots", robots_ids_);
  
  for (size_t i = 0; i < robots_ids_.size(); ++i) {
    auto name = robots_ids_[i];
    auto current_pose_topic = name + "/current_pose";
    auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          current_pose_topic,
          rclcpp::SensorDataQoS(),
          [this, i](geometry_msgs::msg::PoseStamped msg) {
            currentPoseCallback(msg, i);
          }
        );

    position_subs_.push_back(sub);
  }

  
  waypoint_array_subs_ = node->create_subscription<location_msgs::msg::WaypointArray>(
          "waypoints",
          rclcpp::SensorDataQoS(),
          [this](location_msgs::msg::WaypointArray msg) {
            waypointArrayCallback(msg);
          }
        );


  RCLCPP_INFO(logger_, "Configuring %s of type PddlStateGenerator", name.c_str());

}

void PddlStateGenerator::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type PddlStateGenerator", name_.c_str());

}

void PddlStateGenerator::deactivate()
{
  RCLCPP_INFO( logger_, "Deactivating plugin %s of type PddlStateGenerator", name_.c_str());
}

void PddlStateGenerator::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type PddlStateGenerator", name_.c_str());
}

standard_msgs::msg::StringMultiArray PddlStateGenerator::generateState()
{

    return atlantis_state_;
}


void PddlStateGenerator::currentPoseCallback(geometry_msgs::msg::PoseStamped msg, int robotID){

  Eigen::Quaterniond quaternion;
  quaternion.x() = msg.pose.orientation.x;
  quaternion.y() = msg.pose.orientation.y;
  quaternion.z() = msg.pose.orientation.z;
  quaternion.w() = msg.pose.orientation.w;
  auto rpy = atlantis::util::quaternionToEulerAngles(quaternion);
  auto theta = rpy[2]; // Yaw
  auto nearest = findNearestWaypoint(msg.pose.position.x, msg.pose.position.y, theta);
  const std::string prefix = "(at rb" + std::to_string(robotID) + " ";
  const std::string new_state = "(at rb" + std::to_string(robotID) + " " + nearest.name + ")";

  // if already present → do nothing
  auto it = std::find(atlantis_state_.data.begin(),
                    atlantis_state_.data.end(),
                    new_state);
  if (it == atlantis_state_.data.end()) {
    // remove any existing "(at rb<robotID> ...)"
    atlantis_state_.data.erase(std::remove_if(atlantis_state_.data.begin(), atlantis_state_.data.end(),
                                              [&](const std::string& s) { return s.rfind(prefix, 0) == 0;}),atlantis_state_.data.end());

    // add the new one
    atlantis_state_.data.push_back(new_state);
  }

}

void PddlStateGenerator::waypointArrayCallback(location_msgs::msg::WaypointArray msg)
{
    RCLCPP_INFO(logger_, "Received waypoint array for robot");
    for (auto wp: msg.waypoints){
        atlantis_core::Waypoint waypoint;
        waypoint.name = wp.name;
        waypoint.x = wp.pose.position.x;
        waypoint.y = wp.pose.position.y;
        waypoints_.push_back(waypoint);
    }
    
}

atlantis_core::Waypoint PddlStateGenerator::findNearestWaypoint(double x, double y, double theta)
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

rcl_interfaces::msg::SetParametersResult
PddlStateGenerator::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
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
PLUGINLIB_EXPORT_CLASS(atlantis_state::PddlStateGenerator, atlantis_state::StateGenerator)
