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


void PddlStateGenerator::currentPoseCallback(geometry_msgs::msg::PoseStamped msg, int robotID)
{
    atlantis::util::Waypoint nearest = atlantis::util::getNearestWaypoint(waypoints_, msg.pose.position.x, msg.pose.position.y);
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
        atlantis::util::Waypoint waypoint;
        waypoint.name = wp.name;
        waypoint.x = wp.pose.position.x;
        waypoint.y = wp.pose.position.y;
        waypoints_.push_back(waypoint);
    }
    
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
