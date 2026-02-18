#ifndef ATLANTIS_STATE__STATE_GENERATORS__PDDL_STATE_GENERATOR_HPP_
#define ATLANTIS_STATE__STATE_GENERATORS__PDDL_STATE_GENERATOR_HPP_


#include <memory>
#include <vector>
#include <string>

#include "atlantis_state/state_generator.hpp"
#include <location_msgs/msg/waypoint.hpp>
#include <location_msgs/msg/waypoint_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "atlantis_util/utils.h"
namespace atlantis_state
{

class PddlStateGenerator : public StateGenerator
{
public:
  /**
   * @brief constructor
   */
  PddlStateGenerator();

  /**
   * @brief destructor
   */
  ~PddlStateGenerator();

  /**
   * @brief Configure lifecycle node
   * @param parent Weak pointer to the lifecycle node
   * @param name The name of this state generator
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Generate a symbolic PDDL state
   * 
   * @return standard_msgs::msg::StringMultiArray 
   */
  standard_msgs::msg::StringMultiArray generateState() override;

private:
    
  void currentPoseCallback(geometry_msgs::msg::PoseStamped msg, int robotID);

  void waypointArrayCallback(location_msgs::msg::WaypointArray msg);

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);


  rclcpp::Logger logger_{rclcpp::get_logger("PddlStateGenerator")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string  name_;
  std::mutex  mutex_;
  std::vector<std::string> robots_ids_;
  std::vector<atlantis::util::Waypoint> waypoints_;
  standard_msgs::msg::StringMultiArray atlantis_state_;

  //Subs
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> position_subs_;
  rclcpp::Subscription<location_msgs::msg::WaypointArray>::SharedPtr waypoint_array_subs_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  

#endif  