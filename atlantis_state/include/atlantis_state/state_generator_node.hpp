#ifndef ATLANTIS_STATE_ATLANTIS_STATE_GENERATOR_NODE_HPP_
#define ATLANTIS_STATE_ATLANTIS_STATE_GENERATOR_NODE_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "atlantis_state/state_generator.hpp"



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace atlantis_state
{
/**
 * @brief 
 * 
 */
class StateGeneratorNode : public rclcpp_lifecycle::LifecycleNode
{

public:
  
  StateGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the State Generator object
   * 
   */
  ~StateGeneratorNode();

  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

protected:
  /**
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishState();

    /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  standard_msgs::msg::StringMultiArray generateState();


  atlantis_state::StateGenerator::Ptr state_generator_;
  pluginlib::ClassLoader<atlantis_state::StateGenerator> gp_loader_;
  std::string default_id_;
  std::string default_type_;
  std::string state_generator_id_;
  std::string state_generator_type_;

  rclcpp::TimerBase::SharedPtr timer_state_publisher_; // Used to publish the symbolic state


  // Publishers for the state
  rclcpp_lifecycle::LifecyclePublisher<standard_msgs::msg::StringMultiArray>::SharedPtr state_publisher_;


};

} 

#endif 
