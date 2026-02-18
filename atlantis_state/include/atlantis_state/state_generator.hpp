#ifndef ATLANTIS_STATE_STATE_GENERATOR_HPP_
#define ATLANTIS_STATE_STATE_GENERATOR_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "standard_msgs/msg/string_multi_array.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace atlantis_state
{

/**
 * @brief 
 * 
 */
class StateGenerator
{
public:
  using Ptr = std::shared_ptr<StateGenerator>;

  /**
   * @brief Virtual destructor
   */
  virtual ~StateGenerator() {}

  /**

   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method to generate the symbolic state
   * 
   * @return A symbolic state associated to the problem 
   */
  virtual standard_msgs::msg::StringMultiArray generateState() = 0;
};

} 

#endif  