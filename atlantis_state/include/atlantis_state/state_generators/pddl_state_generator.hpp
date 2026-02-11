#ifndef ATLANTIS_STATE__STATE_GENERATORS__PDDL_STATE_GENERATOR_HPP_
#define ATLANTIS_STATE__STATE_GENERATORS__PDDL_STATE_GENERATOR_HPP_


#include <memory>
#include <vector>
#include <string>

#include "atlantis_state/state_generator.hpp"

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
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
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

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_LATTICE_HPP_
