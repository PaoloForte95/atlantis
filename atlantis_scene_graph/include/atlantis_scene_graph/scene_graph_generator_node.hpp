#ifndef ATLANTIS_SCENE_GRAPH_ATLANTIS_SCENE_GRAPH_GENERATOR_NODE_HPP_
#define ATLANTIS_SCENE_GRAPH_ATLANTIS_SCENE_GRAPH_GENERATOR_NODE_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "atlantis_scene_graph/scene_graph_generator.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace atlantis_scene_graph
{
/**
 * @brief 
 * 
 */
class SceneGraphGeneratorNode : public rclcpp_lifecycle::LifecycleNode
{

public:
  
  SceneGraphGeneratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the State Generator object
   * 
   */
  ~SceneGraphGeneratorNode();

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
   * @brief Publish the scene graph when it is complete
   */
  void publishSceneGraph();

  /**
   * @brief Get the scene graph from the plugin
   * @return Scene graph
   */
  scene_graph_msgs::msg::SceneGraph generateSceneGraph();




  atlantis_scene_graph::SceneGraphGenerator::Ptr scene_graph_generator_;
  pluginlib::ClassLoader<atlantis_scene_graph::SceneGraphGenerator> gp_loader_;
  std::string default_id_;
  std::string default_type_;
  std::string scene_graph_generator_id_;
  std::string scene_graph_generator_type_;
  std::string domain_file_;
  double scene_graph_rate_{1.0};

  rclcpp::TimerBase::SharedPtr timer_scene_graph_publisher_; // Used to publish the scene graph

  // Publishers for the scene graph
  rclcpp_lifecycle::LifecyclePublisher<scene_graph_msgs::msg::SceneGraph>::SharedPtr scene_graph_publisher_;


};

} 

#endif 
