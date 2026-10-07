#ifndef ATLANTIS_SCENE_GRAPH_ATLANTIS_SCENE_GRAPH_GENERATOR_HPP_
#define ATLANTIS_SCENE_GRAPH_ATLANTIS_SCENE_GRAPH_GENERATOR_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "scene_graph_msgs/msg/scene_graph.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace atlantis_scene_graph
{

/**
 * @brief 
 * 
 */
class SceneGraphGenerator
{
public:
  using Ptr = std::shared_ptr<SceneGraphGenerator>;

  /**
   * @brief Virtual destructor
   */
  virtual ~SceneGraphGenerator() {}

  /**

   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) = 0;

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
   * @brief Method to generate the scene graph
   * 
   * @return The current scene graph
   */
  virtual scene_graph_msgs::msg::SceneGraph generateSceneGraph() = 0;

  /**
   * @brief Method to tell if the scene graph is complete and can be published
   * 
   * @return True when the scene graph is complete
   */
  virtual bool isReady() { return true; }

  virtual void setDomainFile(const std::string & domain_file) { domain_file_ = domain_file; }
protected:
  std::string domain_file_;
};

} 

#endif
