#ifndef ATLANTIS_SCENE_GRAPH__SCENE_GRAPH_GENERATORS__AGENT_SCENE_GRAPH_GENERATOR_HPP_
#define ATLANTIS_SCENE_GRAPH__SCENE_GRAPH_GENERATORS__AGENT_SCENE_GRAPH_GENERATOR_HPP_


#include <memory>
#include <vector>
#include <string>
#include <set>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "atlantis_scene_graph/scene_graph_generator.hpp"
#include <location_msgs/msg/waypoint.hpp>
#include <location_msgs/msg/waypoint_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <scene_graph_msgs/msg/scene_graph.hpp>
#include <standard_msgs/msg/plan.hpp>
#include <standard_msgs/msg/event.hpp>
#include <standard_msgs/msg/string_multi_array.hpp>
#include "atlantis_core/types.hpp"
#include "atlantis_util/utils.h"
namespace atlantis_scene_graph
{

struct PredicateSchema
{
  std::string intent;
  std::vector<std::string> types;
};

class AgentSceneGraphGenerator : public SceneGraphGenerator
{
public:
  /**
   * @brief constructor
   */
  AgentSceneGraphGenerator();

  /**
   * @brief destructor
   */
  ~AgentSceneGraphGenerator();

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
   * @brief Generate the scene graph
   *
   * @return scene_graph_msgs::msg::SceneGraph
   */
  scene_graph_msgs::msg::SceneGraph generateSceneGraph() override;

  /**
   * @brief Tell if the robot ids are known and every robot has a location
   *
   * @return True when the scene graph is complete
   */
  bool isReady() override;

private:

  void robotIdsCallback(standard_msgs::msg::StringMultiArray msg);

  void currentPoseCallback(geometry_msgs::msg::PoseStamped msg, const std::string & robot);

  void waypointArrayCallback(location_msgs::msg::WaypointArray msg);

  void planCallback(standard_msgs::msg::Plan msg);

  void eventCallback(standard_msgs::msg::Event msg);


  atlantis_core::Waypoint findNearestWaypoint(double x, double y, double theta);

  void applyEffects(const std::vector<std::string> & effects);

  std::vector<std::string> tokenize(const std::string & fact) const;

  void loadTypes();

  void loadPredicates();

  void readPredicates(const YAML::Node & root);

  void checkSchema();

  void loadInitialState();

  bool isValidFact(const std::vector<std::string> & fact) const;

  void addFactNodes(const std::vector<std::string> & fact);

  bool hasLocation(const std::string & robot) const;

  bool allRobotsLocated() const;

  void updateNodePose(const std::string & id, const geometry_msgs::msg::Pose & pose);

  void setNodeType(const std::string & id, const std::string & type);

  bool hasEdge(const std::string & relation, const std::vector<std::string> & args) const;

  void removeEdge(const std::string & relation, const std::vector<std::string> & args);

  void removeEdges(const std::string & relation, const std::string & first_arg);

  void addEdge(const std::string & relation, const std::vector<std::string> & args);

protected:
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);


  rclcpp::Logger logger_{rclcpp::get_logger("AgentSceneGraphGenerator")};
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string  name_;
  std::mutex  mutex_;
  std::vector<std::string> robots_ids_;
  bool robot_ids_received_{false};
  std::vector<atlantis_core::Waypoint> waypoints_;
  scene_graph_msgs::msg::SceneGraph graph_;
  std::string scene_graph_frame_;
  std::string robot_type_;
  std::string waypoint_type_;
  bool initial_state_built_{false};
  standard_msgs::msg::Plan plan_;
  std::set<int32_t> applied_actions_;
  std::string types_file_;
  std::string predicates_file_;
  std::vector<std::pair<std::string, std::string>> initial_objects_;
  std::vector<std::string> initial_facts_;
  std::unordered_map<std::string, std::string> type_parents_;
  std::unordered_map<std::string, PredicateSchema> predicates_;

  //Subs
  rclcpp::Subscription<standard_msgs::msg::StringMultiArray>::SharedPtr robot_ids_sub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> position_subs_;
  rclcpp::Subscription<location_msgs::msg::WaypointArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<standard_msgs::msg::Plan>::SharedPtr plan_sub_;
  rclcpp::Subscription<standard_msgs::msg::Event>::SharedPtr event_sub_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}

#endif