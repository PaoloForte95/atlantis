#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "atlantis_scene_graph/scene_graph_generator_node.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;


template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterType & param_type,
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, param_type, parameter_descriptor);
  }
}



/// Gets the type of plugin for the selected node and its plugin
/**
 * Gets the type of plugin for the selected node and its plugin.
 * Actually seeks for the value of "<plugin_name>.plugin" parameter.
 *
 * \param[in] node Selected node
 * \param[in] plugin_name The name of plugin the type of which is being searched for
 * \return A string containing the type of plugin (the value of "<plugin_name>.plugin" parameter)
 */
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::PARAMETER_STRING);
  std::string plugin_type;
  try {
    if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
      RCLCPP_FATAL(
        node->get_logger(), "Can not get 'plugin' param value for %s", plugin_name.c_str());
      exit(-1);
    }
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s", plugin_name.c_str());
    exit(-1);
  }

  return plugin_type;
}

namespace atlantis_scene_graph
{

SceneGraphGeneratorNode::SceneGraphGeneratorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("atlantis_scene_graph_generator_node", "", options),
  gp_loader_("atlantis_scene_graph", "atlantis_scene_graph::SceneGraphGenerator"),
  default_id_("AgentSceneGraphGenerator"),
  default_type_("atlantis_scene_graph::SceneGraphGenerator")
{
  RCLCPP_INFO(get_logger(), "Creating scene graph generator node");

  declare_parameter("scene_graph_generator_plugin", default_id_);
  declare_parameter("domain_file", std::string(""));
  declare_parameter("scene_graph_rate", 1.0);
  get_parameter("scene_graph_generator_plugin", scene_graph_generator_id_);
  get_parameter("domain_file", domain_file_);
  get_parameter("scene_graph_rate", scene_graph_rate_);

  if (scene_graph_generator_id_ == default_id_) {
    declare_parameter(default_id_ + ".plugin", default_type_);
  }
}


SceneGraphGeneratorNode::~SceneGraphGeneratorNode()
{

}

CallbackReturn
SceneGraphGeneratorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Scene Graph Generator Node");

  auto node = shared_from_this();

   try {
      scene_graph_generator_type_ = get_plugin_type_param( node, scene_graph_generator_id_);
      scene_graph_generator_ = gp_loader_.createUniqueInstance(scene_graph_generator_type_);
      RCLCPP_INFO( get_logger(), "Created scene graph generator plugin %s of type %s", scene_graph_generator_id_.c_str(), scene_graph_generator_type_.c_str());
      scene_graph_generator_->configure(node, scene_graph_generator_id_);
      scene_graph_generator_->setDomainFile(domain_file_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL( get_logger(), "Failed to create scene graph generator. Exception: %s", ex.what());
      return CallbackReturn::FAILURE;
    }



  // Initialize pubs & subs
  scene_graph_publisher_ = create_publisher<scene_graph_msgs::msg::SceneGraph>(
    "scene_graph",
    rclcpp::QoS(1).transient_local().reliable());


  return CallbackReturn::SUCCESS;
}

CallbackReturn
SceneGraphGeneratorNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{

  scene_graph_publisher_->on_activate();
  scene_graph_generator_->activate();

  const double rate = scene_graph_rate_ > 0.0 ? scene_graph_rate_ : 1.0;
  timer_scene_graph_publisher_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
    std::bind(&SceneGraphGeneratorNode::publishSceneGraph, this));

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SceneGraphGeneratorNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  if (timer_scene_graph_publisher_) {
    timer_scene_graph_publisher_->cancel();
    timer_scene_graph_publisher_.reset();
  }

  scene_graph_publisher_->on_deactivate();
  scene_graph_generator_->deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SceneGraphGeneratorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  timer_scene_graph_publisher_.reset();
  scene_graph_publisher_.reset();
  scene_graph_generator_->cleanup();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SceneGraphGeneratorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}


scene_graph_msgs::msg::SceneGraph SceneGraphGeneratorNode::generateSceneGraph()
{
  return scene_graph_generator_->generateSceneGraph();
}

void SceneGraphGeneratorNode::publishSceneGraph()
{
  if (!scene_graph_generator_ || !scene_graph_publisher_ ||
    !scene_graph_publisher_->is_activated())
  {
    return;
  }

  if (!scene_graph_generator_->isReady()) {
    return;
  }

  scene_graph_publisher_->publish(generateSceneGraph());
}

}  

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(atlantis_scene_graph::SceneGraphGeneratorNode)
