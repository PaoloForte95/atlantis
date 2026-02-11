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

#include "atlantis_state/state_generator_node.hpp"

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

namespace atlantis_state
{

StateGeneratorNode::StateGeneratorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("atlantis_state_generator_node", "", options),
  gp_loader_("atlantis_state", "atlantis_state::StateGenerator"),
  default_id_("PDDL"),
  default_type_("atlantis_state::PddlStateGenerator")
{
  RCLCPP_INFO(get_logger(), "Creating state generator node");

  declare_parameter("state_generator_plugin", default_id_);

  get_parameter("state_generator_plugin", state_generator_id_);
  if (state_generator_id_ == default_id_) {
    declare_parameter(default_id_ + ".plugin", default_type_);
  }

  timer_state_publisher_ = this->create_wall_timer( std::chrono::milliseconds(1000), std::bind(&StateGeneratorNode::publishState, this)); 

}


StateGeneratorNode::~StateGeneratorNode()
{

}

CallbackReturn
StateGeneratorNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring State Generator Node");

  auto node = shared_from_this();

   try {
      state_generator_type_ = get_plugin_type_param( node, state_generator_id_);
      state_generator_ = gp_loader_.createUniqueInstance(state_generator_type_);
      RCLCPP_INFO( get_logger(), "Created state generator plugin %s of type %s", state_generator_id_.c_str(), state_generator_type_.c_str());
      state_generator_->configure(node, state_generator_id_);
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL( get_logger(), "Failed to create state generator. Exception: %s", ex.what());
      return CallbackReturn::FAILURE;
    }



  // Initialize pubs & subs
  state_publisher_ = create_publisher<standard_msgs::msg::StringMultiArray>("symbolic_state", 1);


  return CallbackReturn::SUCCESS;
}

CallbackReturn
StateGeneratorNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  state_publisher_->on_activate();
  state_generator_->activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
StateGeneratorNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  state_publisher_->on_deactivate();
  state_generator_->deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
StateGeneratorNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  state_publisher_.reset();
  state_generator_->cleanup();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
StateGeneratorNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}


standard_msgs::msg::StringMultiArray StateGeneratorNode::generateState(){

  auto current_state = state_generator_->generateState();
  return current_state;

}

void StateGeneratorNode::publishState()
{

  auto current_state = generateState();
  auto msg = std::make_unique<standard_msgs::msg::StringMultiArray>(current_state);
  if (state_publisher_->is_activated() && state_publisher_->get_subscription_count() > 0) {
    state_publisher_->publish(std::move(msg));
  }
}

}  

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(atlantis_state::StateGeneratorNode)
