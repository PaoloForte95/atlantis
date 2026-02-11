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

    standard_msgs::msg::StringMultiArray symbolic_state;
    //RCLCPP_INFO(logger_, "Generating symbolic state");
    symbolic_state.data.push_back("(at rb1 wp1)");

    return symbolic_state;
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
