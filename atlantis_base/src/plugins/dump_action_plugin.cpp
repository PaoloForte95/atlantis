// Copyright 2026 Atlantis

#include <atlantis_base/plugins/dump_action_plugin.hpp>

#include <material_handler_msgs/msg/material_flow.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <thread>

namespace atlantis_base
{

void DumpActionPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode * node,
  std::shared_ptr<atlantis_core::SimulationWorld> world,
  const atlantis_core::PluginConfig & config)
{
  node_ = node;
  world_ = world;
  config_ = config;
  robot_name_ = config.name.substr(0, config.name.find('.'));

  base_sim_ = dynamic_cast<BaseSimulator *>(node);
  if (!base_sim_) {
    RCLCPP_ERROR(node_->get_logger(),
      "DumpActionPlugin requires BaseSimulator host node");
    return;
  }

  server_ = rclcpp_action::create_server<Action>(
    node_,
    config.topic,
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<GoalHandle> gh) {
      std::thread{[this, gh]() { this->execute(gh); }}.detach();
    });
}

void DumpActionPlugin::cleanup()
{
  server_.reset();
}

void DumpActionPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto feedback = std::make_shared<Action::Feedback>();
  auto result = std::make_shared<Action::Result>();
  auto goal = goal_handle->get_goal();

  const auto & material_name = goal->name;
  const auto & location = goal->location;

  double t_start = base_sim_->getSimTime();
  double delta = world_->getLoadedAmount(robot_name_);

  if (delta <= 0.0) {
    result->material_dumped = false;
    goal_handle->abort(result);
    return;
  }

  double current = world_->getMaterialAmount(material_name, location);
  if (current < 0.0) current = 0.0;

  const int num_steps = 5;
  const double step_dt = 0.5;
  double running_total = 0.0;
  for (int step = 0; step < num_steps; ++step) {
    current += delta / num_steps;
    running_total += delta / num_steps;
    world_->setMaterialAmount(material_name, location, current);
    base_sim_->advanceSimTime(step_dt);

    feedback->amount_dumped = running_total;
    goal_handle->publish_feedback(feedback);
  }

  auto pub = base_sim_->getMaterialFlowPublisher();
  if (pub) {
    material_handler_msgs::msg::MaterialFlow msg;
    msg.stamp = node_->now();
    msg.flow = delta;
    pub->publish(msg);
  }

  world_->setLoadedAmount(robot_name_, 0.0);

  double duration = base_sim_->getSimTime() - t_start;
  result->time.sec = static_cast<int32_t>(duration);
  result->time.nanosec = static_cast<uint32_t>((duration - result->time.sec) * 1e9);
  result->material_dumped = true;
  goal_handle->succeed(result);
}

}  // namespace atlantis_base

PLUGINLIB_EXPORT_CLASS(
  atlantis_base::DumpActionPlugin,
  atlantis_core::ActionPlugin)
