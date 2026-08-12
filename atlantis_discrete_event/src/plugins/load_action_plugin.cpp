// Copyright 2026 Atlantis

#include <atlantis_discrete_event/plugins/load_action_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <random>
#include <thread>

namespace atlantis_simulator
{

void LoadActionPlugin::initialize(
  rclcpp_lifecycle::LifecycleNode * node,
  std::shared_ptr<atlantis_core::SimulationWorld> world,
  const atlantis_core::PluginConfig & config)
{
  node_ = node;
  world_ = world;
  config_ = config;
  robot_name_ = config.name.substr(0, config.name.find('.'));

  if (!node_->has_parameter("refilling")) {
    node_->declare_parameter("refilling", false);
  }
  if (!node_->has_parameter("randomness")) {
    node_->declare_parameter("randomness", false);
  }
  node_->get_parameter("refilling", refilling_);
  node_->get_parameter("randomness", randomness_);

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

void LoadActionPlugin::cleanup()
{
  server_.reset();
}

std::string LoadActionPlugin::getName() const
{
  return "load";
}

double LoadActionPlugin::generateRandomValue(double mean, double stddev)
{
  static thread_local std::mt19937 gen{std::random_device{}()};
  std::normal_distribution<double> dist(mean, stddev * mean);
  return std::max(0.0, dist(gen));
}

void LoadActionPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto feedback = std::make_shared<Action::Feedback>();
  auto result = std::make_shared<Action::Result>();
  auto goal = goal_handle->get_goal();

  const auto & material_name = goal->target;
  const auto & location = goal->location;
  double capacity = world_->getCapacity(robot_name_);
  double amount_to_load = capacity;

  double available = world_->getMaterialAmount(material_name, location);
  if (available < 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(), "Material %s not found at %s",
      material_name.c_str(), location.c_str());
    result->success = false;
    result->payload = 0.0;
    goal_handle->abort(result);
    return;
  }
  if (available <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(), "Material %s not available at %s",
      material_name.c_str(), location.c_str());
    result->success = false;
    result->payload = 0.0;
    goal_handle->abort(result);
    return;
  }

  if (goal->amount > 0.0) {
    amount_to_load = std::min(amount_to_load, goal->amount);
  }

  double amount_loaded = 0.0;
  if (available > amount_to_load) {
    amount_loaded = randomness_
      ? generateRandomValue(0.8 * amount_to_load, 0.2)
      : amount_to_load;
  } else {
    amount_loaded = available;
  }

  world_->setLoadedAmount(robot_name_, amount_loaded);
  RCLCPP_INFO(
    node_->get_logger(), "%s loading %f of %s",
    robot_name_.c_str(), amount_loaded, material_name.c_str());

  const int num_steps = 5;
  double remaining = available;
  for (int step = 0; step < num_steps; ++step) {
    if (!refilling_) {
      remaining -= amount_loaded / num_steps;
      world_->setMaterialAmount(material_name, location, remaining);
    }
    feedback->phase = "loading";
    feedback->progress = static_cast<float>(step + 1) / num_steps;
    goal_handle->publish_feedback(feedback);
  }

  result->payload = amount_loaded;
  result->success = true;
  goal_handle->succeed(result);
}

}  // namespace atlantis_simulator

PLUGINLIB_EXPORT_CLASS(
  atlantis_simulator::LoadActionPlugin,
  atlantis_core::ActionPlugin)
