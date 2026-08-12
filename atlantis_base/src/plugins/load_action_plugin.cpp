// Copyright 2026 Atlantis

#include <atlantis_base/plugins/load_action_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <random>
#include <thread>

namespace atlantis_base
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

  base_sim_ = dynamic_cast<BaseSimulator *>(node);
  if (!base_sim_) {
    RCLCPP_ERROR(node_->get_logger(),
      "LoadActionPlugin requires BaseSimulator host node");
    return;
  }

  if (!node_->has_parameter("refilling")) {
    node_->declare_parameter("refilling", false);
  }
  if (!node_->has_parameter("randomness")) {
    node_->declare_parameter("randomness", false);
  }
  if (!node_->has_parameter("uncertainty")) {
    node_->declare_parameter("uncertainty", 0.0);
  }
  node_->get_parameter("refilling", refilling_);
  node_->get_parameter("randomness", randomness_);
  node_->get_parameter("uncertainty", uncertainty_);

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

double LoadActionPlugin::generateRandomValue(double amount,
                                             double uncertainty)
{
    static thread_local std::mt19937 gen{std::random_device{}()};

    const double mean = 0.8 * amount;
    const double stddev = uncertainty * amount;
    std::normal_distribution<double> dist(mean, stddev);
    return std::max(0.0, dist(gen));
}

void LoadActionPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto feedback = std::make_shared<Action::Feedback>();
  auto result = std::make_shared<Action::Result>();
  auto goal = goal_handle->get_goal();

  const auto & material_name = goal->name;
  const auto & location = goal->location;

  double t_start = base_sim_->getSimTime();
  double capacity = world_->getCapacity(robot_name_);
  double amount_to_load = capacity;

  double available = world_->getMaterialAmount(material_name, location);
  if (available <= 0.0) {
    result->material_loaded = false;
    goal_handle->abort(result);
    return;
  }
  if (goal->amount > 0.0) {
    amount_to_load = std::min(amount_to_load, goal->amount);
  }

  double amount_loaded_total = (available > amount_to_load)
    ? (randomness_
        ? generateRandomValue(amount_to_load, uncertainty_)
        : amount_to_load)
    : available;

  const int num_steps = 5;
  const double step_dt = 0.5;
  double remaining = available;
  double running_total = 0.0;
  for (int step = 0; step < num_steps; ++step) {
    running_total += amount_loaded_total / num_steps;
    if (!refilling_) {
      remaining -= amount_loaded_total / num_steps;
      world_->setMaterialAmount(material_name, location, remaining);
    }
    base_sim_->advanceSimTime(step_dt);

    feedback->amount_loaded = running_total;
    goal_handle->publish_feedback(feedback);
  }

  world_->setLoadedAmount(robot_name_, amount_loaded_total);

  double duration = base_sim_->getSimTime() - t_start;
  result->time.sec = static_cast<int32_t>(duration);
  result->time.nanosec = static_cast<uint32_t>((duration - result->time.sec) * 1e9);
  result->material_loaded = true;
  goal_handle->succeed(result);
}

}  // namespace atlantis_base

PLUGINLIB_EXPORT_CLASS(
  atlantis_base::LoadActionPlugin,
  atlantis_core::ActionPlugin)
