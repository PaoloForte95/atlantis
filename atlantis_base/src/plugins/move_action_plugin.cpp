// Copyright 2026 Atlantis

#include <atlantis_base/plugins/move_action_plugin.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <chrono>
#include <cmath>
#include <thread>

namespace atlantis_base
{

void MoveActionPlugin::initialize(
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
    RCLCPP_ERROR(
      node_->get_logger(),
      "MoveActionPlugin requires a BaseSimulator host node");
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

void MoveActionPlugin::cleanup()
{
  server_.reset();
}

// void MoveActionPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
// {
//   auto goal = goal_handle->get_goal();
//   auto result = std::make_shared<Action::Result>();

//   if (!base_sim_) {
//     goal_handle->abort(result);
//     return;
//   }

//   double gx = goal->pose.pose.position.x;
//   double gy = goal->pose.pose.position.y;
//   double qw = goal->pose.pose.orientation.w;
//   double qz = goal->pose.pose.orientation.z;
//   double theta = 2.0 * std::atan2(qz, qw);

//   auto target = world_->findWaypoint(gx, gy, theta);


//   auto * planner = base_sim_->getPlanner(robot_name_);
//   if (!planner) {
//     RCLCPP_ERROR(
//       node_->get_logger(),
//       "No planner for %s", robot_name_.c_str());
//     goal_handle->abort(result);
//     return;
//   }

//   // TODO: real path planning and walking. Placeholder below.
//   double duration = 1.0;
//   base_sim_->advanceSimTime(duration);
//   std::this_thread::sleep_for(std::chrono::milliseconds(100));

//   world_->setRobotLocation(robot_name_, target);

//   goal_handle->succeed(result);
// }

void MoveActionPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Action::Result>();

  if (!base_sim_) {
    goal_handle->abort(result);
    return;
  }

  auto * planner = base_sim_->getPlanner(robot_name_);
  if (!planner) {
    RCLCPP_ERROR(node_->get_logger(), "No planner for %s", robot_name_.c_str());
    goal_handle->abort(result);
    return;
  }

  navigo::Pose goal_pose;
  goal_pose.x = goal->pose.pose.position.x;
  goal_pose.y = goal->pose.pose.position.y;
  goal_pose.theta =
    2.0 * std::atan2(goal->pose.pose.orientation.z, goal->pose.pose.orientation.w);

  auto start_loc = world_->getRobotLocation(robot_name_);
  navigo::Pose start_pose;
  start_pose.x = start_loc.x;
  start_pose.y = start_loc.y;
  start_pose.theta = start_loc.theta;

  const auto & footprint = base_sim_->getFootprint(robot_name_);
  std::vector<double> xcoords, ycoords;
  for (const auto & p : footprint) {
    xcoords.push_back(p.x);
    ycoords.push_back(p.y);
  }

  auto checker = std::make_unique<navigo::GridCollisionChecker>(base_sim_->getCostmap());
  checker->setFootprint(navigo::Footprint(xcoords, ycoords));
  planner->setCollisionChecker(checker.get());

  navigo::Path path = planner->computePath(start_pose, goal_pose);
  if (path.size() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "No path for %s", robot_name_.c_str());
    goal_handle->abort(result);
    return;
  }

  nav_msgs::msg::Path plan;
  plan.header = goal->pose.header;
  for (size_t i = 0; i < path.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = plan.header;
    ps.pose.position.x = path[i].x;
    ps.pose.position.y = path[i].y;
    auto q = atlantis::util::rpyToQuaternion(0.0, 0.0, path[i].theta);
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    plan.poses.push_back(ps);
  }
  RCLCPP_INFO(node_->get_logger(), "Path for %s has %zu waypoints", robot_name_.c_str(), plan.poses.size());

  auto path_pub = base_sim_->getRobotPathPublisher(robot_name_);
  if (path_pub) {
    path_pub->publish(plan);
  }

  double dt = base_sim_->getDt();
  rclcpp::Rate rate((1.0 / dt) * base_sim_->getRealTimeFactor());

  for (size_t i = 0; i < path.size(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }
    atlantis_core::Waypoint wp;
    wp.x = path[i].x;
    wp.y = path[i].y;
    wp.theta = path[i].theta;
    world_->setRobotLocation(robot_name_, wp);
    base_sim_->advanceSimTime(dt);
    rate.sleep();
  }

  const auto & last = path[path.size() - 1];
  double dist = std::hypot(goal_pose.x - last.x, goal_pose.y - last.y);
  if (dist < base_sim_->getGoalTolerance()) {
    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "Goal succeeded for %s", robot_name_.c_str());
  } else {
    goal_handle->abort(result);
    RCLCPP_INFO(node_->get_logger(), "Goal failed for %s", robot_name_.c_str());
  }
}

}  // namespace atlantis_base

PLUGINLIB_EXPORT_CLASS(
  atlantis_base::MoveActionPlugin,
  atlantis_core::ActionPlugin)
