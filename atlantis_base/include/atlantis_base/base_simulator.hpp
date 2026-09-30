// Copyright 2026 Atlantis

#ifndef ATLANTIS_BASE__BASE_SIMULATOR_HPP_
#define ATLANTIS_BASE__BASE_SIMULATOR_HPP_

#include <atlantis_base/rviz_visualization.hpp>
#include <atlantis_core/simulator_core.hpp>
#include <atlantis_util/file_handler.h>
#include <atlantis_util/utils.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <material_handler_msgs/msg/material_flow.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigo/costmap/costmap.h>
#include <navigo/planner/car_planner.h>
#include <navigo/planner/utils.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/int64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace atlantis_base
{

class BaseSimulator : public atlantis_core::SimulatorCore
{
public:
  BaseSimulator(
    const std::string & node_name,
    const std::string & ns = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~BaseSimulator() override;

  // ---- Accessors used by plugins ----
  navigo::CostMap * getCostmap() { return oc_; }
  double getDt() const { return dt_; }
  double getRealTimeFactor() const { return real_time_factor_; }
  double getGoalTolerance() const { return goal_tolerance_; }
  navigo::CarPlanner * getPlanner(const std::string & robot_name);
  const std::vector<geometry_msgs::msg::Point> & getFootprint(const std::string & robot_name) const;

  double getSimTime() const;
  void advanceSimTime(double dt);

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    getRobotPosePublisher(const std::string & robot_name);
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    getRobotPathPublisher(const std::string & robot_name);
  rclcpp_lifecycle::LifecyclePublisher<material_handler_msgs::msg::MaterialFlow>::SharedPtr
    getMaterialFlowPublisher() { return material_flow_pub_; }

  RvizVisualization & getRvizVisualization() { return rviz_viz_; }

  bool usePrecomputedPaths() const {return use_precomputed_paths_;}
  navigo::Path loadPrecomputedPath(const navigo::Pose & start, const navigo::Pose & goal);

protected:

  void onConfigureExtra() override;
  void onActivateExtra() override;
  void onDeactivateExtra() override;
  void onCleanupExtra() override;

  std::vector<std::string> defaultActions() const override { return {}; }
  std::vector<std::string> defaultServices() const override { return {}; }

private:
  void loadBaseParameters();
  void buildCostmap();
  void buildPlanners();
  void buildPerRobotPublishers();
  void loadFootprints();

  void publishClock();
  void publishRobotPoses();

  void publishRobotMarkers();

  void publishRobotTransforms();

  navigo::Pose readFirstPose(const std::string & file_path) const;
  navigo::Pose readLastPose(const std::string & file_path) const;
  bool posesMatch(const navigo::Pose & a, const navigo::Pose & b) const;
  std::string findPathFile(const std::string & folder, const navigo::Pose & start, const navigo::Pose & goal) const;
  navigo::Path loadPath(const std::string & file_path) const;

  // ---- Layer parameters ----
  std::string map_yaml_;
  std::string lattice_primitives_;
  std::string primitives_dir_;
  std::string path_costs_;
  bool use_precomputed_paths_{false};
  bool compute_path_costs_{false};
  std::string precomputed_paths_folder_;
  bool use_trajectory_dt_{false};
  double goal_tolerance_{0.5};
  double max_planning_time_{5.0};
  double dt_{0.1};
  double real_time_factor_{1.0};
  double max_sim_time_{0.0};

  // ---- Layer state ----
  navigo::CostMap * oc_{nullptr};
  std::map<std::string, std::unique_ptr<navigo::GridCollisionChecker>> collision_checkers_;
  std::map<std::string, navigo::CarPlanner *> base_planners_;
  std::map<std::string, std::vector<geometry_msgs::msg::Point>> robot_footprints_;
  RvizVisualization rviz_viz_;

  mutable std::mutex sim_time_mutex_;
  double sim_time_{0.0};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp_lifecycle::LifecyclePublisher<material_handler_msgs::msg::MaterialFlow>::SharedPtr
    material_flow_pub_;
  std::map<std::string,
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
    robot_pose_pubs_;
  std::map<std::string,
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr>
    robot_path_pubs_;

  // ---- Timers ----
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace atlantis_base

#endif  // ATLANTIS_BASE__BASE_SIMULATOR_HPP_