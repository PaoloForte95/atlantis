#ifndef ATLANTIS_BASE_SIMULATOR_H
#define ATLANTIS_BASE_SIMULATOR_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/path.hpp"
#include <rosgraph_msgs/msg/clock.hpp>
#include <atlantis_base/rviz_visualization.hpp>
#include "atlantis_util/file_handler.h"
#include "navigo/planner/car_planner.h"
#include <material_handler_msgs/action/load_material.hpp>
#include <material_handler_msgs/action/dump_material.hpp>
#include <material_handler_msgs/srv/get_material_amount.hpp>
#include <material_handler_msgs/msg/material_flow.hpp>
#include <location_msgs/srv/get_waypoint_list.hpp>
#include <location_msgs/msg/waypoint.hpp>
#include <location_msgs/msg/waypoint_array.hpp>
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GoalHandleNavigatetoPose = rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>;
using GoalHandleLoadCommand = rclcpp_action::ServerGoalHandle<material_handler_msgs::action::LoadMaterial>;
using GoalHandleDumpCommand = rclcpp_action::ServerGoalHandle<material_handler_msgs::action::DumpMaterial>;
using Robots = std::map<int, int>;


namespace atlantis
{

    /**
     * @brief A base visualization based on RVIZ 
     * 
     */
class BaseSimulator : public rclcpp_lifecycle::LifecycleNode{

    public:

        BaseSimulator(const std::string & node_name,
        const std::string & ns = "",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        ~BaseSimulator();

        CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Activate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Deactivate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Reset member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Called when in shutdown state
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
        

        double getMaterialAmount(std::string pile_id, std::string pile_loc);

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, 
                                                std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle, const std::string & server_name);

        virtual void execute(const std::shared_ptr<GoalHandleNavigatetoPose> goal_handle, const std::string & server_name );

        rclcpp_action::GoalResponse handle_load_cmd_goal(const rclcpp_action::GoalUUID & uuid, 
                                                std::shared_ptr<const material_handler_msgs::action::LoadMaterial::Goal> goal);

        rclcpp_action::CancelResponse handle_load_cmd_cancel(const std::shared_ptr<GoalHandleLoadCommand> goal_handle);

        void handle_load_cmd_accepted(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const int robotID);

        void executeLoadCmd(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const int robotID);


        rclcpp_action::GoalResponse handle_dump_cmd_goal(const rclcpp_action::GoalUUID & uuid, 
                                                std::shared_ptr<const material_handler_msgs::action::DumpMaterial::Goal> goal);

        rclcpp_action::CancelResponse handle_dump_cmd_cancel(const std::shared_ptr<GoalHandleDumpCommand> goal_handle);

        void handle_dump_cmd_accepted(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const int robotID);

        void executeDumpCmd(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const int robotID);

        void getMaterialAmountCallback(const std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Request> request,
          std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Response> response);

        void criticalPointCallback(std_msgs::msg::Int64 msg, int robotID);

        virtual geometry_msgs::msg::Pose getWaypoint(std::string wp);
    
        // Simulation time related stuff
        void publishClock();
    
        void publishMaterialFlow();

        void publishRobotPose();

        void publishMaterial();

        void getWaypointList(const std::shared_ptr<location_msgs::srv::GetWaypointList::Request> request, std::shared_ptr<location_msgs::srv::GetWaypointList::Response> response);
    

    public:
        rclcpp::Node::SharedPtr node_;
        std::mutex poses_mutex_;
        std::string lattice_primitives_, primitives_dir_;
        //Actions servers
        std::vector<rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr> action_servers_;
        std::vector<rclcpp_action::Server<material_handler_msgs::action::LoadMaterial>::SharedPtr> load_action_servers_;
        std::vector<rclcpp_action::Server<material_handler_msgs::action::DumpMaterial>::SharedPtr> dump_action_servers_;
        //Pubs
        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
        rclcpp_lifecycle::LifecyclePublisher<location_msgs::msg::WaypointArray>::SharedPtr waypoints_pub_;
        std::map< std::string,rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr> robots_current_pose_;
        std::map< std::string,rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr> robots_paths_;
        std::map< std::string,rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr> current_path_idx_pubs_;
        std::map< std::string,rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr> metrics_pubs_;
        rclcpp_lifecycle::LifecyclePublisher<material_handler_msgs::msg::MaterialFlow>::SharedPtr material_flow_pub_;
        //Subs
        std::vector<rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr> critical_points_subs_;

        //Services
        rclcpp::Service<material_handler_msgs::srv::GetMaterialAmount>::SharedPtr material_amount_server_;
        rclcpp::Service<location_msgs::srv::GetWaypointList>::SharedPtr waypoint_list_server_;

        
        atlantis_base::RvizVisualization rviz_viz_;
        std::string map_yaml_, map_dir_;
        bool started_, use_trajectory_dt_, refilling_ ;
        bool randomness_; //Whenever to enable a random amound of material loaded
        bool use_precomputed_paths_;
        std::string precomputed_paths_folder_;
        double mat_amount_, goal_tolerance_;
        navigo::CostMap* oc_;
        std::map<int, navigo::CarPlanner*> base_planners_;
        std::vector<atlantis::util::Material> materials_;
        std::map<std::string, int> materials_viz_;
        std::vector<std::string> robots_ids_, materials_ids_, waypoints_ids_, metrics_, planners_ids_;
        std::map<std::string, bool> metrics_to_evaluate_;
        Robots robots_;
        std::map<int, std::string> models_, planner_types_;
        std::map<std::string, navigo::Pose> start_poses_ ;
        std::map<std::string, navigo::Pose> current_poses_;
        std::map<std::string, int> path_index_;
        std::vector<atlantis::util::Waypoint> waypoints_;
        std::map<int, int> critical_points_;
        std::map<int, double> capacities_;
        std::map<int,double> amount_loaded_;
        std::map<std::string, std::vector<geometry_msgs::msg::Point>> robot_footprints_;

        rclcpp::TimerBase::SharedPtr timer_sim_time_; // Used to output the clock time
        rclcpp::TimerBase::SharedPtr timer_material_flow_; // Used to output the material flow
        rclcpp::TimerBase::SharedPtr timer_robot_pose_; // Used to publish the current robot pose and path index
        rclcpp::TimerBase::SharedPtr timer_material_publisher_;  // Used to publish the material amount in the different locations
        double max_planning_time_;
        double last_real_time_;
        double sim_time_ = 0.; // The current simulated time step, starting at 0 at simulation start.
        double dt_; // How large time step the simulator will take at each step.
        double real_time_factor_; // How much faster the simulator will run compared to real time .
        double material_flow_amount_ = 0.;
        double max_sim_time_;
       
};

}
#endif