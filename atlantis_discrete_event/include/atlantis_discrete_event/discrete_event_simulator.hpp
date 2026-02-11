// Copyright (c) 2025 Paolo Forte
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ATLANTIS__DISCRETE_EVENT_SIMULATOR_HPP_
#define ATLANTIS__DISCRETE_EVENT_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <material_handler_msgs/action/load_material.hpp>
#include <material_handler_msgs/action/dump_material.hpp>
#include <material_handler_msgs/srv/get_material_amount.hpp>
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"
#include "atlantis_util/file_handler.h"
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GoalHandleNavigatetoPose = rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>;
using GoalHandleLoadCommand = rclcpp_action::ServerGoalHandle<material_handler_msgs::action::LoadMaterial>;
using GoalHandleDumpCommand = rclcpp_action::ServerGoalHandle<material_handler_msgs::action::DumpMaterial>;

using Robots = std::map<int, int>;


namespace atlantis_simulator
{

    /**
     * @brief A base visualization based on RVIZ 
     * 
     */
class DiscreteEventSimulator : public rclcpp_lifecycle::LifecycleNode{

    public:

        DiscreteEventSimulator(const std::string & node_name,
        const std::string & ns = "",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        ~DiscreteEventSimulator();

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
        

    private:

        std::string find_waypoint(double x, double y, double theta, double tolerance = 1.0); 

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

        // Simulation time related stuff
        void publishClock();
            
        private:
        rclcpp::Node::SharedPtr node_;
        //Actions servers
        std::vector<rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr> action_servers_;
        std::vector<rclcpp_action::Server<material_handler_msgs::action::LoadMaterial>::SharedPtr> load_action_servers_;
        std::vector<rclcpp_action::Server<material_handler_msgs::action::DumpMaterial>::SharedPtr> dump_action_servers_;
        std::vector<rclcpp::Service<material_handler_msgs::srv::GetMaterialAmount>::SharedPtr> material_amount_servers_;
        //Pubs
        rclcpp_lifecycle::LifecyclePublisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
        std::map< std::string,rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr> metrics_pubs_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr material_flow_pub_;

        std::vector<atlantis::util::Material> materials_;
        std::vector<std::string> robots_ids_, materials_ids_, waypoints_ids_, metrics_;
        std::map<std::string, bool> metrics_to_evaluate_;
        Robots robots_;
        std::map<int,double> amount_loaded_;
        std::map<int, double> capacities_;
        std::vector<atlantis::util::Waypoint> waypoints_;
        std::map< std::string,std::string> robots_current_locations_;
        
        double sim_time_ = 0.; // The current simulated time step, starting at 0 at simulation start.
        double dt_; // How large time step the simulator will take at each step.
        double real_time_factor_; // How much faster the simulator will run compared to real time .
        double material_flow_amount_ = 0.;
};
}

#endif