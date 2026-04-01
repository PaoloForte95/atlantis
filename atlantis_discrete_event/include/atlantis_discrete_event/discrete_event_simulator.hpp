#ifndef ATLANTIS__DISCRETE_EVENT_SIMULATOR_HPP_
#define ATLANTIS__DISCRETE_EVENT_SIMULATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <standard_msgs/action/move_to_pose.hpp>
#include <standard_msgs/action/load.hpp>
#include <standard_msgs/action/dump.hpp>
#include <material_handler_msgs/srv/get_material_amount.hpp>
#include "std_msgs/msg/float64.hpp"
#include "atlantis_util/file_handler.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<standard_msgs::action::MoveToPose>;
using GoalHandleLoadCommand = rclcpp_action::ServerGoalHandle<standard_msgs::action::Load>;
using GoalHandleDumpCommand = rclcpp_action::ServerGoalHandle<standard_msgs::action::Dump>;

using Robots = std::map<int, int>;


namespace atlantis_simulator
{

    /**
     * @brief A discrete event simulator for material handling robots.
     * No physics simulation — actions complete instantly with console logging.
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

        // MoveToPose action handlers
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, 
                                                std::shared_ptr<const standard_msgs::action::MoveToPose::Goal> goal);

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);

        void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle, const std::string & server_name);

        virtual void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle, const std::string & server_name);

        // Load action handlers
        rclcpp_action::GoalResponse handle_load_cmd_goal(const rclcpp_action::GoalUUID & uuid, 
                                                std::shared_ptr<const standard_msgs::action::Load::Goal> goal);

        rclcpp_action::CancelResponse handle_load_cmd_cancel(const std::shared_ptr<GoalHandleLoadCommand> goal_handle);

        void handle_load_cmd_accepted(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const std::string & robotID);

        void executeLoadCmd(const std::shared_ptr<GoalHandleLoadCommand> goal_handle, const std::string & robotID);

        // Dump action handlers
        rclcpp_action::GoalResponse handle_dump_cmd_goal(const rclcpp_action::GoalUUID & uuid, 
                                                std::shared_ptr<const standard_msgs::action::Dump::Goal> goal);

        rclcpp_action::CancelResponse handle_dump_cmd_cancel(const std::shared_ptr<GoalHandleDumpCommand> goal_handle);

        void handle_dump_cmd_accepted(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const std::string & robotID);

        void executeDumpCmd(const std::shared_ptr<GoalHandleDumpCommand> goal_handle, const std::string & robotID);
        
        // Service callback
        void getMaterialAmountCallback(const std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Request> request,
          std::shared_ptr<material_handler_msgs::srv::GetMaterialAmount::Response> response);
            
    private:
        rclcpp::Node::SharedPtr node_;

        // Action servers
        std::vector<rclcpp_action::Server<standard_msgs::action::MoveToPose>::SharedPtr> action_servers_;
        std::vector<rclcpp_action::Server<standard_msgs::action::Load>::SharedPtr> load_action_servers_;
        std::vector<rclcpp_action::Server<standard_msgs::action::Dump>::SharedPtr> dump_action_servers_;
        std::vector<rclcpp::Service<material_handler_msgs::srv::GetMaterialAmount>::SharedPtr> material_amount_servers_;

        // Publishers
        std::map<std::string, rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr> metrics_pubs_;

        // Data
        std::vector<atlantis::util::Material> materials_;
        std::vector<std::string> robots_ids_, materials_ids_, waypoints_ids_, metrics_;
        std::map<std::string, bool> metrics_to_evaluate_;
        Robots robots_;
        std::map<std::string, double> amount_loaded_;
        std::map<std::string, double> capacities_;
        std::vector<atlantis::util::Waypoint> waypoints_;
        std::map<std::string, std::string> robots_current_locations_;
        
        double material_flow_amount_ = 0.;
        bool refilling_ = false;
        bool randomness_ = false;
        int execution_sec_;
};
}

#endif