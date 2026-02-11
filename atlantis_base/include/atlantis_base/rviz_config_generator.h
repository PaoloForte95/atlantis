#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <fstream>
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace atlantis_base
{


class RvizConfigModifierNode : public rclcpp_lifecycle::LifecycleNode {
public:
    RvizConfigModifierNode(const std::string & node_name,
        const std::string & ns = "",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    void writeModifiedConfigToFile(const YAML::Node &root, const std::string &outputFile);
    void addPathToGroup(YAML::Node &root, const std::vector<std::string> &robotNames);

    void addMarkerDisplayToGroup(YAML::Node &root, const std::vector<std::string> &robotNames, const std::string &groupName, const std::string &topicSuffix);


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
    

};

}
