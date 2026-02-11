#include <atlantis_base/rviz_config_generator.h>
#include "rclcpp/parameter_client.hpp"

namespace atlantis_base
{


RvizConfigModifierNode::RvizConfigModifierNode(const std::string & node_name,
        const std::string & ns,
        const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("rviz_config_modifier",ns,options) {
        // Declare and get input/output file paths
        std::string robotConfigFile, input_file, output_file, simulator_node_name;
        declare_parameter("input_file", "atlantis_def.rviz");
        declare_parameter("output_file", "atlantis.rviz");
        declare_parameter("simulator_node_name", "atlantis_base_simulator");

        get_parameter("simulator_node_name", simulator_node_name);
        get_parameter("input_file", input_file);
        get_parameter("output_file", output_file);
        auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(this, simulator_node_name);

        if (!parameter_client->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Parameter service not available.");
            return;
        }
        
        std::vector<std::string> parameter_names{"robots"};
        auto params = parameter_client->get_parameters(parameter_names);

        std::vector<std::string> robotNames;
         for (const auto &param : params) {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
                auto string_array = param.as_string_array();
                for (const auto& robot : string_array) {
                    robotNames.push_back(robot);
                }
            } 
        }

        // Load the existing RViz configuration file
        YAML::Node root;
        try {
            root = YAML::LoadFile(input_file);
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading RViz configuration file: %s", e.what());
            return;
        }

        // Add paths for robots using their names
        addPathToGroup(root, robotNames);


        addMarkerDisplayToGroup(root, robotNames, "Robots", "marker");
        addMarkerDisplayToGroup(root, robotNames, "Footprints", "footprint_marker");
        addMarkerDisplayToGroup(root, robotNames, "Info", "info_marker");

        // Write the modified configuration back to a file
        writeModifiedConfigToFile(root, output_file);


        
    }



CallbackReturn
RvizConfigModifierNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
}


CallbackReturn
RvizConfigModifierNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return CallbackReturn::SUCCESS;
}


CallbackReturn
RvizConfigModifierNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return CallbackReturn::SUCCESS;
}


CallbackReturn
RvizConfigModifierNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
RvizConfigModifierNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}


// Function to add paths for each robot specified by a vector of robot names
void 
RvizConfigModifierNode::addPathToGroup(YAML::Node &root, const std::vector<std::string> &robotNames) {
    // Find the "Paths" group under the "Visualization Manager -> Displays"
    auto displays = root["Visualization Manager"]["Displays"];
    for (std::size_t i = 0; i < displays.size(); ++i) {
        if (displays[i]["Name"] && displays[i]["Name"].as<std::string>() == "Paths") {
            // Found the Paths group, add a path display for each robot name
            for (const auto &robotName : robotNames) {
                std::string pathName = robotName + " Path";

                YAML::Node path;
                path["Class"] = "rviz_default_plugins/Path";
                path["Enabled"] = true;
                path["Name"] = pathName;
                path["Color"] = "25; 255; 0";  // Example color
                path["Topic"]["Value"] = "/" + robotName + "/path"; // Topic uses the robot name
                path["Line Style"] = "Lines";
                path["Line Width"] = 0.03;

                // Add the new path node to the "Paths" group
                displays[i]["Displays"].push_back(path);
            }
        }
    }
}

// Function to write the modified YAML configuration to a file
void RvizConfigModifierNode::writeModifiedConfigToFile(const YAML::Node &root, const std::string &outputFile) {
    try {
        std::ofstream fout(outputFile);
        if (!fout.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("rviz_config_modifier"), "Error: Unable to open file for writing: %s", outputFile.c_str());
            return;
        }

        // Emit the modified YAML node to the output file
        fout << root;
        fout.close();
        RCLCPP_INFO(rclcpp::get_logger("rviz_config_modifier"), "Modified RViz configuration saved to: %s", outputFile.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rviz_config_modifier"), "Error writing to file: %s", e.what());
    }
}


void RvizConfigModifierNode::addMarkerDisplayToGroup(YAML::Node &root, const std::vector<std::string> &robotNames, const std::string &groupName, const std::string &topicSuffix) {
    // Find the specified group under "Visualization Manager -> Displays"
    auto displays = root["Visualization Manager"]["Displays"];
    for (std::size_t i = 0; i < displays.size(); ++i) {
        if (displays[i]["Name"] && displays[i]["Name"].as<std::string>() == groupName) {
            // Found the group, add a marker display for each robot name
            for (const auto &robotName : robotNames) {
                std::string markerName = robotName + " " + topicSuffix;
                std::string topicName = "/" + robotName + "/" + topicSuffix;

                YAML::Node marker;
                marker["Class"] = "rviz_default_plugins/Marker";
                marker["Enabled"] = true;
                marker["Name"] = markerName;
                marker["Topic"]["Value"] = topicName;  // Topic uses the robot name and suffix

                // Add the new marker node to the specified group
                displays[i]["Displays"].push_back(marker);
            }
        }
    }
}


}