// Copyright (c) 2023 Paolo Forte
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

#ifndef ATLANTIS__BASE_RVIZ_VISUALIZATION_HPP_
#define ATLANTIS__BASE_RVIZ_VISUALIZATION_HPP_


// Boost
#include <boost/program_options.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <navigo/costmap/costmap.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
namespace po = boost::program_options;


namespace atlantis_base
{
    /**
     * @brief A Rviz visualization to visualize the robot moving 
     * 
     */
class RvizVisualization{

    public:

        RvizVisualization();
        
        ~RvizVisualization();

    void startVisualization();


    void publishRobot(int ID, double x, double y, double theta, std::string mesh_model = "", int color = 1);


    void publishMaterial(std::string name, geometry_msgs::msg::Pose pose, int ID, double amount, double start_z = 0, int color = 1);

    void publishFootprint(double x, double y, double theta,
                        std::vector<geometry_msgs::msg::Point> footprint,
                        const std::string &frame_id,
                        int id,
                        double start_z = 0,
                        int color = 1);


    void publishRobotInfo(double x, double y, double theta, double distance, double amount_loaded, const std::string &frame_id, int id);

    void convertCostMapToMsg(const double resolution,
                            const double x_cells,
                            const double y_cells,
                            const std::vector<unsigned char> data,
                            nav_msgs::msg::OccupancyGrid& msg);


    protected:
        std::map<int, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> robots_pub_;
        std::map<int, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> robots_info_pub_;
        std::map<int, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> robots_footprints_pubs_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr materials_marker_pub_;
        std::map<int, visualization_msgs::msg::MarkerArray> material_marker_array_;
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::Logger logger_ {rclcpp::get_logger("Base Simulator")};
        
};
}

#endif
