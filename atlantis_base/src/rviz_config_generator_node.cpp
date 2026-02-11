#include <memory>

#include "atlantis_base/rviz_config_generator.h"
#include "rclcpp/rclcpp.hpp"
#include <boost/program_options.hpp>    
    
    
    int main(int argc, char *argv[]) 
    {

      rclcpp::init(argc, argv);
  
        auto node = std::make_shared<atlantis_base::RvizConfigModifierNode>("rviz_config_generator_node");
        rclcpp_lifecycle::State state;
        node->on_activate(state);
        node->on_configure(state);
        rclcpp::spin(node->get_node_base_interface());
        rclcpp::shutdown();

    return 0;
}
