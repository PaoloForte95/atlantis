#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "atlantis_state/state_generator_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<atlantis_state::StateGeneratorNode>();
  rclcpp_lifecycle::State state;
  node->on_configure(state);
  node->on_activate(state);
  rclcpp::spin(node->get_node_base_interface());
  node->on_deactivate(state);
  rclcpp::shutdown();

  return 0;
 
}