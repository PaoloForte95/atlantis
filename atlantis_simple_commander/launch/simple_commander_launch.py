from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    
    scenario = LaunchConfiguration('scenario')

    declare_scenario_params_file_cmd = DeclareLaunchArgument(
        'scenario',
        default_value="",
        description='Full path to the scenario to simulate')

    start_simple_commander_cmd = Node(
                    name = 'atlantis_simple_commander',
                    package='atlantis_simple_commander',
                    executable='simple_robot_commander',
                    parameters=[scenario],
                    output="screen")
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_scenario_params_file_cmd)
    ld.add_action(start_simple_commander_cmd)
    return ld