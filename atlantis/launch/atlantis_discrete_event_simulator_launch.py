import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,GroupAction,TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
def generate_launch_description():

    launch_dir = get_package_share_directory('atlantis')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')
    scenario = LaunchConfiguration('scenario')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(launch_dir, 'launch', 'atlantis.rviz'),
        description='Full path to the RVIZ config file to use.')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml',
        default_value="",
        description='Full path to map file to load')
    
    declare_scenario_params_file_cmd = DeclareLaunchArgument(
        'scenario',
        default_value="",
        description='Full path to the scenario to simulate')

    simulator_launch_cmd = GroupAction(
         actions=[Node(
                    name = 'atlantis_discrete_event_simulator',
                    package='atlantis_discrete_event',
                    executable='atlantis_discrete_event_simulator',
                    parameters=[scenario],
                    output="screen")],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_scenario_params_file_cmd)
    
    ld.add_action(simulator_launch_cmd)

    return ld