import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,GroupAction,TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    launch_dir = get_package_share_directory('atlantis')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')
    map_yaml = LaunchConfiguration('map_yaml')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(launch_dir, 'launch', 'fleet_coordination_test.rviz'),
        description='Full path to the RVIZ config file to use.')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml',
        default_value="",
        description='Full path to map file to load')
    

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        namespace = namespace,
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
    
    static_transform_node = Node(
                    name= 'world_map_broadcaster',
                    package='tf2_ros',
                    namespace = namespace,
                    executable='static_transform_publisher',
                    arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw','0', '--pitch', '0', '--roll', '0', '--frame-id','world', '--child-frame-id', 'map'])
    
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(start_rviz_cmd)
    ld.add_action(static_transform_node)
    return ld