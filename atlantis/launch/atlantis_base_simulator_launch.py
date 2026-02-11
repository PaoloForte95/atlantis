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
    
    
    declare_scenario_params_file_cmd = DeclareLaunchArgument(
        'scenario',
        default_value="",
        description='Full path to the scenario to simulate')
    
    # generate_rviz_config_file_cmd = Node(
    #         package='atlantis_base',  # Name of your package
    #         executable='rviz_config_generator_node',  # Name of the compiled node executable
    #         name='rviz_config_generator_node',  # Node name
    #         output='screen',
    #         parameters=[
    #         {'simulator_node_name': 'atlantis_base_simulator',
    #          'input_file' : 'src/atlantis/atlantis/launch/atlantis_def.rviz',
    #          'output_file' : 'src/atlantis/atlantis/launch/atlantis.rviz',
    #         }
    #         ]
    #     )
    
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
    
    simulator_launch_cmd = GroupAction(
         actions=[Node(
                    name = 'atlantis_base_simulator',
                    package='atlantis_base',
                    executable='atlantis_base_simulator',
                    parameters=[scenario],
                    output="screen")],
    )

    state_generator_launch_cmd = Node(
                package='atlantis_state',
                executable='state_generator',
                name='state_generator',
                output='screen',
                parameters=[scenario])
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_scenario_params_file_cmd)
    
    
    ld.add_action(static_transform_node)
    
    ld.add_action(start_rviz_cmd)
    ld.add_action(state_generator_launch_cmd)
    ld.add_action(TimerAction(period=1.0, actions=[simulator_launch_cmd]))
    #ld.add_action(TimerAction(period=1.0, actions=[generate_rviz_config_file_cmd]))

    return ld