import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    launch_dir = get_package_share_directory('atlantis')
    simple_cm_dir = get_package_share_directory('atlantis_simple_commander')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    scenario = LaunchConfiguration('scenario')
    use_simple_commander = LaunchConfiguration('use_simple_commander')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_simulator_level_cmd = DeclareLaunchArgument(
        'simulator_level',
        default_value='',
        description='The simulator to use')
    
    declare_scenario_params_file_cmd = DeclareLaunchArgument(
        'scenario',
        default_value="",
        description='Full path to the scenario to simulate')
    
    declare_use_simple_commander_cmd = DeclareLaunchArgument(
        'use_simple_commander',
        default_value='False',
        description='Whenever to use the simple commander')


    start_aiddl_simulator_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'atlantis_discrete_event_simulator_launch.py')),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulator_level'), "DES")),
        launch_arguments={'namespace': namespace,
                          'scenario': scenario,
                          }.items())
    
    start_base_simulator_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'atlantis_base_simulator_launch.py')),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulator_level'), "NPS")),
        launch_arguments={'namespace': namespace,
                          'scenario': scenario,
                          }.items())
    
    # start_gazebo_simulator_cmd =  IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'atlantis_gazebo_simulator_launch.py')),
    #     condition=IfCondition(EqualsSubstitution(LaunchConfiguration('simulator_level'), "PS")),
    #     launch_arguments={'namespace': namespace,
    #                       'scenario': scenario,
    #                       }.items())
    
    start_simple_commander_cmd =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simple_cm_dir,'simple_commander_launch.py')),
        condition=IfCondition(use_simple_commander),
        launch_arguments={'scenario': scenario,
                          }.items())
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_simulator_level_cmd)
    ld.add_action(declare_scenario_params_file_cmd)
    ld.add_action(declare_use_simple_commander_cmd)
    ld.add_action(start_aiddl_simulator_cmd)
    ld.add_action(start_base_simulator_cmd)
    ld.add_action(start_gazebo_simulator_cmd)
    ld.add_action(start_simple_commander_cmd)

    return ld