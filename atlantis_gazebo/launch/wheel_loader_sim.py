# Copyright (c) 2023 Paolo Forte
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch File for the volvo L70 wheel loader """

import os
import yaml
import pprint
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,GroupAction, LogInfo, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString
import functools



def generate_launch_description():
    # Get the launch directory
 

    gazebo_dir = get_package_share_directory('wheel_loader_gazebo')
    description_dir = get_package_share_directory('wheel_loader_description')
    launch_dir = get_package_share_directory('wheel_loader_launch')
    area_filter_dir = get_package_share_directory('navigo2_costmap_2d')
   
    # Create the launch configuration variables
    

    autostart = LaunchConfiguration('autostart')
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    area_filter_yaml_file =  LaunchConfiguration('mask')
    use_area_filter = LaunchConfiguration('use_area_filter')
    log_level = LaunchConfiguration('log_level')
    slam = LaunchConfiguration('slam')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')


    #ld.add_action(LogInfo(msg=['Simulating scenario print2', test]))
    tool_lifecycle_nodes = ['bucket_controller_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('cmd_vel', 'controller/cmd_vel')
                  ]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='Robot Name')

    
    declare_use_area_filter_cmd = DeclareLaunchArgument(
        'use_area_filter',
        default_value='False',
        description='Whether to use the area filter to detect when entering a new area.')
    
    declare_area_filter_yaml_cmd = DeclareLaunchArgument(
        'area_filter_yaml_file',
        default_value=os.path.join(
            area_filter_dir, 'maps', 'empty_segmented.yaml'),
        description='Full path to the segmented map file to load for the filter')

    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Full path to world model file to load')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map file to load')
    
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='The x value of the initial pose')
    
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='The y value of the initial pose')
    
    declare_initial_pose_z_cmd = DeclareLaunchArgument(
        'initial_pose_z',
        default_value='0.4',
        description='The z value of the initial pose')
    
    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='The yaw value of the initial pose')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value='',
        description='Full path to the RVIZ config file to use')
    
    declare_robot_params_file_cmd = DeclareLaunchArgument(
        'robot_params_file',
        default_value='',
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')
    

    spawn_robot_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'wheel_loader_empty.py')),
    launch_arguments={'namespace': namespace,
                        'robot_name': robot_name,
                        'x_pose': initial_pose_x,
                        'y_pose': initial_pose_y,
                        'yaw': initial_pose_yaw,
                        'world':world,
                        'use_gui': 'False',
                        'use_simulator': 'False'
                        }.items())

    default_nav_to_pose_bt_xml = os.path.join(
            get_package_share_directory('navigo2_behavior_tree'),
            'behavior_trees', 'navigate_to_pose_w_selection_replanning_and_recovery.xml')
    


    params_file = LaunchConfiguration('robot_params_file')
    params_file = ReplaceString(
            source_file=params_file,
            replacements={'<robot_namespace>': ('/', robot_name),
                          '//': ('/')
                          })
    nav_instances_cmds = []
    group = GroupAction([
    Node(
                name= 'origin_broadcaster',
                package='tf2_ros',
                namespace = robot_name,
                executable='static_transform_publisher',
                remappings=remappings,
                arguments = ['--frame-id','world', '--child-frame-id', 'map']),
    
    IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(description_dir, 'launch', 'wheel_loader_rviz.py')),
                launch_arguments={
                                'namespace': namespace,
                                'use_namespace': 'True',
                                'use_robot_state_pub': 'False',
                                'use_sim_time': 'True',
                                'use_joint_state_publisher_gui': 'False',
                                'rviz_config': rviz_config_file}.items()),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(area_filter_dir, 'launch', 'area_filter.py')),
        condition=IfCondition(use_area_filter),
        launch_arguments={'namespace': namespace,
                          'mask': area_filter_yaml_file,
                          }.items()),

        Node(
                package='wheel_loader_bucket_controller',
                executable='bucket_controller_server',
                name='bucket_controller_server',
                namespace =robot_name,
                output='screen',
                respawn_delay=2.0,
                parameters=[params_file],
                arguments=['--ros-args', '--log-level', log_level]),

        Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_tool_controller',
                namespace =robot_name,
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': tool_lifecycle_nodes},
                            {'bond_timeout': 25.0},
                            ]),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch', 'wheel_loader_navigation.py')),
        launch_arguments={'namespace': namespace,
                          'robot_name': robot_name,
                          'use_namespace': 'True',
                           'use_simulator': 'False',
                           'use_sim_time': 'True',
                           'use_rviz': 'False',
                           'headless': 'False',
                           'autostart': 'True',
                           'rviz_config_file': rviz_config_file,
                           'map': map_yaml_file,
                           'params_file' : params_file,
                           'use_robot_state_pub': 'False',
                           'controller_prefix': 'controller/',
                           'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                           'use_composition': 'False',
                           'use_selector': 'True',
                            'set_initial_pose': 'True',
                           'initial_pose_x': initial_pose_x,
                           'initial_pose_y': initial_pose_y,
                           'initial_pose_z': initial_pose_z,
                           'initial_pose_yaw': initial_pose_yaw,
                            'slam': slam,
                          }.items()),
        
        ])
    nav_instances_cmds.append(group)

    

    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_area_filter_cmd)
    ld.add_action(declare_area_filter_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_z_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_robot_params_file_cmd)

    ld.add_action(spawn_robot_cmd)
    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld

