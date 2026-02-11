# Copyright (c) 2022 Paolo Forte
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

"""Launch File for the a25 volvo truck """

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString



def generate_launch_description():
    # Get the launch directory

    a25_gazebo_dir = get_package_share_directory('a25_gazebo')
    description_dir = get_package_share_directory('a25_description')
    a25_launch_dir = get_package_share_directory('a25_launch')
    worlds_dir = get_package_share_directory('ecceleron_gazebo_worlds')
    area_filter_dir = get_package_share_directory('navigo2_costmap_2d')
    weather_detector_dir = get_package_share_directory('navigo2_weather_detection')
  
    # Create the launch configuration variables
    

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    use_a25_controller = LaunchConfiguration('use_a25_controller')
    use_ecceleron_odometry = LaunchConfiguration('use_ecceleron_odometry')
    output_control_topic = LaunchConfiguration('output_control_topic')
    angular_controller_type = LaunchConfiguration('angular_controller_type')
    linear_controller_type = LaunchConfiguration('linear_controller_type')
    use_gui = LaunchConfiguration('use_gui')
    area_filter_yaml_file =  LaunchConfiguration('mask')

    robots = [{'name': 'robot1', 'platform': 'a25', 'x_pose': '0.0', 'y_pose': '-2.0', 'z_pose': '0.4',
                           'roll': '0.0', 'pitch': '0.0', 'yaw': '1.57'}]

    #robots = [{'name': 'robot1', 'platform': 'a25', 'x_pose': '22.0', 'y_pose': '41.3', 'z_pose': '0.4',
                           #'roll': '0.0', 'pitch': '0.0', 'yaw': '1.57'}]

    #Field
    #robots = [
        #{'name': 'robot1', 'platform': 'a25', 'x_pose': '-4.45', 'y_pose': '10.0', 'z_pose': '0.4',
                           #'roll': '0.0', 'pitch': '0.0', 'yaw': '0.00'}]

    #Small city
    # robots = [
    #     {'name': 'robot1', 'platform': 'a25', 'x_pose': '-40.0', 'y_pose': '0.0', 'z_pose': '0.4',
    #                        'roll': '0.0', 'pitch': '0.0', 'yaw': '0.0'}]
    
    #Mine
    #robots = [
        #{'name': 'robot1', 'platform': 'a25', 'x_pose': '-32.0', 'y_pose': '63.9', 'z_pose': '0.4',
                           #'roll': '0.0', 'pitch': '0.0', 'yaw': '0.00'}]
  
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('cmd_vel', 'controller/cmd_vel')
                  ]


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(description_dir, 'launch', 'a25_namespaced.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(worlds_dir, 'worlds', 'empty.world'),
        description='Full path to world model file to load')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            worlds_dir, 'maps', 'empty.yaml'),
        description='Full path to map file to load')

    declare_robot_params_file_cmd = DeclareLaunchArgument(
        'robot_params_file',
        default_value=os.path.join(a25_launch_dir, 'params', 'a25_navigation_configs.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    declare_use_a25_controller_cmd = DeclareLaunchArgument(
        'use_a25_controller',
        default_value='false',
        description='Whether to start the a25 low level controller')
    
    declare_use_ecceleron_odometry_cmd = DeclareLaunchArgument(
        'use_ecceleron_odometry',
        default_value='false',
        description='Whether to use the ecceleron odometry')

    declare_output_control_topic_cmd = DeclareLaunchArgument(
        'output_control_topic',
        default_value='/cmd_vel_to_machine',
        description='The control topic name')

    declare_angular_controller_type_cmd = DeclareLaunchArgument(
        'angular_controller_type',
        default_value= '0',
        description='The angular control type to use ')

    declare_linear_controller_type_cmd = DeclareLaunchArgument(
        'linear_controller_type',
        default_value= '0',
        description='The linear control type to use ')

    declare_use_gui_cmd = DeclareLaunchArgument(
        'use_gui',
        default_value='False',
        description='Whether to start the GAZEBO Client')
    
    
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(a25_gazebo_dir, 'launch', 'a25_empty.py')),
        launch_arguments={'namespace': robot['name'],
                          'robot_name': robot['name'],
                          'x_pose': robot['x_pose'],
                          'y_pose': robot['y_pose'],
                          'yaw': robot['yaw'],
                          'world':world,
                          'use_gui': use_gui,
                          }.items()))

    default_nav_to_pose_bt_xml = os.path.join(
            get_package_share_directory('ecceleron_bringup'),
            'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
    
    nav_instances_cmds = []
    for robot in robots:
        #params_file = LaunchConfiguration(robot['name'] + '_params_file')
        params_file = LaunchConfiguration('robot_params_file')
        params_file = ReplaceString(
            source_file=params_file,
            replacements={'<robot_namespace>': ('/', robot['name']),
                          '//': ('/')
                          })
        
        group = GroupAction([
        Node(
                    name= robot['name']+'_origin_broadcaster',
                    package='tf2_ros',
                    namespace = robot['name'],
                    executable='static_transform_publisher',
                    remappings=remappings,
                    arguments = ['--frame-id','world', '--child-frame-id', 'map']),
        
        Node(
                    condition=IfCondition(use_ecceleron_odometry),
                    name= robot['name']+'_odom_to_world',
                    package='tf2_ros',
                    namespace = robot['name'],
                    executable='static_transform_publisher',
                    remappings=remappings,
                    arguments = ['--frame-id','odom', '--child-frame-id', 'world']),


        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                            os.path.join(description_dir, 'launch', 'a25_rviz.py')),
                    launch_arguments={
                                    'namespace': robot['name'],
                                    'use_namespace': 'True',
                                    'use_robot_state_pub': 'False',
                                    'use_sim_time': 'True',
                                    'use_joint_state_publisher_gui': 'False',
                                    'rviz_config': rviz_config_file}.items()),

        Node(
            condition=IfCondition(use_a25_controller),
            package="a25_controller",
            executable="velocity_steering_controller_node",
            name="a25_controller",
            remappings=remappings,
            namespace=robot['name'],
            output='screen',
            parameters=[
                {'output_control_topic': output_control_topic,
                #'steering_valve_fuzzy_model_parameters_path': steering_valve_fuzzy_model_parameters_path,
                'angular_controller_type' : angular_controller_type,
                'linear_controller_type' : linear_controller_type,
                'input_reference_topic': "/controller/cmd_vel",
                'input_state_topic': "/velocity_steering"
                }]
        ), 

        Node(
            condition=IfCondition(use_ecceleron_odometry),
            package="ecceleron_odometry",
            executable="articulated_odometry_node",
            name="articulated_odometry_node",
            remappings=remappings,
            namespace=robot['name'],
            output='screen',
            parameters=[
                {'length_front_axis': '1.2',
                'length_rear_axis' : '3.8',
                'steering_angle_offset' : '0.0'
                }]
        ), 
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(a25_launch_dir, 'launch', 'a25_navigation.py')),
        launch_arguments={'namespace': robot['name'],
                          'robot_name': robot['name'],
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
                           'set_initial_pose': 'True',
                           'initial_pose_x': robot['x_pose'],
                           'initial_pose_y': robot['y_pose'],
                           'initial_pose_z': robot['z_pose'],
                           'initial_pose_yaw': robot['yaw'],
                           
                          }.items()),
        
        ])
        nav_instances_cmds.append(group)

    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_a25_controller_cmd)
    ld.add_action(declare_use_ecceleron_odometry_cmd)
    ld.add_action(declare_output_control_topic_cmd)
    ld.add_action(declare_angular_controller_type_cmd)
    ld.add_action(declare_linear_controller_type_cmd)
    ld.add_action(declare_use_gui_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)
    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)


    return ld