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
import yaml
import pprint
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition



def load_yaml(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)
    
ld = LaunchDescription()
use_gui = LaunchConfiguration('use_gui')
gazebo_dir = get_package_share_directory('atlantis_gazebo')

def set_up_simulation(context):
    scenario = context.launch_configurations['scenario']
    ld.add_action(LogInfo(msg=['Simulating scenario: ', scenario]))
    config = load_yaml(scenario)
    if 'atlantis_gazebo_simulator' in config:
        scenario_param = config['atlantis_gazebo_simulator'].get('ros__parameters', {})
    else:
        scenario_param = {}
    
    # launch_configs = {}
    
    # def generate_launch_configurations(data, prefix=''):
        
    #     nonlocal launch_configs
    #     for key, value in data.items():
    #         if isinstance(value, dict):
    #             generate_launch_configurations(value, prefix + key + '_')
    #         else:
    #             launch_configs[prefix + key] = LaunchConfiguration(prefix + key, default=value)
    #             context.launch_configurations[prefix + key] = value

    #generate_launch_configurations(scenario_param)

    start_robots_cmds = []
    for robot in scenario_param["robots"]:
        rb = scenario_param[robot]
        name = "robot" + str(rb["ID"])
        initial_pose = rb['initial_pose']
        
        group = GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'wheel_loader_sim.py')),
            condition= IfCondition(EqualsSubstitution(LaunchConfiguration('machine'), "wheel_loader")),
            launch_arguments={'namespace': name,
                          'robot_name': name,
                           'use_sim_time': 'True',
                           'rviz_config_file': scenario_param["rviz_config_file"],
                           'map': scenario_param["map"],
                           'world': scenario_param["world"],
                           'robot_params_file' : rb["param_file"],
                           'initial_pose_x': str(initial_pose['x']),
                           'initial_pose_y': str(initial_pose['y']),
                           'initial_pose_z': '0.4',
                           'initial_pose_yaw': str(initial_pose['yaw']),
                           'slam': 'False',
                            'autostart': 'True',
                          }.items()),
    
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'a25_sim.py')),
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration('machine'), "a25")),
                launch_arguments={'namespace': name,
                          'robot_name': name,
                           'use_sim_time': 'True',
                           'rviz_config_file': scenario_param["rviz_config_file"],
                           'map': scenario_param["map"],
                           'world': scenario_param["world"],
                           'robot_params_file' : rb["param_file"],
                           'initial_pose_x': str(initial_pose['x']),
                           'initial_pose_y': str(initial_pose['y']),
                           'initial_pose_z': '0.4',
                           'initial_pose_yaw': str(initial_pose['yaw']),
                           'slam': 'False',
                            'autostart': 'True',
                          }.items())
            ])
        start_robots_cmds.append(group)


    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', scenario_param['world']],
        cwd=[gazebo_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
         condition=IfCondition(use_gui),
         cmd=['gzclient'],
         cwd=[gazebo_dir], output='screen')
    
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
        
    for start_robots_cmd in start_robots_cmds:
        ld.add_action(start_robots_cmd)
    


def generate_launch_description():
    # Default dirs

    # Create the launch configuration variables
    declare_machine_type_cmd = DeclareLaunchArgument(
        'machine',
        default_value='a25',
        description='Full path to the RVIZ config file to use')
    
    
    declare_use_gui_cmd = DeclareLaunchArgument(
        'use_gui',
        default_value='False',
        description='Whether to start the GAZEBO Client')

    ld.add_action(OpaqueFunction(function=set_up_simulation))

    # Declare the launch options
    ld.add_action(declare_machine_type_cmd)
    ld.add_action(declare_use_gui_cmd)
    

    return ld