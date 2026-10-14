from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description():
    bag_name = 'plan_run_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    record_arg = DeclareLaunchArgument('record', default_value='true')
    foxglove_arg = DeclareLaunchArgument('foxglove', default_value='true')

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('foxglove')),
        parameters=[{'port': 8765}],
        output='screen'
    )

    bag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record')),
        cmd=['ros2', 'bag', 'record', '-o', bag_name,
             '/plan_actions', '/atlantis_scene_graph'],
        output='screen'
    )


    return LaunchDescription([
        record_arg,
        foxglove_arg,
        foxglove_bridge,
        bag_record,
    ])