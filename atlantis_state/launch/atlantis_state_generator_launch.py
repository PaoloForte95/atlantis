
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atlantis_state',
            executable='atlantis_state_generator',
            name='atlantis_state_generator',
            output='screen',
            parameters=[],
        ),
    ])
