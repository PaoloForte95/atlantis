
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atlantis_scene_graph',
            executable='atlantis_scene_graph_generator',
            name='atlantis_scene_graph_generator',
            output='screen',
            parameters=[],
        ),
    ])
