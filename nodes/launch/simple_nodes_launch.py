# nodes/launch/simple_nodes_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='physical_ai_nodes',
            executable='simple_talker',
            name='simple_talker',
            output='screen'
        ),
        Node(
            package='physical_ai_nodes',
            executable='simple_listener',
            name='simple_listener',
            output='screen'
        )
    ])