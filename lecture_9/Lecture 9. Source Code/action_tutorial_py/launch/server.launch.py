from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_tutorial_py',
            executable='fibonacci_action_server',
            name='fibonacci_action_server'
        )
    ])