from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            namespace='sim1',
            output='screen',
            respawn=True,
        ),

        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_key',
            namespace='sim1',
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            namespace='sim2',
            output='screen',
            respawn=True,
        ),

        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_key',
            namespace='sim2',
            output='screen',
            prefix='xterm -e',
        ),
    ])