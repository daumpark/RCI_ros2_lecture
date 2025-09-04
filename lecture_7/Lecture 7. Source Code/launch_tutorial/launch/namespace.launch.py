from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1',
            namespace='sim1',
        ),
        Node (
            package='turtlesim',
            executable='turtle_teleop_key',
            name='telop_key',
            output='screen',
            prefix='xterm -e',
            namespace='sim1',
        ),

        Node (
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1',
            namespace='sim2',
        ),

        Node (
            package='turtlesim',
            executable='turtle_teleop_key',
            name='telop_key',
            output='screen',
            prefix='xterm -e',
            namespace='sim2',
        ),
    ])