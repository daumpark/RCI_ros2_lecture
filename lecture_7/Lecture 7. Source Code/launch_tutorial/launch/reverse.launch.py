from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1',
            remappings=[('/turtle1/cmd_vel', '/turtle1/cmd_vel_reverse')],
        ),

        Node (
            package='launch_tutorial',
            executable='reverse_node',
            name='reverse',
        ),

        Node (
            package='turtlesim',
            executable='turtle_teleop_key',
            name='turtle1',
            output='screen',
            prefix='xterm -e',
        ),


    ])