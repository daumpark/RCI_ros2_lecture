from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1',
        ),

        Node(
            package='launch_tutorial',
            executable='random_velocity_node',
            name='random_velocity_node',
            parameters=[{'cmd_vel_rate': 1.0}]
        )
    ])
