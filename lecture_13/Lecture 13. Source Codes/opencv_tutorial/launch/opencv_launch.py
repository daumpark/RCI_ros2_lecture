from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='opencv_tutorial',
            executable='opencv_node',
            name='opencv_node',
            output='screen',
        ),
    ])
