from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node (
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1',
        ),
        Node (
            package='topic_tutorial_py',
            executable='pub_vel_node',
            name='vel_publisher',
            output='screen'
        ),
        Node (
            package='topic_tutorial_py',
            executable='sub_pose_node',
            name='pose_subscriber',
            output='screen'
        ),


    ])