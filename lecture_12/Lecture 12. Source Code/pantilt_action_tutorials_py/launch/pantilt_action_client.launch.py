from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    LogInfo,
    Shutdown,
)

def generate_launch_description():

    action_client_node = Node(
        package="pantilt_action_tutorials_py",
        executable="pantilt_action_client",
        parameters=[{
            'joint_name': "tilt_joint",
            'desired_angle': 10,
            'stepsize': 1,
        }],
    )

    # Description
    ld =  LaunchDescription()

    ld.add_action(action_client_node)

    return ld




