from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    LogInfo,
    Shutdown,
)


def launch_setup(context, *args, **kwargs):

    joint_name_value = LaunchConfiguration("joint_name")
    desired_angle_value = LaunchConfiguration("desired_angle")
    stepsize_value = LaunchConfiguration("stepsize")

    action_client_node = Node(
        package="pantilt_action_tutorials_py",
        executable="pantilt_action_client",
        parameters=[{
            'joint_name': joint_name_value,
            'desired_angle': desired_angle_value,
            'stepsize': stepsize_value,
        }],
    )

    nodes_to_start = [
        action_client_node,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    # Add arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_name",
            default_value='tilt_joint',
            description='Specifies the joint to move'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "desired_angle",
            default_value='60',
            description='Specifies the joint to move'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "stepsize",
            default_value='5',
            description='Specifies the joint to move'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])



