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

    mode_value = LaunchConfiguration("mode")

    description_package = LaunchConfiguration("description_package", default='pantilt_action_tutorials_py')
    description_file = LaunchConfiguration("description_file", default='pan_tilt.xacro')
    
    launch_rviz = LaunchConfiguration("launch_rviz", default='true')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("pantilt_action_tutorials_py"), "rviz", "pan_tilt.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    action_server_node = Node(
        package="pantilt_action_tutorials_py",
        executable="pantilt_action_server",
        parameters=[{
            'mode': mode_value,
        }],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        on_exit=Shutdown(),
    )

    nodes_to_start = [
        action_server_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    # Add arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value='REJECT', #ABORT
            description='Specifies how to handle a new goal petition when the server is executing a previous petition'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])







