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

    # General arguments
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


    # Description
    ld =  LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(action_server_node)    

    return ld




