from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo, Shutdown
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():   
    launch_rviz = LaunchConfiguration("launch_rviz", default='true')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("urdf_tutorial"), "rviz", "display_urdf.rviz"]
    )

    # Get the absolute path to the URDF file
    urdf_path = os.path.join(get_package_share_path('urdf_tutorial'), 'urdf', 'pan_tilt.urdf')

    # Log the URDF path for debugging
    log_urdf_path = LogInfo(
        msg=f"URDF Path: {urdf_path}"
    )

    # Load the URDF file directly into the robot description
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    robot_description = {"robot_description": robot_description_content}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": False}, robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
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

    # Launch description
    ld = LaunchDescription()

    ld.add_action(log_urdf_path)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz_node)

    return ld
