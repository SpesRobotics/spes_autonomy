from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    package_dir = get_package_share_directory("spesbot_description")
    xacro_file = os.path.join(package_dir, "urdf", "spesbot.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    config = os.path.join(
        get_package_share_directory("spesbot_isaac"), "config", "spesbot_control.yaml"
    )

    ld = LaunchDescription(
        [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{"robot_description": robot_description}, config],
                remappings=[
                    ("/diff_drive_controller/cmd_vel_unstamped", "cmd_vel"),
                    ("/diff_drive_controller/odom", "odom"),
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager-timeout",
                    "50",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "diff_drive_controller",
                    "--controller-manager-timeout",
                    "50",
                ],
            ),
        ]
    )

    return ld
