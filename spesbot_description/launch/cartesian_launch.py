import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    robot_description_path = os.path.join(get_package_share_directory(
        'spesbot_description'), 'urdf', 'spesbot.xacro')
    robot_description = xacro.process_file(robot_description_path, mappings={}).toprettyxml(indent='  ')
    controllers = os.path.join(get_package_share_directory(
        'spesbot_hardware'), 'resource', 'controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers
        ],
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame')
        ],
        output='screen'
    )

    cartesian_motion_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_motion_controller'],
        output='screen'
    )

    motion_control_handle_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['motion_control_handle'],
        output='screen'
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'spesbot_description'), 'resource', 'cartesian_demo.rviz')],
        output='screen'
    )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher,
        cartesian_motion_controller_spawner,
        motion_control_handle_spawner,
        joint_state_broadcaster_spawner,
        rviz
    ])
