import pathlib
import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('spesbot_driver')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'ros2_control.urdf')).read_text()
    controller_params_file = os.path.join(package_dir, 'resource', 'ros2_control.yaml')

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/diffdrive_controller/odom', 'odom'),
        ],
        output='screen'
    )

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller', '--controller-manager-timeout', '50'],
    )

    v4l2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video4',
                'brightness': 0,
                'contrast': 62,
                'saturation': 74,
                'sharpness': 100,
            }
        ],
    )

    tf_base_link_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ],
    )

    tf_base_link_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint'
        ],
    )

    lidar = Node(
        package='hls_lfcd_lds_driver',
        executable='hlds_laser_publisher',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0', 'frame_id': 'laser'}
        ]
    )

    return LaunchDescription([
        diffdrive_controller_spawner,
        tf_base_link_laser,
        controller_manager_node,
        lidar,
        tf_base_link_base_footprint,
        v4l2
    ])
