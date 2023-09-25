import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('spesbot_webots'), 'launch', 'webots_launch.py')
        ),
        condition=launch.conditions.IfCondition(use_sim_time),
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('spesbot_hardware'), 'launch', 'hardware_launch.py')
        ),
        condition=launch.conditions.UnlessCondition(use_sim_time),
        launch_arguments={
            'camera': 'realsense',
        }.items()
    )

    move = Node(
        package='spes_move',
        executable='move',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'angular.max_velocity': 0.3,
            'angular.max_acceleration': 0.3,
            'angular.tolerance': 0.001,
            'update_rate': 100,
        }],
    )

    behavior = Node(
        package='spes_behavior',
        executable='behavior',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'behavior': 'test_translate_obstacle',
        }],
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('spes_tests'), 'resource', 'apriltag.yaml'),
        ],
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
    )

    tf_target_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--yaw', '1.5708',
            '--pitch', '1.5708',
            '--frame-id', 'target_frame_raw',
            '--child-frame-id', 'target_frame'
        ],
    )

    return launch.LaunchDescription([
        webots,
        move,
        behavior,
        hardware,
        apriltag,
        tf_target_frame,
    ])
