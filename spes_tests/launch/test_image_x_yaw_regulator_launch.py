import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true'
    )

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('spesbot_webots'),
                'launch',
                'webots_launch.py',
            )
        ),
        condition=launch.conditions.IfCondition(use_sim_time),
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('spesbot_hardware'),
                'launch',
                'hardware_launch.py',
            )
        ),
        condition=launch.conditions.UnlessCondition(use_sim_time),
    )

    # Note: This will work only in Docker
    object_tracker = ExecuteProcess(
        cmd=[
            '/spesbot/ros2_ws/src/spesbot/cuda/object_tracker/object_tracker',
            '--source',
            'ros',
        ],
        output='screen',
        condition=launch.conditions.IfCondition(use_sim_time),
    )

    move = Node(
        package='spes_move',
        executable='move',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'angular.max_velocity': 0.3,
                'angular.max_acceleration': 0.3,
                'angular.tolerance': 0.001,
                'update_rate': 100,
            }
        ],
    )

    image_x_yaw_regulator = Node(
        package='spes_move',
        executable='image_x_yaw_regulator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    behavior = Node(
        package='spes_behavior',
        executable='behavior',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'behavior': 'test_image_x_yaw_regulator',
            }
        ],
    )

    return launch.LaunchDescription(
        [webots, move, image_x_yaw_regulator, object_tracker, behavior, hardware]
    )
