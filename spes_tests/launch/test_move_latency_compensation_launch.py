import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'spesbot_webots'), 'launch', 'webots_launch.py')
        )
    )

    move = Node(
        package='spes_move',
        executable='move',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'angular.max_velocity': 0.3,
            'angular.max_acceleration': 0.3,
            'angular.tolerance': 0.001,
            'update_rate': 100,
        }],
    )

    test_move_latency_compensation = Node(
        package='spes_tests',
        executable='test_move_latency_compensation',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return launch.LaunchDescription([
        webots,
        move,
        test_move_latency_compensation
    ])
