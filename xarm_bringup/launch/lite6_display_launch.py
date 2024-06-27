import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('urdf_tutorial'),
                    'launch',
                    'display.launch.py'
                )
            ),
            launch_arguments=[
                ('model', os.path.join(get_package_share_directory(
                    'xarm_bringup'), 'urdf', 'lite6.urdf.xacro'))
            ]
        )
    ])
