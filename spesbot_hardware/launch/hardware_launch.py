import pathlib
import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    package_dir = get_package_share_directory('spesbot_hardware')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'description.urdf')).read_text()
    controller_params_file = os.path.join(package_dir, 'resource', 'controllers.yaml')
    
    camera = LaunchConfiguration('camera', default='')

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
                'camera_frame_id': 'camera',
            }
        ],
        condition=IfCondition(PythonExpression(['"', camera, '" == "v4l2"'])),
    )

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        output='screen',
        reamappings=[
            ('/color/camera_info', '/camera_info'),
            ('/color/image_raw', '/image_raw'),
        ],
        parameters=[{
            'base_frame_id': 'camera',
        }],
        condition=IfCondition(PythonExpression(['"', camera, '" == "realsense"'])),
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

    move_command = Node(
        package='spes_move',
        executable='move',
        output='screen',
        parameters=[
            {
                'angular.max_acceleration' : 0.5,
                'angular.max_velocity' : 1.2,
                'linear.max_acceleration' : 0.8,
                'linear.max_velocity' : 0.3
            }
        ],
    )

    return LaunchDescription([
        diffdrive_controller_spawner,
        tf_base_link_laser,
        controller_manager_node,
        move_command,
        # lidar,
        tf_base_link_base_footprint,
        v4l2,
        realsense
    ])
