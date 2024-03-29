import os
import pathlib
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    world = launch.substitutions.LaunchConfiguration('world', default='sample.wbt')

    package_dir = get_package_share_directory('spesbot_webots')

    controller_params = os.path.join(get_package_share_directory('spesbot_hardware'),
                             'resource', 'controllers.yaml')

    robot_description = pathlib.Path(
        os.path.join(package_dir, 'resource', 'description.urdf')).read_text()

    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'data',
                                               'worlds', world]), ros2_supervisor=True)
    webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',  # debugging
        emulate_tty=True,  # debugging
        parameters=[
            {
                'use_sim_time': True,
                'robot_description': robot_description
            },
            controller_params
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('/diffdrive_controller/odom', 'odom'),
        ],
        ros_arguments=['--log-level', 'warn'],
        additional_env={'WEBOTS_CONTROLLER_URL': 'spesbot'},
    )

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        emulate_tty=True,
        arguments=[
            'diffdrive_controller',
            '--controller-manager-timeout',
            '50'
        ]
    )

    tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ]
    )

    tf_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--frame-id', 'odom',
            '--child-frame-id', 'base_footprint'
        ]
    )

    tf_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ]
    )

    behavior = Node(
        package='spes_behavior',
        executable='behavior',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'behavior': 'test_translate',
            }
        ],
    )

    move = Node(
        package='spes_move',
        executable='move',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,
                'angular.max_velocity': 0.3,
                'angular.max_acceleration': 0.3,
                'angular.tolerance': 0.001,
                'update_rate': 100,
            }
        ],
    )

    return launch.LaunchDescription([
        webots,
        behavior,
        move,
        webots._supervisor,
        webots_robot_driver,
        diffdrive_controller_spawner,
        tf_lidar,
        tf_base_footprint,
        tf_map,
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        ))
    ])
