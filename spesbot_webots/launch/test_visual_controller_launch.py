import os
import pathlib
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('spesbot_webots')

    controller_params = os.path.join(get_package_share_directory('spesbot_hardware'),
                             'resource', 'controllers.yaml')
    
    robot_description = pathlib.Path(
        os.path.join(package_dir, 'resource', 'description.urdf')).read_text()
    
    webots = WebotsLauncher(world=os.path.join(package_dir, 'data',
                                               'worlds', 'eurobot.wbt'), ros2_supervisor=True)
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

    virtual_controller = Node(
        package='spesbot_webots',
        executable='visual_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    move_command = Node(
        package='spes_move',
        executable='move',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'angular.max_acceleration' : 0.5,
            'angular.max_velocity' : 1.2,
            'linear.max_acceleration' : 0.8,
            'linear.max_velocity' : 0.3
        }],
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
    
    return launch.LaunchDescription([
        webots,
        webots._supervisor,
        webots_robot_driver,
        diffdrive_controller_spawner,
        virtual_controller,
        move_command,
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        ))
    ])