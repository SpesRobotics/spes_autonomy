import os
import pathlib
import yaml
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node

def get_controller_spawners(controller_params_file):
    with open(controller_params_file, 'r') as f:
        controller_params = yaml.safe_load(f)

    controller_names = list(controller_params['controller_manager']['ros__parameters'].keys())

    # Create controller spawners
    controller_spawners = []
    for controller_name in controller_names:
        if controller_name in ['update_rate', 'publish_rate']:
            continue
        
        controller_spawners.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            emulate_tty=True,
            arguments=[
                controller_name,
                '--controller-manager-timeout',
                '50',
                '--controller-manager',
                'controller_manager',
            ])
        )
    return controller_spawners



def generate_launch_description():
    # HOTFIX: https://github.com/cyberbotics/webots_ros2/issues/567
    os.environ['LD_LIBRARY_PATH'] += ':/opt/ros/humble/lib/controller'

    package_dir = get_package_share_directory('spesbot_webots')

    controller_params = os.path.join(get_package_share_directory('spesbot_hardware'),
                             'resource', 'ros2_control.yaml')

    robot_description = pathlib.Path(
        os.path.join(package_dir, 'resource', 'robot_description.urdf')).read_text()
    
    webots = WebotsLauncher(world=os.path.join(package_dir, 'data',
                                               'worlds', 'spesbot.wbt'), ros2_supervisor=True)
                                    

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
            ('/diffdrive_controller/odom', 'odom')
        ],
        ros_arguments=['--log-level', 'warn'],
        additional_env={'WEBOTS_CONTROLLER_URL': 'spesbot'},
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,
        webots._supervisor,
        webots_robot_driver,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        ))
    ] + get_controller_spawners(controller_params))