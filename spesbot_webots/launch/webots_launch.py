import os
import pathlib
import yaml
import launch

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node


def generate_launch_description():
    # HOTFIX: https://github.com/cyberbotics/webots_ros2/issues/567
    os.environ['LD_LIBRARY_PATH'] += ':/opt/ros/humble/lib/controller'

    package_dir = get_package_share_directory('spesbot_webots')

    controller_params = os.path.join(get_package_share_directory('spesbot_hardware'),
                             'resource', 'controllers.yaml')
    
    with open(controller_params, 'r') as f:
        controller_data = yaml.safe_load(f)
    controller_names = list(controller_data['controller_manager']['ros__parameters'].keys())

    robot_description = pathlib.Path(
        os.path.join(package_dir, 'resource', 'description.urdf')).read_text()
    
    camera_description = pathlib.Path(
        os.path.join(package_dir, 'resource', 'camera.urdf')).read_text()
    
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
            ('/diffdrive_controller/odom', 'odom'),
            ('/spesbot/camera', 'camera')
        ],
        ros_arguments=['--log-level', 'warn'],
        additional_env={'WEBOTS_CONTROLLER_URL': 'spesbot'},
    )
    webots_camera_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',  # debugging
        emulate_tty=True,  # debugging
        parameters=[{
            'robot_description': camera_description,
            'use_sim_time': True
        }],
        ros_arguments=['--log-level', 'warn'],
        additional_env={'WEBOTS_CONTROLLER_URL': 'camera'}
    )

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
    


    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,
        webots._supervisor,
        webots_robot_driver,
        webots_camera_driver,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        ))
    ] + controller_spawners)