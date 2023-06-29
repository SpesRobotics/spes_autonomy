import os
import pathlib

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    # HOTFIX: https://github.com/cyberbotics/webots_ros2/issues/567
    os.environ['LD_LIBRARY_PATH'] += ':/opt/ros/humble/lib/controller'

    package_dir = get_package_share_directory('spesbot_webots')


    webots = WebotsLauncher(world=os.path.join(package_dir, 'data',
                                               'worlds', 'spesbot.wbt'), ros2_supervisor=False)

   

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        # Start the Webots node
        webots,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        ))
    ])