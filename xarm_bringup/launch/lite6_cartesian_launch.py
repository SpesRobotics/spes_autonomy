import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import launch
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xml.etree.ElementTree as ET
from launch.actions import OpaqueFunction, DeclareLaunchArgument


def create_image_compression_nodes(topic_names):
    nodes = []
    for topic_name in topic_names:
        node = Node(
            package='image_transport',
            executable='republish',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', topic_name),
                ('out/compressed', topic_name + '/compressed'),
            ],
            output='screen',
        )
        nodes.append(node)
    return nodes

configurable_parameters = [{'name': 'serial_no', 'default': "'021222071076'", 'description': 'choose device by serial number'},]

def launch_setup(context):
    robot_description_path = os.path.join(
        get_package_share_directory('xarm_bringup'), 'urdf', 'lite6.urdf.xacro')

    controllers = os.path.join(get_package_share_directory(
        'xarm_bringup'), 'resource', 'controllers.yaml')

    use_rviz = LaunchConfiguration('rviz', default=True)
    use_sim = LaunchConfiguration('sim', default=True)
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.184')

    ros2_control_plugin = 'uf_robot_hardware/UFRobotSystemHardware' if use_sim.perform(context) == 'false' else 'topic_based_ros2_control/TopicBasedSystem'
    print('[SYSTEM INFO] Started controler: ', ros2_control_plugin)

    robot_description = xacro.process_file(robot_description_path, mappings={'robot_ip': robot_ip.perform(context), 'ros2_control_plugin': ros2_control_plugin}).toprettyxml(indent='  ')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers
        ],
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ('cartesian_motion_controller/current_pose', 'current_pose')
        ],
        output='screen'
    )

    cartesian_motion_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_motion_controller'],
        output='screen'
    )

    motion_control_handle_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['motion_control_handle'],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(
            'xarm_bringup'), 'resource', 'lite6_cartesian.rviz')],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--inactive'],
        output='screen'
    )

    sixd_speed_limiter = Node(
        package='xarm_bringup',
        executable='sixd_speed_limiter',
        output='screen'
    )

    realsence_camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters = [{'serial_no': "021222071076",}],
        remappings=[
            ('/camera/camera/color/image_raw', '/rgb')
        ],
        output='screen',
        condition=UnlessCondition(use_sim)
    )

    gripper_service = Node(
        package='xarm_api',
        executable='xarm_driver_node',
        output='screen',
        parameters=[
            {'robot_ip': robot_ip},
            {'services.open_lite6_gripper': True},
            {'services.close_lite6_gripper': True},
            {'services.stop_lite6_gripper': True},
            {'services.set_mode': True},
            {'services.set_state': True},
        ],
        condition=UnlessCondition(use_sim)
    )

    image_compression_nodes = create_image_compression_nodes(['/rgb'])

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'address': '192.168.2.168'}],
        output='screen',
    )

    return [
        joint_trajectory_controller,
        controller_manager,
        robot_state_publisher,
        cartesian_motion_controller_spawner,
        motion_control_handle_spawner,
        joint_state_broadcaster_spawner,
        rviz,
        position_controller,
        sixd_speed_limiter,
        realsence_camera,
        gripper_service,
        foxglove_bridge
    ]+image_compression_nodes


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
