import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController

import tf2_ros
import time
from enum import Enum

class EnvStates(Enum):
    IDLE = 0
    CLOSE_GRIPPER = 1
    OPEN_GRIPPER = 2
    RESET = 3
    UP = 4
    DOWN = 5
    MOVE = 6


class Test(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_frame', 10)
        self.timer = self.create_timer(0.01, self.publish_pose)

        
        self.publisher_joint_init = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.switch_service = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.switch_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Switch service not available, waiting again...')

        self.joint_state = JointTrajectory()
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()

        point.positions = [0.00148, 0.06095, 1.164, -0.00033, 1.122, -0.00093]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        # self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.points = [point]
        self.joint_state.joint_names = self.joint_names


        self.start_time = time.time()
    
        self.is_arm_init = False

    def switch_motion_controller(self, activate_controllers, deactivate_controllers) :
        request = SwitchController.Request()
        request.activate_controllers = [activate_controllers]
        request.deactivate_controllers = [deactivate_controllers]
        request.strictness = 2
        request.activate_asap = True
        self.get_logger().info(f'------------------')
        self.switch_service.call(request)

  

    def publish_pose(self):
        # self.get_logger().info(f'..............')
        if not self.is_arm_init:
            self.get_logger().info(f'Arm init...')
            self.switch_motion_controller('joint_trajectory_controller', 'cartesian_motion_controller')
            # self.publisher_joint_init.publish(self.joint_state)
            # self.switch_motion_controller('cartesian_motion_controller', 'joint_trajectory_controller')
            self.is_arm_init = True
            self.get_logger().info(f'Arm inited...')

        if time.time() - self.start_time > 5.0:
            self.is_arm_init = False
            self.start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
