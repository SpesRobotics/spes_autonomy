import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController

import tf2_ros
import time
from enum import Enum
import subprocess, re

class EnvStates(Enum):
    IDLE = 0
    CLOSE_GRIPPER = 1
    OPEN_GRIPPER = 2
    RESET = 3
    UP = 4
    DOWN = 5
    MOVE = 6



def call_ros2_service(activate_controllers, deactivate_controllers):
    service_name = '/controller_manager/switch_controller'
    service_type = 'controller_manager_msgs/srv/SwitchController'
    strictness = '2'
    activate_asap = 'true'

    command = f'ros2 service call {service_name} {service_type} "{{activate_controllers: [\"{activate_controllers}\"], deactivate_controllers: [\"{deactivate_controllers}\"], strictness: {strictness}, activate_asap: {activate_asap}}}"'
    try:
        result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        match = re.search(r'response:\n(.*)', result.stdout, re.DOTALL)
        print(f"{activate_controllers}:", match.group(1).strip())
    except subprocess.CalledProcessError as e:
        print(f"Error calling ROS 2 service: {e}")



class Test(Node):
    def __init__(self):
        super().__init__('test_subscriber')
    
        self.publisher_ = self.create_publisher(PoseStamped, '/target_frame', 10)
        self.timer = self.create_timer(0.01, self.publish_pose)

        self.publisher_gripper = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.publisher_respawn = self.create_publisher(Twist, '/respawn', 10)
        self.publisher_joint_init = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.switch_service = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not self.switch_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Switch service not available, waiting again...')

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.current_pose = PoseStamped()

        self.joint_state = JointTrajectory()
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()

        point.positions = [0.00148, 0.06095, 1.164, -0.00033, 1.122, -0.00093]
        
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        self.joint_state.points = [point]
        self.joint_state.joint_names = self.joint_names

        msg_arr = Twist()
        self.publisher_respawn.publish(msg_arr)

        self.start_time = time.time()
        self.is_object_picked = False
        self.next_episode_trigger = False
        self.is_arm_init = False
        self.is_trajectory_controler_active = False

        self.transform = None
        self.state = EnvStates.IDLE
    

    def switch_motion_controller(self, activate_controllers, deactivate_controllers) :
        request = SwitchController.Request()
        request.activate_controllers = [activate_controllers]
        request.deactivate_controllers = [deactivate_controllers]
        request.strictness = 2
        request.activate_asap = True

        self.switch_service.call(request)

    def switch_motion_controller_ex(self, activate_controllers, deactivate_controllers):
        request = SwitchController.Request()
        request.activate_controllers = [activate_controllers]
        request.deactivate_controllers = [deactivate_controllers]
        request.strictness = 2
        request.activate_asap = True
        
        result = self.switch_service.call(request)

        
        time.sleep(10)
        print(result)
        
        print('+++')
        # rclpy.spin_until_future_complete(self, future)
        # self.get_logger().error('++++++++++++++++++++++++++++++++++++=')
        # if future.result() is not None:
        #     self.get_logger().info('Switched to Cartesian motion controller successfully')
        # else:
        #     self.get_logger().error('Failed to switch controllers:')
        #     self.get_logger().error(str(future.exception()))

        #     response = future.result()
        #     if response is not None:
        #         self.get_logger().error(f'Response: {response}')



    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tfBuffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    def publish_pose(self):

        epsilon = 0.003
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = 'link_base'

        gripper2target = self.get_transform('gripper_base_link', 'pick_target')
        if gripper2target is None:
            return
        
        if not self.is_arm_init:
            call_ros2_service('joint_trajectory_controller', 'cartesian_motion_controller')
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.publisher_joint_init.publish(self.joint_state)
            self.is_arm_init = True
            self.get_logger().info(f'Arm inited...')
            self.start_time = time.time()

        
        if self.state == EnvStates.IDLE:
            if time.time() - self.start_time > 3.0:
                call_ros2_service('cartesian_motion_controller', 'joint_trajectory_controller')
                self.is_trajectory_controler_active = False
                self.transform = self.get_transform('link_base', 'pick_target')
            
                if self.transform is None:
                    return
                self.get_logger().info('Start...')

                self.state = EnvStates.MOVE

        elif self.state == EnvStates.MOVE:
            if gripper2target.transform.translation.x > epsilon or gripper2target.transform.translation.y > epsilon or gripper2target.transform.translation.z > epsilon:
                self.current_pose.pose.position.z = self.transform.transform.translation.z
            else:
                self.current_pose.pose.position.z = 0.095

            self.current_pose.pose.position.x = self.transform.transform.translation.x
            self.current_pose.pose.position.y = self.transform.transform.translation.y

            self.current_pose.pose.orientation.x = self.transform.transform.rotation.x
            self.current_pose.pose.orientation.y = self.transform.transform.rotation.y
            self.current_pose.pose.orientation.z = self.transform.transform.rotation.z
            self.current_pose.pose.orientation.w = self.transform.transform.rotation.w

            if round(gripper2target.transform.translation.z, 2) < -0.0245:
                self.state = EnvStates.CLOSE_GRIPPER

        elif self.state == EnvStates.CLOSE_GRIPPER:
            if not self.is_object_picked:
                self.state = EnvStates.UP

                self.start_time = time.time()
                self.is_object_picked = True

                msg = Float64MultiArray()
                msg.data = [-0.01]
                self.publisher_gripper.publish(msg)
                self.get_logger().info(f'Close gripper! {round(gripper2target.transform.translation.z, 2)  + 0.026}')
        
        elif self.state == EnvStates.UP:
            if time.time() - self.start_time > 3.0:
                self.current_pose.pose.position.z = 0.2
                self.state = EnvStates.DOWN
                self.start_time = time.time()

                self.get_logger().info('Move upward...')
        
        elif self.state == EnvStates.DOWN:
            if time.time() - self.start_time > 3.0:
                self.current_pose.pose.position.z = 0.095
                self.state = EnvStates.OPEN_GRIPPER
                self.start_time = time.time()

                self.get_logger().info('Move downward...')

        elif self.state == EnvStates.OPEN_GRIPPER:
            if time.time() - self.start_time > 3.0 and self.is_object_picked:
                self.state = EnvStates.RESET
                self.is_object_picked = False

                self.start_time = time.time()
                self.next_episode_trigger = True

                msg = Float64MultiArray()
                msg.data = [0.0]
                self.publisher_gripper.publish(msg)
                self.get_logger().info('Open gripper!')

        elif self.state == EnvStates.RESET:
            if not self.is_trajectory_controler_active:
                call_ros2_service('joint_trajectory_controller', 'cartesian_motion_controller')
                self.is_trajectory_controler_active = True
                self.joint_state.header.stamp = self.get_clock().now().to_msg()
                self.publisher_joint_init.publish(self.joint_state)
                self.get_logger().info(f'Arm inited...')
            
            if time.time() - self.start_time > 10.0 and self.next_episode_trigger:
                self.state = EnvStates.IDLE
                self.start_time = time.time()
                self.next_episode_trigger = False

                msg_arr = Twist()
                self.publisher_respawn.publish(msg_arr)
                self.get_logger().info('Move object to the random position!')
            
        else:
            self.state = EnvStates.IDLE
            self.start_time = time.time()

        if self.state is not EnvStates.RESET:
            self.publisher_.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
