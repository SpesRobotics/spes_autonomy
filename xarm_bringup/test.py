import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose, TransformStamped
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ros2_numpy as rnp
import transforms3d as t3d

import tf2_ros
import time
from datetime import datetime
from enum import Enum
import subprocess
import re
import cv2
import os
import numpy as np



class EnvStates(Enum):
    IDLE = 0
    CLOSE_GRIPPER = 1
    OPEN_GRIPPER = 2
    RESET = 3
    UP = 4
    DOWN = 5
    MOVE = 6
    GO_CLOSE = 7


class GripperStatus(Enum):
    OPEN = 0
    CLOSE = 1


DATA_DIR = 'DATA'


def are_transforms_close(a, b=None, linear_tol=0.03, angular_tol=0.03):
    if b is None:
        b = np.eye(4)
    c = np.linalg.inv(a) @ b
    xyz = c[:3, 3]
    if np.any(np.abs(xyz) > linear_tol):
        return False
    rpy = t3d.euler.mat2euler(c[:3, :3])
    if np.any(np.abs(np.array(rpy)) > angular_tol):
        return False
    return True


def call_ros2_service(activate_controllers, deactivate_controllers):
    service_name = '/controller_manager/switch_controller'
    service_type = 'controller_manager_msgs/srv/SwitchController'
    strictness = '2'
    activate_asap = 'true'

    command = f'ros2 service call {service_name} {service_type} "{{activate_controllers: [\"{activate_controllers}\"], deactivate_controllers: [\"{deactivate_controllers}\"], strictness: {strictness}, activate_asap: {activate_asap}}}"'
    try:
        result = subprocess.run(command, shell=True,
                                check=True, capture_output=True, text=True)
        match = re.search(r'response:\n(.*)', result.stdout, re.DOTALL)
        print(f"{activate_controllers}:", match.group(1).strip())
    except subprocess.CalledProcessError as e:
        print(f"Error calling ROS 2 service: {e}")


class Test(Node):
    def __init__(self):
        super().__init__('test_subscriber')

        self.publisher_ = self.create_publisher(
            PoseStamped, '/target_frame', 1)
        self.timer = self.create_timer(0.1, self.publish_pose)

        self.publisher_speed_limiter = self.create_publisher(
            PoseStamped, '/target_frame_raw', 1)

        self.publisher_gripper = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 1)
        
        self.publisher_respawn = self.create_publisher(Twist, '/respawn', 1)
        
        self.publisher_joint_init = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 1)

        self.image_subscriber = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            1)
        self.image_subscriber

        self.current_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            1)
        self.current_pose_subscriber

        self.cv_bridge = CvBridge()
        self.image = None
        self.obtained_object_pose = PoseStamped()

        self.action_file = None
        self.observation_file = None
        self.reward_file = None
        self.images_folder = None
        
        self.images_counter = 0
        self.episode_frame_counter = 0
        self.gripper_status = GripperStatus.OPEN

        self.switch_service = self.create_client(
            SwitchController, '/controller_manager/switch_controller')
        while not self.switch_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Switch service not available, waiting again...')

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.current_pose = PoseStamped()
        self.current_relative_pose = TransformStamped()

        self.joint_state = JointTrajectory()
        self.joint_names = ['joint1', 'joint2',
                            'joint3', 'joint4', 'joint5', 'joint6']

        point = JointTrajectoryPoint()
        point.positions = [0.00148, 0.06095, 1.164, -0.00033, 1.122, -0.00093]
        point.time_from_start.sec = 3
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
        self.new_episode = True
        self.skip_saving = False


        self.transform = None
        self.state = EnvStates.IDLE
        self.previous_state = self.state
        
        self.is_end_episode = False
        self.end_episode_cnt = 0
        

    def switch_motion_controller(self, activate_controllers, deactivate_controllers):
        request = SwitchController.Request()
        request.activate_controllers = [activate_controllers]
        request.deactivate_controllers = [deactivate_controllers]
        request.strictness = 2
        request.activate_asap = True

        self.switch_service.call(request)

    def image_callback(self, msg):
        try:
            cv2_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2_img = cv2.resize(cv2_img, (96, 96))
            self.image = cv2_img
        except CvBridgeError as e:
            print(e)

    def current_pose_callback(self, msg):
        self.obtained_object_pose = msg

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tfBuffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get transform: {e}")
            return None

    def save_episode(self, dir_path):
        date = datetime.now()
        episode_name = date.strftime("%Y_%m_%d_%H_%M_%S")
        self.images_folder = dir_path + '/' + episode_name
        action_folder = dir_path + '/actions'
        observation_folder = dir_path + '/observations'
        rewards_folder = dir_path + '/rewards'

        if not os.path.exists(dir_path):
            os.mkdir(dir_path)

        if not os.path.exists(action_folder):
            os.mkdir(action_folder)

        if not os.path.exists(self.images_folder):
            os.mkdir(self.images_folder)
        
        if not os.path.exists(observation_folder):
            os.mkdir(observation_folder)
        
        # if not os.path.exists(rewards_folder):
        #     os.mkdir(rewards_folder)

        action_file_neme = action_folder + '/' + episode_name+'.txt'
        observation_file_neme = observation_folder + '/' + episode_name+'.txt'
        rewards_file_neme = rewards_folder + '/' + episode_name+'.txt'

        self.action_file = open( action_file_neme, "a")
        self.observation_file = open( observation_file_neme, "a")
        # self.reward_file = open(rewards_file_neme, "a")
        self.get_logger().info(f'Episode {episode_name} saving started...')


    def save_current_frame(self):
        if self.action_file is None or self.images_folder is None:
            return
        if self.image is None or self.obtained_object_pose is None:
            return

        self.episode_frame_counter += 1

        x_pos = self.obtained_object_pose.pose.position.x
        y_pos = self.obtained_object_pose.pose.position.y
        z_pos = self.obtained_object_pose.pose.position.z

        x_ori = self.obtained_object_pose.pose.orientation.x
        y_ori = self.obtained_object_pose.pose.orientation.y
        z_ori = self.obtained_object_pose.pose.orientation.z
        w_ori = self.obtained_object_pose.pose.orientation.w

        gripper_status = 0
        if self.gripper_status == GripperStatus.CLOSE:
            gripper_status = 1
        
        # observation = (f"[{x_pos}, {y_pos}, {z_pos}, {x_ori}, {y_ori}, {z_ori}, {w_ori}, {gripper_status}]\n")

        observation = (f"[{x_pos}, {y_pos}, {z_pos}, {x_ori}, {y_ori}, {z_ori}, {w_ori}]\n")

        x_action_pos = self.current_relative_pose.transform.translation.x
        y_action_pos = self.current_relative_pose.transform.translation.y
        z_action_pos = self.current_relative_pose.transform.translation.z

        x_action_ori = self.current_relative_pose.transform.rotation.x
        y_action_ori = self.current_relative_pose.transform.rotation.y
        z_action_ori = self.current_relative_pose.transform.rotation.z
        w_action_ori = self.current_relative_pose.transform.rotation.w

        # action = (f"[{x_action_pos}, {y_action_pos}, {z_action_pos}, {x_action_ori}, {y_action_ori}, {z_action_ori}, {w_action_ori}, {gripper_status}]\n")
        action = (f"[{x_action_pos}, {y_action_pos}, {z_action_pos}, {x_action_ori}, {y_action_ori}, {z_action_ori}, {w_action_ori}]\n")
        if self.is_end_episode:
            action = (f"[{0.0}, {0.0}, {0.0}, {0.0}, {0.0}, {0.0}, {0.0}]\n")

        # reward = (1 / (self.euclidean_distance + self.angle_distance)) * 0.01

        image_name = self.images_folder + '/' + str(self.images_counter) + '.jpg'
        cv2.imwrite(image_name, self.image)
        self.observation_file.write(observation)
        self.action_file.write(action)
        # self.reward_file.write(str(reward)+ '\n')

        self.images_counter += 1

    def publish_pose(self):
        if self.previous_state != self.state:
            self.previous_state = self.state

        if not self.skip_saving:
                self.save_current_frame()

        
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = 'link_base'

        gripper_target_tf = self.get_transform('gripper_base_link', 'pick_target')
        base_gripper_tf = self.get_transform('link_base', 'gripper_base_link')
        
        if gripper_target_tf is None:
            return
        
        if base_gripper_tf is None:
            return
        
        self.current_relative_pose = gripper_target_tf

        gripper_target = rnp.numpify(gripper_target_tf.transform)
        base_gripper = rnp.numpify(base_gripper_tf.transform)

        target_transform =  base_gripper @ gripper_target
        pose = rnp.msgify(Pose, target_transform)

        if not self.is_arm_init:
            call_ros2_service('joint_trajectory_controller',
                              'cartesian_motion_controller')
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.publisher_joint_init.publish(self.joint_state)
            self.is_arm_init = True
            self.get_logger().info(f'Arm inited...')
            self.start_time = time.time()
        
        if self.state == EnvStates.IDLE:

            if time.time() - self.start_time > 3.0:
                call_ros2_service('cartesian_motion_controller',
                                  'joint_trajectory_controller')
                self.is_trajectory_controler_active = False
                self.transform = self.get_transform('link_base', 'pick_target') # ?????????????????????

                if self.transform is None:
                    return
                self.get_logger().info('Start...')

                self.state = EnvStates.MOVE

                if self.new_episode:
                    self.save_episode(DATA_DIR)
                    self.new_episode = False
                    self.skip_saving = False
                    self.images_counter = 0
                    self.episode_frame_counter = 0
                    self.is_end_episode = False
                    self.end_episode_cnt = 0

        elif self.state == EnvStates.MOVE:
            # self.euclidean_distance = np.sqrt(gripper2target.transform.translation.x**2 + gripper2target.transform.translation.y**2)
            # self.angle_distance = 2 * math.acos(gripper2target.transform.rotation.w)
            self.current_pose.pose = pose
            if are_transforms_close(gripper_target):
                self.state = EnvStates.GO_CLOSE
                # self.skip_saving = True
                self.current_pose.pose.position.z = 0.09
                self.start_time  = time.time()
        elif self.state == EnvStates.GO_CLOSE:
            if time.time() - self.start_time > 2:
                if round(gripper_target_tf.transform.translation.z, 4) <= -0.0255:
                    if self.end_episode_cnt > 5:
                        self.state = EnvStates.CLOSE_GRIPPER
                    self.end_episode_cnt += 1
                    self.is_end_episode = True
                self.get_logger().info(f'{round(gripper_target_tf.transform.translation.z, 4)}')

        elif self.state == EnvStates.CLOSE_GRIPPER:
            self.skip_saving  =True
            if not self.is_object_picked:
                self.state = EnvStates.UP
                self.gripper_status = GripperStatus.CLOSE

                self.start_time = time.time()
                self.is_object_picked = True

                msg = Float64MultiArray()
                msg.data = [-0.01]
                self.publisher_gripper.publish(msg)
                self.get_logger().info(
                    f'Close gripper! {round(gripper_target_tf.transform.translation.z, 2)}')

        elif self.state == EnvStates.UP:
            if time.time() - self.start_time > 0.1:
                self.current_pose.pose.position.z = 0.35
                self.state = EnvStates.DOWN
                self.start_time = time.time()

                self.get_logger().info('Move upward...')

        elif self.state == EnvStates.DOWN:
            # self.skip_saving = True
            if time.time() - self.start_time > 3.0:
                self.skip_saving = True
                self.current_pose.pose.position.z = 0.09
                self.state = EnvStates.OPEN_GRIPPER
                self.gripper_status = GripperStatus.OPEN
                self.start_time = time.time()

                self.get_logger().info('Move downward...')

        elif self.state == EnvStates.OPEN_GRIPPER:
            if time.time() - self.start_time > 3.5 and self.is_object_picked:
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
                call_ros2_service('joint_trajectory_controller',
                                  'cartesian_motion_controller')
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
                
                self.new_episode = True

        else:
            self.state = EnvStates.IDLE
            self.start_time = time.time()

        if self.state is not EnvStates.RESET:
            self.publisher_speed_limiter.publish(self.current_pose)


def test_tf_close():
    a = t3d.affines.compose(
        [0, 0, 0],
        t3d.euler.euler2mat(0, 0, 0),
        [1, 1, 1]
    )
    b = t3d.affines.compose(
        [0, 0, 0],
        t3d.euler.euler2mat(0, 0, 0),
        [1, 1, 1]
    )
    assert are_transforms_close(a, b) == True
    b = t3d.affines.compose(
        [0, 0, 0],
        t3d.euler.euler2mat(0.3, 0, 0),
        [1, 1, 1]
    )
    assert are_transforms_close(a, b) == False
    b = t3d.affines.compose(
        [0, 0, 0],
        t3d.euler.euler2mat(0.01, 0.01, -0.01),
        [1, 1, 1]
    )
    assert are_transforms_close(a, b) == True

def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# a_b_tf = ..
# b_a_tf = np.linalg.inv(a_b_tf)