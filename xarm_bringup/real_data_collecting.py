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
import sys, select, termios, tty

import tf2_ros
import time
from datetime import datetime
import re
import cv2
import os
import numpy as np
from collections import deque
import threading

DATA_DIR = 'DATA_REAL'

settings = termios.tcgetattr(sys.stdin)
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class RealDataCollecting(Node):
    def __init__(self):
        super().__init__('real_data_collecting')
        self.timer = self.create_timer(0.1, self.save_data_frame)

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

        self.target_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/target_frame_raw',
            self.target_pose_callback,
            1)
        self.target_pose_subscriber

        self.cv_bridge = CvBridge()
        self.image = None
        self.obtained_object_pose = PoseStamped()

        self.action_file = None
        self.observation_file = None
        self.images_folder = None

        self.images_counter = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.target_pose_relative = PoseStamped()
        self.target_pose = PoseStamped()
        self.absolute_pose = PoseStamped()
        self.start_time = time.time()

        self.next_episode_trigger = False
        self.new_episode = True

        self.is_end_episode = False
        self.end_episode_cnt = 0


        self.listener_thread = threading.Thread(target=self.start_keyboard_listener, daemon=True)
        self.listener_thread.start()
        self.exit = False

        self.start_saving = False
        self.is_teleoperation_stareted = False
        self.observation_queue = deque(maxlen=10)

    
    def start_keyboard_listener(self):
        while True:
            key = getKey()
            self.on_key_press(key)
            
            if key == 'q':
                self.exit = True
                self.get_logger().info('Quit!')
                break

    def on_key_press(self, key):
        try:
            if key == 's':
                self.is_end_episode = True
                self.get_logger().info('Episode saving...')
            if key == 'w':
                self.start_saving = False
                self.is_end_episode = True
                self.get_logger().info('Wait....')
            if key == 'n':
                self.start_saving = True
                self.new_episode = True
                self.get_logger().info('New episode started by keyboard input.')
        except AttributeError:
            pass

    def image_callback(self, msg):
        try:
            cv2_img = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2_img = cv2.resize(cv2_img, (96, 96))
            self.image = cv2_img
        except CvBridgeError as e:
            print(e)

    def current_pose_callback(self, msg):
        self.observation_queue.append(msg)
        # self.obtained_object_pose = msg

    def target_pose_callback(self, msg):
        # self.get_logger().info(f'{msg}')
        self.target_pose = msg
        self.is_teleoperation_stareted = True

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

        if not os.path.exists(dir_path):
            os.mkdir(dir_path)

        if not os.path.exists(action_folder):
            os.mkdir(action_folder)

        if not os.path.exists(self.images_folder):
            os.mkdir(self.images_folder)
        
        if not os.path.exists(observation_folder):
            os.mkdir(observation_folder)
        

        action_file_neme = action_folder + '/' + episode_name+'.txt'
        observation_file_neme = observation_folder + '/' + episode_name+'.txt'

        self.action_file = open( action_file_neme, "a")
        self.observation_file = open( observation_file_neme, "a")
        
        self.get_logger().info(f'Episode {episode_name} saving started...')


    def save_current_frame(self):
        if self.action_file is None or self.images_folder is None:
            return
        if self.image is None:
            return


        x_pos = self.obtained_object_pose.pose.position.x
        y_pos = self.obtained_object_pose.pose.position.y
        z_pos = self.obtained_object_pose.pose.position.z

        x_ori = self.obtained_object_pose.pose.orientation.x
        y_ori = self.obtained_object_pose.pose.orientation.y
        z_ori = self.obtained_object_pose.pose.orientation.z
        w_ori = self.obtained_object_pose.pose.orientation.w

        observation = (f"[{x_pos}, {y_pos}, {z_pos}, {x_ori}, {y_ori}, {z_ori}, {w_ori}]\n")

        x_action_pos = self.target_pose_relative.position.x
        y_action_pos = self.target_pose_relative.position.y
        z_action_pos = self.target_pose_relative.position.z
        x_action_ori = self.target_pose_relative.orientation.x
        y_action_ori = self.target_pose_relative.orientation.y
        z_action_ori = self.target_pose_relative.orientation.z
        w_action_ori = self.target_pose_relative.orientation.w

        action = (f"[{x_action_pos}, {y_action_pos}, {z_action_pos}, {x_action_ori}, {y_action_ori}, {z_action_ori}, {w_action_ori}]\n")
        if self.is_end_episode:
            action = (f"[{0.0}, {0.0}, {0.0}, {0.0}, {0.0}, {0.0}, {1.0}]\n")

        image_name = self.images_folder + '/' + str(self.images_counter) + '.jpg'
        cv2.imwrite(image_name, self.image)
        self.observation_file.write(observation)
        self.action_file.write(action)

        self.images_counter += 1


    def save_data_frame(self):
        if not self.is_teleoperation_stareted:
            # self.get_logger().info(f'Wait msg from  the space mause...')
            return
        
        if self.is_end_episode:
            if self.end_episode_cnt > 5:
                self.is_end_episode = False
                self.start_saving = False
            else:
                self.save_current_frame()
            self.end_episode_cnt += 1
            self.get_logger().info(f'Episode saving...')
            return
        
        if not self.start_saving:
            return

        if self.new_episode:
            self.save_episode(DATA_DIR)
            self.new_episode = False
            self.images_counter = 0
            self.is_end_episode = False
            self.end_episode_cnt = 0
            self.is_teleoperation_stareted = False
            time.sleep(3.5)
        
        self.obtained_object_pose = self.observation_queue[0]
        if self.obtained_object_pose is None:
            return
        base_gripper_tf = self.get_transform('link_base', 'gripper_base_link')
        if base_gripper_tf is None:
            return
        

        base_gripper = rnp.numpify(base_gripper_tf.transform)
        gripper_target = rnp.numpify(self.target_pose.pose)

        base_current_pose = rnp.numpify(self.obtained_object_pose.pose)
        base_target  = rnp.numpify(self.target_pose.pose)

        relative_pose = np.linalg.inv(base_current_pose) @ base_target

        relative_pose_msg = rnp.msgify(Pose, relative_pose)
        
        # gripper_target_inverse = np.linalg.inv(gripper_target)

        # target_transform =  base_gripper @ gripper_target_inverse
        # pose = rnp.msgify(Pose, target_transform)

        self.target_pose_relative = relative_pose_msg

        self.save_current_frame()

        if self.exit:
            self.get_logger().info(f"Finished operation. Shutting down node...")
            rclpy.shutdown()
            self.listener_thread.join()
            exit(0)
              
        
def main(args=None):
    rclpy.init(args=args)
    real_data_collecting = RealDataCollecting()
    rclpy.spin(real_data_collecting)
    real_data_collecting.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

        
