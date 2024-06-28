import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import transforms3d

frequency = 0.01
class XArmManualControl(Node):
    def __init__(self):
        super().__init__('xarm_manual_control')

        self.publisher_ = self.create_publisher(PoseStamped, '/target_frame', 10)
        self.timer = self.create_timer(0.01, self.publish_pose)

        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.current_pose = PoseStamped()

        self.current_pose_x = 0.087
        self.current_pose_y = 0.0
        self.current_pose_z = 0.15359

        self.current_orientation_x = np.pi
        self.current_orientation_y = 0.0
        self.current_orientation_z = 0.0

    def publish_pose(self):
        # self.get_logger().info(f'{self.current_pose}')
        self.publisher_.publish(self.current_pose)

    def cmd_vel_callback(self, msg):
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = 'link_base'
        
        self.current_pose_x += msg.linear.x * frequency 
        self.current_pose_y += msg.linear.y * frequency 
        self.current_pose_z += msg.linear.z * frequency 
        
        self.current_orientation_x += msg.angular.x * frequency
        self.current_orientation_y += msg.angular.y * frequency
        self.current_orientation_z += msg.angular.z * frequency

        self.current_pose.pose.position.x = self.current_pose_x
        self.current_pose.pose.position.y = self.current_pose_y
        self.current_pose.pose.position.z = self.current_pose_z

        quaternion = transforms3d.euler.euler2quat(self.current_orientation_x, self.current_orientation_y, self.current_orientation_z)
        # self.get_logger().info(f'{quaternion}')

        self.current_pose.pose.orientation.x = quaternion[1]
        self.current_pose.pose.orientation.y = quaternion[2]
        self.current_pose.pose.orientation.z = quaternion[3]
        self.current_pose.pose.orientation.w = quaternion[0]

        # self.get_logger().info(f'{self.current_pose.pose.position.x}, {self.current_pose.pose.position.y}, {self.current_pose.pose.position.z}')

def main(args=None):
    rclpy.init(args=args)
    xarm_manual_control = XArmManualControl()
    rclpy.spin(xarm_manual_control)
    xarm_manual_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
