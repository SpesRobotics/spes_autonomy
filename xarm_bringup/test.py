import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray
import tf2_ros
import time
from enum import Enum

class EnvStates(Enum):
    IDLE = 0
    CLOSE_GRIPPER = 1
    OPEN_GRIPPER = 2
    RESET = 3
    

class Test(Node):
    def __init__(self):
        super().__init__('test_subscriber')
    
        self.publisher_ = self.create_publisher(
            PoseStamped, '/target_frame', 10)
        self.timer = self.create_timer(0.01, self.publish_pose)

        self.publisher_gripper = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.publisher_respawn = self.create_publisher(Twist, '/respawn', 10)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.current_pose = PoseStamped()

        self.start_time = time.time()
        self.is_object_picked = False
        self.next_episode_trigger = False

        self.state = EnvStates.IDLE

    
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

        transform = self.get_transform('link_base', 'pick_target')

        gripper2target = self.get_transform('gripper_base_link', 'pick_target')

        if transform is None or gripper2target is None:
            return
        
        if gripper2target.transform.translation.x > epsilon or gripper2target.transform.translation.y > epsilon or gripper2target.transform.translation.z > epsilon:
            self.current_pose.pose.position.z = transform.transform.translation.z
        else:
            self.current_pose.pose.position.z = 0.095
            # self.get_logger().info(f'{gripper2target}')
            

        self.current_pose.pose.position.x = transform.transform.translation.x
        self.current_pose.pose.position.y = transform.transform.translation.y
        

        self.current_pose.pose.orientation.x = transform.transform.rotation.x
        self.current_pose.pose.orientation.y = transform.transform.rotation.y
        self.current_pose.pose.orientation.z = transform.transform.rotation.z
        self.current_pose.pose.orientation.w = transform.transform.rotation.w

        self.publisher_.publish(self.current_pose)

        if self.state == EnvStates.IDLE:
            if round(gripper2target.transform.translation.z, 2)  + 0.026 < epsilon  and not self.is_object_picked:
                self.state = EnvStates.CLOSE_GRIPPER

                self.start_time = time.time()
                self.is_object_picked = True

                msg = Float64MultiArray()
                msg.data = [-0.01]
                self.publisher_gripper.publish(msg)
                self.get_logger().info(f'Close gripper! {round(gripper2target.transform.translation.z, 2)  + 0.026}')

        elif self.state == EnvStates.CLOSE_GRIPPER:
            if time.time() - self.start_time > 3.0 and self.is_object_picked:
                self.state = EnvStates.OPEN_GRIPPER

                self.start_time = time.time()
                self.is_object_picked = False

                msg = Float64MultiArray()
                msg.data = [0.0]
                self.publisher_gripper.publish(msg)
                self.get_logger().info(f'Open gripper!')

        elif self.state == EnvStates.OPEN_GRIPPER:
            if time.time() - self.start_time > 2.0 and not self.next_episode_trigger:
                self.state = EnvStates.RESET

                self.start_time = time.time()
                self.next_episode_trigger = True

                msg_arr = Twist()
                self.publisher_respawn.publish(msg_arr)
                self.get_logger().info(f'Move object to the random position!')

        elif self.state == EnvStates.RESET:
            self.state = EnvStates.IDLE
            self.next_episode_trigger = False
        else:
            self.state = EnvStates.IDLE

        
        # if self.state == EnvStates.CLOSE_GRIPPER:
        #     msg = Float64MultiArray()
        #     msg.data = [-0.01]
        #     self.publisher_gripper.publish(msg)
        #     self.get_logger().info(f'Close gripper! {round(gripper2target.transform.translation.z, 2)  + 0.026}')

        # elif self.state == EnvStates.OPEN_GRIPPER:
        #     msg = Float64MultiArray()
        #     msg.data = [0.0]
        #     self.publisher_gripper.publish(msg)
        #     self.get_logger().info(f'Open gripper!')

        # # randomize object
        # elif self.state == EnvStates.RESET:
        #     msg_arr = Twist()
        #     self.publisher_respawn.publish(msg_arr)
        #     self.get_logger().info(f'Move object to the random position!')




def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
