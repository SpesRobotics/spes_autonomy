import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys, select, termios, tty
import transforms3d

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
        ,    

i : forward (+x)
, : backward (-x)
u : left (-y)
o : right (+x)
j : rotate_left
l : rotate_right

t : up (+z)
b : down (-z)

g : open/close gripper

anything else : stop

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        ',':(-1,0,0,0),
        'u':(0,1,0,0),
        'o':(0,-1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
       }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('xarm_manual_control')

        self.publisher_ = self.create_publisher(PoseStamped, '/target_frame', 10)
        self.timer = self.create_timer(0.01, self.publish_pose)

        self.publisher_gripper = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        self.speed = 0.5
        self.turn = 1.0
        self.frequency = 0.01

        self.current_pose = PoseStamped()

        self.current_pose_x = 0.087
        self.current_pose_y = 0.0
        self.current_pose_z = 0.15359

        self.current_orientation_x = np.pi
        self.current_orientation_y = 0.0
        self.current_orientation_z = 0.0

        self.is_gripper_open = True

    def publish_pose(self):
        self.publisher_.publish(self.current_pose)


    def run(self):
        print(msg)
        try:
            while True:
                key = getKey()
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    th = moveBindings[key][3]
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                    if key == '\x03':
                        break
                
                if key == 'g':
                    msg_arr = Float64MultiArray()
                    if self.is_gripper_open:
                        msg_arr.data = [-0.01]
                    else:
                        msg_arr.data = [0.0]

                    self.is_gripper_open = (not self.is_gripper_open)
                    self.publisher_gripper.publish(msg_arr)

                self.current_pose.header.stamp = self.get_clock().now().to_msg()
                self.current_pose.header.frame_id = 'link_base'

                # linear motion
                self.current_pose_x += x * self.frequency * self.speed
                self.current_pose_y += y * self.frequency * self.speed
                self.current_pose_z += z * self.frequency * self.speed

                self.current_pose.pose.position.x = self.current_pose_x
                self.current_pose.pose.position.y = self.current_pose_y
                self.current_pose.pose.position.z = self.current_pose_z

                # angular motion
                self.current_orientation_x += 0.0 * self.frequency
                self.current_orientation_y += 0.0 * self.frequency
                self.current_orientation_z += th * self.turn * self.frequency

                quaternion = transforms3d.euler.euler2quat(self.current_orientation_x, self.current_orientation_y, self.current_orientation_z)

                self.current_pose.pose.orientation.x = quaternion[1]
                self.current_pose.pose.orientation.y = quaternion[2]
                self.current_pose.pose.orientation.z = quaternion[3]
                self.current_pose.pose.orientation.w = quaternion[0]

                self.publisher_.publish(self.current_pose)

        except Exception as e:
            self.get_logger().error(str(e))

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_keyboard = TeleopTwistKeyboard()
    teleop_twist_keyboard.run()
    teleop_twist_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
