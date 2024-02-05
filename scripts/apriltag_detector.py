from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy, cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from dt_apriltags import Detector
from sensor_msgs.msg import Image
import numpy as np
import transforms3d as t3d

CAMERA_MATRIX = np.array([[248.091261, 0.0, 338.72890467],
                            [0.0, 249.17817251, 208.70472081],
                            [0.0, 0.0, 1.0]])

# create image_row topic with command: ros2 run v4l2_camera v4l2_camera_node
class AprilTagBroadcaster(Node):

    def __init__(self):    
        print("Initializing AprilTag Broadcaster Node =====")

        super().__init__('apriltag_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        
        self.node = rclpy.create_node('row_image_subscriber')
        self.__image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.__on_image_callback,
            1)
        self.__image_subscription
        self.bridge = CvBridge()
        self.__frame = None

        self.at_detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

        self.camera_params = ( CAMERA_MATRIX[0,0], CAMERA_MATRIX[1,1], CAMERA_MATRIX[0,2], CAMERA_MATRIX[1,2] )

    def __on_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.__frame = cv_image
        except CvBridgeError as e:
            print(e)
            return

        
    def broadcast_transform(self):
        if self.__frame is None:
            return
        h, w, _ = self.__frame.shape

        color_img = self.__frame[:, round(w/2):w, :]
        image = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(image, True, self.camera_params, 0.09)
        # print(tags)
        for tag in tags:
            tag_transform = TransformStamped()
            print(tag)
    
            tag_id = tag.tag_id
            tag_position = tag.pose_t
            tag_orientation = tag.pose_R

            tag_transform.header.stamp = self.get_clock().now().to_msg()
            tag_transform.header.frame_id = 'camera_frame'
            tag_transform.child_frame_id = 'apriltag_' + str(tag_id)
            
            tag_transform.transform.translation.x = tag_position[0, 0]
            tag_transform.transform.translation.y = tag_position[1, 0]
            tag_transform.transform.translation.z = tag_position[2, 0]
            
            tag_transform.transform.rotation.x = tag_orientation[0][0]
            tag_transform.transform.rotation.y = tag_orientation[1][0]
            tag_transform.transform.rotation.z = tag_orientation[2][0]
            tag_transform.transform.rotation.w = tag_orientation[2][2]
        
            quaternion = t3d.quaternions.mat2quat(tag_orientation)
            tag_transform.transform.rotation.x = quaternion[0]
            tag_transform.transform.rotation.y = quaternion[1]
            tag_transform.transform.rotation.z = quaternion[2]
            tag_transform.transform.rotation.w = quaternion[3]

            self.tf_broadcaster.sendTransform(tag_transform)

def main(args=None):
    rclpy.init(args=args)
    apriltag_broadcaster = AprilTagBroadcaster()
    rclpy.spin(apriltag_broadcaster)
    apriltag_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
