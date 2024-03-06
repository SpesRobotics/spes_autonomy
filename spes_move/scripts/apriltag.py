from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from cv_bridge import CvBridge
import rclpy
import cv2
from dt_apriltags import Detector
from sensor_msgs.msg import Image
import time
import numpy as np

class AprilTagBroadcaster(Node):

    def __init__(self):    
        print("Initializing AprilTag Broadcaster Node =====")

        super().__init__('apriltag_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        print("Initializing AprilTag Broadcaster Node")
        self.__camera = None
        while self.__camera is None:
            self.__camera = cv2.VideoCapture(-1)
            time.sleep(1)
        # self.cv_bridge = CvBridge()
        # self.__message = None
        # self.__cv_image = None
        # self.__subscriber = self.create_subscription(
        #     Image, '/image_raw', self.__callback, 1
        # )

    # def __callback(self, msg):
    #     self.__message = msg
    #     try:
    #         self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #     except Exception as e:
    #         print("Error converting image:", e)

    def broadcast_transform(self):
        print("-----")
        return_value, frame = self.__camera.read()
        h, w, _ = frame.shape

        color_img = frame[:, round(w/2):w, :]
        image = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        at_detector = Detector(families='tag36h11',
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

        camera_matrix = np.array([[248.091261, 0.0, 338.72890467],
                            [0.0, 249.17817251, 208.70472081],
                            [0.0, 0.0, 1.0]])

        camera_params = ( camera_matrix[0,0], camera_matrix[1,1], camera_matrix[0,2], camera_matrix[1,2] )
        tags = at_detector.detect(image, True, camera_params, 0.09)
        # print(tags)

    
        tag_transform = TransformStamped()
        tag_transform.header.stamp = self.get_clock().now().to_msg()
        tag_transform.header.frame_id = 'camera_frame'
        tag_transform.child_frame_id = 'apriltag'
        tag_transform.transform.translation.x = 1.0
        tag_transform.transform.translation.y = 2.0
        tag_transform.transform.translation.z = 0.0

        tag_transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tag_transform)

def main(args=None):
    rclpy.init(args=args)
    apriltag_broadcaster = AprilTagBroadcaster()
    rclpy.spin(apriltag_broadcaster)
    apriltag_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
