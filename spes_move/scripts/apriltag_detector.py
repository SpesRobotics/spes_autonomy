from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy, cv2, time
from rclpy.node import Node
from cv_bridge import CvBridge
from dt_apriltags import Detector
from sensor_msgs.msg import Image
import numpy as np
import transforms3d as t3d

# sudo modprobe v4l2loopback video_nr=20,30 exclusive_caps=1 card_label="Spes Camera"
# ffmpeg -i /dev/video0 -codec copy -f v4l2 /dev/video20 -codec copy -f v4l2 /dev/video30

CAMERA_MATRIX = np.array([[922.64240542, 0.0, 621.33488741],
                          [0.0, 905.74778606, 386.01481495],
                          [0.0, 0.0, 1.0]])

class OpencvCameraReader:
    def __init__(self, source):
        self.__capture = None
        while self.__capture == None:
            self.__capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
            self.__capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.__capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            time.sleep(1)

    def is_open(self):
        return self.__capture.isOpened()

    def read(self):
        success, image = self.__capture.read()
        return success, image

    def close(self):
        self.__capture.release()

class AprilTagBroadcaster(Node):

    def __init__(self):    
        super().__init__('apriltag_broadcaster_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        self.__image_reader = OpencvCameraReader(30)

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
        success, image = self.__image_reader.read()
        
        if not success:
            return
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(image, True, self.camera_params, 0.09)

        for tag in tags:
            tag_transform = TransformStamped()
            self.get_logger().info(f'Apriltag found: {tag.tag_id}')

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
            tag_transform.transform.rotation.x = quaternion[1]
            tag_transform.transform.rotation.y = quaternion[2]
            tag_transform.transform.rotation.z = quaternion[3]
            tag_transform.transform.rotation.w = quaternion[0]

            self.tf_broadcaster.sendTransform(tag_transform)

def main(args=None):
    rclpy.init(args=args)
    apriltag_broadcaster = AprilTagBroadcaster()
    rclpy.spin(apriltag_broadcaster)
    apriltag_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
