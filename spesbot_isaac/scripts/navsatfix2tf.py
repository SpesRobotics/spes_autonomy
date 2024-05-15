#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import NavSatFix
import pymap3d as pm
import ros2_numpy as rnp
import numpy as np


class NavSatToTF(Node):
    def __init__(self):
        super().__init__("navsat_to_tf_node")
        self.subscription = self.create_subscription(
            NavSatFix, "/navsatfix", self.navsat_callback, 10
        )
        self.tf_publisher = self.create_publisher(TFMessage, "/tf", 10)

        self.ref_lat = 45.2586161
        self.ref_lon = 19.8066591
        self.ref_alt = 76.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def navsat_callback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        x, y, z = pm.geodetic2enu(
            lat=lat,
            lon=lon,
            h=alt,
            lat0=self.ref_lat,
            lon0=self.ref_lon,
            h0=self.ref_alt,
        )

        tf_map_base_link = np.eye(4)
        tf_map_base_link[:3, 3] = np.array([x, y, z])
        tf_odom_base_link = None

        try:
            odom_base_link_msg = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            )
            tf_odom_base_link = rnp.numpify(odom_base_link_msg.transform)
        except Exception as e:
            return

        tf_map_odom = tf_map_base_link @ np.linalg.inv(tf_odom_base_link)

        map_odom = TransformStamped()
        map_odom.header.stamp = self.get_clock().now().to_msg()
        map_odom.header.frame_id = "map"
        map_odom.child_frame_id = "odom"
        map_odom.transform = rnp.msgify(Transform, tf_map_odom)
        map_odom_msg = TFMessage(transforms=[map_odom])
        self.tf_publisher.publish(map_odom_msg)


def main(args=None):
    rclpy.init(args=args)
    navsat_to_tf_node = NavSatToTF()
    rclpy.spin(navsat_to_tf_node)
    navsat_to_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
