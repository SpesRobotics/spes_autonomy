#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix
import pymap3d as pm


class TFToNavSatNode(Node):
    def __init__(self):
        super().__init__("tf_to_navsat_node")
        self.subscription = self.create_subscription(
            TFMessage, "/isaac/tf", self.tf_callback, 10
        )
        self.publisher = self.create_publisher(NavSatFix, "/navsatfix", 10)

        self.ref_lat = 45.2586161
        self.ref_lon = 19.8066591
        self.ref_alt = 76.0

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if (
                transform.header.frame_id == "World"
                and transform.child_frame_id == "base_link"
            ):
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z

                lat, lon, alt = pm.enu2geodetic(
                    e=x, n=y, u=z, lat0=self.ref_lat, lon0=self.ref_lon, h0=self.ref_alt
                )

                navsat_msg = NavSatFix()
                navsat_msg.header.stamp = self.get_clock().now().to_msg()
                navsat_msg.header.frame_id = "map"
                navsat_msg.latitude = lat
                navsat_msg.longitude = lon
                navsat_msg.altitude = alt
                navsat_msg.position_covariance
                self.publisher.publish(navsat_msg)


def main(args=None):
    rclpy.init(args=args)
    tf_to_navsat_node = TFToNavSatNode()
    rclpy.spin(tf_to_navsat_node)
    tf_to_navsat_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
