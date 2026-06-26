#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class DepthViewer(Node):

    def __init__(self):
        super().__init__('depth_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10
        )

        self.depth_img = None

        cv2.namedWindow("Depth")
        cv2.setMouseCallback("Depth", self.mouse_callback)

        self.get_logger().info("Depth viewer started")

    def mouse_callback(self, event, x, y, flags, param):

        if self.depth_img is None:
            return

        if event == cv2.EVENT_MOUSEMOVE:

            depth = self.depth_img[y, x]

            if np.isfinite(depth):
                print(
                    f"x={x:3d} y={y:3d} depth={depth:.3f} m",
                    end="\r"
                )

    def depth_callback(self, msg):

        try:

            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='32FC1'
            )

            self.depth_img = depth

            valid = depth[np.isfinite(depth)]

            if len(valid) == 0:
                return

            center_depth = depth[
                depth.shape[0] // 2,
                depth.shape[1] // 2
            ]

            print(
                f"\ncenter={center_depth:.3f}m "
                f"min={valid.min():.3f}m "
                f"max={valid.max():.3f}m"
            )

            depth_vis = depth.copy()

            depth_vis[~np.isfinite(depth_vis)] = 0

            depth_vis = cv2.normalize(
                depth_vis,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            )

            depth_vis = depth_vis.astype(np.uint8)

            depth_vis = cv2.applyColorMap(
                depth_vis,
                cv2.COLORMAP_JET
            )

            cv2.imshow("Depth", depth_vis)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(str(e))


def main():

    rclpy.init()

    node = DepthViewer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()