#!/usr/bin/env python3
"""
Image Decoder and Rectification Node Skeleton for SAR Quadcopter.

Subscribes to /camera/image_raw, performs format conversion, debayering,
or lens undistortion/rectification if needed, and forwards the processed
frame for downstream perception nodes.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageDecoder(Node):
    """ROS 2 Node skeleton for image decoding and preprocessing."""

    def __init__(self) -> None:
        super().__init__('image_decoder')

        # ----------------------------------------------------------------------
        # Parameter Declarations
        # ----------------------------------------------------------------------
        self.declare_parameter('enable_rectification', False)
        self.declare_parameter('output_encoding', 'bgr8')
        self.declare_parameter('downsample_factor', 1.0)

        self.enable_rectification = (
            self.get_parameter('enable_rectification').get_parameter_value().bool_value
        )
        self.output_encoding = (
            self.get_parameter('output_encoding').get_parameter_value().string_value
        )

        self.get_logger().info(
            f'Image Decoder initialized (rectification={self.enable_rectification}, '
            f'encoding={self.output_encoding})'
        )

        # ----------------------------------------------------------------------
        # ROS 2 Subscriptions & Publishers
        # ----------------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            10
        )

        # Future processed image publisher (e.g. rectified)
        self.processed_image_pub = self.create_publisher(
            Image,
            '/camera/image_rect',
            10
        )

    def _image_callback(self, msg: Image) -> None:
        """Callback received on new raw image."""
        # ----------------------------------------------------------------------
        # TODO: Image Decoding and Preprocessing Implementation
        # ----------------------------------------------------------------------
        # TODO:
        #   1. Convert ROS Image message to OpenCV Mat using cv_bridge or numpy buffer:
        #      - img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(...)
        #   2. Apply color conversion if needed (e.g., Bayer to BGR, RGB to Grayscale).
        #   3. If self.enable_rectification:
        #      - Query camera intrinsics and distortion parameters from camera_info.
        #      - Apply cv2.undistort or cv2.remap.
        #   4. Convert back to sensor_msgs/msg/Image.
        #   5. Preserve original timestamp from msg.header.stamp for SLAM synchronization.
        #   6. Publish to self.processed_image_pub.
        pass


from rclpy.executors import ExternalShutdownException


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageDecoder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
