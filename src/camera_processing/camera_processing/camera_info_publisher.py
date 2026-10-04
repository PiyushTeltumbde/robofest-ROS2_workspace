#!/usr/bin/env python3
"""
Camera Info Publisher Node Skeleton for SAR Quadcopter.

Publishes camera calibration metadata (sensor_msgs/msg/CameraInfo) on
/camera/camera_info to accompany /camera/image_raw frames for visual SLAM
and 3D mapping nodes.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    """ROS 2 Node skeleton for publishing CameraInfo metadata."""

    def __init__(self) -> None:
        super().__init__('camera_info_publisher')

        # ----------------------------------------------------------------------
        # Parameter Declarations
        # ----------------------------------------------------------------------
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('distortion_model', 'plumb_bob')
        self.declare_parameter('camera_name', 'sar_quad_camera')

        # Intrinsic parameters placeholders (fx, cx, fy, cy)
        self.declare_parameter('camera_matrix', [500.0, 0.0, 320.0,
                                                 0.0, 500.0, 240.0,
                                                 0.0, 0.0, 1.0])
        self.declare_parameter('distortion_coefficients', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('rectification_matrix', [1.0, 0.0, 0.0,
                                                        0.0, 1.0, 0.0,
                                                        0.0, 0.0, 1.0])
        self.declare_parameter('projection_matrix', [500.0, 0.0, 320.0, 0.0,
                                                     0.0, 500.0, 240.0, 0.0,
                                                     0.0, 0.0, 1.0, 0.0])
        self.declare_parameter('publish_rate_hz', 30.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.distortion_model = self.get_parameter('distortion_model').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.get_logger().info(
            f'Camera Info Publisher initialized -> Frame ID: {self.frame_id}, '
            f'Resolution: {self.image_width}x{self.image_height}'
        )

        # ----------------------------------------------------------------------
        # ROS 2 Publisher
        # ----------------------------------------------------------------------
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # ----------------------------------------------------------------------
        # TODO: Camera Calibration Initialization & Sync Strategy
        # ----------------------------------------------------------------------
        # TODO:
        #   1. Load calibration file from YAML path (camera_info_manager or pyyaml).
        #   2. Populate self.camera_info template msg with:
        #      - width, height
        #      - distortion_model ("plumb_bob" or "radtan" or "fisheye")
        #      - D (distortion coefficients)
        #      - K (intrinsic 3x3 matrix)
        #      - R (rectification 3x3 matrix)
        #      - P (projection 3x4 matrix)
        #   3. Sync option A: Publish camera_info in lockstep when /camera/image_raw arrives
        #      using message_filters or direct subscriber synchronization.
        #   4. Sync option B: Periodic broadcast matching sensor stream frequency.


from rclpy.executors import ExternalShutdownException


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraInfoPublisher()
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
