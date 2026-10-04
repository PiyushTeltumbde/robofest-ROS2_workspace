#!/usr/bin/env python3
"""
TCP Frame Receiver Node for Search-and-Rescue Quadcopter.

Acts as a TCP server receiving JPEG camera frames transmitted from
Raspberry Pi 5 (TCP client) using a custom framing protocol:
    [4-byte big-endian payload length][JPEG bytes]
Decodes JPEG frames with OpenCV and publishes sensor_msgs/msg/Image
messages to /camera/image_raw.
"""

import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class TcpFrameReceiver(Node):
    """ROS 2 Node for receiving TCP camera stream from Raspberry Pi 5."""

    def __init__(self) -> None:
        super().__init__('camera_receiver')

        # ----------------------------------------------------------------------
        # Parameter Declarations
        # ----------------------------------------------------------------------
        self.declare_parameter('server_ip', '0.0.0.0')
        self.declare_parameter('server_port', 5000)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('encoding', 'bgr8')

        self.server_ip = self.get_parameter('server_ip').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.frame_width = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height = self.get_parameter('frame_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

        self.get_logger().info(
            f'Initializing TCP Frame Receiver -> Target: {self.server_ip}:{self.server_port} '
            f'({self.frame_width}x{self.frame_height} @ {self.fps} FPS, {self.encoding})'
        )

        # ----------------------------------------------------------------------
        # ROS 2 Publisher
        # ----------------------------------------------------------------------
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # ----------------------------------------------------------------------
        # cv_bridge Initialization
        # ----------------------------------------------------------------------
        if CvBridge is not None:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warning(
                'cv_bridge is not installed. Frame publishing will be unavailable until installed.'
            )

        if cv2 is None:
            self.get_logger().warning(
                'OpenCV (cv2) is not installed. Frame decoding will be unavailable until installed.'
            )

        # ----------------------------------------------------------------------
        # Statistics Tracking
        # ----------------------------------------------------------------------
        self.frames_received = 0
        self.frames_decoded = 0
        self.frames_published = 0
        self.frames_dropped = 0
        self.last_stats_time = time.time()
        self.last_published_count = 0

        # ----------------------------------------------------------------------
        # TCP Server Socket Setup
        # ----------------------------------------------------------------------
        self.running = True
        self.current_client_sock = None
        self.client_lock = threading.Lock()

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.server_ip, self.server_port))
        self.server_socket.listen(1)
        self.server_socket.settimeout(1.0)

        self.get_logger().info(f'TCP server listening on {self.server_ip}:{self.server_port}')

        # ----------------------------------------------------------------------
        # Background Worker Thread
        # ----------------------------------------------------------------------
        self.server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self.server_thread.start()

    def recv_exactly(self, sock: socket.socket, num_bytes: int) -> bytes | None:
        """Continuously receive from sock until exactly num_bytes has arrived.

        Returns None if the connection is closed, times out during shutdown,
        or an error occurs.
        """
        buffer = bytearray()
        while len(buffer) < num_bytes:
            if not self.running:
                return None
            try:
                chunk = sock.recv(num_bytes - len(buffer))
            except socket.timeout:
                self._log_periodic_stats()
                continue
            except (socket.error, ConnectionResetError, BrokenPipeError):
                return None
            if not chunk:
                # Connection closed by client
                return None
            buffer.extend(chunk)
        return bytes(buffer)

    def _server_loop(self) -> None:
        """Main TCP server accept loop running on background thread."""
        while self.running:
            try:
                try:
                    client_sock, client_addr = self.server_socket.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if not self.running:
                        break
                    continue

                client_ip, client_port = client_addr[0], client_addr[1]
                self.get_logger().info(f'Client connected from {client_ip}:{client_port}')
                with self.client_lock:
                    self.current_client_sock = client_sock
                client_sock.settimeout(1.0)

                try:
                    self._handle_client(client_sock)
                except Exception as e:
                    self.get_logger().warning(f'Connection error: {e}')
                finally:
                    with self.client_lock:
                        if self.current_client_sock is not None:
                            try:
                                self.current_client_sock.close()
                            except Exception:
                                pass
                            self.current_client_sock = None
                    self.get_logger().info('Client disconnected. Returning to accept()...')
            except Exception as e:
                if not self.running:
                    break
                self.get_logger().error(f'Unexpected error in server loop: {e}')

    def _handle_client(self, client_sock: socket.socket) -> None:
        """Receive framing protocol and publish frames for connected client."""
        max_payload_size = 10 * 1024 * 1024  # 10 MB

        while self.running:
            # 1. Read exactly 4 bytes
            header = self.recv_exactly(client_sock, 4)
            if header is None:
                break

            self.frames_received += 1

            # 2. Decode unsigned 32-bit big-endian payload size
            payload_size = struct.unpack('>I', header)[0]

            # 3. Validate payload size
            if payload_size <= 0 or payload_size > max_payload_size:
                self.frames_dropped += 1
                self.get_logger().warning(
                    f'Invalid payload size: {payload_size} bytes (limit: 1..{max_payload_size}).'
                )
                break

            # 4. Receive exactly payload_size bytes
            jpeg_bytes = self.recv_exactly(client_sock, payload_size)
            if jpeg_bytes is None:
                self.frames_dropped += 1
                break

            # 5. Decode JPEG frame using OpenCV
            if cv2 is None or self.bridge is None:
                self.frames_dropped += 1
                self.get_logger().warning(
                    'Missing cv2 or cv_bridge; cannot decode/publish frame.'
                )
                self._log_periodic_stats()
                continue

            try:
                np_arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            except Exception as e:
                self.frames_dropped += 1
                self.get_logger().warning(f'JPEG decoding exception: {e}')
                self._log_periodic_stats()
                continue

            if image is None:
                self.frames_dropped += 1
                self.get_logger().warning('JPEG decoding failed (cv2.imdecode returned None).')
                self._log_periodic_stats()
                continue

            self.frames_decoded += 1

            # 6. Publish ROS Image immediately
            try:
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_optical_frame'
                self.image_pub.publish(msg)
                self.frames_published += 1
            except Exception as e:
                self.frames_dropped += 1
                self.get_logger().warning(f'Failed to publish image message: {e}')

            self._log_periodic_stats()

    def _log_periodic_stats(self) -> None:
        """Log reception statistics approximately once every 5 seconds."""
        now = time.time()
        elapsed = now - self.last_stats_time
        if elapsed >= 5.0:
            interval_published = self.frames_published - self.last_published_count
            fps = interval_published / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(
                f'received: {self.frames_received}\n'
                f'decoded: {self.frames_decoded}\n'
                f'published: {self.frames_published}\n'
                f'dropped: {self.frames_dropped}\n'
                f'FPS: approximately {fps:.1f}'
            )
            self.last_stats_time = now
            self.last_published_count = self.frames_published

    def destroy_node(self) -> bool:
        """Clean shutdown of sockets and background server thread."""
        self.running = False

        with self.client_lock:
            if self.current_client_sock is not None:
                try:
                    self.current_client_sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    self.current_client_sock.close()
                except Exception:
                    pass
                self.current_client_sock = None

        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except Exception:
                pass
            self.server_socket = None

        if self.server_thread is not None and self.server_thread.is_alive():
            self.server_thread.join(timeout=2.0)

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpFrameReceiver()
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
