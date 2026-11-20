"""ROS 2 node: get webcam frames into ROS as sensor_msgs/Image.

source:=tcp   (default, for WSL2) - listen on `port`; windows/cam_streamer.py on the
              Windows side connects to localhost:<port> and streams JPEG frames.
              (WSL2 forwards Windows localhost to WSL, so nothing to configure.)
source:=local - open a V4L2 camera directly with OpenCV (native Linux only; the
              WSL kernel has no camera drivers).

Publishes /camera/image_raw (bgr8) and /camera/image_raw/compressed (jpeg).
Runs with the ~/RobotArm/.venv_vision interpreter (needs OpenCV); see launch file.
"""

import array
import socket
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from arm_vision_teleop.stream_protocol import recv_frame


class CameraBridge(Node):

    def __init__(self):
        super().__init__('camera_bridge')
        p = self.declare_parameter
        self.source = p('source', 'tcp').value
        self.port = int(p('port', 5005).value)
        self.bind = p('bind_address', '0.0.0.0').value
        self.device = int(p('device', 0).value)
        self.width = int(p('width', 640).value)
        self.height = int(p('height', 480).value)
        self.fps = float(p('fps', 30.0).value)
        self.frame_id = p('frame_id', 'camera').value
        topic = p('image_topic', '/camera/image_raw').value

        self.pub = self.create_publisher(Image, topic, 2)
        self.cpub = self.create_publisher(CompressedImage, topic + '/compressed', 2)
        self.running = True
        self.socks = []      # open sockets, closed on shutdown to unblock the thread
        self.frames = 0
        self.create_timer(5.0, self._report)

        target = {'tcp': self._tcp_loop, 'local': self._local_loop}.get(self.source)
        if target is None:
            raise ValueError("source must be 'tcp' or 'local'")
        self.thread = threading.Thread(target=target, daemon=True)
        self.thread.start()

    # ---------------------------------------------------------------- publish
    def _publish(self, bgr, jpeg=None):
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        # array.array('B') takes rclpy's fast path; bytes would be validated element by element.
        msg.data = array.array('B', np.ascontiguousarray(bgr).tobytes())
        self.pub.publish(msg)
        if jpeg is not None and self.cpub.get_subscription_count():
            cmsg = CompressedImage()
            cmsg.header = msg.header
            cmsg.format = 'jpeg'
            cmsg.data = array.array('B', jpeg)
            self.cpub.publish(cmsg)
        self.frames += 1

    def _report(self):
        if self.frames:
            self.get_logger().info(f'{self.frames / 5.0:.1f} fps', throttle_duration_sec=30.0)
        self.frames = 0

    # ---------------------------------------------------------------- sources
    def _tcp_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.bind, self.port))
        srv.listen(1)
        srv.settimeout(1.0)
        self.socks.append(srv)
        self.get_logger().info(
            f'Waiting for the Windows camera streamer on port {self.port}. On Windows run:\n'
            f'    py cam_streamer.py --port {self.port}')
        while self.running and rclpy.ok():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            self.get_logger().info(f'Camera streamer connected from {addr[0]}')
            conn.settimeout(5.0)
            self.socks.append(conn)
            try:
                while self.running and rclpy.ok():
                    jpeg = recv_frame(conn)
                    bgr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                    if bgr is None:
                        self.get_logger().warn('undecodable frame', throttle_duration_sec=2.0)
                        continue
                    self._publish(bgr, jpeg)
            except Exception as e:  # noqa: B902  (incl. publish errors at shutdown)
                if self.running and rclpy.ok():
                    self.get_logger().warn(f'Camera streamer disconnected ({e}); waiting again.')
            finally:
                conn.close()
                self.socks.remove(conn)
        srv.close()

    def stop(self):
        """Stop the capture thread *before* destroy_node: destroying the publisher while
        the thread is still publishing deadlocks (Ctrl+C then needs SIGKILL)."""
        self.running = False
        for s in list(self.socks):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            s.close()
        self.thread.join(timeout=3.0)

    def _local_loop(self):
        cap = cv2.VideoCapture(self.device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not cap.isOpened():
            self.get_logger().error(
                f'Cannot open /dev/video{self.device}. On WSL2 use source:=tcp instead.')
            return
        self.get_logger().info(f'Streaming /dev/video{self.device}')
        while self.running and rclpy.ok():
            ok, bgr = cap.read()
            if not ok:
                self.get_logger().warn('camera read failed', throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue
            self._publish(bgr)
        cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
        # else: Ctrl+C invalidated the context mid-spin; a normal shutdown.
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
