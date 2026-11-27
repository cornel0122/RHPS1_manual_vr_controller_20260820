#!/usr/bin/env python3
"""Terminal PC: receive HMD JPEG frames from operator_pc/tools/hmd_jpeg_forwarder.py over TCP and republish
them locally as sensor_msgs/CompressedImage for the Vulkan HMD overlay.

Listens on --port (default 30555). One sender at a time; a new connection replaces the old one.
Wire format per frame: 4-byte big-endian length + JPEG bytes.
"""
import argparse
import socket
import struct
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

MAX_FRAME = 8 * 1024 * 1024


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("sender closed the connection")
        buf.extend(chunk)
    return bytes(buf)


class Receiver(Node):
    def __init__(self, topic, port, frame_id):
        super().__init__("rhps1_hmd_jpeg_receiver")
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(CompressedImage, topic, qos)
        self.frame_id = frame_id
        self.received = 0
        self.port = port
        self.running = True
        threading.Thread(target=self.server, daemon=True).start()
        self.create_timer(5.0, lambda: self.get_logger().info(f"received={self.received}"))
        self.get_logger().info(f"Listening on :{port}, publishing {topic}")

    def server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self.port))
        srv.listen(1)
        while self.running:
            conn, addr = srv.accept()
            self.get_logger().info(f"Sender connected: {addr[0]}")
            try:
                with conn:
                    while self.running:
                        (length,) = struct.unpack(">I", recv_exact(conn, 4))
                        if length == 0 or length > MAX_FRAME:
                            raise ValueError(f"bad frame length {length}")
                        data = recv_exact(conn, length)
                        msg = CompressedImage()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self.frame_id
                        msg.format = "jpeg"
                        msg.data = data
                        self.pub.publish(msg)
                        self.received += 1
            except (OSError, ValueError, ConnectionError) as exc:
                self.get_logger().warning(f"Sender {addr[0]} disconnected: {exc}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=30555)
    parser.add_argument("--topic", default="/rtgr/local/zed/left/image_raw/compressed")
    parser.add_argument("--frame-id", default="head_camera_optical_frame")
    args = parser.parse_args()
    rclpy.init()
    node = Receiver(args.topic, args.port, args.frame_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():           # a real error, not the context being shut down by SIGTERM/SIGINT
            raise
    finally:
        node.running = False
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
