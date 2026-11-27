#!/usr/bin/env python3
"""Forward the HMD JPEG topic from the Operator PC to another PC over TCP (latest frame only).

Used when the HMD/SteamVR/VIVE run on the Terminal PC (see terminal_pc/README.md). The image chain on this
PC stays unchanged (local ROS domain 201, CycloneDDS, localhost only); this node subscribes to the JPEG
locally and sends it to terminal_pc/hmd_jpeg_receiver.py, which republishes it on the Terminal PC.

Wire format per frame: 4-byte big-endian length + JPEG bytes. A slow link never queues: only the newest
frame is sent. Reconnects automatically.
"""
import argparse
import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class Forwarder(Node):
    def __init__(self, topic, host, port):
        super().__init__("rhps1_hmd_jpeg_forwarder")
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, topic, self.on_image, qos)
        self.host, self.port = host, port
        self.latest = None
        self.cond = threading.Condition()
        self.received = self.sent = 0
        self.running = True
        threading.Thread(target=self.sender, daemon=True).start()
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"Forwarding {topic} to {host}:{port}")

    def on_image(self, msg):
        with self.cond:
            self.latest = bytes(msg.data)
            self.received += 1
            self.cond.notify()

    def sender(self):
        while self.running:
            try:
                with socket.create_connection((self.host, self.port), timeout=3.0) as sock:
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    sock.settimeout(5.0)
                    self.get_logger().info(f"Connected to {self.host}:{self.port}")
                    while self.running:
                        with self.cond:
                            while self.latest is None and self.running:
                                self.cond.wait(0.5)
                            frame, self.latest = self.latest, None
                        if frame is None:
                            continue
                        sock.sendall(struct.pack(">I", len(frame)) + frame)
                        self.sent += 1
            except OSError as exc:
                self.get_logger().warning(f"No connection to {self.host}:{self.port} ({exc}); retrying in 2 s")
                time.sleep(2.0)

    def report(self):
        self.get_logger().info(f"received={self.received} sent={self.sent}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="Terminal PC address")
    parser.add_argument("--port", type=int, default=30555)
    parser.add_argument("--topic", default="/rtgr/local/zed/left/image_raw/compressed")
    args = parser.parse_args()
    rclpy.init()
    node = Forwarder(args.topic, args.host, args.port)
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
