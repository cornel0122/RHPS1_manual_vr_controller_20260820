#!/usr/bin/env python3
import argparse
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def load_zed_sdk():
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise RuntimeError(
            "pyzed.sl is unavailable. Install the ZED SDK Python API on the PC "
            "that receives the Vision PC stream."
        ) from exc
    return sl


def split_endpoint(endpoint):
    host, separator, port_text = endpoint.rpartition(":")
    if not separator or not host:
        return endpoint, 30000
    return host, int(port_text)


class ZedStreamPublisher(Node):
    def __init__(self, sl, endpoint, topic, frame_id, max_fps, preview):
        super().__init__("rhps1_zed_stream_ros2_publisher")
        self.sl = sl
        self.bridge = CvBridge()
        self.frame_id = frame_id
        self.preview = preview
        self.minimum_period = 1.0 / max_fps if max_fps > 0.0 else 0.0
        self.last_publish = 0.0
        self.frames = 0

        host, port = split_endpoint(endpoint)
        init = sl.InitParameters()
        init.set_from_stream(host, port)
        init.depth_mode = sl.DEPTH_MODE.NONE
        init.sdk_verbose = 1

        self.camera = sl.Camera()
        status = self.camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Cannot open ZED stream {host}:{port}: {status}")

        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.publisher = self.create_publisher(Image, topic, qos)
        self.runtime = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.get_logger().info(f"Receiving ZED stream {host}:{port}; publishing {topic}")

    def capture_once(self):
        if self.camera.grab(self.runtime) != self.sl.ERROR_CODE.SUCCESS:
            return

        now = time.monotonic()
        if self.minimum_period > 0.0 and now - self.last_publish < self.minimum_period:
            return

        self.camera.retrieve_image(self.image, self.sl.VIEW.LEFT)
        frame = np.asarray(self.image.get_data())
        if frame.ndim != 3 or frame.shape[2] not in (3, 4):
            self.get_logger().warning(f"Unexpected ZED image shape: {frame.shape}")
            return

        encoding = "bgra8" if frame.shape[2] == 4 else "bgr8"
        msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(frame), encoding=encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)
        self.last_publish = now
        self.frames += 1
        if self.frames == 1 or self.frames % 100 == 0:
            self.get_logger().info(f"Published {self.frames} ZED frame(s): {msg.width}x{msg.height} {encoding}")

        if self.preview:
            import cv2

            cv2.imshow("RHPS1 ZED stream", frame)
            cv2.waitKey(1)

    def close(self):
        if self.preview:
            try:
                import cv2

                cv2.destroyAllWindows()
            except Exception:
                pass
        self.camera.close()


def main():
    parser = argparse.ArgumentParser(description="Republish an RHPS1 ZED SDK network stream as a ROS 2 Image.")
    parser.add_argument("--stream", default="127.0.0.1:30000", help="Vision PC ZED stream endpoint.")
    parser.add_argument("--topic", default="/rtgr/local/zed/left/image_raw")
    parser.add_argument("--frame-id", default="zed_left_camera_optical_frame")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--preview", action="store_true", help="Also show the received image on the desktop.")
    args = parser.parse_args()

    sl = load_zed_sdk()
    rclpy.init(args=None)
    node = ZedStreamPublisher(sl, args.stream, args.topic, args.frame_id, args.fps, args.preview)
    stop = threading.Event()

    def handle_signal(*_):
        stop.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    try:
        while rclpy.ok() and not stop.is_set():
            node.capture_once()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"zed_stream_ros2_publisher failed: {exc}", file=sys.stderr)
        sys.exit(1)
