#!/usr/bin/env python3
"""One ZED receiver for the RHPS1 head camera stream (Vision PC -> Operator PC).

Publishes, each optional:
  --topic              raw BGRA Image (desktop tools; "" or --no-raw to disable)
  --compressed-topic   JPEG CompressedImage for the HMD overlay (--hmd-width x --hmd-height)
  --inference-topic / --depth-topic / --camera-info-topic
                       RGB + metric depth (ZED NEURAL depth computed on this PC) + CameraInfo for the
                       Cornel intent recogniser, at --inference-width x --inference-height
Reading the same ZED stream with several processes causes corrupted chunks and frozen images, so run a
single receiver and let every consumer subscribe to it (design by Codex, 2026-09-30).
"""
import argparse
import signal
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


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
    def __init__(
        self,
        sl,
        endpoint,
        topic,
        frame_id,
        max_fps,
        preview,
        compressed_topic="",
        jpeg_quality=95,
        hmd_width=1280,
        hmd_height=720,
        inference_topic="",
        depth_topic="",
        camera_info_topic="",
        inference_width=640,
        inference_height=360,
    ):
        super().__init__("rhps1_zed_stream_ros2_publisher")
        self.sl = sl
        self.bridge = CvBridge()
        self.frame_id = frame_id
        self.preview = preview
        self.jpeg_quality = min(max(int(jpeg_quality), 1), 100)
        self.hmd_width = max(int(hmd_width), 1)
        self.hmd_height = max(int(hmd_height), 1)
        self.inference_width = max(int(inference_width), 1)
        self.inference_height = max(int(inference_height), 1)
        self.depth_enabled = bool(inference_topic and depth_topic and camera_info_topic)
        self.minimum_period = 1.0 / max_fps if max_fps > 0.0 else 0.0
        self.last_publish = 0.0
        self.frames = 0

        host, port = split_endpoint(endpoint)
        init = sl.InitParameters()
        init.set_from_stream(host, port)
        init.depth_mode = sl.DEPTH_MODE.NEURAL if self.depth_enabled else sl.DEPTH_MODE.NONE
        init.coordinate_units = sl.UNIT.METER
        init.sdk_verbose = 1

        self.camera = sl.Camera()
        status = self.camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Cannot open ZED stream {host}:{port}: {status}")

        camera_info = self.camera.get_camera_information()
        self.calibration = camera_info.camera_configuration.calibration_parameters.left_cam

        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.publisher = self.create_publisher(Image, topic, qos) if topic else None
        self.compressed_publisher = (
            self.create_publisher(CompressedImage, compressed_topic, qos)
            if compressed_topic else None
        )
        self.inference_publisher = None
        self.depth_publisher = None
        self.camera_info_publisher = None
        if self.depth_enabled:
            self.inference_publisher = self.create_publisher(Image, inference_topic, qos)
            self.depth_publisher = self.create_publisher(Image, depth_topic, qos)
            self.camera_info_publisher = self.create_publisher(CameraInfo, camera_info_topic, qos)

        self.runtime = sl.RuntimeParameters()
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.get_logger().info(f"Receiving ZED stream {host}:{port}")
        if self.publisher is not None:
            self.get_logger().info(f"Publishing raw HMD RGB: {topic}")
        if self.compressed_publisher is not None:
            self.get_logger().info(
                f"Publishing JPEG HMD RGB: {compressed_topic} "
                f"at {self.hmd_width}x{self.hmd_height}, quality={self.jpeg_quality}"
            )
        if self.depth_enabled:
            self.get_logger().info(
                "Publishing CHMM RGB/depth/CameraInfo: "
                f"{inference_topic}, {depth_topic}, {camera_info_topic} "
                f"at {self.inference_width}x{self.inference_height}"
            )

    def _camera_info_message(self, stamp, source_width, source_height):
        sx = self.inference_width / float(max(source_width, 1))
        sy = self.inference_height / float(max(source_height, 1))
        fx = float(self.calibration.fx) * sx
        fy = float(self.calibration.fy) * sy
        cx = float(self.calibration.cx) * sx
        cy = float(self.calibration.cy) * sy

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.inference_width
        msg.height = self.inference_height
        msg.distortion_model = "plumb_bob"
        distortion = np.asarray(getattr(self.calibration, "disto", []), dtype=float).reshape(-1)
        msg.d = distortion.tolist()
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

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

        stamp = self.get_clock().now().to_msg()
        encoding = "bgra8" if frame.shape[2] == 4 else "bgr8"
        if self.publisher is not None:
            msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(frame), encoding=encoding)
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)

        if self.compressed_publisher is not None:
            hmd_frame = cv2.resize(
                frame[..., :3], (self.hmd_width, self.hmd_height), interpolation=cv2.INTER_AREA
            )
            encoded, jpeg = cv2.imencode(
                ".jpg", hmd_frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if encoded:
                compressed = CompressedImage()
                compressed.header.stamp = stamp
                compressed.header.frame_id = self.frame_id
                compressed.format = "jpeg"
                compressed.data = jpeg.tobytes()
                self.compressed_publisher.publish(compressed)

        if self.depth_enabled:
            self.camera.retrieve_measure(self.depth, self.sl.MEASURE.DEPTH)
            depth = np.asarray(self.depth.get_data(), dtype=np.float32)
            if depth.ndim == 3:
                depth = depth[..., 0]

            bgr = frame[..., :3]
            inference_size = (self.inference_width, self.inference_height)
            inference_rgb = cv2.resize(bgr, inference_size, interpolation=cv2.INTER_AREA)
            inference_depth = cv2.resize(depth, inference_size, interpolation=cv2.INTER_NEAREST)

            rgb_msg = self.bridge.cv2_to_imgmsg(
                np.ascontiguousarray(inference_rgb), encoding="bgr8"
            )
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = self.frame_id
            depth_msg = self.bridge.cv2_to_imgmsg(
                np.ascontiguousarray(inference_depth, dtype=np.float32), encoding="32FC1"
            )
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id

            self.inference_publisher.publish(rgb_msg)
            self.depth_publisher.publish(depth_msg)
            self.camera_info_publisher.publish(
                self._camera_info_message(stamp, frame.shape[1], frame.shape[0])
            )

        self.last_publish = now
        self.frames += 1
        if self.frames == 1 or self.frames % 100 == 0:
            self.get_logger().info(
                f"Published {self.frames} ZED frame(s): {frame.shape[1]}x{frame.shape[0]} {encoding}"
            )

        if self.preview:
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
    parser.add_argument("--compressed-topic", default="")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--hmd-width", type=int, default=1280)
    parser.add_argument("--hmd-height", type=int, default=720)
    parser.add_argument("--inference-topic", default="")
    parser.add_argument("--depth-topic", default="")
    parser.add_argument("--camera-info-topic", default="")
    parser.add_argument("--inference-width", type=int, default=640)
    parser.add_argument("--inference-height", type=int, default=360)
    parser.add_argument("--frame-id", default="zed_left_camera_optical_frame")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--no-raw", action="store_true", help="Do not publish the raw Image topic (same as --topic '').")
    parser.add_argument("--compressed-max-width", type=int, default=0,
                        help="Alias kept for older launchers: HMD JPEG width with 16:9 height (overrides --hmd-*).")
    parser.add_argument("--preview", action="store_true", help="Also show the received image on the desktop.")
    args = parser.parse_args()
    if args.no_raw:
        args.topic = ""
    if args.compressed_max_width > 0:
        args.hmd_width = args.compressed_max_width
        args.hmd_height = int(round(args.compressed_max_width * 9 / 16))

    sl = load_zed_sdk()
    rclpy.init(args=None)
    node = ZedStreamPublisher(
        sl, args.stream, args.topic, args.frame_id, args.fps, args.preview,
        compressed_topic=args.compressed_topic,
        jpeg_quality=args.jpeg_quality,
        hmd_width=args.hmd_width,
        hmd_height=args.hmd_height,
        inference_topic=args.inference_topic,
        depth_topic=args.depth_topic,
        camera_info_topic=args.camera_info_topic,
        inference_width=args.inference_width,
        inference_height=args.inference_height,
    )
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
