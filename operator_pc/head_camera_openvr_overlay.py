#!/usr/bin/env python3
import argparse
import os
import signal
import sys
import threading
import time
from ctypes import c_ubyte

import numpy as np
import openvr
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class HeadCameraOverlay(Node):
    def __init__(self, topic: str, width_m: float, distance_m: float, fps: float, debug_marker: bool):
        super().__init__("head_camera_openvr_overlay")
        self.bridge = CvBridge()
        self.overlay = openvr.VROverlay()
        self.overlay_handle = self._create_overlay()
        self.width_m = width_m
        self.distance_m = distance_m
        self.min_period = 1.0 / fps if fps > 0.0 else 0.0
        self.last_publish = 0.0
        self.frames_received = 0
        self.frames_sent = 0
        self.frames_skipped_pending = 0
        self.events_drained = 0
        self.image_loaded_events = 0
        self.raw_submission_pending = False
        self.raw_submission_started = 0.0
        self.debug_marker = debug_marker
        self.rgba_buffer = None
        self.lock = threading.Lock()
        self.last_error = ""
        self.last_image_time = time.monotonic()

        self._configure_overlay()
        qos = QoSProfile(depth=1)
        qos.history = HistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(Image, topic, self.on_image, qos)
        # SetOverlayRaw 是异步操作，较高频率轮询完成事件可防止提交队列持续增长。
        self.event_timer = self.create_timer(0.02, self.on_event_timer)
        self.heartbeat_timer = self.create_timer(5.0, self.on_heartbeat)
        self.get_logger().info(f"Showing ROS image topic '{topic}' in SteamVR overlay")

    def _drain_overlay_events(self):
        # SetOverlayRaw 完成后会产生 ImageLoaded 事件；必须持续清空，否则事件队列会逐渐塞满。
        drained = 0
        while True:
            event = openvr.VREvent_t()
            available, event = self.overlay.pollNextOverlayEvent(self.overlay_handle, event)
            if not available:
                break
            drained += 1
            if event.eventType == openvr.VREvent_ImageLoaded:
                self.image_loaded_events += 1
                self.raw_submission_pending = False
            elif event.eventType == openvr.VREvent_ImageFailed:
                self.raw_submission_pending = False
                self.get_logger().warning("SteamVR reported VREvent_ImageFailed for raw overlay")
        self.events_drained += drained
        return drained

    def on_event_timer(self):
        with self.lock:
            self._drain_overlay_events()

    def on_heartbeat(self):
        age = time.monotonic() - self.last_image_time
        self.get_logger().info(
            "Overlay heartbeat: "
            f"received={self.frames_received}, submitted={self.frames_sent}, "
            f"pending_skips={self.frames_skipped_pending}, image_loaded={self.image_loaded_events}, "
            f"pending={self.raw_submission_pending}, events_drained={self.events_drained}, "
            f"last_frame_age={age:.3f}s"
        )

    def _create_overlay(self):
        # OpenVR 的 overlay key 在运行时全局唯一。旧进程异常退出或 SteamVR
        # 尚未清理旧 handle 时，复用固定 key 会得到没有文本的 OpenVR 异常。
        # 每个进程使用独立 key，进程退出后 SteamVR 会自动销毁对应 overlay。
        key = f"rtgr.head_camera_overlay.{os.getpid()}"
        try:
            return self.overlay.createOverlay(key, "RHPS1 Robot Head Camera")
        except Exception as exc:
            raise RuntimeError(
                f"createOverlay(key={key}) failed: {type(exc).__name__}, "
                f"args={exc.args!r}, repr={exc!r}"
            ) from exc

    def _configure_overlay(self):
        self.overlay.setOverlayWidthInMeters(self.overlay_handle, self.width_m)
        self.overlay.setOverlayAlpha(self.overlay_handle, 1.0)
        self.overlay.setOverlayColor(self.overlay_handle, 1.0, 1.0, 1.0)
        self.overlay.setOverlaySortOrder(self.overlay_handle, 100)
        self.overlay.setOverlayFlag(self.overlay_handle, openvr.VROverlayFlags_IgnoreTextureAlpha, True)
        self.overlay.setOverlayInputMethod(self.overlay_handle, openvr.VROverlayInputMethod_None)

        transform = openvr.HmdMatrix34_t()
        transform.m[0][0] = 1.0
        transform.m[0][1] = 0.0
        transform.m[0][2] = 0.0
        transform.m[0][3] = 0.0
        transform.m[1][0] = 0.0
        transform.m[1][1] = 1.0
        transform.m[1][2] = 0.0
        transform.m[1][3] = 0.0
        transform.m[2][0] = 0.0
        transform.m[2][1] = 0.0
        transform.m[2][2] = 1.0
        transform.m[2][3] = -abs(self.distance_m)
        self.overlay.setOverlayTransformTrackedDeviceRelative(self.overlay_handle, openvr.k_unTrackedDeviceIndex_Hmd, transform)
        self.overlay.showOverlay(self.overlay_handle)

    def on_image(self, msg: Image):
        self.frames_received += 1
        self.last_image_time = time.monotonic()
        if self.frames_received == 1:
            self.get_logger().info(
                f"First image received: {msg.width}x{msg.height}, encoding={msg.encoding}, step={msg.step}"
            )

        now = time.monotonic()
        if self.min_period > 0.0 and now - self.last_publish < self.min_period:
            return

        with self.lock:
            self._drain_overlay_events()
            if self.raw_submission_pending:
                pending_age = now - self.raw_submission_started
                if pending_age < 0.5:
                    self.frames_skipped_pending += 1
                    return

                # SteamVR 未确认上一帧时清空旧纹理，避免 raw image 队列永久堵塞。
                self.get_logger().warning(
                    f"Raw overlay frame acknowledgement timed out after {pending_age:.3f}s; resetting texture"
                )
                try:
                    self.overlay.clearOverlayTexture(self.overlay_handle)
                except Exception as exc:
                    self.get_logger().warning(f"clearOverlayTexture failed: {exc}")
                self.raw_submission_pending = False

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            image = np.ascontiguousarray(image, dtype=np.uint8)
            if image.ndim != 3 or image.shape[2] != 3:
                raise ValueError(f"expected rgb8 image, got shape {image.shape}")

            height, width, _ = image.shape
            if self.debug_marker:
                marker_size = max(8, min(width, height) // 16)
                phase = self.frames_sent % 4
                x = 4 + phase * (marker_size + 4)
                y = 4
                image[y : y + marker_size, x : x + marker_size, :] = (255, 32, 32)

            if self.rgba_buffer is None or self.rgba_buffer.shape[:2] != (height, width):
                self.rgba_buffer = np.empty((height, width, 4), dtype=np.uint8)
            rgba = self.rgba_buffer
            rgba[:, :, :3] = image
            rgba[:, :, 3] = 255
            # OpenVR 在本次调用中同步读取缓冲区，直接共享 NumPy 内存可避免每帧再次复制约 1.2 MB。
            raw = (c_ubyte * rgba.size).from_buffer(rgba)
            with self.lock:
                self._drain_overlay_events()
                if self.raw_submission_pending:
                    self.frames_skipped_pending += 1
                    return
                self.overlay.setOverlayRaw(self.overlay_handle, raw, width, height, 4)
                self.raw_submission_pending = True
                self.raw_submission_started = now
                self._drain_overlay_events()
                # overlay 在初始化时已经显示；逐帧重复 showOverlay 会增加 compositor 消息压力。
            self.last_publish = now
            self.frames_sent += 1
            if self.frames_sent == 1 or self.frames_sent % 50 == 0:
                self.get_logger().info(f"Submitted {self.frames_sent} frame(s) to OpenVR overlay")
        except Exception as exc:
            text = str(exc)
            if text != self.last_error:
                self.get_logger().error(f"Failed to update OpenVR overlay: {text}")
                self.last_error = text

    def close(self):
        try:
            self.overlay.hideOverlay(self.overlay_handle)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="Show a ROS2 Image topic as a SteamVR/OpenVR overlay.")
    parser.add_argument("--topic", default="/rtgr/sim/head_camera/rgb/image_raw")
    parser.add_argument("--width-m", type=float, default=1.4)
    parser.add_argument("--distance-m", type=float, default=1.2)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--no-debug-marker", action="store_true", help="Do not draw the small moving red marker.")
    args = parser.parse_args()

    openvr.init(openvr.VRApplication_Overlay)
    rclpy.init(args=None)
    node = HeadCameraOverlay(args.topic, args.width_m, args.distance_m, args.fps, not args.no_debug_marker)

    stop = threading.Event()

    def handle_signal(*_):
        stop.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        openvr.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(
            "head_camera_openvr_overlay failed: "
            f"type={type(exc).__name__}, args={exc.args!r}, repr={exc!r}",
            file=sys.stderr,
        )
        sys.exit(1)
