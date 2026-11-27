#!/usr/bin/env python3
"""Record exactly what the HMD overlay shows: the HMD JPEG topic, to an MP4 file (+ stills on demand).

The Vulkan overlay displays this topic unchanged, so the video is the HMD view (SteamVR's own "Display VR
View" mirror is white on the Operator PC's Intel iGPU and is not needed for this). Subscribe-only: the HMD
and the intent recogniser are not affected.

    s + Enter : save the current frame as PNG        q + Enter (or Ctrl+C) : stop and close the video
"""
import argparse
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class Recorder(Node):
    def __init__(self, topic, out_dir, fps, stamp_text):
        super().__init__("rhps1_hmd_view_recorder")
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, topic, self.on_image, qos)
        self.out_dir, self.fps, self.stamp_text = out_dir, fps, stamp_text
        self.base = datetime.now().strftime("hmd_view_%Y%m%d_%H%M%S")
        self.video_path = os.path.join(out_dir, self.base + ".mp4")
        self.writer = None
        self.last = None
        self.frames = self.stills = 0
        self.lock = threading.Lock()
        self.create_timer(5.0, lambda: self.get_logger().info(f"recorded {self.frames} frame(s) -> {self.video_path}"))
        self.get_logger().info(f"Recording {topic} -> {self.video_path}  (s+Enter: still, q+Enter: stop)")

    def on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        if self.stamp_text:
            cv2.putText(frame, datetime.now().strftime("%H:%M:%S.%f")[:-3], (16, 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        with self.lock:
            if self.writer is None:
                h, w = frame.shape[:2]
                self.writer = cv2.VideoWriter(self.video_path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
                if not self.writer.isOpened():
                    self.get_logger().error(f"cannot open {self.video_path}")
                    rclpy.try_shutdown()
                    return
            self.writer.write(frame)
            self.last = frame
            self.frames += 1

    def save_still(self):
        with self.lock:
            frame = None if self.last is None else self.last.copy()
        if frame is None:
            print("no frame yet", flush=True)
            return
        self.stills += 1
        path = os.path.join(self.out_dir, f"{self.base}_still{self.stills:02d}.png")
        cv2.imwrite(path, frame)
        print(f"saved {path}", flush=True)

    def close(self):
        with self.lock:
            if self.writer is not None:
                self.writer.release()
                self.writer = None
        print(f"closed {self.video_path} ({self.frames} frames)", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/rtgr/local/zed/left/image_raw/compressed")
    parser.add_argument("--out-dir", default=os.path.expanduser("~/rhps1_hmd_recordings"))
    parser.add_argument("--fps", type=float, default=15.0, help="video frame rate (the HMD stream is 15 Hz)")
    parser.add_argument("--no-time", action="store_true", help="do not draw the wall-clock time on the video")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    rclpy.init()
    node = Recorder(args.topic, args.out_dir, args.fps, not args.no_time)

    def keyboard():
        for line in sys.stdin:
            key = line.strip().lower()[:1]
            if key == "s":
                node.save_still()
            elif key == "q":
                rclpy.try_shutdown()
                return

    threading.Thread(target=keyboard, daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
