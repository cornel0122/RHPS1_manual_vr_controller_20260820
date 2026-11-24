#!/usr/bin/env python3

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class TestPatternPublisher(Node):
    def __init__(self, topic: str, width: int, height: int, fps: float) -> None:
        super().__init__("overlay_test_pattern_publisher")
        self.publisher = self.create_publisher(CompressedImage, topic, qos_profile_sensor_data)
        self.width = width
        self.height = height
        self.frame = 0
        self.started = time.monotonic()
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(
            f"Publishing animated JPEG test pattern on '{topic}' at {width}x{height}, {fps:.1f} FPS"
        )

    def publish_frame(self) -> None:
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        colors = (
            (255, 255, 255),
            (0, 255, 255),
            (255, 255, 0),
            (0, 255, 0),
            (255, 0, 255),
            (0, 0, 255),
            (255, 0, 0),
        )
        stripe_width = max(1, self.width // len(colors))
        offset = (self.frame * 4) % self.width
        for index, color in enumerate(colors):
            left = (index * stripe_width + offset) % self.width
            right = min(self.width, left + stripe_width)
            image[:, left:right] = color
            if right - left < stripe_width:
                image[:, : stripe_width - (right - left)] = color

        elapsed = time.monotonic() - self.started
        cv2.putText(
            image,
            f"Vulkan overlay  frame={self.frame}  time={elapsed:7.1f}s",
            (40, self.height - 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (20, 20, 20),
            5,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"Vulkan overlay  frame={self.frame}  time={elapsed:7.1f}s",
            (40, self.height - 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            self.get_logger().error("Failed to encode test frame")
            return

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.publisher.publish(message)
        self.frame += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish an animated ROS 2 JPEG test pattern.")
    parser.add_argument("--topic", default="/rtgr/test/overlay/compressed")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = TestPatternPublisher(args.topic, args.width, args.height, args.fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
