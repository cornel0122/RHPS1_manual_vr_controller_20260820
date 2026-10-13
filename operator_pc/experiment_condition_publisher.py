#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


CONDITIONS = {
    1: "no-delay / no-assistance",
    2: "no-delay / assistance",
    3: "1.5s-delay / no-assistance",
    4: "1.5s-delay / assistance",
}


class ExperimentConditionPublisher(Node):
    def __init__(self, condition: int, session: str, topic: str) -> None:
        super().__init__("ana_experiment_condition_publisher")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._message = String()
        self._message.data = f"{condition}:{session}"
        self._publisher = self.create_publisher(String, topic, qos)
        self._timer = self.create_timer(0.5, self._publish)
        self._publish()
        self.get_logger().info(
            f"Experiment session={session}, condition={condition} "
            f"({CONDITIONS[condition]}), topic={topic}"
        )

    def _publish(self) -> None:
        self._publisher.publish(self._message)


def main() -> None:
    parser = argparse.ArgumentParser(description="Publish the RHPS1 experiment condition.")
    parser.add_argument("condition", type=int, choices=sorted(CONDITIONS))
    parser.add_argument("--session", required=True)
    parser.add_argument("--topic", default="/ana/experiment_condition")
    args = parser.parse_args()

    rclpy.init()
    node = ExperimentConditionPublisher(args.condition, args.session, args.topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
