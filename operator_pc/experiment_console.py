#!/usr/bin/env python3
"""RHPS1 experiment condition console (Operator PC).

Runs next to the VIVE talker (run_vive_talker_rhps1) and switches the experiment
condition of the running *_experiment controller without restarting anything.

  1-4  send a condition (each switch gets a new session ID; the robot holds still
       until the operator presses menu again)
  s    show the current status
  q    quit

The controller acknowledges every applied condition on /ana/experiment_condition_ack.
Every command and acknowledgement is appended to a CSV session log.
"""
import argparse
import csv
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

CONDITIONS = {
    1: "无延迟 / 无辅助",
    2: "无延迟 / 有辅助",
    3: "1.5 s 延迟 / 无辅助",
    4: "1.5 s 延迟 / 有辅助",
}
ACK_TIMEOUT = 3.0


class ExperimentConsole(Node):
    def __init__(self, topic: str, ack_topic: str, log_path: str) -> None:
        super().__init__("ana_experiment_console")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, topic, qos)
        self.create_subscription(String, ack_topic, self.on_ack, qos)
        self.lock = threading.Lock()
        self.message = None          # last command, re-published so a restarted controller gets it
        self.sent_at = None
        self.pending = None          # (condition, session) waiting for an acknowledgement
        self.acked = None            # (condition, session, delay, assistance)
        self.log_path = log_path
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        new_file = not os.path.exists(log_path)
        self.log_file = open(log_path, "a", newline="")
        self.csv = csv.writer(self.log_file)
        if new_file:
            self.csv.writerow(["wall_time", "event", "condition", "session", "delay_s", "assistance"])
        self.create_timer(0.5, self.tick)

    def record(self, event, condition="", session="", delay="", assistance="") -> None:
        self.csv.writerow([datetime.now().isoformat(timespec="milliseconds"), event, condition, session, delay,
                           assistance])
        self.log_file.flush()

    def send(self, condition: int) -> None:
        session = f"{datetime.now().strftime('%Y%m%dT%H%M%S')}-c{condition}"
        with self.lock:
            self.message = String(data=f"{condition}:{session}")
            self.pending = (condition, session)
            self.sent_at = time.monotonic()
        self.pub.publish(self.message)
        self.record("sent", condition, session)
        print(f"→ 已发送条件 {condition}（{CONDITIONS[condition]}），session {session}。等待 controller 确认…",
              flush=True)

    def on_ack(self, msg: String) -> None:
        try:
            condition, session, delay, assistance = msg.data.split(";")
            ack = (int(condition), session, float(delay), assistance == "1")
        except ValueError:
            print(f"! 无法解析确认消息: {msg.data}", flush=True)
            return
        with self.lock:
            matched = self.pending is not None and self.pending[1] == ack[1]
            if matched:
                self.pending = None
            new = ack != self.acked
            self.acked = ack
        if new:
            self.record("acknowledged", ack[0], ack[1], f"{ack[2]:.2f}", int(ack[3]))
            print(f"✓ controller 已切换：条件 {ack[0]}（{CONDITIONS.get(ack[0], '?')}），延迟 {ack[2]:.1f} s，"
                  f"辅助 {'开' if ack[3] else '关'}。机器人保持不动，按 menu 开始。", flush=True)
            if not matched and self.pending is None and self.message is None:
                print("  （这是 controller 之前已在运行的条件）", flush=True)

    def tick(self) -> None:
        with self.lock:
            message = self.message
            pending = self.pending
            sent_at = self.sent_at
        if message is not None:
            self.pub.publish(message)
        if pending is not None and sent_at is not None and time.monotonic() - sent_at > ACK_TIMEOUT:
            print(f"! {ACK_TIMEOUT:.0f} s 内未收到 controller 确认（条件 {pending[0]}）。检查：controller 是否已启动、"
                  "是否为 *_experiment 变体、ROS domain/Zenoh 是否连通。消息会继续重发。", flush=True)
            self.record("ack_timeout", pending[0], pending[1])
            with self.lock:
                self.sent_at = time.monotonic()

    def status(self) -> None:
        with self.lock:
            acked, pending = self.acked, self.pending
        if acked:
            print(f"  当前（controller 确认）：条件 {acked[0]}（{CONDITIONS.get(acked[0], '?')}），session {acked[1]}",
                  flush=True)
        else:
            print("  controller 尚未确认任何条件", flush=True)
        if pending:
            print(f"  等待确认：条件 {pending[0]}，session {pending[1]}", flush=True)
        print(f"  记录文件：{self.log_path}", flush=True)


def main() -> None:
    default_log = os.path.expanduser("~/.local/share/rhps1_cornel_vive_controller/experiment_sessions.csv")
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/ana/experiment_condition")
    parser.add_argument("--ack-topic", default="/ana/experiment_condition_ack")
    parser.add_argument("--log", default=default_log)
    parser.add_argument("--initial", type=int, choices=sorted(CONDITIONS),
                        help="send this condition immediately at start")
    args = parser.parse_args()

    rclpy.init()
    node = ExperimentConsole(args.topic, args.ack_topic, args.log)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    print("RHPS1 实验条件控制台")
    for key, name in CONDITIONS.items():
        print(f"  {key} = {name}")
    print("  s = 状态    q = 退出\n切换条件后机器人会保持不动，操作者按 menu 后才继续跟随。", flush=True)
    if args.initial:
        node.send(args.initial)
    try:
        while True:
            try:
                line = input("> ").strip().lower()
            except EOFError:
                break
            if line in ("q", "quit", "exit"):
                break
            if line == "s":
                node.status()
            elif line.isdigit() and int(line) in CONDITIONS:
                node.send(int(line))
            elif line:
                print("  请输入 1-4、s 或 q", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.record("console_exit")
        node.log_file.close()
        # Stop the spinning thread before tearing down the node and the context.
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
