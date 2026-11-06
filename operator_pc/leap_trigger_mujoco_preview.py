#!/usr/bin/env python3
"""Preview the RHPS1 LEAP Hand trigger mapping without starting mc_rtc.

This tool loads the fixed-base RHPS1 LEAP MuJoCo model and writes only the
selected hand's 16 finger joint positions.  It can be driven with the keyboard
or, when --topic is supplied, by a live vr_interface_msgs/Controller topic.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer


DEFAULT_MODEL = Path(
    "/home/li/luyisi/guillaume_code/rhps1_fixed_mujoco/RHPS1_leap_leap_fixed.xml"
)

FINGER_SUFFIXES = (
    "MCP_1",
    "PIP_1",
    "DIP_1",
    "FINGERTIP_1",
    "MCP_2",
    "PIP_2",
    "DIP_2",
    "FINGERTIP_2",
    "MCP_3",
    "PIP_3",
    "DIP_3",
    "FINGERTIP_3",
    "THUMB_CMC",
    "THUMB_PIP",
    "THUMB_DIP",
    "THUMB_FINGERTIP",
)


class PreviewState:
    def __init__(self, trigger: float) -> None:
        self.trigger = max(0.0, min(1.0, trigger))
        self.last_printed: Optional[float] = None


def trigger_to_opening(trigger: float, deadzone: float = 0.05) -> float:
    """Match ViveHandBridge::analogGripperOpening before its low-pass filter."""
    raw = max(0.0, min(1.0, trigger))
    if raw >= 1.0 - deadzone:
        normalized = 1.0
    elif raw > deadzone:
        normalized = (raw - deadzone) / (1.0 - 2.0 * deadzone)
    else:
        normalized = 0.0
    # These are the current ViveHandBridge openValue_ and closedValue_ defaults.
    return 0.70 + normalized * (0.05 - 0.70)


def set_joint(model: mujoco.MjModel, data: mujoco.MjData, name: str, value: float) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise RuntimeError(f"MuJoCo joint not found: {name}")
    data.qpos[model.jnt_qposadr[joint_id]] = value


def set_presentation_pose(model: mujoco.MjModel, data: mujoco.MjData, side: str) -> None:
    """Move the selected arm away from the torso so the fingers are visible."""
    prefix = "R" if side == "right" else "L"
    set_joint(model, data, f"{prefix}_SHOULDER_P", 0.0)
    set_joint(model, data, f"{prefix}_SHOULDER_R", -1.2 if side == "right" else 1.2)
    set_joint(model, data, f"{prefix}_SHOULDER_Y", 0.0)
    set_joint(model, data, f"{prefix}_ELBOW_P", 0.0)


def apply_hand_opening(
    model: mujoco.MjModel, data: mujoco.MjData, side: str, opening: float
) -> None:
    prefix = "R" if side == "right" else "L"
    for suffix in FINGER_SUFFIXES:
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{prefix}_{suffix}"
        )
        if joint_id < 0:
            raise RuntimeError(f"LEAP joint not found: {prefix}_{suffix}")
        lower, upper = model.jnt_range[joint_id]
        # RHPS1 registers both LEAP grippers with reverse_limits=true.  In
        # mc_control::Gripper, opening=1 therefore means the lower joint limit.
        target = upper + opening * (lower - upper)
        data.qpos[model.jnt_qposadr[joint_id]] = target


def print_state(state: PreviewState, force: bool = False) -> None:
    if not force and state.last_printed is not None and abs(state.trigger - state.last_printed) < 0.01:
        return
    opening = trigger_to_opening(state.trigger)
    print(
        f"trigger={state.trigger:5.2f} ({100.0 * state.trigger:5.1f}%)"
        f"  ->  gripper opening={opening:5.3f} ({100.0 * opening:5.1f}% open)",
        flush=True,
    )
    state.last_printed = state.trigger


def setup_ros_topic(topic: str, state: PreviewState):
    try:
        import rclpy
        from vr_interface_msgs.msg import Controller
    except ImportError as exc:
        raise RuntimeError(
            "ROS topic mode needs the Vive ROS environment. Use the launcher "
            "run_leap_trigger_mujoco_preview instead of calling this file directly."
        ) from exc

    rclpy.init(args=None)
    node = rclpy.create_node("leap_trigger_mujoco_preview")

    def callback(msg: Controller) -> None:
        state.trigger = max(0.0, min(1.0, float(msg.trigger)))

    node.create_subscription(Controller, topic, callback, 10)
    return rclpy, node


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Preview the Cornel Vive trigger-to-LEAP-Hand mapping in MuJoCo."
    )
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--topic", help="Optional live Controller topic, e.g. /controller_1")
    parser.add_argument("--trigger", type=float, default=0.0, help="Initial trigger value in [0, 1]")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.model.is_file():
        print(f"MuJoCo model not found: {args.model}", file=sys.stderr)
        return 2

    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    state = PreviewState(args.trigger)
    set_presentation_pose(model, data, args.side)

    ros = None
    node = None
    if args.topic:
        ros, node = setup_ros_topic(args.topic, state)
        print(f"Live input: {args.topic}")
    else:
        print("Keyboard: 0=released, 5=half, 1=fully pulled, [ and ]=change by 5%, space=toggle")

    def set_trigger(value: float) -> None:
        if args.topic:
            return
        state.trigger = max(0.0, min(1.0, value))
        print_state(state, force=True)

    def key_callback(keycode: int) -> None:
        if keycode == ord("0"):
            set_trigger(0.0)
        elif keycode == ord("5"):
            set_trigger(0.5)
        elif keycode == ord("1"):
            set_trigger(1.0)
        elif keycode == ord("["):
            set_trigger(state.trigger - 0.05)
        elif keycode == ord("]"):
            set_trigger(state.trigger + 0.05)
        elif keycode == ord(" "):
            set_trigger(1.0 if state.trigger < 0.5 else 0.0)

    apply_hand_opening(model, data, args.side, trigger_to_opening(state.trigger))
    mujoco.mj_forward(model, data)
    print_state(state, force=True)

    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
            prefix = "R" if args.side == "right" else "L"
            body_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}_MCP_2_LINK"
            )
            viewer.cam.lookat[:] = data.xpos[body_id]
            viewer.cam.distance = 0.32
            viewer.cam.azimuth = 225.0 if args.side == "right" else 135.0
            viewer.cam.elevation = -10.0

            while viewer.is_running():
                if node is not None:
                    ros.spin_once(node, timeout_sec=0.0)
                apply_hand_opening(model, data, args.side, trigger_to_opening(state.trigger))
                mujoco.mj_forward(model, data)
                print_state(state)
                viewer.sync()
                time.sleep(1.0 / 60.0)
    finally:
        if node is not None:
            node.destroy_node()
            ros.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
