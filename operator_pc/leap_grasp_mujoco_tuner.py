#!/usr/bin/env python3
"""Standalone MuJoCo pose editor for the RHPS1 right arm and LEAP Hand.

This program is deliberately isolated from ROS, mc_rtc, SteamVR and robot
hardware.  It changes only MuJoCo qpos values and stores optional presets in a
user JSON file.  The bottle is a visual reference, not a simulated grasp test.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, MutableMapping, Sequence

import mujoco
import mujoco.viewer
import numpy as np


DEFAULT_MODEL = Path(
    "/home/li/luyisi/guillaume_code/rhps1_fixed_mujoco/RHPS1_leap_leap_fixed.xml"
)
DEFAULT_PRESETS = (
    Path.home()
    / ".local"
    / "share"
    / "rhps1_cornel_vive_controller"
    / "leap_grasp_mujoco_presets.json"
)

ARM_JOINTS = (
    "R_SHOULDER_P",
    "R_SHOULDER_R",
    "R_SHOULDER_Y",
    "R_ELBOW_P",
    "R_ELBOW_Y",
    "R_WRIST_R",
    "R_WRIST_Y",
)

HAND_GROUPS: Mapping[str, Sequence[str]] = {
    "Finger 1": ("R_MCP_1", "R_PIP_1", "R_DIP_1", "R_FINGERTIP_1"),
    "Finger 2": ("R_MCP_2", "R_PIP_2", "R_DIP_2", "R_FINGERTIP_2"),
    "Finger 3": ("R_MCP_3", "R_PIP_3", "R_DIP_3", "R_FINGERTIP_3"),
    "Thumb": (
        "R_THUMB_CMC",
        "R_THUMB_PIP",
        "R_THUMB_DIP",
        "R_THUMB_FINGERTIP",
    ),
}
HAND_JOINTS = tuple(joint for group in HAND_GROUPS.values() for joint in group)

# Same default as controller/etc/ANAAvatarController.in.yaml.  Loading this
# preset is an editor action only and has no connection to the real robot.
RIGHT_ARM_FRONT_DEG = {
    name: value
    for name, value in zip(ARM_JOINTS, (-30.0, -20.0, 5.0, -120.0, -20.0, 0.0, 20.0))
}

DEFAULT_BOTTLE = {
    "x": 0.47,
    "y": -0.145,
    "z": 0.61,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "radius": 0.033,
    "height": 0.20,
}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def rotation_from_rpy_deg(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r, p, y = (math.radians(value) for value in (roll, pitch, yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def joint_id(model: mujoco.MjModel, name: str) -> int:
    result = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if result < 0:
        raise RuntimeError(f"MuJoCo joint not found: {name}")
    return result


def joint_limits_deg(model: mujoco.MjModel, name: str) -> tuple[float, float]:
    index = joint_id(model, name)
    lower, upper = model.jnt_range[index]
    return math.degrees(float(lower)), math.degrees(float(upper))


def validate_model(model: mujoco.MjModel) -> None:
    for name in (*ARM_JOINTS, *HAND_JOINTS):
        index = joint_id(model, name)
        if model.jnt_type[index] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"Expected scalar hinge joint: {name}")
    for name in ("R_HAND_LINK", "R_MCP_2_LINK"):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
            raise RuntimeError(f"MuJoCo body not found: {name}")


class PresetStore:
    """Small JSON store used only by this standalone editor."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.candidates: MutableMapping[str, dict] = {}
        self.quick: MutableMapping[str, dict] = {}
        self.last_pose: MutableMapping[str, object] = {}
        self.load()

    def load(self) -> None:
        if not self.path.is_file():
            return
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(raw, dict):
                raise ValueError("top level must be an object")
            self.candidates = dict(raw.get("candidates", {}))
            self.quick = dict(raw.get("quick_endpoints", {}))
            self.last_pose = dict(raw.get("last_pose", {}))
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            print(f"Warning: ignored invalid preset file {self.path}: {exc}", file=sys.stderr)

    def save(self) -> None:
        payload = {
            "version": 1,
            "note": "Standalone MuJoCo editor values; not automatically sent to RHPS1",
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "candidates": self.candidates,
            "quick_endpoints": self.quick,
            "last_pose": self.last_pose,
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(temporary, self.path)


class TunerApp:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        presets_path: Path,
        show_bottle: bool,
    ) -> None:
        # Imported lazily so --check works on a machine without an X display.
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.model = model
        self.data = data
        self.store = PresetStore(presets_path)
        self.closed = False
        self.viewer = None
        self.camera_request = "hand"
        self.status_var = None

        self.root = tk.Tk()
        self.root.title("RHPS1 right LEAP grasp tuner — MuJoCo only")
        self.root.geometry("920x900")
        self.root.minsize(780, 650)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.arm_vars: Dict[str, object] = {}
        self.hand_vars: Dict[str, object] = {}
        self.quick_vars: Dict[str, object] = {}
        self.quick_summary_vars: Dict[str, object] = {}
        self.candidate_status_vars: Dict[str, object] = {}
        self.bottle_vars: Dict[str, object] = {}
        self.show_bottle_var = tk.BooleanVar(value=show_bottle)
        self.show_frame_var = tk.BooleanVar(value=True)

        self._build_header()
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        arm_tab = ttk.Frame(notebook)
        hand_tab = ttk.Frame(notebook)
        quick_tab = ttk.Frame(notebook)
        scene_tab = ttk.Frame(notebook)
        notebook.add(arm_tab, text="Right arm (7)")
        notebook.add(hand_tab, text="Right LEAP Hand (16)")
        notebook.add(quick_tab, text="Quick / A-B-C")
        notebook.add(scene_tab, text="Bottle / output")

        self._build_arm_tab(arm_tab)
        self._build_hand_tab(hand_tab)
        self._build_quick_tab(quick_tab)
        self._build_scene_tab(scene_tab)

        self.load_front_arm()
        self.reset_hand_zero()
        self._load_quick_from_store()
        self._set_status(
            "Local preview only: sliders update MuJoCo immediately; nothing is sent to ROS or RHPS1."
        )

    def _build_header(self) -> None:
        tk, ttk = self.tk, self.ttk
        frame = ttk.Frame(self.root, padding=8)
        frame.pack(fill="x")
        ttk.Label(
            frame,
            text="Standalone RHPS1 right-arm + 16-joint LEAP Hand editor",
            font=("TkDefaultFont", 13, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            frame,
            text=(
                "No ROS / SteamVR / mc_rtc / hardware connection. "
                "The bottle is visual-only; contact success here does not prove a real grasp."
            ),
            foreground="#8b2500",
            wraplength=880,
        ).pack(anchor="w", pady=(3, 5))
        controls = ttk.Frame(frame)
        controls.pack(fill="x")
        ttk.Button(controls, text="Camera: hand", command=lambda: self._request_camera("hand")).pack(
            side="left"
        )
        ttk.Button(controls, text="Camera: hand + bottle", command=lambda: self._request_camera("pair")).pack(
            side="left", padx=4
        )
        ttk.Button(controls, text="Camera: full robot", command=lambda: self._request_camera("full")).pack(
            side="left"
        )
        ttk.Checkbutton(controls, text="Show hand frame XYZ", variable=self.show_frame_var).pack(
            side="left", padx=(16, 4)
        )
        ttk.Checkbutton(controls, text="Show bottle", variable=self.show_bottle_var).pack(side="left")
        self.status_var = tk.StringVar(value="Starting")
        ttk.Label(frame, textvariable=self.status_var, wraplength=880).pack(anchor="w", pady=(6, 0))

    def _scrollable(self, parent):
        tk, ttk = self.tk, self.ttk
        canvas = tk.Canvas(parent, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        content = ttk.Frame(canvas)
        content.bind(
            "<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        window = canvas.create_window((0, 0), window=content, anchor="nw")
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        return content

    def _joint_slider(self, parent, row: int, name: str, target: Dict[str, object]) -> None:
        tk, ttk = self.tk, self.ttk
        lower, upper = joint_limits_deg(self.model, name)
        variable = tk.DoubleVar(value=0.0)
        value_text = tk.StringVar(value="0.0°")
        target[name] = variable

        def update_text(*_unused) -> None:
            value_text.set(f"{float(variable.get()):7.1f}°")

        variable.trace_add("write", update_text)

        ttk.Label(parent, text=name, width=22).grid(row=row, column=0, sticky="w", padx=6, pady=2)
        scale = tk.Scale(
            parent,
            from_=lower,
            to=upper,
            orient="horizontal",
            resolution=0.1,
            showvalue=False,
            variable=variable,
            length=510,
        )
        scale.grid(row=row, column=1, sticky="ew", padx=4)
        ttk.Label(parent, textvariable=value_text, width=10, anchor="e").grid(
            row=row, column=2, sticky="e", padx=6
        )
        parent.columnconfigure(1, weight=1)

    def _value_slider(
        self,
        parent,
        row: int,
        label: str,
        key: str,
        lower: float,
        upper: float,
        resolution: float,
        suffix: str,
    ) -> None:
        tk, ttk = self.tk, self.ttk
        variable = tk.DoubleVar(value=float(DEFAULT_BOTTLE[key]))
        value_text = tk.StringVar(value=f"{DEFAULT_BOTTLE[key]:.3f}{suffix}")
        self.bottle_vars[key] = variable

        def update_text(*_unused) -> None:
            value_text.set(f"{float(variable.get()):.3f}{suffix}")

        variable.trace_add("write", update_text)
        ttk.Label(parent, text=label, width=20).grid(row=row, column=0, sticky="w", padx=6, pady=2)
        scale = tk.Scale(
            parent,
            from_=lower,
            to=upper,
            orient="horizontal",
            resolution=resolution,
            showvalue=False,
            variable=variable,
            length=510,
        )
        scale.grid(row=row, column=1, sticky="ew", padx=4)
        ttk.Label(parent, textvariable=value_text, width=13, anchor="e").grid(
            row=row, column=2, sticky="e", padx=6
        )
        parent.columnconfigure(1, weight=1)

    def _build_arm_tab(self, parent) -> None:
        ttk = self.ttk
        buttons = ttk.Frame(parent, padding=6)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="Load controller front pose", command=self.load_front_arm).pack(
            side="left"
        )
        ttk.Button(buttons, text="Zero right arm", command=self.reset_arm_zero).pack(
            side="left", padx=4
        )
        ttk.Label(
            buttons,
            text="Sliders are degrees; limits come from the actual MuJoCo model.",
        ).pack(side="left", padx=12)
        content = self._scrollable(parent)
        for row, name in enumerate(ARM_JOINTS):
            self._joint_slider(content, row, name, self.arm_vars)

    def _build_hand_tab(self, parent) -> None:
        ttk = self.ttk
        buttons = ttk.Frame(parent, padding=6)
        buttons.pack(fill="x")
        ttk.Button(buttons, text="All joints = 0°", command=self.reset_hand_zero).pack(side="left")
        ttk.Button(buttons, text="All model midpoints", command=self.hand_midpoints).pack(
            side="left", padx=4
        )
        ttk.Label(
            buttons,
            text="No common open/close direction is assumed; tune each joint from the model view.",
        ).pack(side="left", padx=12)

        content = self._scrollable(parent)
        row = 0
        for group, names in HAND_GROUPS.items():
            ttk.Label(content, text=group, font=("TkDefaultFont", 11, "bold")).grid(
                row=row, column=0, columnspan=3, sticky="w", padx=6, pady=(10, 2)
            )
            row += 1
            for name in names:
                self._joint_slider(content, row, name, self.hand_vars)
                row += 1

    def _build_quick_tab(self, parent) -> None:
        tk, ttk = self.tk, self.ttk
        content = self._scrollable(parent)
        row = 0
        ttk.Label(
            content,
            text=(
                "Each quick slider interpolates only its four joints between endpoints captured from the "
                "current hand pose. Endpoints and A/B/C are stored locally in JSON."
            ),
            wraplength=840,
        ).grid(row=row, column=0, columnspan=5, sticky="w", padx=8, pady=8)
        row += 1
        for group in HAND_GROUPS:
            ttk.Label(content, text=group, font=("TkDefaultFont", 11, "bold")).grid(
                row=row, column=0, sticky="w", padx=8, pady=(10, 2)
            )
            variable = tk.DoubleVar(value=0.0)
            self.quick_vars[group] = variable
            tk.Scale(
                content,
                from_=0.0,
                to=100.0,
                orient="horizontal",
                resolution=1.0,
                variable=variable,
                command=lambda raw, name=group: self.apply_quick(name, float(raw)),
                length=380,
            ).grid(row=row, column=1, sticky="ew", padx=4)
            ttk.Button(
                content, text="Set 0%", command=lambda name=group: self.set_quick_endpoint(name, "low")
            ).grid(row=row, column=2, padx=3)
            ttk.Button(
                content,
                text="Set 100%",
                command=lambda name=group: self.set_quick_endpoint(name, "high"),
            ).grid(row=row, column=3, padx=3)
            row += 1
            summary = tk.StringVar(value="Endpoints: not initialized")
            self.quick_summary_vars[group] = summary
            ttk.Label(content, textvariable=summary, wraplength=790).grid(
                row=row, column=0, columnspan=3, sticky="w", padx=8, pady=(0, 2)
            )
            ttk.Button(
                content,
                text="Demo: 0° → model midpoint",
                command=lambda name=group: self.reset_quick_demo(name),
            ).grid(row=row, column=3, sticky="e", padx=3)
            row += 1

        ttk.Separator(content).grid(row=row, column=0, columnspan=5, sticky="ew", padx=8, pady=14)
        row += 1
        ttk.Label(
            content,
            text="Candidate slots save arm + hand + bottle reference together:",
            font=("TkDefaultFont", 11, "bold"),
        ).grid(row=row, column=0, columnspan=5, sticky="w", padx=8, pady=4)
        row += 1
        for label in ("A", "B", "C"):
            ttk.Label(content, text=f"Candidate {label}", width=18).grid(
                row=row, column=0, sticky="w", padx=8, pady=4
            )
            ttk.Button(
                content, text=f"Save as {label}", command=lambda slot=label: self.save_candidate(slot)
            ).grid(row=row, column=1, sticky="w", padx=4)
            ttk.Button(
                content, text=f"Load {label}", command=lambda slot=label: self.load_candidate(slot)
            ).grid(row=row, column=2, sticky="w", padx=4)
            status = tk.StringVar(value="EMPTY — save before loading")
            self.candidate_status_vars[label] = status
            ttk.Label(content, textvariable=status).grid(row=row, column=3, sticky="w", padx=8)
            row += 1
        self._refresh_candidate_status()
        content.columnconfigure(1, weight=1)

    def _build_scene_tab(self, parent) -> None:
        ttk = self.ttk
        content = self._scrollable(parent)
        ttk.Label(
            content,
            text=(
                "Bottle centre/size and orientation are only a visual reference for relative geometry. "
                "No collision, force, friction or grasp stability is evaluated."
            ),
            foreground="#8b2500",
            wraplength=840,
        ).grid(row=0, column=0, columnspan=3, sticky="w", padx=8, pady=8)
        specs = (
            ("Centre X", "x", -0.20, 1.00, 0.005, " m"),
            ("Centre Y", "y", -0.80, 0.80, 0.005, " m"),
            ("Centre Z", "z", 0.10, 1.50, 0.005, " m"),
            ("Roll", "roll", -180.0, 180.0, 1.0, "°"),
            ("Pitch", "pitch", -180.0, 180.0, 1.0, "°"),
            ("Yaw", "yaw", -180.0, 180.0, 1.0, "°"),
            ("Radius", "radius", 0.015, 0.070, 0.001, " m"),
            ("Height", "height", 0.080, 0.350, 0.005, " m"),
        )
        for row, spec in enumerate(specs, start=1):
            self._value_slider(content, row, *spec)

        controls = ttk.Frame(content)
        controls.grid(row=len(specs) + 1, column=0, columnspan=3, sticky="w", padx=8, pady=14)
        ttk.Button(controls, text="Reset bottle", command=self.reset_bottle).pack(side="left")
        ttk.Button(controls, text="Print current pose", command=self.print_pose).pack(
            side="left", padx=4
        )
        ttk.Button(controls, text="Copy current JSON", command=self.copy_pose_json).pack(side="left")
        ttk.Button(controls, text="Save last pose", command=self.save_last_pose).pack(
            side="left", padx=4
        )
        ttk.Label(
            content,
            text=f"Preset file: {self.store.path}",
            wraplength=840,
        ).grid(row=len(specs) + 2, column=0, columnspan=3, sticky="w", padx=8, pady=4)

    def _set_status(self, text: str) -> None:
        if self.status_var is not None:
            self.status_var.set(text)

    def _request_camera(self, mode: str) -> None:
        self.camera_request = mode

    def current_pose(self) -> dict:
        return {
            "arm_deg": {name: round(float(var.get()), 3) for name, var in self.arm_vars.items()},
            "hand_deg": {name: round(float(var.get()), 3) for name, var in self.hand_vars.items()},
            "bottle": {
                name: round(float(var.get()), 5) for name, var in self.bottle_vars.items()
            },
        }

    def _apply_pose_dict(self, pose: Mapping[str, object]) -> None:
        for section_name, variables in (("arm_deg", self.arm_vars), ("hand_deg", self.hand_vars)):
            values = pose.get(section_name, {})
            if not isinstance(values, dict):
                continue
            for name, raw in values.items():
                if name not in variables:
                    continue
                lower, upper = joint_limits_deg(self.model, name)
                variables[name].set(clamp(float(raw), lower, upper))
        bottle = pose.get("bottle", {})
        if isinstance(bottle, dict):
            for name, raw in bottle.items():
                if name in self.bottle_vars:
                    self.bottle_vars[name].set(float(raw))

    def load_front_arm(self) -> None:
        for name, value in RIGHT_ARM_FRONT_DEG.items():
            if name in self.arm_vars:
                lower, upper = joint_limits_deg(self.model, name)
                self.arm_vars[name].set(clamp(value, lower, upper))
        self.camera_request = "hand"
        self._set_status("Loaded the controller's default right-arm front pose into local MuJoCo.")

    def reset_arm_zero(self) -> None:
        for name, variable in self.arm_vars.items():
            lower, upper = joint_limits_deg(self.model, name)
            variable.set(clamp(0.0, lower, upper))
        self._set_status("Right-arm joints reset to 0° in local MuJoCo.")

    def reset_hand_zero(self) -> None:
        for name, variable in self.hand_vars.items():
            lower, upper = joint_limits_deg(self.model, name)
            variable.set(clamp(0.0, lower, upper))
        self._set_status("Right-hand joints reset to 0° in local MuJoCo.")

    def hand_midpoints(self) -> None:
        for name, variable in self.hand_vars.items():
            lower, upper = joint_limits_deg(self.model, name)
            variable.set(0.5 * (lower + upper))
        self._set_status("Right-hand joints set to their model-range midpoints.")

    def reset_bottle(self) -> None:
        for name, value in DEFAULT_BOTTLE.items():
            self.bottle_vars[name].set(value)
        self.camera_request = "pair"
        self._set_status("Bottle visual reference reset.")

    def _quick_entry(self, group: str) -> dict:
        names = HAND_GROUPS[group]
        fallback = {name: float(self.hand_vars[name].get()) for name in names}
        entry = self.store.quick.setdefault(group, {"low_deg": fallback, "high_deg": fallback})
        if not isinstance(entry.get("low_deg"), dict):
            entry["low_deg"] = dict(fallback)
        if not isinstance(entry.get("high_deg"), dict):
            entry["high_deg"] = dict(fallback)
        return entry

    def _load_quick_from_store(self) -> None:
        for group in HAND_GROUPS:
            self._quick_entry(group)
            self._refresh_quick_summary(group)

    def _refresh_quick_summary(self, group: str) -> None:
        target = self.quick_summary_vars.get(group)
        if target is None:
            return
        entry = self._quick_entry(group)
        names = HAND_GROUPS[group]
        low = ", ".join(f"{float(entry['low_deg'].get(name, 0.0)):.1f}" for name in names)
        high = ", ".join(f"{float(entry['high_deg'].get(name, 0.0)):.1f}" for name in names)
        target.set(f"0% [{low}]°   →   100% [{high}]°")

    def reset_quick_demo(self, group: str) -> None:
        """Create visibly different MuJoCo-only endpoints without claiming a grasp pose."""
        entry = self._quick_entry(group)
        low = {}
        high = {}
        for name in HAND_GROUPS[group]:
            lower, upper = joint_limits_deg(self.model, name)
            low[name] = clamp(0.0, lower, upper)
            high[name] = 0.5 * (lower + upper)
        entry["low_deg"] = low
        entry["high_deg"] = high
        self.quick_vars[group].set(0.0)
        for name, value in low.items():
            self.hand_vars[name].set(value)
        self._refresh_quick_summary(group)
        self.store.save()
        self._set_status(
            f"Initialized {group} demo endpoints. These are model-range examples, not a validated grasp."
        )

    def set_quick_endpoint(self, group: str, endpoint: str) -> None:
        key = "low_deg" if endpoint == "low" else "high_deg"
        entry = self._quick_entry(group)
        entry[key] = {name: round(float(self.hand_vars[name].get()), 3) for name in HAND_GROUPS[group]}
        self._refresh_quick_summary(group)
        self.store.save()
        self._set_status(f"Saved {group} {endpoint} endpoint locally; no robot connection exists.")

    def apply_quick(self, group: str, percent: float) -> None:
        entry = self._quick_entry(group)
        ratio = clamp(percent, 0.0, 100.0) / 100.0
        for name in HAND_GROUPS[group]:
            current = float(self.hand_vars[name].get())
            low = float(entry["low_deg"].get(name, current))
            high = float(entry["high_deg"].get(name, current))
            lower_limit, upper_limit = joint_limits_deg(self.model, name)
            self.hand_vars[name].set(
                clamp((1.0 - ratio) * low + ratio * high, lower_limit, upper_limit)
            )

    def save_candidate(self, label: str) -> None:
        pose = self.current_pose()
        pose["saved_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        self.store.candidates[label] = pose
        self.store.last_pose = self.current_pose()
        self.store.save()
        self._refresh_candidate_status()
        self._set_status(f"Saved candidate {label} to {self.store.path}")

    def load_candidate(self, label: str) -> None:
        pose = self.store.candidates.get(label)
        if not isinstance(pose, dict):
            self._set_status(f"Candidate {label} is empty.")
            return
        self._apply_pose_dict(pose)
        self.camera_request = "pair"
        self._set_status(f"Loaded candidate {label} into local MuJoCo only.")

    def _refresh_candidate_status(self) -> None:
        for label, target in self.candidate_status_vars.items():
            pose = self.store.candidates.get(label)
            if not isinstance(pose, dict):
                target.set("EMPTY — save before loading")
                continue
            saved_at = pose.get("saved_at", "saved")
            target.set(str(saved_at))

    def save_last_pose(self) -> None:
        self.store.last_pose = self.current_pose()
        self.store.save()
        self._set_status(f"Saved current pose to {self.store.path}")

    def print_pose(self) -> None:
        payload = json.dumps(self.current_pose(), indent=2, sort_keys=True)
        print("\nRHPS1 LEAP MuJoCo pose:\n" + payload, flush=True)
        self._set_status("Printed current pose to the launch terminal.")

    def copy_pose_json(self) -> None:
        payload = json.dumps(self.current_pose(), indent=2, sort_keys=True)
        self.root.clipboard_clear()
        self.root.clipboard_append(payload)
        self.root.update_idletasks()
        self._set_status("Copied current pose JSON to the clipboard.")

    def apply_qpos(self) -> None:
        for name, variable in (*self.arm_vars.items(), *self.hand_vars.items()):
            index = joint_id(self.model, name)
            self.data.qpos[self.model.jnt_qposadr[index]] = math.radians(float(variable.get()))

    def bottle(self) -> dict:
        return {name: float(variable.get()) for name, variable in self.bottle_vars.items()}

    def pump(self) -> bool:
        if self.closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except self.tk.TclError:
            self.closed = True
        return not self.closed

    def close(self) -> None:
        if self.closed:
            return
        try:
            self.store.last_pose = self.current_pose()
            self.store.save()
        except (OSError, ValueError) as exc:
            print(f"Warning: could not save final tuner pose: {exc}", file=sys.stderr)
        self.closed = True
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass


def add_user_geom(
    scene: mujoco.MjvScene,
    geom_type: mujoco.mjtGeom,
    size: Iterable[float],
    pos: Iterable[float],
    mat: np.ndarray,
    rgba: Iterable[float],
) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        geom_type,
        np.asarray(tuple(size), dtype=float),
        np.asarray(tuple(pos), dtype=float),
        np.asarray(mat, dtype=float).reshape(9),
        np.asarray(tuple(rgba), dtype=np.float32),
    )
    scene.ngeom += 1


def add_connector(
    scene: mujoco.MjvScene,
    geom_type: mujoco.mjtGeom,
    width: float,
    start: np.ndarray,
    end: np.ndarray,
    rgba: Iterable[float],
) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        geom_type,
        np.zeros(3),
        np.zeros(3),
        np.eye(3).reshape(9),
        np.asarray(tuple(rgba), dtype=np.float32),
    )
    mujoco.mjv_connector(geom, geom_type, width, start, end)
    scene.ngeom += 1


def update_user_scene(viewer, app: TunerApp) -> None:
    scene = viewer.user_scn
    scene.ngeom = 0

    if bool(app.show_bottle_var.get()):
        bottle = app.bottle()
        centre = np.array((bottle["x"], bottle["y"], bottle["z"]), dtype=float)
        rotation = rotation_from_rpy_deg(bottle["roll"], bottle["pitch"], bottle["yaw"])
        radius = bottle["radius"]
        height = bottle["height"]
        add_user_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            (radius, 0.5 * height, 0.0),
            centre,
            rotation,
            (0.10, 0.45, 0.85, 0.55),
        )
        neck_height = min(0.045, 0.25 * height)
        neck_centre = centre + rotation @ np.array((0.0, 0.0, 0.5 * (height + neck_height)))
        add_user_geom(
            scene,
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            (0.48 * radius, 0.5 * neck_height, 0.0),
            neck_centre,
            rotation,
            (0.08, 0.36, 0.72, 0.70),
        )

    if bool(app.show_frame_var.get()):
        body_id = mujoco.mj_name2id(app.model, mujoco.mjtObj.mjOBJ_BODY, "R_HAND_LINK")
        origin = np.asarray(app.data.xpos[body_id], dtype=float)
        rotation = np.asarray(app.data.xmat[body_id], dtype=float).reshape(3, 3)
        colors = ((0.95, 0.10, 0.10, 1.0), (0.10, 0.90, 0.10, 1.0), (0.10, 0.35, 1.0, 1.0))
        for axis, color in enumerate(colors):
            add_connector(
                scene,
                mujoco.mjtGeom.mjGEOM_CAPSULE,
                0.004,
                origin,
                origin + 0.10 * rotation[:, axis],
                color,
            )


def apply_camera_request(viewer, app: TunerApp) -> None:
    mode = app.camera_request
    if not mode:
        return
    app.camera_request = ""
    if mode == "full":
        viewer.cam.lookat[:] = (0.0, 0.0, 0.50)
        viewer.cam.distance = 2.1
        viewer.cam.azimuth = 145.0
        viewer.cam.elevation = -8.0
        return

    hand_id = mujoco.mj_name2id(app.model, mujoco.mjtObj.mjOBJ_BODY, "R_MCP_2_LINK")
    hand = np.asarray(app.data.xpos[hand_id], dtype=float)
    if mode == "pair" and bool(app.show_bottle_var.get()):
        bottle = app.bottle()
        target = 0.5 * (hand + np.array((bottle["x"], bottle["y"], bottle["z"])))
        viewer.cam.lookat[:] = target
        viewer.cam.distance = 0.65
    else:
        viewer.cam.lookat[:] = hand
        viewer.cam.distance = 0.46
    viewer.cam.azimuth = 225.0
    viewer.cam.elevation = -12.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone MuJoCo tuner for the RHPS1 right arm and 16-joint LEAP Hand."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="RHPS1 LEAP MuJoCo XML")
    parser.add_argument(
        "--presets", type=Path, default=DEFAULT_PRESETS, help="local JSON file for A/B/C and endpoints"
    )
    parser.add_argument("--no-bottle", action="store_true", help="start with the visual bottle hidden")
    parser.add_argument(
        "--check", action="store_true", help="validate dependencies/model/joints without opening windows"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.model.is_file():
        print(f"MuJoCo model not found: {args.model}", file=sys.stderr)
        return 2
    try:
        model = mujoco.MjModel.from_xml_path(str(args.model))
        validate_model(model)
    except Exception as exc:
        print(f"Could not load a compatible RHPS1 LEAP model: {exc}", file=sys.stderr)
        return 2

    if args.check:
        print(f"OK model: {args.model}")
        print(f"OK right arm joints: {len(ARM_JOINTS)}")
        print(f"OK right LEAP joints: {len(HAND_JOINTS)}")
        print(f"Preset path: {args.presets}")
        return 0

    data = mujoco.MjData(model)
    try:
        app = TunerApp(model, data, args.presets, show_bottle=not args.no_bottle)
    except Exception as exc:
        print(f"Could not open the tuner control window: {exc}", file=sys.stderr)
        return 3

    try:
        app.apply_qpos()
        mujoco.mj_forward(model, data)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            app.viewer = viewer
            while viewer.is_running() and app.pump():
                app.apply_qpos()
                mujoco.mj_forward(model, data)
                update_user_scene(viewer, app)
                apply_camera_request(viewer, app)
                viewer.sync()
                time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
