#!/usr/bin/env python3
"""Physics lab for comparing LEAP Hand closure strategies on a free bottle.

Purely local MuJoCo experiment: no ROS, mc_rtc, SteamVR or hardware.

The right wrist and LEAP Hand are copied out of the RHPS1 fixed-base model.
The wrist rides on three stiff PD-driven slide joints, which stand in for a
stiff, position-controlled arm (orientation fixed during a trial).  The 16 finger joints are dynamic and driven by a
PD loop with a per-joint torque limit, a rough stand-in for current-limited
Dynamixel servos.  A free cylinder ("bottle") stands on a table.

Each trial: approach -> close with a strategy -> hold -> lift -> hold.
Reported metrics:
  push_cm   largest horizontal bottle displacement before the lift
  tilt_deg  largest bottle tilt before the lift
  lifted    bottle followed the hand upwards at the end of the trial
  first     the hand link that touched the bottle first

Joint semantics (measured with forward kinematics, the names are misleading):
  MCP_k        finger k root flexion
  PIP_k        finger k sideways abduction (+-60 deg)
  DIP_k        finger k middle flexion
  FINGERTIP_k  finger k distal flexion
  THUMB_CMC    thumb swing across the palm (opposition)
Hand frame (= R_WRIST_Y_LINK): +x palm normal, -z finger direction,
-y thumb side.  All joint values use the controller URDF limits.

The model is a qualitative tool: friction, compliance and servo behaviour are
approximations.  Use it to rank strategies and to find failure mechanisms,
not to predict real success rates.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import mujoco
import numpy as np

DEFAULT_ROBOT_XML = Path(
    "/home/li/luyisi/guillaume_code/rhps1_fixed_mujoco/RHPS1_leap_leap_fixed.xml"
)

FINGERS = (1, 2, 3)
FINGER_JOINTS = tuple(
    f"{kind}_{k}" for k in FINGERS for kind in ("MCP", "PIP", "DIP", "FINGERTIP")
)
THUMB_JOINTS = ("THUMB_CMC", "THUMB_PIP", "THUMB_DIP", "THUMB_FINGERTIP")
HAND_JOINTS = FINGER_JOINTS + THUMB_JOINTS

# Limits from RHPS1main_leap_leap.urdf (what the real controller uses).
URDF_LIMITS: Dict[str, Tuple[float, float]] = {
    "MCP": (-0.314, 2.230),
    "PIP": (-1.047, 1.047),
    "DIP": (-0.506, 1.885),
    "FINGERTIP": (-0.366, 2.042),
    "THUMB_CMC": (-0.349, 2.094),
    "THUMB_PIP": (-0.470, 2.443),
    "THUMB_DIP": (-1.200, 1.900),
    "THUMB_FINGERTIP": (-1.340, 1.880),
}


def joint_kind(name: str) -> str:
    return name if name.startswith("THUMB") else name.rsplit("_", 1)[0]


def limits(name: str) -> Tuple[float, float]:
    return URDF_LIMITS[joint_kind(name)]


def trigger_pose(closure: float, abduction: Optional[float] = None) -> Dict[str, float]:
    """The real controller's rear-trigger target (reverse_limits: lower=open)."""
    pose = {}
    for name in HAND_JOINTS:
        lo, hi = limits(name)
        pose[name] = lo + closure * (hi - lo)
        if abduction is not None and joint_kind(name) == "PIP":
            pose[name] = abduction
    return pose


# --------------------------------------------------------------------------
# Strategies
# --------------------------------------------------------------------------


@dataclass
class Stage:
    """Move the listed joints to their targets; others keep their target."""

    label: str
    targets: Dict[str, float]
    duration: float
    torque_limit: Optional[float] = None  # overrides the default during the stage


@dataclass
class Strategy:
    name: str
    description: str
    preshape: Dict[str, float]  # held during the approach
    stages: List[Stage] = field(default_factory=list)


def deg(values: Dict[str, float]) -> Dict[str, float]:
    return {name: math.radians(value) for name, value in values.items()}


def fingers(mcp: float, abd: float, dip: float, tip: float) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for k in FINGERS:
        out.update({f"MCP_{k}": mcp, f"PIP_{k}": abd, f"DIP_{k}": dip, f"FINGERTIP_{k}": tip})
    return deg(out)


def thumb(cmc: float, pip: float, dip: float, tip: float) -> Dict[str, float]:
    return deg({"THUMB_CMC": cmc, "THUMB_PIP": pip, "THUMB_DIP": dip, "THUMB_FINGERTIP": tip})


def build_strategies(close_time: float) -> Dict[str, Strategy]:
    steps = 6
    trig = [
        Stage(f"trigger {100 * c:.0f}%", trigger_pose(c), close_time / steps)
        for c in np.linspace(1.0 / steps, 1.0, steps)
    ]
    trig_noabd = [
        Stage(f"trigger(no abd) {100 * c:.0f}%", trigger_pose(c, abduction=0.0), close_time / steps)
        for c in np.linspace(1.0 / steps, 1.0, steps)
    ]

    # Designed A/B/C/D.  Values were chosen with the FK/contact analysis in
    # this file (see --explain) for a ~6.6 cm cylinder against the palm.
    a_pre = {**fingers(15, 0, 10, 5), **thumb(25, 30, -20, 0)}
    b_thumb = thumb(85, 60, 10, 15)
    c_wrap = fingers(75, 0, 30, 10)
    d_lock = {**fingers(95, 0, 60, 35), **thumb(95, 60, 25, 30)}

    return {
        "trigger": Strategy(
            "trigger",
            "Baseline: real rear-trigger path, all 16 joints lower->upper together",
            trigger_pose(0.0),
            trig,
        ),
        "trigger_noabd": Strategy(
            "trigger_noabd",
            "Diagnostic: trigger path with the sideways (PIP_k) joints held at 0",
            trigger_pose(0.0, abduction=0.0),
            trig_noabd,
        ),
        "abcd": Strategy(
            "abcd",
            "A pre-shape C -> B thumb barrier -> C fingers wrap from MCP -> D lock",
            a_pre,
            [
                Stage("B thumb barrier", b_thumb, 0.8),
                Stage("C fingers wrap (MCP first)", c_wrap, 1.0),
                Stage("D lock (distal + thumb)", d_lock, 0.8),
            ],
        ),
    }


# --------------------------------------------------------------------------
# Model construction
# --------------------------------------------------------------------------


@dataclass
class Scene:
    bottle_radius: float = 0.033
    bottle_height: float = 0.20
    bottle_mass: float = 0.35
    bottle_friction: float = 0.7
    finger_friction: float = 1.0
    table_friction: float = 0.4
    armature: float = 0.003
    damping: float = 0.03


def build_xml(robot_xml: Path, scene: Scene) -> str:
    src = ET.parse(robot_xml).getroot()
    root = ET.Element("mujoco", model="leap_grasp_physics_lab")
    for tag in ("compiler", "visual", "asset"):
        for element in src.findall(tag):
            root.append(copy.deepcopy(element))
    ET.SubElement(
        root,
        "option",
        timestep="0.001",
        cone="elliptic",
        impratio="10",
        solver="Newton",
        iterations="100",
        tolerance="1e-10",
    )
    default = copy.deepcopy(src.find("default"))
    default_joint = default.find("joint")
    default_joint.set("armature", str(scene.armature))
    default_joint.set("damping", str(scene.damping))
    collision = default.find("default[@class='collision']/geom")
    collision.set("condim", "4")
    collision.set("friction", f"{scene.finger_friction} 0.01 0.0001")
    root.append(default)

    wrist = None
    for body in src.iter("body"):
        if body.get("name") == "R_WRIST_Y_LINK":
            wrist = copy.deepcopy(body)
            break
    if wrist is None:
        raise RuntimeError("R_WRIST_Y_LINK not found in robot model")
    for joint in wrist.findall("joint"):
        wrist.remove(joint)
    for child in list(wrist.findall("body")):
        if child.get("name") in ("R_HAND_LINK", "r-one-finger-link"):
            wrist.remove(child)
    # Hand convex hulls overlap each other in ordinary poses; keep only
    # hand<->environment contacts (bit 2 hand, bit 1 environment).
    for geom in wrist.iter("geom"):
        if geom.get("class") == "collision":
            geom.set("contype", "2")
            geom.set("conaffinity", "1")
    for joint in wrist.iter("joint"):
        name = joint.get("name", "")
        if name == "R_THUMB_PIP":
            lo, hi = URDF_LIMITS["THUMB_PIP"]
            joint.set("range", f"{lo} {hi}")

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0.3 -0.6 1.2", dir="-0.3 0.6 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "light", pos="-0.4 0.4 1.0", dir="0.4 -0.4 -1", diffuse="0.4 0.4 0.4")
    ET.SubElement(
        world,
        "geom",
        name="table",
        type="box",
        size="0.4 0.4 0.02",
        pos="0 0 -0.02",
        rgba="0.55 0.45 0.35 1",
        friction=f"{scene.table_friction} 0.005 0.0001",
        condim="3",
        contype="1",
        conaffinity="3",
    )
    r, h = scene.bottle_radius, scene.bottle_height
    bottle = ET.SubElement(world, "body", name="bottle", pos=f"0 0 {h / 2 + 0.0005}")
    ET.SubElement(bottle, "joint", name="bottle_free", type="free", armature="0", damping="0")
    ET.SubElement(
        bottle,
        "geom",
        name="bottle_geom",
        type="cylinder",
        size=f"{r} {h / 2}",
        mass=str(scene.bottle_mass),
        rgba="0.15 0.5 0.9 0.75",
        friction=f"{scene.bottle_friction} 0.01 0.0001",
        condim="4",
        contype="1",
        conaffinity="3",
    )
    ET.SubElement(
        bottle,
        "geom",
        type="cylinder",
        size=f"{0.45 * r} 0.02",
        pos=f"0 0 {h / 2 + 0.02}",
        mass="0.005",
        rgba="0.1 0.35 0.7 0.8",
        contype="0",
        conaffinity="0",
    )
    # Three stiff slide joints carry the wrist so that it has a real velocity
    # (a mocap body has zero velocity in the solver and cannot drag an object
    # by friction).  The wrist orientation is set per trial via body_quat.
    slider = ET.SubElement(world, "body", name="wrist_slider")
    ET.SubElement(slider, "inertial", pos="0 0 0", mass="0.2", diaginertia="1e-4 1e-4 1e-4")
    for axis, label in (("1 0 0", "x"), ("0 1 0", "y"), ("0 0 1", "z")):
        ET.SubElement(
            slider, "joint", name=f"wrist_{label}", type="slide", axis=axis,
            limited="false", armature="0", damping="0",
        )
    slider.append(wrist)

    body_names = {b.get("name") for b in root.iter("body")}
    contact = ET.SubElement(root, "contact")
    for exclude in src.iter("exclude"):
        if exclude.get("body1") in body_names and exclude.get("body2") in body_names:
            contact.append(copy.deepcopy(exclude))
    return ET.tostring(root, encoding="unicode")


# --------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------


@dataclass
class TrialConfig:
    strategy: str = "abcd"
    grasp_x: float = 0.087  # bottle axis in hand frame (palm normal)
    grasp_y: float = 0.0  # along the bottle axis (finger row)
    grasp_z: float = -0.195  # along the finger direction
    grasp_height: float = 0.09  # table -> grasp point
    offset_x: float = 0.0  # bottle placement error in hand frame [m]
    offset_z: float = 0.0
    approach_dist: float = 0.12
    approach_speed: float = 0.06
    close_during_approach: bool = False
    roll_deg: float = 0.0  # rotation about the approach axis; 0 = thumb up
    kp: float = 3.0
    kd: float = 0.05
    torque_limit: float = 0.35
    wrist_kp: float = 20000.0  # N/m, stiff position-controlled arm
    wrist_kd: float = 400.0
    joint_speed: float = 2.0  # rad/s, target slew limit
    close_time: float = 1.5
    hold_time: float = 0.6
    lift_height: float = 0.08
    lift_speed: float = 0.05


def hand_rotation(roll_deg: float) -> np.ndarray:
    # columns: hand x (palm normal) -> world +x, hand y -> world -z (thumb up),
    # hand z -> world +y.  Then roll about the approach axis.
    base = np.column_stack(((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)))
    a = math.radians(roll_deg)
    roll = np.array(((1, 0, 0), (0, math.cos(a), -math.sin(a)), (0, math.sin(a), math.cos(a))))
    return base @ roll


def mat_to_quat(matrix: np.ndarray) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix, dtype=float).reshape(9))
    return quat


class Lab:
    def __init__(self, robot_xml: Path, scene: Scene) -> None:
        self.scene = scene
        self.model = mujoco.MjModel.from_xml_string(build_xml(robot_xml, scene))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.wrist_body = m.body("R_WRIST_Y_LINK").id
        self.slide_q = np.array([m.jnt_qposadr[m.joint(f"wrist_{a}").id] for a in "xyz"])
        self.slide_v = np.array([m.jnt_dofadr[m.joint(f"wrist_{a}").id] for a in "xyz"])
        self.bottle_body = m.body("bottle").id
        self.bottle_qadr = m.jnt_qposadr[m.joint("bottle_free").id]
        self.joint_q = np.array([m.jnt_qposadr[m.joint("R_" + n).id] for n in HAND_JOINTS])
        self.joint_v = np.array([m.jnt_dofadr[m.joint("R_" + n).id] for n in HAND_JOINTS])
        self.lo = np.array([limits(n)[0] for n in HAND_JOINTS])
        self.hi = np.array([limits(n)[1] for n in HAND_JOINTS])
        self.hand_bodies = {
            i: m.body(i).name
            for i in range(m.nbody)
            if m.body(i).name.startswith("R_")
        }
        self.bottle_geom = m.geom("bottle_geom").id

    # -- helpers ----------------------------------------------------------
    def vec(self, pose: Dict[str, float], base: Optional[np.ndarray] = None) -> np.ndarray:
        out = np.zeros(len(HAND_JOINTS)) if base is None else base.copy()
        for i, name in enumerate(HAND_JOINTS):
            if name in pose:
                out[i] = pose[name]
        return np.clip(out, self.lo, self.hi)

    def bottle_pos(self) -> np.ndarray:
        return self.data.xpos[self.bottle_body].copy()

    def bottle_tilt_deg(self) -> float:
        z_axis = self.data.xmat[self.bottle_body].reshape(3, 3)[:, 2]
        return math.degrees(math.acos(float(np.clip(z_axis[2], -1.0, 1.0))))

    def touching(self) -> List[str]:
        names = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            pair = (c.geom1, c.geom2)
            if self.bottle_geom not in pair:
                continue
            other = c.geom2 if c.geom1 == self.bottle_geom else c.geom1
            body = self.model.geom_bodyid[other]
            if body in self.hand_bodies:
                names.append(self.hand_bodies[body])
        return names

    # -- trial ------------------------------------------------------------
    def run(self, cfg: TrialConfig, strategy: Strategy, viewer=None, realtime: bool = False) -> dict:
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        rotation = hand_rotation(cfg.roll_deg)
        grasp_hand = np.array((cfg.grasp_x, cfg.grasp_y, cfg.grasp_z))
        # Bottle placement error is expressed in the hand frame (x: towards
        # palm normal, z: along fingers); the hand pose stays nominal.
        offset_world = rotation @ np.array((cfg.offset_x, 0.0, cfg.offset_z))
        offset_world[2] = 0.0
        d.qpos[self.bottle_qadr : self.bottle_qadr + 3] = (
            offset_world[0],
            offset_world[1],
            self.scene.bottle_height / 2 + 0.0005,
        )
        final = np.array((0.0, 0.0, cfg.grasp_height)) - rotation @ grasp_hand
        start = final - cfg.approach_dist * rotation[:, 0]
        quat = mat_to_quat(rotation)
        m.body_quat[self.wrist_body] = quat
        m.body_pos[self.wrist_body] = 0.0
        d.qpos[self.slide_q] = start
        wrist_target = start.copy()

        target = self.vec(strategy.preshape)
        d.qpos[self.joint_q] = target
        mujoco.mj_forward(m, d)
        reference = target.copy()
        bottle0 = self.bottle_pos()

        # Build a timeline of (phase, duration, wrist motion, joint target, torque limit)
        timeline: List[tuple] = []
        approach_time = cfg.approach_dist / cfg.approach_speed
        stage_targets = []
        acc = target.copy()
        for stage in strategy.stages:
            acc = self.vec(stage.targets, acc)
            stage_targets.append((stage, acc.copy()))
        if cfg.close_during_approach:
            # start closing for the last 40% of the approach
            timeline.append(("approach", approach_time * 0.6, "approach", target, None, stage_targets and None))
            remaining = approach_time * 0.4
            for stage, tgt in stage_targets:
                timeline.append((stage.label, stage.duration, "approach", tgt, stage.torque_limit, None))
            timeline.append(("finish approach", max(0.0, remaining), "approach", stage_targets[-1][1], None, None))
        else:
            timeline.append(("approach", approach_time, "approach", target, None, None))
            for stage, tgt in stage_targets:
                timeline.append((stage.label, stage.duration, "hold", tgt, stage.torque_limit, None))
        final_target = stage_targets[-1][1] if stage_targets else target
        timeline.append(("settle", cfg.hold_time, "hold", final_target, None, None))
        timeline.append(("lift", cfg.lift_height / cfg.lift_speed, "lift", final_target, None, None))
        timeline.append(("hold lifted", 1.0, "lifted", final_target, None, None))

        dt = m.opt.timestep
        t = 0.0
        approach_elapsed = 0.0
        lift_elapsed = 0.0
        first_contact = None
        first_contact_phase = None
        push = 0.0
        tilt = 0.0
        events = []
        render_every = max(1, int(round(1.0 / 60.0 / dt)))
        step = 0
        wall0 = time.monotonic()
        for phase, duration, motion, tgt, tau_override, _ in timeline:
            tau_limit = cfg.torque_limit if tau_override is None else tau_override
            n = max(1, int(round(duration / dt)))
            events.append((round(t, 3), phase))
            for _ in range(n):
                if motion == "approach":
                    approach_elapsed = min(approach_time, approach_elapsed + dt)
                    s = approach_elapsed / approach_time
                    wrist_target = start + (final - start) * s
                elif motion == "lift":
                    lift_elapsed = min(cfg.lift_height / cfg.lift_speed, lift_elapsed + dt)
                    wrist_target = final + np.array((0.0, 0.0, cfg.lift_speed * lift_elapsed))

                step_max = cfg.joint_speed * dt
                reference += np.clip(tgt - reference, -step_max, step_max)
                q = d.qpos[self.joint_q]
                qd = d.qvel[self.joint_v]
                tau = cfg.kp * (reference - q) - cfg.kd * qd
                tau = np.clip(tau, -tau_limit, tau_limit)
                d.qfrc_applied[:] = 0.0
                d.qfrc_applied[self.joint_v] = tau + d.qfrc_bias[self.joint_v]
                d.qfrc_applied[self.slide_v] = (
                    cfg.wrist_kp * (wrist_target - d.qpos[self.slide_q])
                    - cfg.wrist_kd * d.qvel[self.slide_v]
                    + d.qfrc_bias[self.slide_v]
                )
                mujoco.mj_step(m, d)
                t += dt
                step += 1

                if first_contact is None:
                    touching = self.touching()
                    if touching:
                        first_contact = touching[0]
                        first_contact_phase = phase
                if motion not in ("lift", "lifted"):
                    delta = self.bottle_pos() - bottle0
                    push = max(push, float(np.hypot(delta[0] - offset_world[0] * 0, delta[1])))
                    tilt = max(tilt, self.bottle_tilt_deg())
                if viewer is not None and step % render_every == 0:
                    if not viewer.is_running():
                        return {}
                    viewer.sync()
                    if realtime:
                        lag = t - (time.monotonic() - wall0)
                        if lag > 0:
                            time.sleep(lag)

        rise = float(self.bottle_pos()[2] - bottle0[2])
        lifted = rise > 0.6 * cfg.lift_height and self.bottle_tilt_deg() < 30.0
        return {
            "strategy": strategy.name,
            "offset_x_cm": round(100 * cfg.offset_x, 1),
            "offset_z_cm": round(100 * cfg.offset_z, 1),
            "push_cm": round(100 * push, 2),
            "tilt_deg": round(tilt, 1),
            "rise_cm": round(100 * rise, 1),
            "lifted": bool(lifted),
            "first": first_contact or "-",
            "first_phase": first_contact_phase or "-",
            "events": events,
        }


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def print_table(rows: Sequence[dict]) -> None:
    header = f"{'strategy':15s} {'dx':>5s} {'dz':>5s} {'push':>6s} {'tilt':>5s} {'rise':>5s}  {'lifted':6s}  first contact (phase)"
    print(header)
    print("-" * len(header))
    for r in rows:
        print(
            f"{r['strategy']:15s} {r['offset_x_cm']:5.1f} {r['offset_z_cm']:5.1f} "
            f"{r['push_cm']:6.2f} {r['tilt_deg']:5.1f} {r['rise_cm']:5.1f}  "
            f"{'YES' if r['lifted'] else 'no':6s}  {r['first']} ({r['first_phase']})"
        )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--robot-xml", type=Path, default=DEFAULT_ROBOT_XML)
    p.add_argument("--strategy", default="abcd", help="trigger | trigger_noabd | abcd | all")
    p.add_argument("--view", action="store_true", help="open the MuJoCo viewer (real time)")
    p.add_argument("--sweep", action="store_true", help="also vary bottle placement error")
    p.add_argument("--close-during-approach", action="store_true")
    p.add_argument("--offset-x", type=float, default=0.0, help="bottle error along palm normal [cm]")
    p.add_argument("--offset-z", type=float, default=0.0, help="bottle error along fingers [cm]")
    p.add_argument("--grasp-height", type=float, default=9.0, help="[cm] above the table")
    p.add_argument("--roll", type=float, default=0.0, help="hand roll about approach axis [deg]")
    p.add_argument("--torque-limit", type=float, default=0.35, help="[Nm] per finger joint")
    p.add_argument("--joint-speed", type=float, default=2.0, help="[rad/s] target slew limit")
    p.add_argument("--close-time", type=float, default=1.5, help="[s] trigger strategies")
    p.add_argument("--bottle-mass", type=float, default=0.35, help="[kg]")
    p.add_argument("--bottle-radius", type=float, default=3.3, help="[cm]")
    p.add_argument("--json", type=Path, help="write results to this JSON file")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    scene = Scene(bottle_mass=args.bottle_mass, bottle_radius=args.bottle_radius / 100.0)
    lab = Lab(args.robot_xml, scene)
    strategies = build_strategies(args.close_time)
    names = list(strategies) if args.strategy == "all" else [args.strategy]
    for name in names:
        if name not in strategies:
            print(f"Unknown strategy {name}; choose from {', '.join(strategies)} or all", file=sys.stderr)
            return 2

    base = TrialConfig(
        grasp_height=args.grasp_height / 100.0,
        roll_deg=args.roll,
        torque_limit=args.torque_limit,
        joint_speed=args.joint_speed,
        close_time=args.close_time,
        close_during_approach=args.close_during_approach,
        offset_x=args.offset_x / 100.0,
        offset_z=args.offset_z / 100.0,
    )
    offsets = [(base.offset_x, base.offset_z)]
    if args.sweep:
        offsets = [(dx / 100.0, dz / 100.0) for dx in (-1.0, 0.0, 1.5, 3.0) for dz in (-2.0, 0.0, 2.0)]

    if args.view:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(lab.model, lab.data) as viewer:
            viewer.cam.lookat[:] = (0.0, 0.0, 0.10)
            viewer.cam.distance = 0.75
            viewer.cam.azimuth = 200.0
            viewer.cam.elevation = -20.0
            while viewer.is_running():
                for name in names:
                    for dx, dz in offsets:
                        cfg = copy.replace(base, strategy=name, offset_x=dx, offset_z=dz) if hasattr(copy, "replace") else TrialConfig(**{**base.__dict__, "strategy": name, "offset_x": dx, "offset_z": dz})
                        print(f"\n▶ {name}: {strategies[name].description}  (dx={100*dx:.1f} cm, dz={100*dz:.1f} cm)", flush=True)
                        result = lab.run(cfg, strategies[name], viewer=viewer, realtime=True)
                        if not result:
                            return 0
                        print_table([result])
                        time.sleep(1.0)
        return 0

    rows = []
    for name in names:
        for dx, dz in offsets:
            cfg = TrialConfig(**{**base.__dict__, "strategy": name, "offset_x": dx, "offset_z": dz})
            rows.append(lab.run(cfg, strategies[name]))
    print_table(rows)
    if args.json:
        args.json.write_text(json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
