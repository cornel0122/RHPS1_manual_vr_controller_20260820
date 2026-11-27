#!/usr/bin/env python3
"""LEAP right hand: turn the on-site joint check into the leap_ring direction table (joint_map / joint_sign /
joint_offset_deg) and check what the ring A/B/C/D poses become on the real hand.

Field procedure: docs/LEAP_ring现场操作单.md.

    # 1. before going on site: a sheet to fill in (prefilled with the 2026-11-24 hypothesis)
    python3 tools/leap_ring_calibration.py template --prefill-1124 -o leap_ring_sheet.csv
    # 2. on site: fill in moved_joint / plus_is_model_plus / zero_cmd_deg, then
    python3 tools/leap_ring_calibration.py compute leap_ring_sheet.csv

Formula used by the controller (ViveHandBridge, leap_ring):
    slot command[i] = joint_sign[i] * model[joint_map[i]] + joint_offset_deg[i]      (degrees)
and the mc_rtc gripper then clamps slot i to the URDF range of the slot's OWN joint name.

Sheet columns (one row per slot = the joint name you command in RViz):
    slot               joint name commanded in RViz (fixed)
    moved_joint        which physical joint actually moved, using the model names (MCP_1 = index flexion at the
                       base, PIP_1 = index sideways, DIP_1, FINGERTIP_1, ... THUMB_CMC, THUMB_PIP, THUMB_DIP,
                       THUMB_FINGERTIP)
    plus_is_model_plus Y if increasing the slot command moved that physical joint in ITS model "+" direction
                       (flexion for MCP/DIP/FINGERTIP, away from the thumb for PIP_k, towards the palm for THUMB_CMC,
                       see the sheet), N if the opposite
    zero_cmd_deg       slot command [deg] at which the physical joint looks like the model at 0 (RViz screenshot
                       with every command at 0)
"""
import argparse
import csv
import sys

FINGER_JOINTS = [f"{kind}_{k}" for k in (1, 2, 3) for kind in ("MCP", "PIP", "DIP", "FINGERTIP")]
THUMB_JOINTS = ["THUMB_CMC", "THUMB_PIP", "THUMB_DIP", "THUMB_FINGERTIP"]
JOINTS = FINGER_JOINTS + THUMB_JOINTS

# Model "+" meaning (from MuJoCo forward kinematics, see docs/LEAP关节方向核对_测试步骤.md).
PLUS_MEANING = {
    "MCP": "根部弯曲 (flex at the base)",
    "PIP": "远离拇指侧摆 (sideways, away from the thumb)",
    "DIP": "中节弯曲 (flex)",
    "FINGERTIP": "指尖弯曲 (flex)",
    "THUMB_CMC": "拇指转向手心/对掌 (towards the palm)",
    "THUMB_PIP": "拇指绕自身轴转动 (rotation about its own axis)",
    "THUMB_DIP": "拇指弯曲 (flex)",
    "THUMB_FINGERTIP": "拇指指尖弯曲 (flex)",
}

# URDF limits [deg] of RHPS1main_leap_leap.urdf (same as leap_grasp_physics_lab.py, radians there).
URDF_LIMITS_DEG = {
    "MCP": (-17.99, 127.77),
    "PIP": (-59.99, 59.99),
    "DIP": (-28.99, 108.0),
    "FINGERTIP": (-20.97, 117.0),
    "THUMB_CMC": (-20.0, 119.98),
    "THUMB_PIP": (-26.93, 139.97),
    "THUMB_DIP": (-68.75, 108.86),
    "THUMB_FINGERTIP": (-76.78, 107.72),
}

# leap_ring keyframes [deg] in model order (controller/etc/ANAAvatarController.in.yaml).
RING = {
    "A": [20, 0, 10, 5, 20, 0, 10, 5, 20, 0, 10, 5, 80, 0, -30, 0],
    "B": [90, 0, 50, 70, 60, 0, 20, 5, 60, 0, 20, 5, 80, 0, -30, 0],
    "C": [90, 0, 50, 70, 60, 0, 20, 5, 60, 0, 20, 5, 85, 0, 60, 60],
    "D": [100, 0, 60, 75, 95, 0, 55, 35, 95, 0, 55, 35, 85, 0, 60, 60],
}

# 2026-11-24 real-robot test + URDF check: on the four fingers MCP_k and PIP_k are swapped, all reversed.
HYP_1124_MAP = [1, 0, 2, 3, 5, 4, 6, 7, 9, 8, 10, 11, 12, 13, 14, 15]
# "H1" offsets (2026-11-27, in the controller config): real angle = slot upper limit - command for the
# flexion-driving slots (the hand is bent at command 0 and the linear trigger reaches a fist); abduction
# slots (model abduction is always 0) at 0.
HYP_H1_ZERO = {"MCP": 0, "PIP": 60, "DIP": 108, "FINGERTIP": 117,
               "THUMB_CMC": 120, "THUMB_PIP": 139.9, "THUMB_DIP": 108.8, "THUMB_FINGERTIP": 107.7}
FIELDS = ["slot", "moved_joint", "plus_is_model_plus", "zero_cmd_deg", "plus_meaning_of_moved_joint", "note"]


def kind(name):
    return name if name.startswith("THUMB") else name.rsplit("_", 1)[0]


def limits(name):
    return URDF_LIMITS_DEG[kind(name)]


def cmd_template(args):
    rows = []
    for i, slot in enumerate(JOINTS):
        moved = JOINTS[HYP_1124_MAP[i]] if args.prefill_1124 else ""
        rows.append({
            "slot": slot,
            "moved_joint": moved,
            "plus_is_model_plus": "N" if args.prefill_1124 else "",
            "zero_cmd_deg": HYP_H1_ZERO[kind(slot)] if args.prefill_1124 else "",
            "plus_meaning_of_moved_joint": PLUS_MEANING[kind(moved)] if moved else "",
            "note": "11-24 + H1 hypothesis: check" if args.prefill_1124 else "",
        })
    out = open(args.output, "w", newline="", encoding="utf-8") if args.output else sys.stdout
    writer = csv.DictWriter(out, fieldnames=FIELDS)
    writer.writeheader()
    writer.writerows(rows)
    if args.output:
        out.close()
        print(f"wrote {args.output}")


def read_sheet(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    by_slot = {r["slot"].strip(): r for r in rows}
    missing = [s for s in JOINTS if s not in by_slot]
    if missing:
        sys.exit(f"sheet is missing slots: {missing}")
    return [by_slot[s] for s in JOINTS]


def cmd_compute(args):
    rows = read_sheet(args.sheet)
    problems, notes = [], []
    joint_map, joint_sign, offsets = [], [], []
    for i, row in enumerate(rows):
        slot = JOINTS[i]
        moved = row["moved_joint"].strip()
        if moved not in JOINTS:
            problems.append(f"{slot}: moved_joint '{moved}' is not a joint name")
            moved = slot
        yn = row["plus_is_model_plus"].strip().upper()
        if yn not in ("Y", "N"):
            problems.append(f"{slot}: plus_is_model_plus must be Y or N (got '{row['plus_is_model_plus']}')")
        zero = row["zero_cmd_deg"].strip()
        try:
            zero_val = float(zero) if zero else 0.0
        except ValueError:
            problems.append(f"{slot}: zero_cmd_deg '{zero}' is not a number")
            zero_val = 0.0
        if not zero:
            notes.append(f"{slot}: zero_cmd_deg not measured, using 0")
        joint_map.append(JOINTS.index(moved))
        joint_sign.append(1 if yn == "Y" else -1)
        offsets.append(round(zero_val, 1))

    if sorted(joint_map) != list(range(16)):
        dup = sorted({JOINTS[m] for m in joint_map if joint_map.count(m) > 1})
        problems.append(f"joint_map is not a permutation (each physical joint must be moved by exactly one slot); "
                        f"repeated: {dup}")

    print("=== leap_ring direction table (paste into controller/etc/ANAAvatarController.in.yaml) ===")
    print(f"      joint_map: [{', '.join(str(m) for m in joint_map)}]")
    print(f"      joint_sign: [{', '.join(str(s) for s in joint_sign)}]")
    print(f"      joint_offset_deg: [{', '.join(f'{o:g}' for o in offsets)}]")
    print("      direction_verified: false   # true only after the RViz slow preview of A/B/C/D on the robot")

    print("\n=== ring poses on the real hand: slot commands [deg] (* = clamped by the slot's URDF range) ===")
    clamped_total = 0
    header = f"{'slot':16s} {'drives':16s} " + " ".join(f"{k:>9s}" for k in RING)
    print(header)
    for i, slot in enumerate(JOINTS):
        lo, hi = limits(slot)
        cells = []
        for key, pose in RING.items():
            cmd = joint_sign[i] * pose[joint_map[i]] + offsets[i]
            clamped = not (lo - 1e-6 <= cmd <= hi + 1e-6)
            clamped_total += clamped
            cells.append(f"{cmd:8.1f}{'*' if clamped else ' '}")
        print(f"{slot:16s} {JOINTS[joint_map[i]]:16s} " + " ".join(cells))
    if clamped_total:
        problems.append(f"{clamped_total} slot command(s) fall outside the slot's URDF range and would be clamped: "
                        "the real pose would differ from the ring design. Re-check zero_cmd_deg, or the mapping must "
                        "be fixed in the LEAPHandPlugin configuration instead of the controller.")

    if notes:
        print("\nnotes:")
        for n in notes:
            print(f"  - {n}")
    if problems:
        print("\nPROBLEMS (do not enable the ring path until solved):")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("\nOK: permutation valid, no clamping. Next: RViz slow preview of A/B/C/D (direction_verified stays false).")
    return 0


STAGE_GROUPS = {
    "index": [0, 1, 2, 3],
    "middle": [4, 5, 6, 7],
    "ring": [8, 9, 10, 11],
    "others": [4, 5, 6, 7, 8, 9, 10, 11],
    "thumb": [12, 13, 14, 15],
}


def parse_pose(text, what):
    values = [float(v) for v in text.replace("[", " ").replace("]", " ").replace(",", " ").split()]
    if len(values) != 16:
        sys.exit(f"--{what} needs 16 values in slot order, got {len(values)}")
    return values


def cmd_from_poses(args):
    """Teach-by-demonstration: two poses saved on the real hand (RViz SAVE EDITOR AS ...) in SLOT order.

    A = open (released trigger); D = closed (fully pulled). B and C close the finger groups one after the
    other, following the ring design: index wraps first, then the thumb closes, then the others.
    The table is identity, so the keyframes are slot commands as taught (no model convention needed).
    """
    open_q, closed_q = parse_pose(args.open, "open"), parse_pose(args.closed, "closed")
    stages = [s.strip() for s in args.stages.split(",") if s.strip()]
    if len(stages) != 3 or any(s not in STAGE_GROUPS for s in stages):
        sys.exit(f"--stages needs three of {sorted(STAGE_GROUPS)}, e.g. index,thumb,others")
    frames, current = {"A": open_q}, list(open_q)
    for key, stage in zip(("B", "C", "D"), stages):
        for i in STAGE_GROUPS[stage]:
            current[i] = closed_q[i]
        frames[key] = list(current)
    if frames["D"] != closed_q:
        missing = [JOINTS[i] for i in range(16) if frames["D"][i] != closed_q[i]]
        sys.exit(f"the three stages do not cover every slot (missing {missing}); use e.g. index,thumb,others")

    clamped = []
    for key, pose in frames.items():
        for i, v in enumerate(pose):
            lo, hi = limits(JOINTS[i])
            if not lo - 1e-6 <= v <= hi + 1e-6:
                clamped.append(f"{key}:{JOINTS[i]}={v:g}")

    fmt = lambda q: ", ".join(f"{v:g}" for v in q)
    print("=== paste into leap_ring (controller/etc/ANAAvatarController.in.yaml) ===")
    print("      # Taught on the real hand (slot order, deg); identity table, so these are slot commands.")
    labels = {"A": "open (released trigger)", "B": f"{stages[0]} closed", "C": f"+ {stages[1]} closed",
              "D": "closed (fully pulled)"}
    for key in ("A", "B", "C", "D"):
        print(f"      {key}: [{fmt(frames[key])}]   # {labels[key]}")
    print(f"      breakpoints: [{', '.join(f'{b:g}' for b in args.breakpoints)}]")
    print("      joint_map: [0, 1, 2, 3,  4, 5, 6, 7,  8, 9, 10, 11,  12, 13, 14, 15]")
    print("      joint_sign: [1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1,  1, 1, 1, 1]")
    print("      joint_offset_deg: [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0]")
    print("      direction_verified: false   # true only after the RViz slow preview of A/B/C/D on the robot")
    if clamped:
        print("\nPROBLEMS: values outside the slot URDF range (would be clamped): " + ", ".join(clamped))
        return 1
    print("\nOK. Next: RViz slow preview of A/B/C/D, then unlock (direction_verified: true).")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("from-poses", help="build A/B/C/D from an open and a closed pose taught on the real hand")
    f.add_argument("--open", required=True, help="16 values [deg], slot order, as logged by SAVE EDITOR AS ...")
    f.add_argument("--closed", required=True, help="16 values [deg], slot order")
    f.add_argument("--stages", default="index,thumb,others",
                   help="order in which finger groups close between A and D (index, thumb, others, middle, ring)")
    f.add_argument("--breakpoints", type=float, nargs=4, default=[0.0, 0.30, 0.65, 1.0])
    t = sub.add_parser("template", help="write the sheet to fill in on site")
    t.add_argument("--prefill-1124", action="store_true", help="prefill moved_joint/direction with the 11-24 hypothesis")
    t.add_argument("-o", "--output")
    c = sub.add_parser("compute", help="compute the leap_ring table from a filled sheet")
    c.add_argument("sheet")
    args = parser.parse_args()
    if args.cmd == "template":
        cmd_template(args)
        return 0
    if args.cmd == "from-poses":
        return cmd_from_poses(args)
    return cmd_compute(args)


if __name__ == "__main__":
    sys.exit(main())
