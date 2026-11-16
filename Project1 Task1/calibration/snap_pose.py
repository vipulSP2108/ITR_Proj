#!/usr/bin/env python3
"""Debug helper: optionally move joints 11-14 to raw counts, then save a camera snapshot.

    python calibration/snap_pose.py --out name.jpg                       # snapshot only
    python calibration/snap_pose.py --raw 0 2048 2048 2048 --out a.jpg   # move, then snapshot
    python calibration/snap_pose.py --gripper 2000 --out b.jpg
Several poses in one go:  --raw ... --raw ...   (outputs name_0.jpg, name_1.jpg ...)
"""
import os, sys, argparse
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hw_common import LiveCamera, connect_arm, read_joints, move_joints, gripper_goto, gripper_pos, T1_DIR, GRIPPER_ID

OUT = os.path.join(T1_DIR, "calibration", "output")
os.makedirs(OUT, exist_ok=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", type=int, nargs=4, action="append", metavar=("J11", "J12", "J13", "J14"))
    ap.add_argument("--gripper", type=int)
    ap.add_argument("--out", default="snap.jpg")
    ap.add_argument("--no-arm", action="store_true")
    a = ap.parse_args()

    cam = LiveCamera()
    arm = None
    try:
        if not a.no_arm:
            arm = connect_arm()
            print("start joints:", read_joints(arm), "gripper:", gripper_pos(arm))
            if a.gripper is not None:
                print("gripper ->", gripper_goto(arm, a.gripper))
        poses = a.raw or [None]
        for i, pose in enumerate(poses):
            if pose is not None and arm is not None:
                print(f"pose {i}: target {pose} -> reached {move_joints(arm, pose)}")
            name = a.out if len(poses) == 1 else a.out.replace(".jpg", f"_{i}.jpg")
            cv2.imwrite(os.path.join(OUT, name), cam.read())
            print("saved", os.path.join(OUT, name))
    finally:
        cam.release()
        if arm is not None:
            arm.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
