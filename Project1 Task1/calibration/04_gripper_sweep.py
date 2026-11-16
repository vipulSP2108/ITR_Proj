#!/usr/bin/env python3
"""Measure how far apart the fingers are for each gripper count (run when the gripper behaves oddly).

The arm holds its straight-out pose; the gripper steps from its homed end stop across the whole
travel. In each picture the finger area that differs from the fully-closed picture is measured -
the step with the most change is fully OPEN. Prints the table and saves calibration/output/gripper_sweep.jpg.
"""
import os, sys, time
import numpy as np
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hw_common import LiveCamera, connect_arm, move_joints, gripper_goto, gripper_pos, T1_DIR, GRIP_OPEN_STOP


def main():
    cam, arm = LiveCamera(), connect_arm()
    rows, tiles = [], []
    try:
        move_joints(arm, [0, 2048, 2048, 2048]); time.sleep(0.5)
        gripper_goto(arm, GRIP_OPEN_STOP - 30, timeout=4); time.sleep(0.6)
        ref = cam.read_avg(4)
        cmd = GRIP_OPEN_STOP - 30
        while cmd > GRIP_OPEN_STOP - 4300:
            cmd -= 250
            got = gripper_goto(arm, cmd, timeout=4); time.sleep(0.5)
            img = cam.read_avg(3)
            d = (cv2.absdiff(img, ref).max(axis=2) > 40)
            ys, xs = np.nonzero(d)
            span = int(ys.max() - ys.min()) if len(ys) > 200 else 0
            rows.append((cmd, got, int(d.sum()), span))
            print(f"command {cmd:6d} -> at {got:6d}   changed pixels {int(d.sum()):6d}   vertical spread {span:4d} px", flush=True)
            t = cv2.resize(img, (320, 180)); cv2.putText(t, f"{cmd} ({got})", (5, 20), 0, 0.6, (0, 255, 255), 2); tiles.append(t)
            if abs(got - cmd) > 80:
                print("   stalled - end of travel")
                break
        gripper_goto(arm, GRIP_OPEN_STOP - 30, timeout=4)
    finally:
        cam.release(); arm.disconnect(disable_torque=False)
    while len(tiles) % 5:
        tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(os.path.join(T1_DIR, "calibration", "output", "gripper_sweep.jpg"),
                np.vstack([np.hstack(tiles[i:i + 5]) for i in range(0, len(tiles), 5)]))
    best = max(rows, key=lambda r: r[2])
    print(f"widest opening at command {best[0]} (position {best[1]})")


if __name__ == "__main__":
    main()
