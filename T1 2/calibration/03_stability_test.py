#!/usr/bin/env python3
"""
STEP 0b - Does anything slide while the arm moves?  (1 minute, nothing touches the table)

Takes a picture from the scan pose, swings the arm through large fast moves, returns to the
scan pose and takes a second picture. Compares the two: the ArUco marker (on the mat) and the
whole left and right halves of the picture. Everything should stay within ~1 px.

    python calibration/03_stability_test.py
"""
import os, sys, time
import numpy as np
import cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hw_common import LiveCamera, connect_arm, move_joints, T1_DIR, SCAN_RAW, SAFE
import importlib
cal = importlib.import_module("01_calibrate_camera_robot")

SCAN = SCAN_RAW
# big moves over the mat, all inside the allowed base range (never towards the camera stand)
SWINGS = [[-450, 2300, 1800, 2700], [400, 2300, 1800, 2700], [0, 2400, 1700, 2900],
          [-300, 2350, 1750, 2650], [450, 2350, 1750, 2650], [0, 2250, 1850, 2700]]


def shift(a, b):
    ga, gb = [cv2.cvtColor(x, cv2.COLOR_BGR2GRAY).astype(np.float32) for x in (a, b)]
    (dx, dy), _ = cv2.phaseCorrelate(ga, gb)
    return float(np.hypot(dx, dy))


def main():
    cam = LiveCamera()
    arm = connect_arm()
    try:
        move_joints(arm, SCAN); time.sleep(1.0)
        before = cam.read_avg(6)
        for rnd in range(2):
            for p in SWINGS:
                move_joints(arm, p, max_step=25, settle=0.1)
        move_joints(arm, SCAN); time.sleep(1.5)
        after = cam.read_avg(6)
    finally:
        cam.release()
        arm.disconnect(disable_torque=False)
    out = os.path.join(T1_DIR, "calibration", "output")
    cv2.imwrite(os.path.join(out, "stability_before.png"), before)
    cv2.imwrite(os.path.join(out, "stability_after.png"), after)
    def all_markers(img):                              # marker 4 (4x4) and, if present, the ChArUco board markers (6x6)
        out = {f"4x4 id{k}": v for k, v in cal.detect_markers(img).items()}
        det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50))
        c, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if ids is not None:
            out.update({f"board id{int(i)}": cc.reshape(4, 2) for cc, i in zip(c, ids.ravel())})
        return out
    mb, ma = all_markers(before), all_markers(after)
    ok = True
    for i in sorted(set(mb) & set(ma)):
        d = float(np.linalg.norm(mb[i] - ma[i], axis=1).max())
        ok &= d <= 1.5
        print(f"marker {i:12s} moved {d:.1f} px  {'OK' if d <= 1.5 else '<-- the MAT or the ROBOT BASE slid'}")
    if not (set(mb) & set(ma)):
        print("no marker seen in both pictures - put marker 4 flat on the mat in view of the camera")
        ok = False
    w = before.shape[1] // 2
    for name, sl in (("left half of the picture", np.s_[:, :w]), ("right half of the picture", np.s_[:, w:])):
        d = shift(before[sl], after[sl])
        ok &= d <= 1.5
        print(f"{name:26s} moved {d:.1f} px  {'OK' if d <= 1.5 else '<-- moved'}")
    print("STABLE - go ahead and calibrate" if ok else
          "NOT STABLE - clamp the robot base and tape the mat down, then run this test again")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
