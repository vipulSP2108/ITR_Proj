#!/usr/bin/env python3
"""
STEP 2 - Verify (and optionally fine-tune) the camera <-> robot calibration.

The camera finds every ArUco marker on the table, converts each marker centre to robot
coordinates with the calibration, and the arm puts its closed fingertip on that centre.
A photo of every touch is saved to calibration/output/verify_marker_<id>.jpg.

    python calibration/02_verify_calibration.py           # automatic: touch + photos
    python calibration/02_verify_calibration.py --jog     # you nudge the tip onto each centre
                                                          # (w/s = +/-X, a/d = +/-Y, 2 mm steps);
                                                          # the average nudge is stored as
                                                          # xy_correction_mm in the calibration file
    python calibration/02_verify_calibration.py --xy 200 50   # touch an arbitrary robot point

Place a ruler from the robot base if you want to check real millimetres: the script prints
the robot X/Y (mm, measured from the base rotation axis) of every point it touches.
"""
import os, sys, math, argparse, time
import numpy as np
import cv2
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hw_common import (T1_DIR, LiveCamera, connect_arm, move_joints, read_joints, gripper_goto,
                       GRIP_CLOSED, GRIP_OPEN, load_calib, ik_cmd)
from src.vision.camera_model import CameraModel
from src.planning import omx_kinematics as K

OUT = os.path.join(T1_DIR, "calibration", "output")
HOME = [0, 2048, 2048, 2048]
TOUCH_CLEARANCE = 25.0        # fingertip height above the mat (mm) - high enough never to drag on it
TILT = 45.0                   # tool angle below horizontal; tilted so the camera can see the fingertip


def detect_markers(img):
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    return {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.flatten())}


def touch(arm, calib, cm, x, y, hover=True):
    """Fingertip to (x, y), TOUCH_CLEARANCE above the mat. Returns the tool angle used, or None."""
    tip_len = calib["fingertip_len_mm"]
    z = cm.table_z + TOUCH_CLEARANCE
    raw, dn = ik_cmd(x, y, z, TILT, tip_len, calib, min_down_deg=20.0)
    if raw is None:
        return None
    if hover:
        raw_h, _ = ik_cmd(x, y, z + 60, dn, tip_len, calib, min_down_deg=20.0)
        if raw_h is not None:
            move_joints(arm, raw_h)
    move_joints(arm, raw, max_step=12)
    return dn


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jog", action="store_true")
    ap.add_argument("--tilt", type=float, default=45.0, help="tool angle below horizontal (90 = straight down)")
    ap.add_argument("--xy", type=float, nargs=2, action="append", help="extra robot point(s) to touch (mm)")
    a = ap.parse_args()
    global TILT
    TILT = a.tilt

    calib = load_calib()
    cm = CameraModel()
    cam = LiveCamera()
    arm = connect_arm()
    nudges = []
    try:
        cur = read_joints(arm)
        move_joints(arm, [cur[0]] + HOME[1:])
        gripper_goto(arm, GRIP_CLOSED)
        move_joints(arm, calib["scan_pose_raw"])
        time.sleep(0.5)
        scan = cam.read_avg(4)
        markers = detect_markers(scan)
        print("markers seen:", sorted(markers))
        targets = []
        for mid, c in sorted(markers.items()):
            u, v = c.mean(0)
            x, y = cm.pixel_to_robot(u, v)
            old = calib.get("markers", {}).get(mid)
            note = ""
            if old:
                d = math.hypot(x - old["robot_xy_mm"][0], y - old["robot_xy_mm"][1])
                note = f"  (moved {d:.1f} mm since calibration)" + ("  <-- camera or strip moved!" if d > 4 else "")
            print(f"  marker {mid}: pixel ({u:.0f},{v:.0f}) -> robot X={x:.1f} Y={y:.1f} mm{note}")
            targets.append((f"marker_{mid}", x, y, cm.robot_to_pixel(x, y, cm.table_z + TOUCH_CLEARANCE)))
        for i, (x, y) in enumerate(a.xy or []):
            targets.append((f"point_{i}", x, y, cm.robot_to_pixel(x, y, cm.table_z + TOUCH_CLEARANCE)))

        for name, x, y, (u, v) in targets:
            dn = touch(arm, calib, cm, x, y)
            if dn is None:
                print(f"  {name}: ({x:.0f},{y:.0f}) is out of reach - skipped")
                continue
            dx = dy = 0.0
            if a.jog:
                print(f"  {name}: nudge the fingertip onto the target. w/s = +X/-X, a/d = +Y/-Y, Enter = accept")
                while True:
                    k = input("    > ").strip().lower()
                    if k == "":
                        break
                    for ch in k:
                        dx += {"w": 2, "s": -2}.get(ch, 0)
                        dy += {"a": 2, "d": -2}.get(ch, 0)
                    touch(arm, calib, cm, x + dx, y + dy, hover=False)
                nudges.append((dx, dy))
                print(f"    accepted nudge dx={dx:+.0f} dy={dy:+.0f} mm")
            time.sleep(0.4)
            img = cam.read_avg(3)
            cv2.drawMarker(img, (int(u), int(v)), (0, 255, 255), cv2.MARKER_CROSS, 40, 1)
            cv2.circle(img, (int(u), int(v)), 18, (0, 255, 255), 1)
            cv2.putText(img, f"{name}: robot ({x:.0f},{y:.0f}) mm, tool {dn:.0f} deg down", (20, 40), 0, 0.9, (0, 255, 255), 2)
            cv2.imwrite(os.path.join(OUT, f"verify_{name}.jpg"), img)
            print(f"  {name}: touched robot ({x:.1f},{y:.1f}) -> calibration/output/verify_{name}.jpg")
            cur = read_joints(arm)                       # lift straight up before travelling on
            raw_h, _ = ik_cmd(x + dx, y + dy, cm.table_z + TOUCH_CLEARANCE + 60, dn, calib["fingertip_len_mm"], calib, min_down_deg=20.0)
            if raw_h is not None:
                move_joints(arm, raw_h)
        if a.jog and nudges:
            corr = np.mean(nudges, axis=0) + np.array(calib.get("xy_correction_mm", [0, 0]))
            calib["xy_correction_mm"] = [float(corr[0]), float(corr[1])]
            with open(os.path.join(T1_DIR, "config", "camera_robot_calib.yaml"), "w") as f:
                yaml.safe_dump(calib, f, sort_keys=False)
            print(f"saved average correction X={corr[0]:+.1f} Y={corr[1]:+.1f} mm")
        move_joints(arm, HOME)
        gripper_goto(arm, GRIP_OPEN)
    finally:
        cam.release()
        arm.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
